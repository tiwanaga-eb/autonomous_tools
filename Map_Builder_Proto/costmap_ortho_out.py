import laspy
import numpy as np
from scipy.stats import binned_statistic_2d
from scipy.ndimage import sobel, generic_filter
import rasterio
from rasterio.transform import from_origin
from pyproj import CRS, Transformer

# ============================================================
# Utility
# ============================================================
def make_edges(vmin: float, vmax: float, step: float) -> np.ndarray:
    if step <= 0:
        raise ValueError("grid_size(step) must be > 0")
    if not np.isfinite(vmin) or not np.isfinite(vmax):
        raise ValueError("Invalid min/max (NaN or inf).")
    if vmax <= vmin:
        return np.array([vmin, vmin + step], dtype=float)
    edges = np.arange(vmin, vmax + step, step, dtype=float)
    if edges.size < 2:
        edges = np.array([vmin, vmin + step], dtype=float)
    return edges

def fill_nan_with_mean_3x3(arr: np.ndarray) -> np.ndarray:
    nan_mask = np.isnan(arr)
    if not nan_mask.any():
        return arr
    filtered = generic_filter(arr, np.nanmean, size=3, mode="mirror")
    out = arr.copy()
    out[nan_mask] = filtered[nan_mask]
    return out

def assert_nonempty_finite(name: str, a: np.ndarray) -> None:
    if a.size == 0:
        raise ValueError(f"{name} is empty.")
    if not np.isfinite(a).any():
        raise ValueError(f"{name} has no finite values (all NaN/Inf).")

def cost_to_rgb(cost_u8: np.ndarray,
                threshold: int = 20,
                nodata_mask: np.ndarray | None = None) -> np.ndarray:
    """
    cost_u8 (0-255) を RGB (uint8) に変換
      - 0..threshold: 緑→黄→赤 のグラデ
      - >threshold: グレー
      - nodata_mask=True はグレー（濃いめ）
    戻り: (H, W, 3) uint8
    """
    h, w = cost_u8.shape
    rgb = np.zeros((h, w, 3), dtype=np.uint8)

    # デフォルト：>threshold を薄いグレー
    rgb[:] = np.array([128, 128, 128], dtype=np.uint8)

    in_range = cost_u8 <= threshold
    t = np.zeros_like(cost_u8, dtype=np.float32)
    t[in_range] = cost_u8[in_range].astype(np.float32) / float(threshold)  # 0..1

    # 0..0.5: green(0,255,0) -> yellow(255,255,0)
    # 0.5..1: yellow(255,255,0) -> red(255,0,0)
    first = in_range & (t <= 0.5)
    second = in_range & (t > 0.5)

    # first half
    tt = np.zeros_like(t)
    tt[first] = (t[first] / 0.5)
    rgb[first, 0] = (255 * tt[first]).astype(np.uint8)     # R: 0 -> 255
    rgb[first, 1] = 255                                    # G: 255
    rgb[first, 2] = 0                                      # B: 0

    # second half
    tt[:] = 0
    tt[second] = ((t[second] - 0.5) / 0.5)
    rgb[second, 0] = 255                                   # R: 255
    rgb[second, 1] = (255 * (1.0 - tt[second])).astype(np.uint8)  # G: 255 -> 0
    rgb[second, 2] = 0

    # nodata は濃いグレー
    if nodata_mask is not None and nodata_mask.any():
        rgb[nodata_mask] = np.array([80, 80, 80], dtype=np.uint8)

    return rgb

# ============================================================
# User params
# ============================================================
# las_path = "merged_epsg6677.las"  # EPSG6677
las_path = "yoshimitsu.las"  # EPSG6677
# las_path = "pointcloud_0515_M4E_8060_100_EdgeRTK.las"  # EPSG4979 (とコメントあり)
out_tif  = "cost_map_yoshimitsu.tif"

grid_size = 0.1

ground_percentile = 1.0
ground_buffer_m   = 100.0

# ★重要：LASにEPSGが無いなら必ず入れる（例：6677, 6668, 4326など）
source_epsg = 6677

# 出力EPSG（Default）
target_epsg = 6677

nodata = 255 

# cost の色分け閾値
color_threshold = 60

# slope/rough の重み
w_slope = 500.0
w_rough = 100.0

# -----------------------------
# NEW: scale-stable parameters
# -----------------------------
# roughness を何mスケールで評価するか（grid_sizeを変えても物理窓は同じ）
rough_window_m = 1.0

# 正規化を max ではなく分位点で（grid_size変更でスケールが暴れにくい）
norm_percentile = 99.0

# 分位点で割ったあと 0..1 にクリップするか（推奨：True）
clip01 = True

# ============================================================
# Main
# ============================================================
def main():
    # -----------------------------
    # Read LAS
    # -----------------------------
    las = laspy.read(las_path)
    x = np.asarray(las.x, dtype=float)
    y = np.asarray(las.y, dtype=float)
    z = np.asarray(las.z, dtype=float)

    if x.size == 0:
        raise ValueError("LAS has no points.")

    # -----------------------------
    # Remove NaN/Inf points
    # -----------------------------
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[valid], y[valid], z[valid]
    assert_nonempty_finite("x", x)
    assert_nonempty_finite("y", y)
    assert_nonempty_finite("z", z)

    print(f"[INFO] points(after clean)={x.size}")
    print(f"[INFO] Z min/max={np.nanmin(z):.3f}/{np.nanmax(z):.3f}")

    # -----------------------------
    # CRS
    # -----------------------------
    src_crs = None
    epsg_from_las = getattr(las.header, "epsg", None)
    if epsg_from_las:
        try:
            src_crs = CRS.from_epsg(int(epsg_from_las))
            print(f"[INFO] LAS header EPSG: {epsg_from_las}")
        except Exception:
            src_crs = None

    if src_crs is None:
        if source_epsg is None:
            raise ValueError("Source CRS unknown. Set source_epsg.")
        src_crs = CRS.from_epsg(int(source_epsg))
        print(f"[INFO] source_epsg(manual): {source_epsg}")

    dst_crs = CRS.from_epsg(int(target_epsg))
    print(f"[INFO] target_epsg: {target_epsg}")

    # -----------------------------
    # Reproject XY if needed
    # -----------------------------
    if src_crs != dst_crs:
        transformer = Transformer.from_crs(src_crs, dst_crs, always_xy=True)
        x, y = transformer.transform(x, y)
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        print("[INFO] Reprojected XY to target CRS.")

    # -----------------------------
    # Ground filter (percentile+buffer)
    # -----------------------------
    z_thr = np.nanpercentile(z, ground_percentile)
    thr = z_thr + ground_buffer_m
    ground_mask = z < thr
    ground_count = int(np.count_nonzero(ground_mask))
    print(f"[INFO] z_thr={z_thr:.3f}, thr={thr:.3f}, ground_count={ground_count}")

    if ground_count < 10:
        raise ValueError("Too few ground points. Increase buffer/percentile.")

    xg, yg, zg = x[ground_mask], y[ground_mask], z[ground_mask]
    assert_nonempty_finite("xg", xg)
    assert_nonempty_finite("yg", yg)
    assert_nonempty_finite("zg", zg)

    # -----------------------------
    # Grid edges
    # -----------------------------
    x_edges = make_edges(np.nanmin(xg), np.nanmax(xg), grid_size)
    y_edges = make_edges(np.nanmin(yg), np.nanmax(yg), grid_size)

    nx = len(x_edges) - 1
    ny = len(y_edges) - 1
    print(f"[INFO] grid nx={nx}, ny={ny}, cell={grid_size}")

    # -----------------------------
    # Elevation raster (mean)
    # -----------------------------
    elev_xy, _, _, _ = binned_statistic_2d(
        xg, yg, zg,
        statistic="mean",
        bins=[x_edges, y_edges],
    )

    # (x,y)->(row=y,col=x), north-up
    elev = np.flipud(elev_xy.T)  # (ny, nx)

    # -----------------------------
    # slope / roughness / cost (scale-stable)
    # -----------------------------
    elev_filled = fill_nan_with_mean_3x3(elev)

    # 1) slope: m/m にする（grid_sizeで割る）
    dzdx = sobel(elev_filled, axis=1, mode="nearest") / grid_size
    dzdy = sobel(elev_filled, axis=0, mode="nearest") / grid_size
    slope = np.hypot(dzdx, dzdy)  # ≈ |grad z| [m/m]

    # 2) roughness: 固定メートル窓を grid に換算
    k = max(3, int(round(rough_window_m / grid_size)))
    if k % 2 == 0:
        k += 1
    roughness = generic_filter(elev_filled, np.nanstd, size=k, mode="mirror")  # [m]
    print(f"[INFO] roughness window: {rough_window_m} m -> {k} cells")

    # 3) 正規化: maxではなく分位点（外れ値耐性、解像度変更耐性）
    slope_ref = np.nanpercentile(slope, norm_percentile)
    rough_ref = np.nanpercentile(roughness, norm_percentile)

    slope_ref = slope_ref if np.isfinite(slope_ref) and slope_ref > 0 else 1e-6
    rough_ref = rough_ref if np.isfinite(rough_ref) and rough_ref > 0 else 1e-6

    slope_n = slope / slope_ref
    rough_n = roughness / rough_ref

    if clip01:
        slope_n = np.clip(slope_n, 0.0, 1.0)
        rough_n = np.clip(rough_n, 0.0, 1.0)

    cost = slope_n * w_slope + rough_n * w_rough

    print(f"[INFO] slope_ref(p{norm_percentile})={slope_ref:.6f}, rough_ref(p{norm_percentile})={rough_ref:.6f}")
    print(f"[INFO] cost min/max={np.nanmin(cost):.3f}/{np.nanmax(cost):.3f}")

    # NoData マスク（元の elev が NaN のセル）
    nodata_mask = np.isnan(elev)

    # cost を uint8 に
    cost[nodata_mask] = nodata
    cost_u8 = np.clip(cost, 0, 255).astype(np.uint8)

    # -----------------------------
    # Convert to RGB
    # -----------------------------
    rgb = cost_to_rgb(cost_u8, threshold=color_threshold, nodata_mask=nodata_mask)
    # rasterio は (bands, H, W)
    rgb_bands = np.moveaxis(rgb, 2, 0)  # (3, ny, nx)

    # -----------------------------
    # GeoTIFF write (RGB)
    # -----------------------------
    minx = x_edges[0]
    maxy = y_edges[-1]
    transform = from_origin(minx, maxy, grid_size, grid_size)

    with rasterio.open(
        out_tif,
        "w",
        driver="GTiff",
        height=rgb.shape[0],
        width=rgb.shape[1],
        count=3,
        dtype=rasterio.uint8,
        crs=dst_crs,
        transform=transform,
        compress="DEFLATE",
        photometric="RGB",
        interleave="pixel",
    ) as dst:
        dst.write(rgb_bands[0], 1)  # R
        dst.write(rgb_bands[1], 2)  # G
        dst.write(rgb_bands[2], 3)  # B

    print(f"[OK] Saved RGB GeoTIFF: {out_tif}")
    print(f"[OK] shape={rgb.shape}, EPSG={target_epsg}, threshold={color_threshold}")

if __name__ == "__main__":
    main()