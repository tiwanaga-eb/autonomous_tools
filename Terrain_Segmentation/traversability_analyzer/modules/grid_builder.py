"""
grid_builder.py - Build DSM, DTM, elevation raster, and density map from point cloud.

Output:
  dsm          - (ny, nx) float64, Digital Surface Model (max Z)
  dtm          - (ny, nx) float64, Digital Terrain Model (local min-based)
  elev         - (ny, nx) float64, mean elevation of ground points
  density_map  - (ny, nx) float64, point count per cell
  x_edges      - (nx+1,) float64
  y_edges      - (ny+1,) float64
"""
import numpy as np
from scipy.stats import binned_statistic_2d
from scipy.ndimage import generic_filter


# ============================================================
# Utility (ported from existing code)
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


# ============================================================
# Grid builder
# ============================================================

def build(points: np.ndarray, config: dict) -> dict:
    """
    Build raster grids from point cloud.

    Args:
        points: ndarray (N, 6) [X, Y, Z, R, G, B]
        config: loaded config dict

    Returns:
        dict with keys: dsm, dtm, elev, density_map, x_edges, y_edges
    """
    grid_size   = float(config["grid"]["resolution_m"])
    local_buf   = float(config["ground_filter"]["local_buffer_m"])

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    assert_nonempty_finite("x", x)
    assert_nonempty_finite("y", y)
    assert_nonempty_finite("z", z)

    x_edges = make_edges(np.nanmin(x), np.nanmax(x), grid_size)
    y_edges = make_edges(np.nanmin(y), np.nanmax(y), grid_size)

    nx = len(x_edges) - 1
    ny = len(y_edges) - 1
    print(f"[INFO] Grid: nx={nx}, ny={ny}, resolution={grid_size} m")

    # --------------------------------------------------
    # DSM: max Z per cell
    # --------------------------------------------------
    dsm_xy, _, _, _ = binned_statistic_2d(
        x, y, z,
        statistic="max",
        bins=[x_edges, y_edges],
    )
    dsm = np.flipud(dsm_xy.T)  # (ny, nx), north-up

    # --------------------------------------------------
    # DTM: local min-based method
    # --------------------------------------------------
    # Step 1: min Z per cell
    dtm_raw_xy, _, _, _ = binned_statistic_2d(
        x, y, z,
        statistic="min",
        bins=[x_edges, y_edges],
    )
    dtm_raw = np.flipud(dtm_raw_xy.T)  # (ny, nx)

    # Step 2: fill NaN with 3x3 mean
    dtm_filled = fill_nan_with_mean_3x3(dtm_raw)

    # Step 3: ground mask — points within local_buffer_m of local DTM
    # Map each point to its grid cell
    xi = np.searchsorted(x_edges, x, side="right") - 1
    yi = np.searchsorted(y_edges, y, side="right") - 1

    # Clamp to valid range
    xi = np.clip(xi, 0, nx - 1)
    yi = np.clip(yi, 0, ny - 1)

    # dtm_filled is (ny, nx); row = ny - 1 - yi_grid, where yi_grid = yi index in y_edges order
    # y_edges is in ascending order, dtm_filled is north-up (flipud), so row = ny - 1 - yi
    row = (ny - 1) - yi
    row = np.clip(row, 0, ny - 1)

    dtm_at_point = dtm_filled[row, xi]
    ground_mask = z < (dtm_at_point + local_buf)

    ground_count = int(np.count_nonzero(ground_mask))
    print(f"[INFO] Ground points (local DTM): {ground_count} / {len(z)}")

    if ground_count < 10:
        print("[WARN] Too few ground points with local DTM. Falling back to global percentile.")
        fb_pct = float(config["ground_filter"]["fallback_global_percentile"])
        fb_buf = float(config["ground_filter"]["fallback_global_buffer_m"])
        z_thr = np.nanpercentile(z, fb_pct)
        ground_mask = z < (z_thr + fb_buf)
        ground_count = int(np.count_nonzero(ground_mask))
        print(f"[INFO] Fallback ground points: {ground_count}")

    xg = x[ground_mask]
    yg = y[ground_mask]
    zg = z[ground_mask]

    # Step 4: mean elevation of ground points
    elev_xy, _, _, _ = binned_statistic_2d(
        xg, yg, zg,
        statistic="mean",
        bins=[x_edges, y_edges],
    )
    elev = np.flipud(elev_xy.T)  # (ny, nx)

    # --------------------------------------------------
    # DTM (final): use ground-mean as DTM
    # --------------------------------------------------
    dtm = fill_nan_with_mean_3x3(elev)

    # --------------------------------------------------
    # Density map: count per cell (all points)
    # --------------------------------------------------
    density_xy, _, _, _ = binned_statistic_2d(
        x, y, z,
        statistic="count",
        bins=[x_edges, y_edges],
    )
    density_map = np.flipud(density_xy.T)  # (ny, nx)

    return {
        "dsm":         dsm,
        "dtm":         dtm,
        "elev":        elev,
        "density_map": density_map,
        "x_edges":     x_edges,
        "y_edges":     y_edges,
    }
