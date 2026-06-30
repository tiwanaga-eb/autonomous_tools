"""LAS / point cloud -> raw scalar cost (float32) + DTM (ground elevation).

設計書 §11 / §21 (B1, B3) + ユーザーフィードバック反映。

改良点（旧 costmap_ortho_out.py の課題を是正）:
- 地表抽出: セル内 **最小Z = 地表(DTM)** を使い、樹冠/構造物の混入を避ける
  （平均Zだと植生天端を含み傾斜・粗さがノイズ化していた）。
- 粗さ: セル内 **(最大Z − 最小Z)** を「植生・段差」の指標に（バラ地の道は低、植生は高）。
- 正規化: **物理スケール**（slope[deg]/slope_limit, 高低差/canopy_ref）で安定化
  （p99分位点正規化は外れ値でコストが一様化していた → 廃止）。
- slope は np.gradient による物理 m/m（未較正 sobel を是正）。
- 出力 cost(float32) は走行判定の「正」、RGB は colorize.py（表示専用）。
"""
from __future__ import annotations

import numpy as np
from rasterio.transform import from_origin
from scipy.interpolate import griddata
from scipy.ndimage import generic_filter, uniform_filter
from scipy.stats import binned_statistic_2d

from ..models.costmap import CostmapParams


def _make_edges(vmin: float, vmax: float, step: float) -> np.ndarray:
    if step <= 0:
        raise ValueError("grid_size_m must be > 0")
    if not (np.isfinite(vmin) and np.isfinite(vmax)):
        raise ValueError("Invalid min/max (NaN or inf).")
    if vmax <= vmin:
        return np.array([vmin, vmin + step], dtype=float)
    edges = np.arange(vmin, vmax + step, step, dtype=float)
    if edges.size < 2:
        edges = np.array([vmin, vmin + step], dtype=float)
    return edges


def _fill_nan_3x3(arr: np.ndarray) -> np.ndarray:
    nan = np.isnan(arr)
    if not nan.any():
        return arr
    filled = generic_filter(arr, np.nanmean, size=3, mode="mirror")
    out = arr.copy()
    out[nan] = filled[nan]
    return out


def _binned(x, y, z, xe, ye, stat) -> np.ndarray:
    """north-up の (ny, nx) 統計グリッド。empty セルは NaN。"""
    g, _, _, _ = binned_statistic_2d(x, y, z, statistic=stat, bins=[xe, ye])
    return np.flipud(g.T)


def elevation_grid(x, y, z, grid_size: float):
    """Mean-Z raster on a north-up grid. Returns (elev[ny,nx], rasterio_transform)。

    （後方互換のため残置。コスト生成は地表=最小Zを使う build_costmap_arrays を参照）。
    """
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    z = np.asarray(z, float)
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[valid], y[valid], z[valid]
    if x.size == 0:
        raise ValueError("No finite points.")
    xe = _make_edges(np.nanmin(x), np.nanmax(x), grid_size)
    ye = _make_edges(np.nanmin(y), np.nanmax(y), grid_size)
    elev = _binned(x, y, z, xe, ye, "mean")
    transform = from_origin(xe[0], ye[-1], grid_size, grid_size)
    return elev, transform


def _local_std(arr: np.ndarray, k: int) -> np.ndarray:
    """局所標準偏差（uniform_filter による高速版）。地表の凹凸＝粗さの指標。"""
    k = max(3, int(k))
    base = np.nanmean(arr) if np.isfinite(arr).any() else 0.0
    a = np.nan_to_num(arr, nan=float(base))
    m = uniform_filter(a, size=k, mode="nearest")
    m2 = uniform_filter(a * a, size=k, mode="nearest")
    return np.sqrt(np.clip(m2 - m * m, 0.0, None))


def _interpolate_dtm(zmin: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """疎な セル最小Z(zmin) から連続DTMを線形補間。凸包外は最近傍で埋める。

    Returns (ground, inside_hull)。inside_hull=線形補間が定義された(=データ凸包内)領域。
    """
    ny, nx = zmin.shape
    valid = np.isfinite(zmin)
    rr, cc = np.nonzero(valid)
    vals = zmin[valid]
    pts = np.column_stack([rr, cc]).astype(float)
    gy, gx = np.mgrid[0:ny, 0:nx]
    xi = np.column_stack([gy.ravel(), gx.ravel()]).astype(float)
    lin = griddata(pts, vals, xi, method="linear").reshape(ny, nx)
    inside = np.isfinite(lin)
    near = griddata(pts, vals, xi, method="nearest").reshape(ny, nx)
    ground = np.where(inside, lin, near)
    return ground, inside


def slope_grid(elev: np.ndarray, grid_size: float) -> np.ndarray:
    """Slope magnitude |grad z| in m/m, calibrated central differences."""
    e = _fill_nan_3x3(elev)
    gy, gx = np.gradient(e, grid_size)
    return np.hypot(gx, gy)


def roughness_grid(elev: np.ndarray, grid_size: float, window_m: float) -> np.ndarray:
    """Local std-dev over a physical window（後方互換のため残置）。"""
    e = _fill_nan_3x3(elev)
    k = max(3, int(round(window_m / grid_size)))
    if k % 2 == 0:
        k += 1
    return generic_filter(e, np.nanstd, size=k, mode="mirror")


def build_costmap_arrays(x, y, z, params: CostmapParams | None = None) -> dict:
    """点群 -> 生cost(float32) + DTM + 中間配列。

    返り値 keys: cost, dsm(=DTM 地表標高), transform, nodata_mask, slope, roughness(=セル内高低差)。
    """
    params = params or CostmapParams()
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    z = np.asarray(z, float)
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[valid], y[valid], z[valid]
    if x.size == 0:
        raise ValueError("No finite points.")

    gs = params.grid_size_m
    xe = _make_edges(x.min(), x.max(), gs)
    ye = _make_edges(y.min(), y.max(), gs)
    transform = from_origin(xe[0], ye[-1], gs, gs)

    # 地表(DTM)近似: 厳密な最小Zは1点の外れ値（地中ノイズ/マルチパス）に弱いので、
    # セル内の低パーセンタイル(既定5%)を地表とする（ground_percentile=0で従来のmin）。
    pgl = params.ground_percentile
    if pgl and pgl > 0:
        zmin = _binned(x, y, z, xe, ye, lambda v: float(np.percentile(v, pgl)) if len(v) else np.nan)
    else:
        zmin = _binned(x, y, z, xe, ye, "min")
    zmax = _binned(x, y, z, xe, ye, "max")   # 天端

    # 点数が少なすぎるセルは地表推定が不信頼 → NaN にして補間/穴埋めに委ねる。
    if params.min_points_per_cell and params.min_points_per_cell > 1:
        cnt = _binned(x, y, z, xe, ye, "count")
        zmin = np.where(np.nan_to_num(cnt, nan=0.0) >= params.min_points_per_cell, zmin, np.nan)

    valid = np.isfinite(zmin)
    frac_valid = float(valid.mean()) if valid.size else 0.0

    if frac_valid >= 0.6 or not valid.any():
        # 密: セル最小Zをそのまま使い、小さな穴だけ埋める（高速）
        ground = _fill_nan_3x3(_fill_nan_3x3(zmin))
        nodata_mask = ~valid
    else:
        # 疎: 点(セル最小Z)から連続DTMを補間（fine grid でも slope が意味を持つ）
        ground, inside_hull = _interpolate_dtm(zmin)
        nodata_mask = ~inside_hull

    canopy = np.where(np.isfinite(zmax) & np.isfinite(zmin), zmax - zmin, 0.0)  # セル内高低差（密で有効）

    slope = slope_grid(ground, gs)                    # m/m（地表ベース）
    slope_deg = np.degrees(np.arctan(slope))

    # 境界アーティファクト除去: nodata（凸包外/穴）を NN/補間で埋めた縁は段差→偽の急斜面になり、
    # それが hard-obstacle 化してデータ縁に壁を作る。データ側1セルの縁リングの slope を無効化。
    if nodata_mask.any() and (~nodata_mask).any():
        from scipy.ndimage import binary_dilation

        edge_ring = binary_dilation(nodata_mask, iterations=1) & ~nodata_mask
        slope_deg = slope_deg.copy()
        slope_deg[edge_ring] = 0.0

    # 粗さ = DTM局所標準偏差（任意密度で機能）と セル内高低差(植生) の大きい方
    k = max(3, int(round(params.rough_window_m / gs)))
    rough_local = _local_std(ground, k)
    rough_signal = np.maximum(rough_local, np.nan_to_num(canopy))

    slope_limit = params.slope_limit_deg if params.slope_limit_deg is not None else 20.0
    slope_n = np.clip(slope_deg / max(slope_limit, 1e-6), 0.0, 1.0)
    rough_n = np.clip(rough_signal / max(params.canopy_ref_m, 1e-6), 0.0, 1.0)

    cost = (params.w_slope * slope_n + params.w_rough * rough_n).astype(np.float32)

    if params.slope_limit_deg is not None:
        cost[slope_deg > params.slope_limit_deg] = np.float32(params.obstacle_value)
    cost[nodata_mask] = np.float32(params.obstacle_value)

    dsm = ground.astype(np.float32)
    dsm[nodata_mask] = np.nan
    return {
        "cost": cost,
        "dsm": dsm,
        "transform": transform,
        "nodata_mask": nodata_mask,
        "slope": slope,
        "roughness": rough_signal,
    }
