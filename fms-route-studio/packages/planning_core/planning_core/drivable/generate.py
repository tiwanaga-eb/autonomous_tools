"""走行可能領域（Drivable Area）の生成（設計書 §12）。

コストマップ(生cost) → 閾値 → 形態学(close/open) → 車両クリアランス収縮 →
連結成分フィルタ → 非破壊の人/AI編集(include/exclude) を適用 → boolean マスク。

走行可能領域はプランナーの「ハード制約」、cost は領域内の「ソフトコスト」(§9.5)。
編集は base とは別に保持し、閾値再生成しても手修正が残る（§12.4）。
"""
from __future__ import annotations

import numpy as np
from rasterio import features
from scipy.ndimage import (
    binary_closing,
    binary_erosion,
    binary_fill_holes,
    binary_opening,
    distance_transform_edt,
    label,
    median_filter,
)


def _cells(meters: float, gs: float) -> int:
    return max(0, int(round(meters / gs)))


def _remove_small(mask: np.ndarray, min_area_m2: float, gs: float) -> np.ndarray:
    if min_area_m2 <= 0 or not mask.any():
        return mask
    lab, n = label(mask)
    if n == 0:
        return mask
    counts = np.bincount(lab.ravel())
    min_cells = min_area_m2 / (gs * gs)
    keep = counts >= min_cells
    keep[0] = False  # background
    return keep[lab]


def _fill_small_holes(mask: np.ndarray, max_hole_m2: float, gs: float) -> np.ndarray:
    """面積 max_hole_m2 以下の閉じ穴（領域に囲まれた背景）を埋める。0以下なら何もしない。"""
    if max_hole_m2 <= 0 or not mask.any():
        return mask
    filled = binary_fill_holes(mask)
    holes = filled & ~mask
    if not holes.any():
        return mask
    lab, n = label(holes)
    counts = np.bincount(lab.ravel())
    max_cells = max_hole_m2 / (gs * gs)
    small = np.where(counts <= max_cells)[0]
    small = small[small != 0]
    return mask | np.isin(lab, small)


def _keep_largest(mask: np.ndarray) -> np.ndarray:
    """最大の連結成分だけ残す（島ノイズを捨てて1つの走行可能領域に）。"""
    if not mask.any():
        return mask
    lab, n = label(mask)
    if n <= 1:
        return mask
    counts = np.bincount(lab.ravel())
    counts[0] = 0
    return lab == int(counts.argmax())


def _apply_edits(mask: np.ndarray, edits, transform, shape) -> np.ndarray:
    """非破壊編集: include=領域追加 / exclude=領域除外（world座標ポリゴン）。"""
    for e in edits or []:
        ring = e.get("polygon")
        op = e.get("op")
        if not ring or len(ring) < 3:
            continue
        coords = [[float(p[0]), float(p[1])] for p in ring]
        coords.append(coords[0])  # close ring
        geom = {"type": "Polygon", "coordinates": [coords]}
        r = features.rasterize(
            [(geom, 1)], out_shape=shape, transform=transform, fill=0, dtype="uint8"
        ).astype(bool)
        if op == "include":
            mask = mask | r
        elif op == "exclude":
            mask = mask & ~r
    return mask


def generate_drivable(
    cost: np.ndarray,
    transform,
    *,
    threshold: float,
    obstacle_value: float = 1e9,
    close_m: float = 1.0,
    open_m: float = 0.5,
    min_area_m2: float = 20.0,
    clearance_m: float = 0.0,
    smooth_m: float = 0.0,
    max_hole_m2: float = 0.0,
    keep_largest: bool = False,
    edits=None,
    base_override: np.ndarray | None = None,
) -> tuple[np.ndarray, dict]:
    """Return (drivable_mask[bool], stats)。

    パイプライン: 閾値 → close(隙間橋渡し) → open(スペックル除去) → smooth(境界平滑) →
    fill holes(小穴埋め) → erosion(車両クリアランス) → 小領域除去/最大領域のみ → 人/AI編集。
    """
    cost = np.asarray(cost, dtype=float)
    gs = abs(transform.a)  # cell size [m]

    # base_override（CV/AI セグメンテーションの基底）があれば閾値の代わりに使う。
    # 以降の close/open/smooth/fill/clearance/連結成分/編集は共通パイプラインを通す。
    if base_override is not None:
        base = np.asarray(base_override, dtype=bool) & (cost < obstacle_value)
    else:
        base = (cost <= threshold) & (cost < obstacle_value)

    ci, oi, ei = _cells(close_m, gs), _cells(open_m, gs), _cells(clearance_m, gs)
    if ci > 0:
        base = binary_closing(base, iterations=ci)
    if oi > 0:
        base = binary_opening(base, iterations=oi)
    si = _cells(smooth_m, gs)
    if si > 0:
        base = median_filter(base.astype(np.uint8), size=2 * si + 1) > 0
    base = _fill_small_holes(base, max_hole_m2, gs)

    # 連結成分フィルタ（小領域除去・最大領域）は **erosion の前**に行う。
    # erosion(車両クリアランス)を先にやると細い首が切れて keep_largest が向こう側の正当な
    # 走行可能域を丸ごと捨ててしまう。面積閾値も収縮前の実地面で測る（レビュー指摘）。
    mask = _remove_small(base, min_area_m2, gs)
    if keep_largest:
        mask = _keep_largest(mask)
    if ei > 0:
        mask = binary_erosion(mask, iterations=ei)  # 最後に車両クリアランス収縮
    mask = _apply_edits(mask, edits, transform, cost.shape)
    return mask, drivable_stats(mask, gs)


def drivable_stats(mask: np.ndarray, gs: float) -> dict:
    lab, n = label(mask)
    area = float(mask.sum()) * gs * gs
    largest = 0.0
    width_median = 0.0
    width_max = 0.0
    if n > 0 and mask.any():
        counts = np.bincount(lab.ravel())[1:]
        if counts.size:
            largest = float(counts.max()) * gs * gs
        # 道幅 ≈ 2 × (走行可能セルから縁までの距離)
        dt = distance_transform_edt(mask) * gs
        widths = 2.0 * dt[mask]
        width_median = float(np.median(widths))
        width_max = float(widths.max())
    hole_count = 0
    if mask.any():
        holes = binary_fill_holes(mask) & ~mask
        if holes.any():
            hole_count = int(label(holes)[1])
    return {
        "area_m2": round(area, 1),
        "island_count": int(n),
        "hole_count": hole_count,
        "largest_island_m2": round(largest, 1),
        "width_median_m": round(width_median, 2),
        "width_max_m": round(width_max, 2),
    }


def mask_to_rgba(mask: np.ndarray, color=(34, 197, 94), alpha: int = 200) -> np.ndarray:
    """走行可能領域を半透明の単色RGBAに（範囲外は透明）。表示用。"""
    h, w = mask.shape
    rgba = np.zeros((h, w, 4), dtype=np.uint8)
    rgba[mask, 0] = color[0]
    rgba[mask, 1] = color[1]
    rgba[mask, 2] = color[2]
    rgba[mask, 3] = alpha
    return rgba
