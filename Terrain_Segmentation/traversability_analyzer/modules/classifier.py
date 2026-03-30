"""
classifier.py - Classify terrain traversability from features.

Classes (uint8):
  PASSABLE   = 0
  RISK       = 1
  IMPASSABLE = 2

Also computes continuous cost map (FMS-compatible, uint8 0-255).
"""
import numpy as np
from scipy.ndimage import binary_opening


PASSABLE   = 0
RISK       = 1
IMPASSABLE = 2


def classify(features: dict, grids: dict, config: dict) -> dict:
    """
    Classify each grid cell.

    Returns:
        dict with keys: class_map, cost_u8, nodata_mask
    """
    thr = config["thresholds"]
    out = config["output"]

    slope        = features["slope"]
    roughness    = features["roughness"]
    height_range = features["height_range"]
    water_score  = features["water_score"]
    density      = features["density"]
    elev         = grids["elev"]

    slope_bench_deg        = float(thr["slope_bench_deg"])
    slope_passable_deg     = float(thr["slope_passable_deg"])
    slope_risk_deg         = float(thr["slope_risk_deg"])
    roughness_risk_m       = float(thr["roughness_risk_m"])
    height_range_passable_m = float(thr["height_range_passable_m"])
    height_range_risk_m    = float(thr["height_range_risk_m"])
    density_min            = float(thr["density_min_points"])
    water_thresh           = float(thr["water_score_thresh"])

    w_slope = float(out["w_slope"])
    w_rough = float(out["w_rough"])
    norm_pct = float(config["features"]["norm_percentile"])

    ny, nx = slope.shape

    # NoData mask: cells where original elev is NaN
    nodata_mask = np.isnan(elev)

    # --------------------------------------------------
    # 優先度0: ベンチ面マスク生成
    # --------------------------------------------------
    is_bench = slope < slope_bench_deg

    # --------------------------------------------------
    # Class map (priority order)
    # --------------------------------------------------
    class_map = np.full((ny, nx), PASSABLE, dtype=np.uint8)

    # Priority 2: RISK
    risk_cond = (
        (slope > slope_passable_deg) |
        (roughness > roughness_risk_m) |
        (height_range > height_range_passable_m)
    )
    class_map[risk_cond] = RISK

    # Priority 1: IMPASSABLE (overrides RISK)
    impassable_cond = (
        ~is_bench |                              # 法面・急傾斜（ベンチ面以外）
        (height_range > height_range_risk_m) |
        (density < density_min) |
        (water_score > water_thresh)
    )
    class_map[impassable_cond] = IMPASSABLE

    # Apply nodata mask
    class_map[nodata_mask] = IMPASSABLE  # treat nodata as impassable in map

    # Morphology opening (3x3) to remove noise
    struct = np.ones((3, 3), dtype=bool)
    for cls in [RISK, IMPASSABLE]:
        mask = class_map == cls
        mask_opened = binary_opening(mask, structure=struct)
        # Pixels removed by opening revert to lower priority
        removed = mask & ~mask_opened
        if cls == IMPASSABLE:
            class_map[removed] = RISK
        else:  # RISK -> PASSABLE
            class_map[removed] = PASSABLE

    # Restore nodata mask after morphology
    class_map[nodata_mask] = IMPASSABLE

    # --------------------------------------------------
    # Continuous cost (FMS-compatible)
    # --------------------------------------------------
    # Normalize with percentile (scale-stable)
    slope_ref = np.nanpercentile(slope, norm_pct)
    rough_ref = np.nanpercentile(roughness, norm_pct)

    slope_ref = slope_ref if np.isfinite(slope_ref) and slope_ref > 0 else 1e-6
    rough_ref = rough_ref if np.isfinite(rough_ref) and rough_ref > 0 else 1e-6

    slope_n = np.clip(slope / slope_ref, 0.0, 1.0)
    rough_n = np.clip(roughness / rough_ref, 0.0, 1.0)

    cost = slope_n * w_slope + rough_n * w_rough

    cost[nodata_mask] = 255
    cost_u8 = np.clip(cost, 0, 255).astype(np.uint8)

    print(f"[INFO] slope_ref(p{norm_pct})={slope_ref:.6f}, rough_ref={rough_ref:.6f}")
    print(f"[INFO] cost min/max={np.nanmin(cost):.2f}/{np.nanmax(cost):.2f}")

    class_counts = {
        "passable":   int(np.sum((class_map == PASSABLE) & ~nodata_mask)),
        "risk":       int(np.sum((class_map == RISK)     & ~nodata_mask)),
        "impassable": int(np.sum((class_map == IMPASSABLE) & ~nodata_mask)),
        "nodata":     int(np.sum(nodata_mask)),
    }
    print(f"[INFO] Classes: {class_counts}")

    return {
        "class_map":   class_map,
        "cost_u8":     cost_u8,
        "nodata_mask": nodata_mask,
    }
