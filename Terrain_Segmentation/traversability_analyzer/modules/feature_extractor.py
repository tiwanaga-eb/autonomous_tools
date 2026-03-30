"""
feature_extractor.py - Extract terrain features from raster grids.

Features:
  slope         - slope in degrees (Sobel-based)
  roughness     - local std of elevation (fixed meter window)
  height_range  - DSM - DTM
  water_score   - [0, 1] based on HSV hue
  density       - point count per cell
"""
import numpy as np
from scipy.ndimage import sobel, generic_filter
from colorsys import rgb_to_hsv

from modules.grid_builder import fill_nan_with_mean_3x3


def extract(grids: dict, points: np.ndarray, config: dict) -> dict:
    """
    Extract features from raster grids.

    Args:
        grids:  output of grid_builder.build()
        points: ndarray (N, 6) [X, Y, Z, R, G, B]
        config: loaded config dict

    Returns:
        dict with keys: slope, roughness, height_range, water_score, density
    """
    grid_size      = float(config["grid"]["resolution_m"])
    rough_window_m = float(config["features"]["rough_window_m"])

    elev        = grids["elev"]        # (ny, nx)
    dsm         = grids["dsm"]         # (ny, nx)
    dtm         = grids["dtm"]         # (ny, nx)
    density_map = grids["density_map"] # (ny, nx)
    x_edges     = grids["x_edges"]
    y_edges     = grids["y_edges"]

    # Fill NaN before computing derivatives
    elev_filled = fill_nan_with_mean_3x3(elev)
    dsm_filled  = fill_nan_with_mean_3x3(dsm)
    dtm_filled  = fill_nan_with_mean_3x3(dtm)

    # --------------------------------------------------
    # Slope (degrees)
    # --------------------------------------------------
    dzdx = sobel(elev_filled, axis=1, mode="nearest") / grid_size
    dzdy = sobel(elev_filled, axis=0, mode="nearest") / grid_size
    slope_rad = np.arctan(np.hypot(dzdx, dzdy))
    slope_deg = np.degrees(slope_rad)

    # --------------------------------------------------
    # Roughness (local std, fixed meter window)
    # --------------------------------------------------
    k = max(3, round(rough_window_m / grid_size))
    if k % 2 == 0:
        k += 1
    roughness = generic_filter(elev_filled, np.nanstd, size=k, mode="mirror")
    print(f"[INFO] roughness window: {rough_window_m} m -> {k} cells")

    # --------------------------------------------------
    # Height range: DSM - DTM
    # --------------------------------------------------
    height_range = dsm_filled - dtm_filled
    height_range = np.clip(height_range, 0.0, None)  # should be non-negative

    # --------------------------------------------------
    # Water score (HSV hue-based)
    # --------------------------------------------------
    water_score = _compute_water_score(
        points, x_edges, y_edges, config
    )

    # --------------------------------------------------
    # Density
    # --------------------------------------------------
    density = density_map.copy()

    return {
        "slope":        slope_deg,
        "roughness":    roughness,
        "height_range": height_range,
        "water_score":  water_score,
        "density":      density,
        "elev_filled":  elev_filled,
    }


def _compute_water_score(
    points: np.ndarray,
    x_edges: np.ndarray,
    y_edges: np.ndarray,
    config: dict,
) -> np.ndarray:
    """
    Compute per-cell water score based on mean RGB -> HSV.

    Hue in [water_hue_min, water_hue_max] (degrees, 0-360)
    and Saturation < 0.4 → high water score.
    """
    hue_min   = float(config["thresholds"]["water_hue_min"])
    hue_max   = float(config["thresholds"]["water_hue_max"])

    nx = len(x_edges) - 1
    ny = len(y_edges) - 1

    x = points[:, 0]
    y = points[:, 1]
    r = points[:, 3]
    g = points[:, 4]
    b = points[:, 5]

    # Map points to grid cells
    xi = np.searchsorted(x_edges, x, side="right") - 1
    yi = np.searchsorted(y_edges, y, side="right") - 1
    xi = np.clip(xi, 0, nx - 1)
    yi = np.clip(yi, 0, ny - 1)
    row = (ny - 1) - yi
    row = np.clip(row, 0, ny - 1)

    # Accumulate RGB per cell
    r_sum  = np.zeros((ny, nx), dtype=float)
    g_sum  = np.zeros((ny, nx), dtype=float)
    b_sum  = np.zeros((ny, nx), dtype=float)
    count  = np.zeros((ny, nx), dtype=float)

    np.add.at(r_sum,  (row, xi), r)
    np.add.at(g_sum,  (row, xi), g)
    np.add.at(b_sum,  (row, xi), b)
    np.add.at(count,  (row, xi), 1.0)

    valid_cells = count > 0
    r_mean = np.where(valid_cells, r_sum / np.maximum(count, 1), 0.5)
    g_mean = np.where(valid_cells, g_sum / np.maximum(count, 1), 0.5)
    b_mean = np.where(valid_cells, b_sum / np.maximum(count, 1), 0.5)

    # Convert mean RGB to HSV per cell
    h_map = np.zeros((ny, nx), dtype=float)
    s_map = np.zeros((ny, nx), dtype=float)

    for iy in range(ny):
        for ix in range(nx):
            rv = float(np.clip(r_mean[iy, ix], 0, 1))
            gv = float(np.clip(g_mean[iy, ix], 0, 1))
            bv = float(np.clip(b_mean[iy, ix], 0, 1))
            h, s, _ = rgb_to_hsv(rv, gv, bv)
            h_map[iy, ix] = h * 360.0  # convert [0,1] -> [0,360]
            s_map[iy, ix] = s

    # Water condition: hue in [hue_min, hue_max] AND saturation < 0.4
    in_hue = (h_map >= hue_min) & (h_map <= hue_max)
    low_sat = s_map < 0.4

    water_score = np.where(in_hue & low_sat, 1.0, 0.0).astype(float)

    return water_score
