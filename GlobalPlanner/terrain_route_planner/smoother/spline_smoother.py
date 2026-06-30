"""Smooth a grid route into a continuous XYZ spline."""
import numpy as np
from scipy.interpolate import splprep, splev
from scipy.ndimage import map_coordinates

from config import SPLINE_SMOOTHING, SPLINE_POINTS


def smooth_route(route_rc: list, height_grid: np.ndarray, geo_transform: dict) -> np.ndarray:
    """Convert row/col route to a smoothed (M, 3) XYZ spline using B-spline interpolation."""
    cell_size = geo_transform["cell_size"]
    x_min = geo_transform["x_min"]
    y_min = geo_transform["y_min"]

    rows_arr = np.array([rc[0] for rc in route_rc], dtype=np.float64)
    cols_arr = np.array([rc[1] for rc in route_rc], dtype=np.float64)

    # Convert to world coordinates (cell centre)
    x_world = x_min + (cols_arr + 0.5) * cell_size
    y_world = y_min + (rows_arr + 0.5) * cell_size

    # Remove duplicate consecutive points (splprep requires unique points)
    coords = np.stack([x_world, y_world], axis=1)
    unique_mask = np.concatenate([[True], np.any(np.diff(coords, axis=0) != 0, axis=1)])
    x_world = x_world[unique_mask]
    y_world = y_world[unique_mask]

    if len(x_world) < 4:
        # Too few points for spline — return linear interpolation
        t = np.linspace(0, 1, SPLINE_POINTS)
        x_smooth = np.interp(t, np.linspace(0, 1, len(x_world)), x_world)
        y_smooth = np.interp(t, np.linspace(0, 1, len(y_world)), y_world)
    else:
        tck, _ = splprep([x_world, y_world], s=SPLINE_SMOOTHING, k=min(3, len(x_world) - 1))
        t = np.linspace(0, 1, SPLINE_POINTS)
        x_smooth, y_smooth = splev(t, tck)

    # Interpolate Z from height_grid at smoothed XY positions
    col_float = (x_smooth - x_min) / cell_size - 0.5
    row_float = (y_smooth - y_min) / cell_size - 0.5

    rows_g, cols_g = height_grid.shape
    col_float = np.clip(col_float, 0, cols_g - 1)
    row_float = np.clip(row_float, 0, rows_g - 1)

    z_smooth = map_coordinates(height_grid, [row_float, col_float], order=1, mode="nearest")

    return np.column_stack([x_smooth, y_smooth, z_smooth])
