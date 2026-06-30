"""Compute slope angle in degrees from a height grid using Sobel gradient."""
import numpy as np
from scipy import ndimage


def compute_slope(height_grid: np.ndarray, cell_size: float) -> np.ndarray:
    """Return per-cell slope angle in degrees; NaN input cells map to 90 degrees."""
    # Replace NaN with local fill to avoid gradient contamination
    filled = np.where(np.isnan(height_grid), 0.0, height_grid)

    dy = ndimage.sobel(filled, axis=0) / (8.0 * cell_size)
    dx = ndimage.sobel(filled, axis=1) / (8.0 * cell_size)

    gradient_magnitude = np.sqrt(dx ** 2 + dy ** 2)
    slope_deg = np.degrees(np.arctan(gradient_magnitude))

    # NaN cells are impassable → 90°
    slope_deg[np.isnan(height_grid)] = 90.0

    return slope_deg
