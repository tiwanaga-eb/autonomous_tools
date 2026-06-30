"""Compute local surface roughness from a height grid."""
import numpy as np
from scipy import ndimage


def _nanstd(values):
    """Compute nanstd for use with generic_filter."""
    v = values[~np.isnan(values)]
    return np.std(v) if len(v) > 1 else 0.0


def compute_roughness(height_grid: np.ndarray) -> np.ndarray:
    """Return per-cell local roughness as nanstd in a 3x3 neighbourhood."""
    roughness = ndimage.generic_filter(height_grid, _nanstd, size=3, mode="nearest")
    return roughness
