"""Compute traversability cost map from slope and roughness grids."""
import numpy as np

from config import SLOPE_LIMIT_DEG, SLOPE_WEIGHT, ROUGH_WEIGHT, OBSTACLE_COST
from .slope import compute_slope
from .roughness import compute_roughness


def compute_cost_map(
    height_grid: np.ndarray,
    cell_size: float,
    slope_limit_deg: float = SLOPE_LIMIT_DEG,
) -> np.ndarray:
    """Build a cost map combining slope and roughness; impassable cells get OBSTACLE_COST."""
    slope_deg = compute_slope(height_grid, cell_size)
    roughness = compute_roughness(height_grid)

    slope_norm = np.clip(slope_deg / slope_limit_deg, 0, None)
    cost = 1.0 + SLOPE_WEIGHT * slope_norm ** 2 + ROUGH_WEIGHT * roughness

    cost[slope_deg >= slope_limit_deg] = OBSTACLE_COST
    cost[np.isnan(height_grid)] = OBSTACLE_COST

    return cost


def get_traversable_mask(cost_map: np.ndarray) -> np.ndarray:
    """Return a boolean mask that is True where a cell is traversable."""
    return cost_map < OBSTACLE_COST
