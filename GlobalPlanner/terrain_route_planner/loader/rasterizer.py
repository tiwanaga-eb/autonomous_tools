"""Rasterize point clouds or TIN surfaces into 2D height grids."""
import numpy as np
from scipy import ndimage

from config import GAP_FILL_RADIUS


def _nanmean_func(values):
    """Compute nanmean for use with generic_filter."""
    v = values[~np.isnan(values)]
    return np.mean(v) if len(v) > 0 else np.nan


def _fill_gaps(height_grid):
    """Fill NaN cells using a nanmean filter, then floor-fill remaining NaNs."""
    filled = ndimage.generic_filter(
        height_grid,
        _nanmean_func,
        size=GAP_FILL_RADIUS * 2 + 1,
        mode="nearest",
    )
    # Where original was NaN but filter gave a value, use it
    result = np.where(np.isnan(height_grid), filled, height_grid)
    # Any remaining NaN → nanmin
    min_val = np.nanmin(result)
    result = np.where(np.isnan(result), min_val, result)
    return result


def rasterize_pointcloud(points: np.ndarray, cell_size: float = 1.0, method: str = "mean") -> tuple:
    """Bin a (N,3) XYZ point cloud into a 2D height grid using fast np.bincount."""
    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()

    cols = int(np.ceil((x_max - x_min) / cell_size)) + 1
    rows = int(np.ceil((y_max - y_min) / cell_size)) + 1

    col_idx = np.floor((x - x_min) / cell_size).astype(np.int64)
    row_idx = np.floor((y - y_min) / cell_size).astype(np.int64)

    # Clip to valid range
    col_idx = np.clip(col_idx, 0, cols - 1)
    row_idx = np.clip(row_idx, 0, rows - 1)

    flat_idx = row_idx * cols + col_idx

    # Accumulate sum and count
    z_sum = np.bincount(flat_idx, weights=z, minlength=rows * cols)
    count = np.bincount(flat_idx, minlength=rows * cols)

    height_grid = np.full(rows * cols, np.nan, dtype=np.float64)
    mask = count > 0
    height_grid[mask] = z_sum[mask] / count[mask]
    height_grid = height_grid.reshape(rows, cols)

    height_grid = _fill_gaps(height_grid)

    geo_transform = {
        "x_min": x_min,
        "y_min": y_min,
        "cell_size": cell_size,
        "cols": cols,
        "rows": rows,
    }
    return height_grid, geo_transform


def rasterize_tin(vertices: np.ndarray, faces: np.ndarray, cell_size: float = 1.0) -> tuple:
    """Rasterize a TIN (vertices+faces) into a 2D height grid via barycentric interpolation."""
    x_v, y_v, z_v = vertices[:, 0], vertices[:, 1], vertices[:, 2]

    x_min, x_max = x_v.min(), x_v.max()
    y_min, y_max = y_v.min(), y_v.max()

    cols = int(np.ceil((x_max - x_min) / cell_size)) + 1
    rows = int(np.ceil((y_max - y_min) / cell_size)) + 1

    height_grid = np.full((rows, cols), np.nan, dtype=np.float64)

    # Cell centres
    cx = x_min + (np.arange(cols) + 0.5) * cell_size
    cy = y_min + (np.arange(rows) + 0.5) * cell_size

    for face in faces:
        i0, i1, i2 = face
        x0, y0, z0 = x_v[i0], y_v[i0], z_v[i0]
        x1, y1, z1 = x_v[i1], y_v[i1], z_v[i1]
        x2, y2, z2 = x_v[i2], y_v[i2], z_v[i2]

        # 2D bounding box of this triangle
        fx_min = min(x0, x1, x2)
        fx_max = max(x0, x1, x2)
        fy_min = min(y0, y1, y2)
        fy_max = max(y0, y1, y2)

        c0 = max(0, int(np.floor((fx_min - x_min) / cell_size)))
        c1 = min(cols - 1, int(np.ceil((fx_max - x_min) / cell_size)))
        r0 = max(0, int(np.floor((fy_min - y_min) / cell_size)))
        r1 = min(rows - 1, int(np.ceil((fy_max - y_min) / cell_size)))

        if c0 > c1 or r0 > r1:
            continue

        # Vectorised barycentric test for candidate cells
        cxs = cx[c0:c1 + 1]
        cys = cy[r0:r1 + 1]
        gx, gy = np.meshgrid(cxs, cys)

        denom = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(denom) < 1e-12:
            continue

        w0 = ((y1 - y2) * (gx - x2) + (x2 - x1) * (gy - y2)) / denom
        w1 = ((y2 - y0) * (gx - x2) + (x0 - x2) * (gy - y2)) / denom
        w2 = 1.0 - w0 - w1

        inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
        z_interp = w0 * z0 + w1 * z1 + w2 * z2

        sub = height_grid[r0:r1 + 1, c0:c1 + 1]
        sub[inside] = z_interp[inside]
        height_grid[r0:r1 + 1, c0:c1 + 1] = sub

    height_grid = _fill_gaps(height_grid)

    geo_transform = {
        "x_min": x_min,
        "y_min": y_min,
        "cell_size": cell_size,
        "cols": cols,
        "rows": rows,
    }
    return height_grid, geo_transform


def blend_grids(tin_grid: np.ndarray, las_grid: np.ndarray, geo_transform: dict) -> np.ndarray:
    """Blend TIN and LAS grids: TIN takes priority, LAS fills NaN gaps."""
    result = np.where(np.isnan(tin_grid), las_grid, tin_grid)
    return result
