"""Render the heatmap: selected area colored by per-point slope deviation, rest grey."""
from __future__ import annotations

import numpy as np
import open3d as o3d
from matplotlib import cm
from matplotlib.colors import Normalize

from analyzer import SlopeResult


_GREY = np.array([0.55, 0.55, 0.55])
_ARROW_COLOR = np.array([1.0, 0.15, 0.0])  # vivid red-orange for visibility


def build_aspect_arrow(
    points: np.ndarray,
    result: SlopeResult,
) -> o3d.geometry.LineSet | None:
    """Return a LineSet arrow on the XY plane pointing in the steepest-descent direction.

    Placed just above the top of the selection so it is always visible.
    Returns None if the fitted plane is essentially horizontal.
    """
    a, b, _ = result.plane_coeffs
    descent_xy = np.array([-a, -b], dtype=np.float64)
    norm = float(np.linalg.norm(descent_xy))
    if norm < 1e-9:
        return None
    descent_xy /= norm

    inside_xy = points[result.mask, :2]
    inside_z = points[result.mask, 2]
    center_xy = inside_xy.mean(axis=0)
    extent = float(np.linalg.norm(inside_xy.max(axis=0) - inside_xy.min(axis=0)))
    shaft_len = extent * 0.45
    head_len = shaft_len * 0.25
    lift = float(np.ptp(inside_z)) * 0.1 + shaft_len * 0.02
    z = float(inside_z.max()) + lift

    start = np.array([center_xy[0] - descent_xy[0] * shaft_len * 0.5,
                      center_xy[1] - descent_xy[1] * shaft_len * 0.5, z])
    tip = np.array([center_xy[0] + descent_xy[0] * shaft_len * 0.5,
                    center_xy[1] + descent_xy[1] * shaft_len * 0.5, z])

    head_angle = np.deg2rad(25.0)
    # vectors from tip back along the shaft, rotated by ±head_angle
    back = -descent_xy
    c, s = np.cos(head_angle), np.sin(head_angle)
    left = np.array([c * back[0] - s * back[1], s * back[0] + c * back[1]])
    right = np.array([c * back[0] + s * back[1], -s * back[0] + c * back[1]])
    head_l = np.array([tip[0] + left[0] * head_len, tip[1] + left[1] * head_len, z])
    head_r = np.array([tip[0] + right[0] * head_len, tip[1] + right[1] * head_len, z])

    verts = np.array([start, tip, head_l, head_r])
    lines = [[0, 1], [1, 2], [1, 3]]
    ls = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(verts),
        lines=o3d.utility.Vector2iVector(lines),
    )
    ls.colors = o3d.utility.Vector3dVector(np.tile(_ARROW_COLOR, (len(lines), 1)))
    return ls


def build_heatmap_cloud(
    points: np.ndarray,
    result: SlopeResult,
    polygon_xy: np.ndarray,
    base_rgb: np.ndarray | None = None,
    cmap_name: str = "RdBu_r",
    spread_deg: float = 5.0,
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.LineSet]:
    """Build a colored PointCloud + LineSet (polygon outline) ready for rendering.

    Points outside the polygon keep their LAS RGB (or a neutral grey if unavailable).
    Points inside are colored by (local_slope - mean_slope) clipped to ±spread_deg.
    """
    if base_rgb is not None:
        colors = base_rgb.astype(np.float64, copy=True)
    else:
        colors = np.tile(_GREY, (len(points), 1))

    deviation = result.point_slope_deg - result.mean_slope_deg
    norm = Normalize(vmin=-spread_deg, vmax=spread_deg, clip=True)
    cmap = cm.get_cmap(cmap_name)
    rgba = cmap(norm(deviation))
    colors[result.mask] = rgba[:, :3]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Polygon outline drawn at the mean Z inside the selection, for visibility
    z_ref = float(points[result.mask, 2].mean())
    verts = np.column_stack([polygon_xy, np.full(len(polygon_xy), z_ref)])
    lines = [[i, (i + 1) % len(verts)] for i in range(len(verts))]
    outline = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(verts),
        lines=o3d.utility.Vector2iVector(lines),
    )
    outline.colors = o3d.utility.Vector3dVector(np.tile([1.0, 1.0, 0.0], (len(lines), 1)))

    return pcd, outline


def show(
    pcd: o3d.geometry.PointCloud,
    outline: o3d.geometry.LineSet,
    title: str,
    arrow: o3d.geometry.LineSet | None = None,
) -> None:
    geoms: list = [pcd, outline]
    if arrow is not None:
        geoms.append(arrow)
    o3d.visualization.draw_geometries(geoms, window_name=title)
