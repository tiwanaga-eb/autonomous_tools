"""Slope analysis: polygon masking, plane fit, per-point slope via local normals."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import open3d as o3d
from matplotlib.path import Path as MplPath


@dataclass
class SlopeResult:
    mask: np.ndarray              # bool (N,) — points inside the polygon
    mean_slope_deg: float         # best-fit plane slope angle (from horizontal)
    mean_slope_percent: float     # rise / run * 100
    aspect_deg: float             # compass bearing of steepest descent, 0°=N, clockwise
    plane_coeffs: tuple[float, float, float]  # z ≈ a*x + b*y + c
    point_slope_deg: np.ndarray   # (M,) per-point slope for points inside polygon


def points_in_polygon(xy: np.ndarray, polygon_xy: np.ndarray) -> np.ndarray:
    return MplPath(polygon_xy).contains_points(xy)


def _fit_plane(pts: np.ndarray) -> tuple[float, float, float]:
    """Least-squares fit z = a*x + b*y + c. Returns (a, b, c)."""
    A = np.column_stack([pts[:, 0], pts[:, 1], np.ones(len(pts))])
    coeffs, *_ = np.linalg.lstsq(A, pts[:, 2], rcond=None)
    return float(coeffs[0]), float(coeffs[1]), float(coeffs[2])


def _per_point_slope_deg(pts_inside: np.ndarray, knn: int = 20) -> np.ndarray:
    """Estimate a local plane per point (via Open3D normal estimation) and return slope angle."""
    sub = o3d.geometry.PointCloud()
    sub.points = o3d.utility.Vector3dVector(pts_inside)
    sub.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn))
    normals = np.asarray(sub.normals)
    # slope = angle between +Z and |normal|; use |nz| so orientation flips don't matter
    nz = np.clip(np.abs(normals[:, 2]), 0.0, 1.0)
    return np.degrees(np.arccos(nz))


def analyze(points: np.ndarray, polygon_xy: np.ndarray) -> SlopeResult:
    mask = points_in_polygon(points[:, :2], polygon_xy)
    n_inside = int(mask.sum())
    if n_inside < 10:
        raise RuntimeError(f"only {n_inside} points inside polygon; need >= 10")

    inside = points[mask]
    a, b, c = _fit_plane(inside)

    slope_tan = float(np.hypot(a, b))
    mean_slope_deg = float(np.degrees(np.arctan(slope_tan)))
    mean_slope_percent = slope_tan * 100.0

    # Compass aspect of steepest descent. +Y is North, +X is East.
    # Descent direction in XY = (-a, -b); bearing from north clockwise = atan2(east, north).
    aspect_deg = float((np.degrees(np.arctan2(-a, -b)) + 360.0) % 360.0)

    per_pt_deg = _per_point_slope_deg(inside)

    return SlopeResult(
        mask=mask,
        mean_slope_deg=mean_slope_deg,
        mean_slope_percent=mean_slope_percent,
        aspect_deg=aspect_deg,
        plane_coeffs=(a, b, c),
        point_slope_deg=per_pt_deg,
    )


def aspect_to_compass(aspect_deg: float) -> str:
    dirs = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    idx = int((aspect_deg + 11.25) // 22.5) % 16
    return dirs[idx]
