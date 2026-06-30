"""3D grade validator: load a LAS, pick a polygon, report slope, render heatmap."""
from __future__ import annotations

import argparse
import sys

import numpy as np

from analyzer import analyze, aspect_to_compass
from loader import load_las
from selector import pick_polygon_vertices
from visualizer import build_aspect_arrow, build_heatmap_cloud, show


def _maybe_downsample(
    points: np.ndarray, rgb: np.ndarray | None, max_points: int
) -> tuple[np.ndarray, np.ndarray | None]:
    if len(points) <= max_points:
        return points, rgb
    rng = np.random.default_rng(0)
    idx = rng.choice(len(points), size=max_points, replace=False)
    idx.sort()
    print(f"Downsampled {len(points):,} -> {max_points:,} points for interaction", file=sys.stderr)
    return points[idx], (rgb[idx] if rgb is not None else None)


def main() -> int:
    ap = argparse.ArgumentParser(description="3D grade validator for LAS point clouds.")
    ap.add_argument("las", help="Path to .las/.laz file")
    ap.add_argument("--max-points", type=int, default=1_500_000,
                    help="Random-downsample to at most this many points for rendering (default 1.5M)")
    ap.add_argument("--spread-deg", type=float, default=5.0,
                    help="Heatmap colormap range ±spread_deg around the mean slope (default 5°)")
    args = ap.parse_args()

    points, rgb = load_las(args.las)
    points, rgb = _maybe_downsample(points, rgb, args.max_points)

    # Recenter for numerical stability; LAS files often use large UTM coords.
    origin = points.mean(axis=0)
    points = points - origin

    polygon_xy = pick_polygon_vertices(points, rgb)
    print(f"Picked {len(polygon_xy)} polygon vertices")

    result = analyze(points, polygon_xy)

    print("\n=== Slope analysis ===")
    print(f"  points in selection : {int(result.mask.sum()):,}")
    print(f"  mean slope          : {result.mean_slope_deg:.2f}°  ({result.mean_slope_percent:.2f}%)")
    print(f"  aspect (descent)    : {result.aspect_deg:.1f}°  ({aspect_to_compass(result.aspect_deg)})")
    print(f"  plane z = {result.plane_coeffs[0]:.4f}*x + {result.plane_coeffs[1]:.4f}*y + {result.plane_coeffs[2]:.2f}")
    print(f"  per-point slope     : mean {result.point_slope_deg.mean():.2f}°, "
          f"std {result.point_slope_deg.std():.2f}°, "
          f"[{result.point_slope_deg.min():.2f}°, {result.point_slope_deg.max():.2f}°]")

    heatmap_pcd, outline = build_heatmap_cloud(
        points, result, polygon_xy, base_rgb=rgb, spread_deg=args.spread_deg
    )
    arrow = build_aspect_arrow(points, result)
    title = (
        f"Slope heatmap  |  mean {result.mean_slope_deg:.2f}°  "
        f"({result.mean_slope_percent:.2f}%)  |  descent -> "
        f"{aspect_to_compass(result.aspect_deg)} ({result.aspect_deg:.0f}°)"
    )
    show(heatmap_pcd, outline, title, arrow=arrow)
    return 0


if __name__ == "__main__":
    sys.exit(main())
