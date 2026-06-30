"""terrain_route_planner — CLI entry point."""
import argparse
import os
import sys

# Allow running from the package directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

from config import DEFAULT_CELL_SIZE, SLOPE_LIMIT_DEG, OBSTACLE_COST


def world_to_rc(x_world, y_world, geo_transform):
    """Convert world (X, Y) coordinates to (row, col) grid indices."""
    cell_size = geo_transform["cell_size"]
    x_min = geo_transform["x_min"]
    y_min = geo_transform["y_min"]
    cols = geo_transform["cols"]
    rows = geo_transform["rows"]

    col = int((x_world - x_min) / cell_size)
    row = int((y_world - y_min) / cell_size)

    if not (0 <= row < rows and 0 <= col < cols):
        return None, None
    return row, col


def nearest_traversable(cost_map, target_world, geo_transform):
    """Find nearest traversable cell to a world-coordinate target."""
    rows_g, cols_g = cost_map.shape
    cell_size = geo_transform["cell_size"]
    x_min = geo_transform["x_min"]
    y_min = geo_transform["y_min"]

    # Try the requested position first; if OOB, clamp it
    col = int(np.clip((target_world[0] - x_min) / cell_size, 0, cols_g - 1))
    row = int(np.clip((target_world[1] - y_min) / cell_size, 0, rows_g - 1))

    # BFS to nearest traversable
    from collections import deque
    visited = np.zeros((rows_g, cols_g), dtype=bool)
    queue = deque([(row, col)])
    visited[row, col] = True
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while queue:
        r, c = queue.popleft()
        if cost_map[r, c] < OBSTACLE_COST:
            return r, c
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows_g and 0 <= nc < cols_g and not visited[nr, nc]:
                visited[nr, nc] = True
                queue.append((nr, nc))
    return None, None


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        prog="terrain_route_planner",
        description="Plan a traversable route across terrain from LAS/LandXML inputs.",
    )
    parser.add_argument("--las", metavar="PATH", help="LAS/LAZ point cloud file")
    parser.add_argument("--xml", metavar="PATH", help="LandXML TIN file")
    parser.add_argument("--start", metavar=("X", "Y"), nargs=2, type=float,
                        help="Start coordinate (world XY)")
    parser.add_argument("--goal", metavar=("X", "Y"), nargs=2, type=float,
                        help="Goal coordinate (world XY)")
    parser.add_argument("--slope", type=float, default=SLOPE_LIMIT_DEG,
                        help=f"Max traversable slope in degrees (default {SLOPE_LIMIT_DEG})")
    parser.add_argument("--cell", type=float, default=DEFAULT_CELL_SIZE,
                        help=f"Raster cell size in metres (default {DEFAULT_CELL_SIZE})")
    parser.add_argument("--export", action="store_true", help="Save route to output/")
    parser.add_argument("--no-3d", action="store_true", help="Skip Open3D 3D viewer")
    return parser.parse_args()


def main():
    """Main CLI execution flow."""
    args = parse_args()

    if not args.las and not args.xml:
        print("Error: Provide at least one of --las or --xml", file=sys.stderr)
        sys.exit(1)

    # --- File existence checks ---
    for path_attr, flag in [(args.las, "--las"), (args.xml, "--xml")]:
        if path_attr and not os.path.exists(path_attr):
            print(f"Error: File not found: {path_attr}", file=sys.stderr)
            sys.exit(1)

    # --- Imports ---
    from loader import load_las, load_landxml_tin, rasterize_pointcloud, rasterize_tin, blend_grids
    from analysis import compute_cost_map, get_traversable_mask
    from planner import find_route
    from smoother import smooth_route
    from visualizer import show_2d, show_3d
    from exporter import export_route

    points = None
    tin_vertices = None
    tin_faces = None
    las_n_points = 0
    tin_n_faces = 0

    # --- Step 1: Load LAS ---
    if args.las:
        points = load_las(args.las)
        las_n_points = len(points)

    # --- Step 2: Load LandXML TIN ---
    if args.xml:
        try:
            tin_vertices, tin_faces = load_landxml_tin(args.xml)
            tin_n_faces = len(tin_faces)
        except ValueError as e:
            print(f"Error: {e}", file=sys.stderr)
            sys.exit(1)

    # --- Step 3: Rasterize ---
    cell_size = args.cell
    las_grid = None
    tin_grid = None
    geo_transform = None

    if points is not None:
        print(f"Rasterizing LAS point cloud (cell={cell_size} m)...")
        las_grid, geo_transform = rasterize_pointcloud(points, cell_size=cell_size)

    if tin_vertices is not None:
        print(f"Rasterizing LandXML TIN (cell={cell_size} m)...")
        tin_grid, tin_geo = rasterize_tin(tin_vertices, tin_faces, cell_size=cell_size)
        if geo_transform is None:
            geo_transform = tin_geo

    if tin_grid is not None and las_grid is not None:
        # Blend: need grids on the same extent — use the larger one
        # For simplicity when both are provided, use TIN geo and rebuild las grid on that
        print("Blending TIN and LAS grids...")
        # Re-rasterize LAS on TIN's extent
        if points is not None:
            # Temporarily override to use tin_geo
            las_grid2, _ = rasterize_pointcloud(points, cell_size=cell_size)
            # Match extents (simple: use tin as base)
            rows_t, cols_t = tin_grid.shape
            rows_l, cols_l = las_grid2.shape
            rows = min(rows_t, rows_l)
            cols = min(cols_t, cols_l)
            height_grid = blend_grids(tin_grid[:rows, :cols], las_grid2[:rows, :cols], geo_transform)
            geo_transform = {
                "x_min": geo_transform["x_min"],
                "y_min": geo_transform["y_min"],
                "cell_size": cell_size,
                "cols": cols,
                "rows": rows,
            }
        else:
            height_grid = tin_grid
    elif tin_grid is not None:
        height_grid = tin_grid
        geo_transform = tin_geo
    else:
        height_grid = las_grid

    # --- Step 4: Compute cost map ---
    print("Computing cost map...")
    cost_map = compute_cost_map(height_grid, cell_size, slope_limit_deg=args.slope)
    traversable_mask = get_traversable_mask(cost_map)
    traversable_pct = 100.0 * traversable_mask.sum() / traversable_mask.size

    rows_g, cols_g = height_grid.shape

    # --- Step 5: Determine start/goal ---
    x_min = geo_transform["x_min"]
    y_min = geo_transform["y_min"]
    x_max = x_min + cols_g * cell_size
    y_max = y_min + rows_g * cell_size

    if args.start:
        sx, sy = args.start
        start_row, start_col = world_to_rc(sx, sy, geo_transform)
        if start_row is None:
            print(
                f"Error: Start ({sx}, {sy}) is out of grid bounds "
                f"X[{x_min:.1f}, {x_max:.1f}] Y[{y_min:.1f}, {y_max:.1f}]",
                file=sys.stderr,
            )
            sys.exit(1)
        start_rc = (start_row, start_col)
        start_world = (sx, sy)
    else:
        # Auto: nearest traversable to 10% inset from (x_min, y_min) corner
        x_off = (x_max - x_min) * 0.1
        y_off = (y_max - y_min) * 0.1
        start_row, start_col = nearest_traversable(cost_map, (x_min + x_off, y_min + y_off), geo_transform)
        if start_row is None:
            print("Error: No traversable start cell found.", file=sys.stderr)
            sys.exit(1)
        start_rc = (start_row, start_col)
        start_world = (x_min + x_off, y_min + y_off)

    if args.goal:
        gx, gy = args.goal
        goal_row, goal_col = world_to_rc(gx, gy, geo_transform)
        if goal_row is None:
            print(
                f"Error: Goal ({gx}, {gy}) is out of grid bounds "
                f"X[{x_min:.1f}, {x_max:.1f}] Y[{y_min:.1f}, {y_max:.1f}]",
                file=sys.stderr,
            )
            sys.exit(1)
        goal_rc = (goal_row, goal_col)
        goal_world = (gx, gy)
    else:
        # Auto: nearest traversable to 10% inset from (x_max, y_max) corner
        x_off = (x_max - x_min) * 0.1
        y_off = (y_max - y_min) * 0.1
        goal_row, goal_col = nearest_traversable(cost_map, (x_max - x_off, y_max - y_off), geo_transform)
        if goal_row is None:
            print("Error: No traversable goal cell found.", file=sys.stderr)
            sys.exit(1)
        goal_rc = (goal_row, goal_col)
        goal_world = (x_max - x_off, y_max - y_off)

    # --- Step 6: A* ---
    print(f"Running A* from {start_rc} to {goal_rc}...")
    route_rc = find_route(cost_map, start_rc, goal_rc, cell_size)
    if not route_rc:
        print(
            f"Error: No traversable path between start and goal. "
            f"Try --slope {int(args.slope) + 5} or verify both points are reachable.",
            file=sys.stderr,
        )
        sys.exit(1)

    # Spline smooth
    print("Smoothing route...")
    route_xyz = smooth_route(route_rc, height_grid, geo_transform)

    # Compute distance
    diffs = np.diff(route_xyz, axis=0)
    total_dist = float(np.sum(np.linalg.norm(diffs, axis=1)))

    # --- Step 7: Summary ---
    las_label = os.path.basename(args.las) if args.las else "N/A"
    xml_label = os.path.basename(args.xml) if args.xml else "N/A"

    print()
    print("─" * 45)
    print(f"Input LAS  : {las_label}  ({las_n_points:,} pts)")
    print(f"Input TIN  : {xml_label}  ({tin_n_faces:,} faces)")
    print(f"Grid       : {rows_g} × {cols_g} cells @ {cell_size} m/cell")
    print(f"Traversable: {traversable_pct:.1f} % of area")
    print(
        f"Start      : ({start_world[0]:.1f}, {start_world[1]:.1f}) "
        f"→ grid row={start_rc[0]}, col={start_rc[1]}"
    )
    print(
        f"Goal       : ({goal_world[0]:.1f}, {goal_world[1]:.1f}) "
        f"→ grid row={goal_rc[0]}, col={goal_rc[1]}"
    )
    print(f"Route      : {len(route_xyz)} waypoints, distance ≈ {total_dist:.1f} m")
    print("─" * 45)

    # --- Step 8: 2D visualisation ---
    show_2d(height_grid, cost_map, route_rc, start_rc, goal_rc, geo_transform)

    # --- Step 9: 3D visualisation ---
    if not args.no_3d:
        show_3d(points, cost_map, route_xyz, geo_transform)

    # --- Step 10: Export ---
    if args.export:
        meta = {
            "input_las": args.las,
            "input_tin": args.xml,
            "cell_size_m": cell_size,
            "slope_limit_deg": args.slope,
        }
        export_route(route_xyz, meta, out_dir="output")


if __name__ == "__main__":
    main()
