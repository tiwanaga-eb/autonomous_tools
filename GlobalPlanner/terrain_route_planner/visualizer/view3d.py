"""3D Open3D visualization of point cloud, cost map colouring, and route."""
import numpy as np

from config import ROUTE_ELEV_OFFSET


def show_3d(
    points_xyz,
    cost_map: np.ndarray,
    route_xyz: np.ndarray,
    geo_transform: dict,
) -> None:
    """Display a 3D Open3D window with cost-coloured point cloud and route line."""
    try:
        import open3d as o3d
    except ImportError:
        print("open3d not installed — skipping 3D view. Run: pip install open3d")
        return

    geometries = []

    # --- Point cloud or mesh colouring ---
    if points_xyz is not None:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_xyz)

        # Map each point to cost map cell
        cell_size = geo_transform["cell_size"]
        x_min = geo_transform["x_min"]
        y_min = geo_transform["y_min"]
        rows_g, cols_g = cost_map.shape

        px, py = points_xyz[:, 0], points_xyz[:, 1]
        ci = np.clip(np.floor((px - x_min) / cell_size).astype(np.int64), 0, cols_g - 1)
        ri = np.clip(np.floor((py - y_min) / cell_size).astype(np.int64), 0, rows_g - 1)
        pt_costs = cost_map[ri, ci]

        colors = np.zeros((len(points_xyz), 3), dtype=np.float64)
        colors[pt_costs <= 2] = [0.2, 0.8, 0.2]       # green
        colors[(pt_costs > 2) & (pt_costs <= 5)] = [0.9, 0.8, 0.1]   # yellow
        colors[(pt_costs > 5) & (pt_costs <= 20)] = [0.9, 0.4, 0.1]  # orange
        colors[pt_costs > 20] = [0.3, 0.3, 0.3]       # dark gray (impassable)

        pcd.colors = o3d.utility.Vector3dVector(colors)
        geometries.append(pcd)
    else:
        # TIN-only mode: build a simple mesh-based height surface
        cell_size = geo_transform["cell_size"]
        x_min = geo_transform["x_min"]
        y_min = geo_transform["y_min"]
        rows_g, cols_g = cost_map.shape

        cx = x_min + (np.arange(cols_g) + 0.5) * cell_size
        cy = y_min + (np.arange(rows_g) + 0.5) * cell_size
        gx, gy = np.meshgrid(cx, cy)

        # Build a point cloud from grid centres — we don't have height_grid here
        # so we use a flat placeholder; caller can pass points_xyz from TIN grid
        print("TIN-only mode: 3D viewer will show cost-coloured grid points.")
        flat_pts = np.column_stack([gx.ravel(), gy.ravel(), np.zeros(rows_g * cols_g)])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(flat_pts)
        geometries.append(pcd)

    # --- Route line set ---
    if route_xyz is not None and len(route_xyz) > 1:
        route_lifted = route_xyz.copy()
        route_lifted[:, 2] += ROUTE_ELEV_OFFSET

        pts = o3d.utility.Vector3dVector(route_lifted)
        lines = [[i, i + 1] for i in range(len(route_lifted) - 1)]
        line_colors = [[1.0, 0.0, 0.0]] * len(lines)  # red

        ls = o3d.geometry.LineSet()
        ls.points = pts
        ls.lines = o3d.utility.Vector2iVector(lines)
        ls.colors = o3d.utility.Vector3dVector(line_colors)
        geometries.append(ls)

        # Start sphere — green
        start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=3.0)
        start_sphere.translate(route_lifted[0])
        start_sphere.paint_uniform_color([0.0, 0.8, 0.0])
        geometries.append(start_sphere)

        # Goal sphere — red
        goal_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=3.0)
        goal_sphere.translate(route_lifted[-1])
        goal_sphere.paint_uniform_color([0.8, 0.0, 0.0])
        geometries.append(goal_sphere)

    o3d.visualization.draw_geometries(
        geometries,
        window_name="Terrain Route Planner — 3D View",
        width=1280,
        height=720,
    )
