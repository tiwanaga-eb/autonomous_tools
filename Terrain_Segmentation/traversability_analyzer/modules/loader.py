"""
loader.py - Point cloud loader for .las/.laz and .ply formats.

Output: ndarray shape=(N, 6), columns=[X, Y, Z, R, G, B]
  RGB is normalized to 0.0-1.0 float64.
"""
import numpy as np
import laspy
import open3d as o3d
from pyproj import CRS, Transformer
from pathlib import Path


def load(file_path: str, config: dict) -> np.ndarray:
    """
    Load point cloud from .las/.laz or .ply file.

    Returns ndarray shape=(N, 6): [X, Y, Z, R, G, B] with RGB in [0, 1].
    """
    path = Path(file_path)
    ext = path.suffix.lower()

    if ext in (".las", ".laz"):
        points = _load_las(path, config)
    elif ext == ".ply":
        points = _load_ply(path, config)
    else:
        raise ValueError(f"Unsupported file format: {ext}. Use .las, .laz, or .ply")

    return points


def _load_las(path: Path, config: dict) -> np.ndarray:
    las = laspy.read(str(path))

    x = np.asarray(las.x, dtype=float)
    y = np.asarray(las.y, dtype=float)
    z = np.asarray(las.z, dtype=float)

    # RGB: laspy stores as uint16 (0-65535) or uint8 depending on format
    try:
        r = np.asarray(las.red,   dtype=float)
        g = np.asarray(las.green, dtype=float)
        b = np.asarray(las.blue,  dtype=float)
        # Normalize to 0-1
        max_val = r.max()
        if max_val > 255.0:
            r /= 65535.0
            g /= 65535.0
            b /= 65535.0
        else:
            r /= 255.0
            g /= 255.0
            b /= 255.0
    except Exception:
        r = np.zeros(x.size, dtype=float)
        g = np.zeros(x.size, dtype=float)
        b = np.zeros(x.size, dtype=float)

    # Remove NaN/Inf
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[valid], y[valid], z[valid]
    r, g, b = r[valid], g[valid], b[valid]

    if x.size == 0:
        raise ValueError("No valid (finite) points after NaN/Inf removal.")

    # CRS reprojection
    src_epsg = config["input"]["source_epsg"]
    dst_epsg = config["input"]["target_epsg"]

    src_crs = None
    epsg_from_las = getattr(las.header, "epsg", None)
    if epsg_from_las:
        try:
            src_crs = CRS.from_epsg(int(epsg_from_las))
            print(f"[INFO] LAS header EPSG: {epsg_from_las}")
        except Exception:
            src_crs = None

    if src_crs is None:
        src_crs = CRS.from_epsg(int(src_epsg))
        print(f"[INFO] source_epsg (config): {src_epsg}")

    dst_crs = CRS.from_epsg(int(dst_epsg))
    print(f"[INFO] target_epsg: {dst_epsg}")

    if src_crs != dst_crs:
        transformer = Transformer.from_crs(src_crs, dst_crs, always_xy=True)
        x, y = transformer.transform(x, y)
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        print("[INFO] Reprojected XY to target CRS.")

    # Statistical Outlier Removal via open3d
    points_xyz = np.column_stack([x, y, z])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz)
    pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    ind = np.asarray(ind)
    print(f"[INFO] SOR: {x.size} -> {len(ind)} points")

    x = x[ind]
    y = y[ind]
    z = z[ind]
    r = r[ind]
    g = g[ind]
    b = b[ind]

    points = np.column_stack([x, y, z, r, g, b])
    print(f"[INFO] Loaded {len(points)} points from {path.name}")
    return points


def _load_ply(path: Path, config: dict) -> np.ndarray:
    pcd = o3d.io.read_point_cloud(str(path))

    pts = np.asarray(pcd.points, dtype=float)
    if len(pts) == 0:
        raise ValueError("PLY file contains no points.")

    # Colors from open3d are already [0, 1]
    if pcd.has_colors():
        colors = np.asarray(pcd.colors, dtype=float)
        r = colors[:, 0]
        g = colors[:, 1]
        b = colors[:, 2]
    else:
        r = np.zeros(len(pts), dtype=float)
        g = np.zeros(len(pts), dtype=float)
        b = np.zeros(len(pts), dtype=float)

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

    # Remove NaN/Inf
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[valid], y[valid], z[valid]
    r, g, b = r[valid], g[valid], b[valid]

    if x.size == 0:
        raise ValueError("No valid (finite) points after NaN/Inf removal.")

    # CRS reprojection
    src_epsg = config["input"]["source_epsg"]
    dst_epsg = config["input"]["target_epsg"]

    src_crs = CRS.from_epsg(int(src_epsg))
    dst_crs = CRS.from_epsg(int(dst_epsg))
    print(f"[INFO] source_epsg: {src_epsg}, target_epsg: {dst_epsg}")

    if src_crs != dst_crs:
        transformer = Transformer.from_crs(src_crs, dst_crs, always_xy=True)
        x, y = transformer.transform(x, y)
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        print("[INFO] Reprojected XY to target CRS.")

    # Statistical Outlier Removal
    points_xyz = np.column_stack([x, y, z])
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(points_xyz)
    _, ind = pcd2.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    ind = np.asarray(ind)
    print(f"[INFO] SOR: {x.size} -> {len(ind)} points")

    x = x[ind]
    y = y[ind]
    z = z[ind]
    r = r[ind]
    g = g[ind]
    b = b[ind]

    points = np.column_stack([x, y, z, r, g, b])
    print(f"[INFO] Loaded {len(points)} points from {path.name}")
    return points
