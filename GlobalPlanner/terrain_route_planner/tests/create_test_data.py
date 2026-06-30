"""Create synthetic LAS and LandXML test data for terrain_route_planner."""
import numpy as np
import os
import sys

# Ensure package root is on path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def create_test_las(filepath: str = "tests/test_site.las", n_points: int = 500_000) -> None:
    """Generate a 500x500 m synthetic LAS file with a flat plateau and diagonal trench."""
    try:
        import laspy
    except ImportError:
        print("laspy not found. Run: pip install laspy[lazrs]")
        sys.exit(1)

    rng = np.random.default_rng(42)

    # Random XY in 500 x 500 m area
    x = rng.uniform(0.0, 500.0, n_points).astype(np.float64)
    y = rng.uniform(0.0, 500.0, n_points).astype(np.float64)

    # Base elevation: flat plateau at 50 m with small noise
    z = np.full(n_points, 50.0) + rng.normal(0, 0.1, n_points)

    # Diagonal trench: where |x - y| < 20, drop elevation by 20 m
    trench_mask = np.abs(x - y) < 20.0
    z[trench_mask] -= 20.0 * (1.0 - np.abs(x[trench_mask] - y[trench_mask]) / 20.0)

    # Create LAS
    header = laspy.LasHeader(point_format=0, version="1.2")
    header.offsets = np.array([0.0, 0.0, 0.0])
    header.scales = np.array([0.001, 0.001, 0.001])

    las = laspy.LasData(header=header)
    las.x = x
    las.y = y
    las.z = z

    os.makedirs(os.path.dirname(os.path.abspath(filepath)) if os.path.dirname(filepath) else ".", exist_ok=True)
    las.write(filepath)
    print(f"Test data created: {filepath}")


def create_test_landxml(filepath: str = "tests/test_surface.xml") -> None:
    """Generate a 10x10 grid LandXML TIN with a flat plateau and a central mound."""
    # 11x11 grid of points spanning 0..500 in X and Y
    nx, ny = 11, 11
    xs = np.linspace(0, 500, nx)
    ys = np.linspace(0, 500, ny)
    gx, gy = np.meshgrid(xs, ys)

    # Base elevation 50 m, with a gentle mound in the centre
    gz = 50.0 + 10.0 * np.exp(-((gx - 250) ** 2 + (gy - 250) ** 2) / (2 * 80 ** 2))

    # Flatten to vertex list
    verts = []
    vid_map = {}
    pid = 1
    for j in range(ny):
        for i in range(nx):
            verts.append((pid, gy[j, i], gx[j, i], gz[j, i]))  # id, northing, easting, elev
            vid_map[(j, i)] = pid
            pid += 1

    # Build triangle faces from grid quads
    faces = []
    for j in range(ny - 1):
        for i in range(nx - 1):
            a = vid_map[(j, i)]
            b = vid_map[(j, i + 1)]
            c = vid_map[(j + 1, i)]
            d = vid_map[(j + 1, i + 1)]
            faces.append((a, b, c))
            faces.append((b, d, c))

    os.makedirs(os.path.dirname(os.path.abspath(filepath)) if os.path.dirname(filepath) else ".", exist_ok=True)

    lines = []
    lines.append('<?xml version="1.0" encoding="UTF-8"?>')
    lines.append('<LandXML xmlns="http://www.landxml.org/schema/LandXML-1.2"')
    lines.append('         version="1.2">')
    lines.append('  <Surfaces>')
    lines.append('    <Surface name="TestSurface">')
    lines.append('      <Definition surfType="TIN">')
    lines.append('        <Pnts>')
    for pid, n, e, elev in verts:
        lines.append(f'          <P id="{pid}">{n:.4f} {e:.4f} {elev:.4f}</P>')
    lines.append('        </Pnts>')
    lines.append('        <Faces>')
    for f in faces:
        lines.append(f'          <F>{f[0]} {f[1]} {f[2]}</F>')
    lines.append('        </Faces>')
    lines.append('      </Definition>')
    lines.append('    </Surface>')
    lines.append('  </Surfaces>')
    lines.append('</LandXML>')

    with open(filepath, "w") as fh:
        fh.write("\n".join(lines))
    print(f"Test data created: {filepath}")


if __name__ == "__main__":
    create_test_las()
    create_test_landxml()
