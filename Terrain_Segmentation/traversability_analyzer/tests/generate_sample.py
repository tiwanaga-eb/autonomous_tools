"""
generate_sample.py - Generate a sample point cloud for testing.

Generates a 100x100m area (EPSG:6677 dummy coordinates) with:
  - Flat area (0-40m in X)
  - Sloped area (40-70m in X)
  - Rocky area / high roughness (70-85m in X)
  - Water area / blue color (85-100m in X)

Output: output/sample.las (RGB, EPSG:6677)

Usage:
    python tests/generate_sample.py
"""
import numpy as np
import laspy
from pathlib import Path


def generate_sample(output_path: str = "output/sample.las", seed: int = 42) -> None:
    rng = np.random.default_rng(seed)

    out_path = Path(output_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Point density: ~20 points/m^2
    density = 20
    total_area = 100 * 100  # m^2
    n_points = density * total_area

    x_all = []
    y_all = []
    z_all = []
    r_all = []
    g_all = []
    b_all = []

    # --------------------------------------------------
    # Area 1: Flat (x: 0-40, y: 0-100)
    # --------------------------------------------------
    n1 = int(n_points * 0.40)
    x1 = rng.uniform(0, 40, n1)
    y1 = rng.uniform(0, 100, n1)
    z1 = rng.normal(10.0, 0.03, n1)  # nearly flat

    # Brownish/grey color for soil
    r1 = rng.uniform(0.50, 0.65, n1)
    g1 = rng.uniform(0.40, 0.55, n1)
    b1 = rng.uniform(0.25, 0.35, n1)

    x_all.append(x1); y_all.append(y1); z_all.append(z1)
    r_all.append(r1); g_all.append(g1); b_all.append(b1)

    # --------------------------------------------------
    # Area 2: Sloped (x: 40-70, y: 0-100)
    # --------------------------------------------------
    n2 = int(n_points * 0.30)
    x2 = rng.uniform(40, 70, n2)
    y2 = rng.uniform(0, 100, n2)
    # 30 deg slope in X direction: tan(30°) ≈ 0.577
    slope_mpm = 0.577
    z2 = 10.0 + slope_mpm * (x2 - 40) + rng.normal(0, 0.05, n2)

    r2 = rng.uniform(0.45, 0.60, n2)
    g2 = rng.uniform(0.35, 0.50, n2)
    b2 = rng.uniform(0.20, 0.30, n2)

    x_all.append(x2); y_all.append(y2); z_all.append(z2)
    r_all.append(r2); g_all.append(g2); b_all.append(b2)

    # --------------------------------------------------
    # Area 3: Rocky / high roughness (x: 70-85, y: 0-100)
    # --------------------------------------------------
    n3 = int(n_points * 0.15)
    x3 = rng.uniform(70, 85, n3)
    y3 = rng.uniform(0, 100, n3)
    # Random large bumps
    z3 = 27.0 + rng.normal(0, 0.8, n3)  # large std = high roughness

    # Rocky grey-ish color
    r3 = rng.uniform(0.40, 0.60, n3)
    g3 = rng.uniform(0.38, 0.58, n3)
    b3 = rng.uniform(0.35, 0.55, n3)

    x_all.append(x3); y_all.append(y3); z_all.append(z3)
    r_all.append(r3); g_all.append(g3); b_all.append(b3)

    # --------------------------------------------------
    # Area 4: Water (x: 85-100, y: 0-100)
    # --------------------------------------------------
    n4 = int(n_points * 0.15)
    x4 = rng.uniform(85, 100, n4)
    y4 = rng.uniform(0, 100, n4)
    z4 = 9.5 + rng.normal(0, 0.02, n4)  # flat, slightly lower

    # Blue/cyan color for water: HSV hue ~200-220 deg, low saturation
    # RGB: bluish-grey — (0.20, 0.30, 0.45)
    r4 = rng.uniform(0.15, 0.25, n4)
    g4 = rng.uniform(0.25, 0.35, n4)
    b4 = rng.uniform(0.45, 0.60, n4)

    x_all.append(x4); y_all.append(y4); z_all.append(z4)
    r_all.append(r4); g_all.append(g4); b_all.append(b4)

    # --------------------------------------------------
    # Combine
    # --------------------------------------------------
    x = np.concatenate(x_all)
    y = np.concatenate(y_all)
    z = np.concatenate(z_all)
    r = np.concatenate(r_all)
    g = np.concatenate(g_all)
    b = np.concatenate(b_all)

    print(f"[INFO] Total points: {len(x)}")
    print(f"[INFO] X: {x.min():.1f} - {x.max():.1f}")
    print(f"[INFO] Y: {y.min():.1f} - {y.max():.1f}")
    print(f"[INFO] Z: {z.min():.3f} - {z.max():.3f}")

    # --------------------------------------------------
    # Write LAS (format 2 supports RGB)
    # --------------------------------------------------
    header = laspy.LasHeader(point_format=2, version="1.4")
    header.offsets = np.array([0.0, 0.0, 0.0])
    header.scales  = np.array([0.001, 0.001, 0.001])

    las = laspy.LasData(header=header)
    las.x = x
    las.y = y
    las.z = z

    # laspy RGB is uint16 (0-65535)
    las.red   = (np.clip(r, 0, 1) * 65535).astype(np.uint16)
    las.green = (np.clip(g, 0, 1) * 65535).astype(np.uint16)
    las.blue  = (np.clip(b, 0, 1) * 65535).astype(np.uint16)

    las.write(str(out_path))
    print(f"[OK] Sample LAS saved: {out_path.resolve()}")


if __name__ == "__main__":
    generate_sample()
