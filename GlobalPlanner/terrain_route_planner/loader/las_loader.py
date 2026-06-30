"""Load a LAS/LAZ point cloud into a (N, 3) float64 XYZ array."""
import numpy as np
import sys

from config import LAS_CHUNK_POINTS


def load_las(filepath: str) -> np.ndarray:
    """Open a LAS/LAZ file in chunks and return all XYZ points as (N, 3) float64."""
    try:
        import laspy
    except ImportError:
        print("laspy not found. Run: pip install laspy[lazrs]", file=sys.stderr)
        sys.exit(1)

    chunks = []
    with laspy.open(filepath) as las_file:
        total = las_file.header.point_count
        for chunk_idx, chunk in enumerate(las_file.chunk_iterator(LAS_CHUNK_POINTS)):
            print(f"\rLoading LAS... {chunk_idx * LAS_CHUNK_POINTS:,} / {total:,} pts", end="", flush=True)
            xyz = np.column_stack([
                np.asarray(chunk.x, dtype=np.float64),
                np.asarray(chunk.y, dtype=np.float64),
                np.asarray(chunk.z, dtype=np.float64),
            ])
            chunks.append(xyz)
    print()  # newline after progress

    points = np.concatenate(chunks, axis=0)
    xmin, ymin, zmin = points.min(axis=0)
    xmax, ymax, zmax = points.max(axis=0)
    N = len(points)
    print(
        f"Loaded {N:,} points  "
        f"bbox: X[{xmin:.1f}, {xmax:.1f}]  "
        f"Y[{ymin:.1f}, {ymax:.1f}]  "
        f"Z[{zmin:.1f}, {zmax:.1f}]"
    )
    return points
