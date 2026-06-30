"""Load a LAS/LAZ file into XYZ + (optional) RGB arrays."""
from __future__ import annotations

import sys

import laspy
import numpy as np

_CHUNK = 2_000_000


def load_las(path: str) -> tuple[np.ndarray, np.ndarray | None]:
    """Return (xyz (N,3) float64, rgb (N,3) float32 in [0,1] or None if absent)."""
    xyz_chunks: list[np.ndarray] = []
    rgb_chunks: list[np.ndarray] = []
    has_rgb: bool | None = None

    with laspy.open(path) as f:
        total = f.header.point_count
        dim_names = set(f.header.point_format.dimension_names)
        has_rgb = {"red", "green", "blue"}.issubset(dim_names)

        read = 0
        for chunk in f.chunk_iterator(_CHUNK):
            xyz_chunks.append(
                np.column_stack(
                    [
                        np.asarray(chunk.x, dtype=np.float64),
                        np.asarray(chunk.y, dtype=np.float64),
                        np.asarray(chunk.z, dtype=np.float64),
                    ]
                )
            )
            if has_rgb:
                rgb_chunks.append(
                    np.column_stack(
                        [
                            np.asarray(chunk.red, dtype=np.float32),
                            np.asarray(chunk.green, dtype=np.float32),
                            np.asarray(chunk.blue, dtype=np.float32),
                        ]
                    )
                )
            read += len(chunk.x)
            print(f"\rLoading LAS... {read:,} / {total:,} pts", end="", flush=True, file=sys.stderr)
    print("", file=sys.stderr)

    xyz = np.concatenate(xyz_chunks, axis=0)
    if not has_rgb:
        print("LAS has no RGB channels; will color by elevation.", file=sys.stderr)
        return xyz, None

    rgb = np.concatenate(rgb_chunks, axis=0)
    # LAS spec stores RGB as uint16, but some files pack 8-bit values in the low byte.
    scale = 65535.0 if rgb.max() > 255.0 else 255.0
    rgb = np.clip(rgb / scale, 0.0, 1.0).astype(np.float32)
    return xyz, rgb
