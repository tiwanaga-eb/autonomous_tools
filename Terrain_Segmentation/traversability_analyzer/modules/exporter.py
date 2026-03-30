"""
exporter.py - Export traversability analysis results.

Outputs:
  traversability_map.png   - RGB PNG
  traversability_map.tif   - RGB GeoTIFF with CRS
  cost_map.tif             - FMS-compatible RGB GeoTIFF
  report.json              - Class area statistics
"""
import json
import numpy as np
import rasterio
from rasterio.transform import from_origin
from pyproj import CRS
from PIL import Image
from pathlib import Path


# ============================================================
# Utility (ported from existing code)
# ============================================================

def cost_to_rgb(
    cost_u8: np.ndarray,
    threshold: int = 20,
    nodata_mask: np.ndarray | None = None,
) -> np.ndarray:
    """
    Convert cost_u8 (0-255) to RGB (uint8).
      0..threshold : green -> yellow -> red gradient
      >threshold   : light grey
      nodata_mask  : dark grey
    Returns (H, W, 3) uint8.
    """
    h, w = cost_u8.shape
    rgb = np.zeros((h, w, 3), dtype=np.uint8)

    # Default: > threshold -> light grey
    rgb[:] = np.array([128, 128, 128], dtype=np.uint8)

    in_range = cost_u8 <= threshold
    t = np.zeros_like(cost_u8, dtype=np.float32)
    t[in_range] = cost_u8[in_range].astype(np.float32) / float(threshold)

    first  = in_range & (t <= 0.5)
    second = in_range & (t > 0.5)

    tt = np.zeros_like(t)
    tt[first] = t[first] / 0.5
    rgb[first, 0] = (255 * tt[first]).astype(np.uint8)
    rgb[first, 1] = 255
    rgb[first, 2] = 0

    tt[:] = 0
    tt[second] = (t[second] - 0.5) / 0.5
    rgb[second, 0] = 255
    rgb[second, 1] = (255 * (1.0 - tt[second])).astype(np.uint8)
    rgb[second, 2] = 0

    if nodata_mask is not None and nodata_mask.any():
        rgb[nodata_mask] = np.array([80, 80, 80], dtype=np.uint8)

    return rgb


# ============================================================
# Exporter
# ============================================================

def export(
    result: dict,
    grids: dict,
    config: dict,
    output_dir: str,
    extra_info: dict | None = None,
) -> None:
    """
    Export all output files.

    Args:
        result:     output of classifier.classify()
        grids:      output of grid_builder.build()
        config:     loaded config dict
        output_dir: path to output directory
        extra_info: optional dict merged into report.json
    """
    out_dir    = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    class_map   = result["class_map"]
    cost_u8     = result["cost_u8"]
    nodata_mask = result["nodata_mask"]
    x_edges     = grids["x_edges"]
    y_edges     = grids["y_edges"]

    grid_size   = float(config["grid"]["resolution_m"])
    target_epsg = int(config["input"]["target_epsg"])
    dst_crs     = CRS.from_epsg(target_epsg)
    transform   = from_origin(x_edges[0], y_edges[-1], grid_size, grid_size)

    colors = config["output"]["colors"]
    cost_thresh = int(config["output"]["cost_color_threshold"])

    # --------------------------------------------------
    # 1. traversability_map.png
    # --------------------------------------------------
    rgb_img = _class_to_rgb(class_map, nodata_mask, colors)
    img = Image.fromarray(rgb_img, mode="RGB")
    png_path = out_dir / "traversability_map.png"
    img.save(str(png_path))
    print(f"[OK] {png_path}")

    # --------------------------------------------------
    # 2. traversability_map.tif
    # --------------------------------------------------
    tif_path = out_dir / "traversability_map.tif"
    _write_geotiff_rgb(rgb_img, tif_path, dst_crs, transform, compress="DEFLATE")
    print(f"[OK] {tif_path}")

    # --------------------------------------------------
    # 3. cost_map.tif
    # --------------------------------------------------
    cost_rgb = cost_to_rgb(cost_u8, threshold=cost_thresh, nodata_mask=nodata_mask)
    cost_tif_path = out_dir / "cost_map.tif"
    _write_geotiff_rgb(cost_rgb, cost_tif_path, dst_crs, transform, compress="DEFLATE")
    print(f"[OK] {cost_tif_path}")

    # --------------------------------------------------
    # 4. report.json
    # --------------------------------------------------
    cell_area = grid_size ** 2
    ny, nx    = class_map.shape
    total     = ny * nx

    n_pass = int(np.sum((class_map == 0) & ~nodata_mask))
    n_risk = int(np.sum((class_map == 1) & ~nodata_mask))
    n_imp  = int(np.sum((class_map == 2) & ~nodata_mask))
    n_nd   = int(np.sum(nodata_mask))

    report = {
        "total_cells": total,
        "passable": {
            "cells":   n_pass,
            "area_m2": round(n_pass * cell_area, 3),
            "ratio":   round(n_pass / total, 6) if total > 0 else 0.0,
        },
        "risk": {
            "cells":   n_risk,
            "area_m2": round(n_risk * cell_area, 3),
            "ratio":   round(n_risk / total, 6) if total > 0 else 0.0,
        },
        "impassable": {
            "cells":   n_imp,
            "area_m2": round(n_imp * cell_area, 3),
            "ratio":   round(n_imp / total, 6) if total > 0 else 0.0,
        },
        "nodata": {
            "cells":   n_nd,
            "area_m2": round(n_nd * cell_area, 3),
            "ratio":   round(n_nd / total, 6) if total > 0 else 0.0,
        },
    }

    if extra_info:
        report.update(extra_info)

    json_path = out_dir / "report.json"
    with open(str(json_path), "w") as f:
        json.dump(report, f, indent=2)
    print(f"[OK] {json_path}")


def _class_to_rgb(
    class_map: np.ndarray,
    nodata_mask: np.ndarray,
    colors: dict,
) -> np.ndarray:
    ny, nx = class_map.shape
    rgb = np.zeros((ny, nx, 3), dtype=np.uint8)

    c_pass = np.array(colors["passable"],   dtype=np.uint8)
    c_risk = np.array(colors["risk"],       dtype=np.uint8)
    c_imp  = np.array(colors["impassable"], dtype=np.uint8)
    c_nd   = np.array(colors["nodata"],     dtype=np.uint8)

    rgb[class_map == 0] = c_pass
    rgb[class_map == 1] = c_risk
    rgb[class_map == 2] = c_imp
    rgb[nodata_mask]    = c_nd

    return rgb


def _write_geotiff_rgb(
    rgb: np.ndarray,
    path: Path,
    crs,
    transform,
    compress: str = "DEFLATE",
) -> None:
    rgb_bands = np.moveaxis(rgb, 2, 0)  # (3, H, W)
    with rasterio.open(
        str(path),
        "w",
        driver="GTiff",
        height=rgb.shape[0],
        width=rgb.shape[1],
        count=3,
        dtype=rasterio.uint8,
        crs=crs,
        transform=transform,
        compress=compress,
        photometric="RGB",
        interleave="pixel",
    ) as dst:
        dst.write(rgb_bands[0], 1)
        dst.write(rgb_bands[1], 2)
        dst.write(rgb_bands[2], 3)
