"""
main.py - Traversability analyzer entry point.

Usage:
    python main.py input.las
    python main.py input.ply
"""
import argparse
import sys
import time
from pathlib import Path

import yaml

import modules.loader as loader
import modules.grid_builder as grid_builder
import modules.feature_extractor as feature_extractor
import modules.classifier as classifier
import modules.exporter as exporter


def load_config(config_path: str = "config.yaml") -> dict:
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(
        description="Terrain traversability analyzer for LAS/PLY point clouds."
    )
    parser.add_argument(
        "input",
        help="Input point cloud file (.las, .laz, or .ply)",
    )
    parser.add_argument(
        "--config",
        default="config.yaml",
        help="Path to config.yaml (default: config.yaml)",
    )
    parser.add_argument(
        "--output",
        default="output",
        help="Output directory (default: output/)",
    )
    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        print(f"[ERROR] Input file not found: {input_path}")
        sys.exit(1)

    config_path = Path(args.config)
    if not config_path.exists():
        print(f"[ERROR] Config file not found: {config_path}")
        sys.exit(1)

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    t_start = time.perf_counter()

    # --------------------------------------------------
    # Step 1: Load point cloud
    # --------------------------------------------------
    print(f"\n[Step 1/5] Loading point cloud: {input_path}")
    config = load_config(str(config_path))
    points = loader.load(str(input_path), config)
    print(f"[Step 1/5] Done. {len(points)} points loaded.")

    # --------------------------------------------------
    # Step 2: Build raster grids
    # --------------------------------------------------
    print("\n[Step 2/5] Building raster grids (DSM, DTM, density)...")
    grids = grid_builder.build(points, config)
    ny, nx = grids["elev"].shape
    print(f"[Step 2/5] Done. Grid size: {ny} x {nx}")

    # --------------------------------------------------
    # Step 3: Extract features
    # --------------------------------------------------
    print("\n[Step 3/5] Extracting terrain features...")
    features = feature_extractor.extract(grids, points, config)
    print("[Step 3/5] Done. Features: slope, roughness, height_range, water_score, density")

    # --------------------------------------------------
    # Step 4: Classify
    # --------------------------------------------------
    print("\n[Step 4/5] Classifying terrain traversability...")
    result = classifier.classify(features, grids, config)
    print("[Step 4/5] Done.")

    # --------------------------------------------------
    # Step 5: Export
    # --------------------------------------------------
    print(f"\n[Step 5/5] Exporting results to {output_dir}/...")
    t_elapsed = time.perf_counter() - t_start
    extra_info = {
        "input_file":    str(input_path),
        "processing_time_s": round(t_elapsed, 3),
    }
    exporter.export(result, grids, config, str(output_dir), extra_info=extra_info)
    print(f"[Step 5/5] Done.")

    t_total = time.perf_counter() - t_start
    print(f"\n[DONE] Total processing time: {t_total:.2f} s")
    print(f"       Output directory: {output_dir.resolve()}")


if __name__ == "__main__":
    main()
