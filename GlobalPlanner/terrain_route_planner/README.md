# terrain_route_planner

A Python CLI tool that plans an optimal traversable route across terrain represented by LAS/LAZ point clouds and/or LandXML TIN surfaces.

## Overview

`terrain_route_planner` ingests real-world terrain data, rasterizes it into a height grid, computes a traversability cost map based on slope and roughness, plans an optimal path using A*, smooths it with a B-spline, and exports the result as JSON/CSV waypoints. It supports 2D matplotlib visualization and 3D Open3D viewing.

## Installation

```bash
pip install -r requirements.txt
```

Dependencies: `numpy`, `scipy`, `matplotlib`, `open3d`, `laspy[lazrs]`, `lxml`

## Quick Start — Real Data

```bash
# Plan a route using a LAS point cloud + LandXML TIN
python main.py --las site.las --xml surface.xml --start 1000 2000 --goal 1500 2800 --export

# LAS only
python main.py --las site.las --start 1000 2000 --goal 1500 2800

# TIN only, no 3D window
python main.py --xml surface.xml --no-3d

# Custom slope limit and cell size
python main.py --las site.las --slope 15 --cell 0.5 --export
```

## Quick Start — Synthetic Test Data

Generate test files and run the full pipeline:

```bash
python tests/create_test_data.py
python main.py --las tests/test_site.las --xml tests/test_surface.xml --no-3d --export
```

Output will be written to `output/route.json` and `output/route.csv`.

## CLI Options

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--las PATH` | str | — | LAS/LAZ point cloud file |
| `--xml PATH` | str | — | LandXML TIN file |
| `--start X Y` | float float | auto | Start coordinate (world easting, northing) |
| `--goal X Y` | float float | auto | Goal coordinate (world easting, northing) |
| `--slope DEG` | float | 20.0 | Maximum traversable slope in degrees |
| `--cell M` | float | 1.0 | Raster cell size in metres |
| `--export` | flag | off | Save route to `output/route.json` + `output/route.csv` |
| `--no-3d` | flag | off | Skip Open3D 3D viewer window |

When `--start` / `--goal` are omitted, the planner automatically selects the nearest traversable cells to the corners of the terrain extent.

## LandXML Export Tips

- Export surfaces using **LandXML 1.2** schema when possible.
- Ensure your surface export includes both `<Pnts>` and `<Faces>` blocks under a `<Surface>/<Definition>` element.
- Point coordinates must follow the LandXML convention: `<P id="N">northing easting elevation</P>`.
- If your software uses LandXML 1.0 or 1.1, the loader will auto-detect the namespace.
- When multiple `<Surface>` elements are present, all surfaces are merged (a warning is printed).

## Slope Guidelines for Dump Trucks

| Grade | Slope (deg) | Recommendation |
|-------|------------|----------------|
| Flat | 0–5° | Fully traversable at speed |
| Gentle | 5–10° | Normal operation |
| Moderate | 10–15° | Reduce speed; consider road conditioning |
| Steep | 15–20° | Borderline; use `--slope 20` with caution |
| Impassable | > 20° | Not recommended; blocked by default |

Adjust `--slope` to match your specific truck model's rated grade capability.

## Output Format

### route.json

```json
{
  "generator": "terrain_route_planner",
  "input_las": "site.las",
  "input_tin": "surface.xml",
  "cell_size_m": 1.0,
  "slope_limit_deg": 20.0,
  "total_distance_m": 567.6,
  "total_waypoints": 500,
  "waypoints": [
    {"index": 0, "x_m": 50.5, "y_m": 50.5, "z_m": 50.02},
    ...
  ]
}
```

### route.csv

```
index,x_m,y_m,z_m
0,50.5,50.5,50.02
1,51.5,51.5,50.03
...
```

## Configuration

All tunable constants live in `config.py`:

| Constant | Default | Description |
|----------|---------|-------------|
| `SLOPE_LIMIT_DEG` | 20.0 | Impassable above this slope |
| `SLOPE_WEIGHT` | 3.0 | Cost penalty weight for slope |
| `ROUGH_WEIGHT` | 2.0 | Cost penalty weight for roughness |
| `OBSTACLE_COST` | 1e9 | Cost assigned to impassable cells |
| `DEFAULT_CELL_SIZE` | 1.0 m | Default raster resolution |
| `GAP_FILL_RADIUS` | 3 | NaN fill filter radius (cells) |
| `LAS_CHUNK_POINTS` | 5,000,000 | Points loaded per chunk |
| `SPLINE_SMOOTHING` | 5.0 | B-spline smoothing factor |
| `SPLINE_POINTS` | 500 | Waypoint count in smoothed output |
| `ROUTE_ELEV_OFFSET` | 0.5 m | Route elevation offset in 3D view |
