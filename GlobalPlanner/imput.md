# Terrain Route Planner v3 — Claude Code Prompt
# 実データ対応版（LAS + LandXML入力、ドローンSfM、中規模現場）

Build a self-contained Python CLI tool called `terrain_route_planner` that plans an optimal
dump-truck route over a real 3D mining / quarry / civil-engineering terrain.

**No ROS2. No robot middleware. Pure Python.**

---

## Inputs

| Data | Format | Notes |
|------|--------|-------|
| Point cloud | `.las` / `.laz` | Drone photogrammetry / SfM, LAS 1.2–1.4 |
| TIN surface  | LandXML (`.xml`) | `<Surface>` → `<Pnts>` + `<Faces>` |
| Start point  | XY world coord (CLI) | same CRS as survey data |
| Goal point   | XY world coord (CLI) | same CRS as survey data |
| Slope limit  | degrees (CLI, default 20°) | truck traversability threshold |

Scale assumption: 200 m – 1 km extent, 10 M – 80 M points (fits in RAM with chunked loading).

---

## Directory layout to create

```
terrain_route_planner/
├── README.md
├── requirements.txt
├── config.py
├── loader/
│   ├── __init__.py
│   ├── las_loader.py         # .las / .laz → numpy XYZ (chunked for large files)
│   ├── landxml_loader.py     # LandXML .xml TIN → vertices + faces
│   └── rasterizer.py         # point cloud / TIN → regular height grid
├── analysis/
│   ├── __init__.py
│   ├── slope.py
│   ├── roughness.py
│   └── cost_map.py
├── planner/
│   ├── __init__.py
│   ├── graph.py
│   └── astar.py
├── smoother/
│   ├── __init__.py
│   └── spline_smoother.py
├── visualizer/
│   ├── __init__.py
│   ├── view2d.py
│   └── view3d.py
├── exporter/
│   ├── __init__.py
│   └── export.py
├── tests/
│   └── create_test_data.py   # generate synthetic .las + LandXML for CI
└── main.py
```

---

## requirements.txt

```
numpy
scipy
matplotlib
open3d
laspy[lazrs]
lxml
```

*(No ROS2, no GDAL, no ezdxf — replaced by lxml for LandXML parsing.)*

---

## config.py

```python
# --- Truck traversability ---
SLOPE_LIMIT_DEG   = 20.0   # degrees — impassable above this
SLOPE_WEIGHT      = 3.0
ROUGH_WEIGHT      = 2.0
OBSTACLE_COST     = 1e9

# --- Rasterization ---
DEFAULT_CELL_SIZE = 1.0    # metres per grid cell
GAP_FILL_RADIUS   = 3      # cells — ndimage fill radius for empty cells

# --- LAS chunked loading ---
LAS_CHUNK_POINTS  = 5_000_000   # points per chunk (tune for available RAM)

# --- Route smoother ---
SPLINE_SMOOTHING  = 5.0
SPLINE_POINTS     = 500

# --- Visualizer ---
ROUTE_ELEV_OFFSET = 0.5    # metres above surface for 3D route line
```

---

## loader/las_loader.py

```python
def load_las(filepath: str) -> np.ndarray:
    """Load .las or .laz, return (N, 3) float64 XYZ. Chunked for large files."""
```

Implementation:
- Open with `laspy.open(filepath)`.
- Read in chunks of `LAS_CHUNK_POINTS` using `las_file.chunk_iterator(LAS_CHUNK_POINTS)`.
- Accumulate into list of arrays, then `np.concatenate`.
- Print progress: `Loading LAS... {chunk_idx * LAS_CHUNK_POINTS:,} / {total:,} pts`
- After load: `Loaded {N:,} points  bbox: X[{xmin:.1f}, {xmax:.1f}]  Y[{ymin:.1f}, {ymax:.1f}]  Z[{zmin:.1f}, {zmax:.1f}]`

---

## loader/landxml_loader.py

LandXML TIN structure to parse:
```xml
<LandXML>
  <Surfaces>
    <Surface name="...">
      <Definition>
        <Pnts>
          <P id="1">northing easting elevation</P>
          ...
        </Pnts>
        <Faces>
          <F>1 2 3</F>
          ...
        </Faces>
      </Definition>
    </Surface>
  </Surfaces>
</LandXML>
```

```python
def load_landxml_tin(filepath: str) -> tuple[np.ndarray, np.ndarray]:
    """
    Parse LandXML .xml TIN file.
    Returns (vertices, faces):
      vertices: (V, 3) float64 — [X, Y, Z]  (easting, northing, elev)
      faces:    (F, 3) int32   — 0-based vertex indices
    """
```

Implementation details:
- Use `lxml.etree.parse()`.
- Handle LandXML namespace automatically:
  `ns = {'lx': 'http://www.landxml.org/schema/LandXML-1.2'}` — also try 1.0 and 1.1 if 1.2 not found.
- `<P>` text = `"northing easting elevation"` — note order: **N E Z**, convert to **X=easting, Y=northing, Z=elev**.
- `<F>` text = `"1 2 3"` — 1-based IDs, convert to 0-based indices.
- If multiple `<Surface>` elements exist, load all and merge (warn user).
- Print: `Loaded TIN: {V:,} vertices, {F:,} faces from {filepath}`
- Raise clear `ValueError` if no `<Pnts>` found (e.g. wrong XML schema).

---

## loader/rasterizer.py

```python
def rasterize_pointcloud(
    points: np.ndarray,       # (N, 3) XYZ
    cell_size: float = 1.0,
    method: str = "mean"      # "mean" | "max" | "min"
) -> tuple[np.ndarray, dict]:
    """
    Bin point cloud onto regular 2-D height grid.
    Returns (height_grid, geo_transform).
    geo_transform = {"x_min", "y_min", "cell_size", "cols", "rows"}
    """
```

- Compute grid extent from point cloud bbox.
- Use `np.digitize` for fast binning. Accumulate with `np.bincount`.
- Empty cells → NaN, then fill with `scipy.ndimage.generic_filter(nanmean, size=GAP_FILL_RADIUS*2+1)`.
- Remaining NaN (edge) → set to `np.nanmin(height_grid)` (safe floor).

```python
def rasterize_tin(
    vertices: np.ndarray,     # (V, 3)
    faces: np.ndarray,        # (F, 3) int32
    cell_size: float = 1.0
) -> tuple[np.ndarray, dict]:
    """
    Rasterize TIN to height grid via barycentric interpolation.
    For each grid cell centre, find containing triangle and interpolate Z.
    """
```

- Build per-triangle 2D bounding box index for fast lookup.
- Barycentric test + interpolation for each cell centre.
- Cells outside all triangles → NaN (fill same as above).

```python
def blend_grids(
    tin_grid: np.ndarray,
    las_grid: np.ndarray,
    geo_transform: dict
) -> np.ndarray:
    """Merge TIN and LAS grids: TIN takes priority, LAS fills NaN gaps."""
```

---

## analysis/slope.py

```python
def compute_slope(height_grid: np.ndarray, cell_size: float) -> np.ndarray:
    """Slope angle in degrees via Sobel gradient. NaN cells → 90°."""
```

---

## analysis/roughness.py

```python
def compute_roughness(height_grid: np.ndarray) -> np.ndarray:
    """Local roughness = nanstd in 3×3 window via generic_filter."""
```

---

## analysis/cost_map.py

```python
def compute_cost_map(
    height_grid: np.ndarray,
    cell_size: float,
    slope_limit_deg: float = SLOPE_LIMIT_DEG
) -> np.ndarray:
    """Traversability cost = f(slope, roughness). Impassable above slope_limit."""
```

Cost formula:
```python
slope_norm = np.clip(slope_deg / slope_limit_deg, 0, None)
cost = 1.0 + SLOPE_WEIGHT * slope_norm**2 + ROUGH_WEIGHT * roughness
cost[slope_deg >= slope_limit_deg] = OBSTACLE_COST
cost[np.isnan(height_grid)]        = OBSTACLE_COST
```

```python
def get_traversable_mask(cost_map: np.ndarray) -> np.ndarray:
    """True where truck can travel."""
```

---

## planner/graph.py + planner/astar.py

Same as v2 spec — 8-connected weighted grid graph, A* with Euclidean heuristic.
`find_route` auto-snaps impassable start/goal to nearest traversable cell and warns.

---

## smoother/spline_smoother.py

```python
def smooth_route(
    route_rc: list[tuple[int, int]],
    height_grid: np.ndarray,
    geo_transform: dict
) -> np.ndarray:
    """Fit scipy spline through raw A* waypoints. Returns (M, 3) XYZ float64."""
```

- `scipy.interpolate.splprep([x_world, y_world], s=SPLINE_SMOOTHING)` → `splev`.
- Interpolate Z from `height_grid` at each smoothed point.

---

## visualizer/view2d.py

```python
def show_2d(height_grid, cost_map, route_rc, start_rc, goal_rc, geo_transform):
```

- 2 subplots: height map (`gist_earth`) + cost map (log scale, `hot_r`).
- Route in red, start ▲ green, goal ★ red.
- Axis ticks in world coordinates (metres), derived from geo_transform.
- Colorbar on each subplot.

---

## visualizer/view3d.py

```python
def show_3d(points_xyz, cost_map, route_xyz, geo_transform):
```

Point cloud coloured by traversability cost:

| cost range  | colour  | hex approx |
|-------------|---------|------------|
| ≤ 2         | green   | `[0.2, 0.8, 0.2]` |
| ≤ 5         | yellow  | `[0.9, 0.8, 0.1]` |
| ≤ 20        | orange  | `[0.9, 0.4, 0.1]` |
| impassable  | dark gray | `[0.3, 0.3, 0.3]` |

- Route: red LineSet at +ROUTE_ELEV_OFFSET above surface.
- Start / goal: green / red spheres, radius = 3 m.

---

## exporter/export.py

```python
def export_route(route_xyz: np.ndarray, meta: dict, out_dir: str = "output"):
```

Saves `output/route.json` and `output/route.csv`.

JSON:
```json
{
  "generator": "terrain_route_planner",
  "input_las": "site.las",
  "input_tin": "surface.xml",
  "cell_size_m": 1.0,
  "slope_limit_deg": 20.0,
  "total_distance_m": 623.4,
  "total_waypoints": 500,
  "waypoints": [
    {"index": 0, "x_m": 1000.0, "y_m": 2000.0, "z_m": 55.2},
    ...
  ]
}
```

CSV columns: `index,x_m,y_m,z_m`

---

## tests/create_test_data.py

Generate minimal synthetic test files so the tool can be verified without real data:

```python
def create_test_las(filepath="tests/test_site.las", n_points=500_000):
    """Write a synthetic .las file: flat plateau with a slope trench."""

def create_test_landxml(filepath="tests/test_surface.xml"):
    """Write a minimal valid LandXML 1.2 file with a triangulated surface."""
```

- `create_test_las`: use `laspy.LasHeader` + `laspy.LasData`. Simulate a 500 m × 500 m
  flat plateau (Z ≈ 50 m) with a diagonal trench (Z drops 20 m, impassable slope).
- `create_test_landxml`: 10×10 grid of points → 162 triangles, flat plateau + mound.
- Print: `Test data created: {filepath}`

---

## main.py — CLI

```
python main.py \
  --las    site.las \
  --xml    surface.xml \
  --start  1000.0 2000.0 \
  --goal   1500.0 2800.0 \
  --slope  20 \
  --cell   1.0 \
  --export \
  --no-3d
```

All arguments:

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--las`  | PATH | — | LAS/LAZ point cloud file |
| `--xml`  | PATH | — | LandXML TIN file |
| `--start`| X Y  | auto | Start coordinate (world XY) |
| `--goal` | X Y  | auto | Goal coordinate (world XY) |
| `--slope`| float| 20.0 | Max traversable slope (degrees) |
| `--cell` | float| 1.0  | Raster cell size (metres) |
| `--export`| flag | off | Save route to output/ |
| `--no-3d`| flag | off | Skip Open3D window |

Execution flow:
1. Load LAS (chunked) if `--las` given.
2. Load LandXML TIN if `--xml` given.
3. Rasterize: blend TIN + LAS if both provided; use whichever is given alone.
4. Compute cost map (slope + roughness).
5. Convert `--start` / `--goal` world XY → grid (row, col).
   - Out-of-bounds → clear error + exit 1.
   - Impassable cell → auto-snap to nearest traversable + warn.
   - If not given → auto: start = traversable cell nearest (x_min+50, y_min+50),
     goal = traversable cell nearest (x_max-50, y_max-50).
6. Build graph → A* → spline smooth.
7. Print summary:
   ```
   ─────────────────────────────────────────
   Input LAS  : site.las  (24,500,000 pts)
   Input TIN  : surface.xml  (18,432 faces)
   Grid       : 720 × 540 cells @ 1.0 m/cell
   Traversable: 71.4 % of area
   Start      : (1000.0, 2000.0) → grid row=190, col=180
   Goal       : (1500.0, 2800.0) → grid row=620, col=480
   Route      : 500 waypoints, distance ≈ 623.4 m
   ─────────────────────────────────────────
   ```
8. Show 2D plot.
9. Show 3D viewer (unless `--no-3d`).
10. Export if `--export`.

---

## Error handling

| Situation | Message | Exit |
|-----------|---------|------|
| Neither `--las` nor `--xml` | "Provide at least one of --las or --xml" | 1 |
| File not found | "File not found: {path}" | 1 |
| LandXML with no `<Pnts>` | "No TIN surface found in XML. Check LandXML schema version and `<Surface>` export settings." | 1 |
| No path found | "No traversable path between start and goal. Try --slope {current+5} or verify both points are reachable." | 1 |
| laspy missing | "Run: pip install laspy[lazrs]" | 1 |
| lxml missing  | "Run: pip install lxml" | 1 |

---

## README.md

Include:
- Overview
- `pip install -r requirements.txt`
- Quick start with real data
- Quick start with synthetic test data: `python tests/create_test_data.py && python main.py --las tests/test_site.las --xml tests/test_surface.xml`
- CLI options table
- LandXML export tips:
  - Civil 3D: `Output → Export to LandXML` → select surface → LandXML 1.2
  - Trimble Business Center / MAGNET Office: Export Surface as LandXML
  - Coordinate order in `<P>`: always Northing Easting Elevation (tool handles this automatically)
- Slope guideline table for dump trucks:

| トラック種別 | 推奨最大勾配 | CLI例 |
|-------------|-------------|-------|
| 超大型 (200 t+) | 8–10° | `--slope 10` |
| 大型 (100 t)    | 10–15° | `--slope 15` |
| 中型 (40–60 t)  | 15–20° | `--slope 20` |

---

## Implementation notes for Claude Code

- All magic numbers live in `config.py`. Every other file does `from config import ...`.
- All public functions must have a one-line docstring.
- After writing all files, run in this exact order and fix any errors:
  ```
  pip install -r requirements.txt
  python tests/create_test_data.py
  python main.py --las tests/test_site.las --xml tests/test_surface.xml --no-3d --export
  ```
- If both commands succeed and `output/route.json` exists, the tool is working.
- Then verify Open3D by running without `--no-3d`.