from fastapi import FastAPI, Response, UploadFile, File
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
import rasterio
from rasterio.plot import reshape_as_image
from rasterio.transform import Affine
from rasterio.warp import reproject, Resampling
from PIL import Image
import numpy as np
import io
import pyproj
from scipy.interpolate import splprep, splev
from pydantic import BaseModel, Field
from typing import List, Optional, Literal, Any, Dict
from pathlib import Path
import os

app = FastAPI(title="FMS Map Building Tool")

# =========================
# Paths
# =========================
BASE_DIR = Path(__file__).resolve().parent
ORTHO_RASTER = BASE_DIR / "ortho.tif"
COST_RASTER  = BASE_DIR / "costmap.tif"

DEFAULT_WORKING_EPSG = 6677


# =========================
# Raster helpers (canonical = ortho grid)
# =========================
def _raster_path(layer: str) -> Path:
    layer = (layer or "ortho").lower()
    return COST_RASTER if layer == "cost" else ORTHO_RASTER

def _open(layer: str):
    p = _raster_path(layer)
    if not p.exists():
        raise FileNotFoundError(f"Missing raster: {layer}")
    return rasterio.open(p)

def _get_epsg(src) -> Optional[int]:
    try:
        if src.crs is None:
            return None
        e = src.crs.to_epsg()
        return int(e) if e is not None else None
    except Exception:
        return None

def _version_of(path: Path) -> int:
    # browser-cache key; changes only when file changes
    if not path.exists():
        return 0
    try:
        return int(path.stat().st_mtime_ns)
    except Exception:
        return 0

def canonical_meta():
    if not ORTHO_RASTER.exists():
        raise FileNotFoundError("Ortho not uploaded yet (canonical grid missing).")
    with _open("ortho") as ortho:
        epsg = _get_epsg(ortho) or DEFAULT_WORKING_EPSG
        return {
            "width": ortho.width,
            "height": ortho.height,
            "crs": str(ortho.crs),
            "epsg": epsg,
            "transform": list(ortho.transform),
        }

def _normalize_to_uint8(arr: np.ndarray) -> np.ndarray:
    if arr.dtype == np.uint8:
        return arr
    a = np.nanmin(arr)
    b = np.nanmax(arr)
    if not np.isfinite(a) or not np.isfinite(b) or abs(b - a) < 1e-12:
        return np.zeros_like(arr, dtype=np.uint8)
    return (255.0 * (arr - a) / (b - a)).clip(0, 255).astype(np.uint8)

def _read_as_display_rgb_on_canonical(layer: str) -> np.ndarray:
    """
    Returns RGB image (H,W,3) on canonical grid (Ortho grid).
    - Ortho: bilinear
    - Cost : nearest (discrete map)
    """
    meta = canonical_meta()
    W, H = meta["width"], meta["height"]
    dst_transform = Affine(*meta["transform"][:6])
    dst_crs = meta["crs"]

    with _open(layer) as src:
        src_crs = src.crs
        src_transform = src.transform
        src_count = src.count
        bands_to_take = 3 if src_count >= 3 else 1

        dst = np.zeros((bands_to_take, H, W), dtype=np.float32)
        resamp = Resampling.bilinear if layer == "ortho" else Resampling.nearest

        for bi in range(bands_to_take):
            src_band = src.read(bi + 1).astype(np.float32)
            reproject(
                source=src_band,
                destination=dst[bi],
                src_transform=src_transform,
                src_crs=src_crs,
                dst_transform=dst_transform,
                dst_crs=dst_crs,
                resampling=resamp,
                num_threads=2,
            )

        dst_u8 = _normalize_to_uint8(dst)
        if bands_to_take == 1:
            dst_u8 = np.repeat(dst_u8, 3, axis=0)

        rgb = reshape_as_image(dst_u8)  # (H,W,3)
        return rgb

def _png_bytes_from_rgb(rgb: np.ndarray) -> io.BytesIO:
    im = Image.fromarray(rgb)
    buf = io.BytesIO()
    im.save(buf, format="PNG")
    buf.seek(0)
    return buf


# =========================
# Geometry / projection utils
# =========================
def proj_xy(xy: np.ndarray, src_epsg: int, dst_epsg: int) -> np.ndarray:
    if int(src_epsg) == int(dst_epsg):
        return xy.copy()
    tr = pyproj.Transformer.from_crs(f"epsg:{int(src_epsg)}", f"epsg:{int(dst_epsg)}", always_xy=True)
    X, Y = tr.transform(xy[:, 0], xy[:, 1])
    return np.stack([X, Y], axis=1)

def map_to_lonlat(xy: np.ndarray, epsg_in: int) -> np.ndarray:
    tr = pyproj.Transformer.from_crs(f"epsg:{int(epsg_in)}", "epsg:4326", always_xy=True)
    lon, lat = tr.transform(xy[:, 0], xy[:, 1])
    return np.stack([lon, lat], axis=1)

def add_heading_ghost(points_xy: np.ndarray, heading_deg: float, dist_m: float, at_start=True):
    th = np.deg2rad(heading_deg)
    vec = np.array([np.cos(th), np.sin(th)], dtype=float)
    if at_start:
        return np.vstack([points_xy[0] + vec * dist_m, points_xy])
    return np.vstack([points_xy, points_xy[-1] - vec * dist_m])

def fit_spline(points_xy: np.ndarray, s=0.0, n=5000):
    tck, _u = splprep([points_xy[:, 0], points_xy[:, 1]], s=s)
    unew = np.linspace(0, 1, n)
    x_s, y_s = splev(unew, tck)
    return np.stack([x_s, y_s], axis=1)

def cumulative_lengths(pts: np.ndarray) -> np.ndarray:
    seg = np.linalg.norm(pts[1:] - pts[:-1], axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])

def resample_by_spacing(pts: np.ndarray, spacing: float) -> np.ndarray:
    if spacing is None or spacing <= 0:
        return pts.copy()
    s = cumulative_lengths(pts)
    total = s[-1]
    if total <= spacing:
        return np.vstack([pts[0], pts[-1]])
    targets = np.arange(0.0, total, spacing)
    targets = np.append(targets, total)
    xs = np.interp(targets, s, pts[:, 0])
    ys = np.interp(targets, s, pts[:, 1])
    return np.stack([xs, ys], axis=1)

def curvature_radius(p1, p2, p3):
    a = np.linalg.norm(p2 - p1)
    b = np.linalg.norm(p3 - p2)
    c = np.linalg.norm(p3 - p1)
    s = (a + b + c) / 2
    area = max(np.sqrt(max(s * (s - a) * (s - b) * (s - c), 0.0)), 1e-12)
    return (a * b * c) / (4 * area)

def min_radius_on_polyline(pts: np.ndarray) -> float:
    """
    polyline pts (N,2) の局所曲率半径の最小値を返す
    """
    if pts is None or len(pts) < 3:
        return float("inf")
    minR = float("inf")
    for i in range(1, len(pts) - 1):
        p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1]
        if np.linalg.norm(p2 - p1) < 1e-6 or np.linalg.norm(p3 - p2) < 1e-6:
            continue
        R = curvature_radius(p1, p2, p3)
        if np.isfinite(R):
            minR = min(minR, float(R))
    return minR

def _crop_curve_between(curve_xy: np.ndarray, start_xy: np.ndarray, goal_xy: np.ndarray):
    i0 = int(np.argmin(np.sum((curve_xy - start_xy) ** 2, axis=1)))
    i1 = int(np.argmin(np.sum((curve_xy - goal_xy) ** 2, axis=1)))
    if i0 > i1:
        i0, i1 = i1, i0
    i0 = max(i0, 0)
    i1 = min(i1, len(curve_xy) - 1)
    return curve_xy[i0 : i1 + 1]

def fit_spline_with_min_radius(points_xy: np.ndarray,
                               start_xy: np.ndarray,
                               goal_xy: np.ndarray,
                               Rmin: Optional[float],
                               n: int = 5000,
                               s_max: float = 1e6,
                               iters: int = 18):
    """
    splprep の smoothing s を増やしながら、
    crop後の曲線の最小曲率半径 >= Rmin になるものを探す。
    """
    if Rmin is None or Rmin <= 0:
        curve = fit_spline(points_xy, s=0.0, n=n)
        curve = _crop_curve_between(curve, start_xy, goal_xy)
        return curve, 0.0, None, min_radius_on_polyline(curve)

    s = 0.0
    best = None
    best_s = None
    best_minR = -1.0

    for _k in range(iters):
        curve = fit_spline(points_xy, s=s, n=n)
        curve = _crop_curve_between(curve, start_xy, goal_xy)

        minR = min_radius_on_polyline(curve)
        if minR > best_minR:
            best_minR = minR
            best = curve
            best_s = s

        if minR >= Rmin:
            return curve, s, None, minR

        if s == 0.0:
            s = 1.0
        else:
            s *= 3.0

        if s > s_max:
            break

    warn = f"Could not satisfy min_turn_radius_m={Rmin:.2f}m (best minR={best_minR:.2f}m, s={best_s})."
    return best, float(best_s or 0.0), warn, float(best_minR)

def thin_by_curvature(points_xy):
    out = [points_xy[0]]
    prev = points_xy[0]
    for i in range(1, len(points_xy) - 1):
        R = curvature_radius(points_xy[i - 1], points_xy[i], points_xy[i + 1])
        d = 1.0 if R < 10 else 2.0 if R < 50 else 5.0
        if np.linalg.norm(points_xy[i] - prev) >= d:
            out.append(points_xy[i])
            prev = points_xy[i]
    out.append(points_xy[-1])
    return np.array(out)

def offset_lane(center_xy: np.ndarray, offset_m: float):
    if not offset_m or abs(offset_m) < 1e-9:
        return None, None
    left = []
    right = []
    n = np.array([0.0, 0.0])
    for i in range(len(center_xy) - 1):
        p1, p2 = center_xy[i], center_xy[i + 1]
        seg = p2 - p1
        if np.linalg.norm(seg) < 1e-9:
            continue
        n = np.array([-seg[1], seg[0]])
        n = n / (np.linalg.norm(n) + 1e-12)
        left.append(p1 + n * offset_m)
        right.append(p1 - n * offset_m)
    left.append(center_xy[-1] + n * offset_m)
    right.append(center_xy[-1] - n * offset_m)
    return np.array(left), np.array(right)


# =========================
# Models
# =========================
class MapPoint(BaseModel):
    x: float
    y: float

class HeadingSpecMap(BaseModel):
    point: MapPoint
    heading_deg: float

class WaypointMapRequest(BaseModel):
    route_name: Optional[str] = "Load_to_Dump"
    canonical_epsg: int
    working_epsg: int = DEFAULT_WORKING_EPSG
    start: HeadingSpecMap
    intermediates: List[MapPoint] = Field(default_factory=list)
    goal: HeadingSpecMap
    constraint_mode: Literal["strict_via", "prefer_min_turn_radius"] = "strict_via"
    resample_mode: Literal["curvature_based", "fixed"] = "fixed"
    fixed_spacing_m: Optional[float] = 2.0
    offset_m: Optional[float] = 3.0

    # ★追加：最小旋回半径[m]（working CRS がm単位前提）
    min_turn_radius_m: Optional[float] = None

class MapPointsRequest(BaseModel):
    points: List[MapPoint] = Field(default_factory=list)


# =========================
# HTML
# =========================
INDEX_HTML = r"""
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>FMS Map Building Tool</title>
<style>
  :root{
    --bg-0:#06162d;
    --bg-1:#0a2a52;
    --ink:#dbeafe;
    --ink-soft:#9ec0de;
    --line:#27517d;
    --panel:#0c2446;
    --panel-strong:#11335f;
    --brand:#38bdf8;
    --brand-strong:#0284c7;
    --accent:#fbbf24;
    --ok:#22c55e;
  }
  *{ box-sizing:border-box; }
  body{
    margin:0;
    color:var(--ink);
    background:
      radial-gradient(circle at 8% 0%, #1a4c7a 0%, transparent 38%),
      radial-gradient(circle at 100% 8%, #0e3a66 0%, transparent 45%),
      linear-gradient(180deg, var(--bg-0), var(--bg-1));
    font-family:"Avenir Next","Trebuchet MS","Segoe UI",sans-serif;
  }
  .page{
    width:calc(100vw - 28px);
    margin:12px 14px 18px;
  }
  .titlebar{
    display:flex;
    align-items:flex-end;
    justify-content:space-between;
    gap:12px;
    margin-bottom:12px;
  }
  .title{
    margin:0;
    font-size:28px;
    letter-spacing:.02em;
    color:#eff6ff;
  }
  .subtitle{
    margin:0;
    color:var(--ink-soft);
    font-size:13px;
  }
  .row{ display:flex; gap:10px; flex-wrap:wrap; align-items:center; }
  .app-shell{
    display:grid;
    grid-template-columns:minmax(360px, 460px) minmax(0, 1fr);
    gap:14px;
    align-items:start;
    transition:grid-template-columns .2s ease, gap .2s ease;
  }
  .app-shell.left-collapsed{
    grid-template-columns:0 minmax(0, 1fr);
    gap:0;
  }
  .control-pane{
    position:sticky;
    top:12px;
    max-height:calc(100vh - 24px);
    overflow:auto;
    padding-right:4px;
    transition:opacity .18s ease, transform .18s ease;
  }
  .app-shell.left-collapsed .control-pane{
    opacity:0;
    pointer-events:none;
    transform:translateX(-20px);
    overflow:hidden;
    max-height:0;
    padding:0;
  }
  .map-pane{
    min-height:80vh;
  }
  .card{
    border:1px solid var(--line);
    border-radius:16px;
    padding:12px;
    background:linear-gradient(180deg, var(--panel-strong), var(--panel));
    margin-bottom:10px;
    box-shadow:0 14px 28px rgba(2, 8, 23, 0.28);
  }
  .card h3{
    margin:0 0 8px 0;
    font-size:14px;
    letter-spacing:.03em;
    color:#d7e9ff;
    text-transform:uppercase;
  }
  .action-card{
    position:sticky;
    top:0;
    z-index:8;
    border-color:#3f6f9d;
  }
  .btn{
    padding:9px 12px;
    border-radius:11px;
    border:1px solid #4a77a5;
    background:#12345f;
    color:#e0efff;
    cursor:pointer;
    display:inline-flex;
    gap:6px;
    align-items:center;
    font-weight:600;
  }
  .btn:hover{ border-color:#70a5d7; background:#1a4577; }
  .btn[data-active="true"]{ background:var(--brand-strong); color:#fff; border-color:var(--brand-strong); }
  #generate{
    background:linear-gradient(180deg, #0ea5e9, #0369a1);
    color:#fff;
    border-color:#0284c7;
  }
  #generate:hover{ filter:brightness(1.05); }
  .btn:disabled{ opacity:.45; cursor:not-allowed; }
  .pill{ padding:3px 9px; border-radius:999px; background:#0f315d; color:#dbeafe; font-size:12px; border:1px solid #4f7ba8; }
  .dot{ width:10px; height:10px; border-radius:50%; display:inline-block; border:1px solid #0002; }
  #wrap{
    position:relative;
    display:inline-block;
    width:100%;
    max-width:100%;
    overflow:hidden;
    border-radius:18px;
    border:1px solid #46729e;
    background:#0b1220;
    touch-action:none;
    box-shadow:0 16px 32px rgba(2, 8, 23, 0.25);
  }
  #viewport{ position:relative; transform-origin:0 0; } /* transform only here */
  #imgOrtho, #imgCost{ display:block; width:100%; max-width:100%; height:auto; user-select:none; -webkit-user-drag:none; }
  #imgCost{ position:absolute; left:0; top:0; opacity:0.5; pointer-events:none; }
  #overlay{ position:absolute; left:0; top:0; transform-origin:0 0; }
  .hud{ position:absolute; right:10px; top:10px; background:#020817cc; color:#dbeafe; padding:6px 8px; border-radius:10px; font-size:12px; pointer-events:none; z-index:5; font-weight:700; border:1px solid #294d73; }
  textarea{ width:100%; height:220px; font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; font-size:12px; }
  .hint{ font-size:12px; color:#9bbfe2; }
  .grid2{ display:grid; grid-template-columns: 1fr 1fr; gap:10px; }
  .field{ display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
  .field input, .field select{
    padding:7px 8px;
    border:1px solid #46739f;
    border-radius:9px;
    background:#0d294e;
    color:#e6f2ff;
  }
  .field strong{ color:#dbeafe; font-size:13px; }
  .legend{ display:flex; gap:14px; flex-wrap:wrap; align-items:center; color:#dbeafe; font-size:12px; margin:10px 0; padding:8px 10px; border-radius:12px; background:#0a2344; border:1px solid var(--line); }
  .ln{ width:20px; height:0; border-top:3px solid; display:inline-block; vertical-align:middle; margin-right:6px; }
  input[type="range"]{ width:220px; }
  .map-toolbar{
    display:flex;
    gap:10px;
    flex-wrap:wrap;
    align-items:center;
    justify-content:space-between;
  }
  #statusMsg{
    margin:10px 0 0;
    padding:8px 10px;
    border-radius:10px;
    background:#082544;
    border:1px solid #3f6d99;
  }
  .saved-list{
    display:flex;
    flex-direction:column;
    gap:6px;
  }
  .saved-item{
    display:flex;
    justify-content:space-between;
    align-items:center;
    gap:8px;
    padding:6px 8px;
    border:1px solid #3f6d99;
    border-radius:8px;
    background:#0a2344;
  }
  .saved-item[data-active="true"]{
    border-color:#7dd3fc;
    box-shadow:0 0 0 1px #7dd3fc66 inset;
  }
  .item-main{
    flex:1;
    text-align:left;
    white-space:nowrap;
    overflow:hidden;
    text-overflow:ellipsis;
  }
  .item-del{
    padding:5px 8px;
    border-radius:8px;
    border:1px solid #9f1239;
    background:#4c1022;
    color:#fecdd3;
    cursor:pointer;
  }
  @media (max-width: 1200px){
    .app-shell{ grid-template-columns:1fr; }
    .app-shell.left-collapsed{ grid-template-columns:1fr; gap:14px; }
    .control-pane{ position:relative; max-height:none; top:auto; overflow:visible; }
    .app-shell.left-collapsed .control-pane{
      opacity:1;
      pointer-events:auto;
      transform:none;
      max-height:none;
      padding-right:4px;
    }
    .action-card{ position:relative; top:auto; }
  }
  @media (max-width: 760px){
    .grid2{ grid-template-columns:1fr; }
    input[type="range"]{ width:170px; }
    .page{ width:95vw; margin:12px auto 18px; }
  }
</style>
</head>
<body>
<div class="page">
  <div class="titlebar">
    <div>
      <h1 class="title">FMS Map Building Tool</h1>
    </div>
    <div class="row">
      <span class="pill" id="canonInfo">Canonical: -</span>
      <span class="pill" id="statusInfo">Status: -</span>
      <button class="btn" id="togglePane">Hide Panel</button>
    </div>
  </div>

  <div class="app-shell" id="appShell">
    <aside class="control-pane">
      <div class="card action-card">
        <h3>Actions</h3>
        <div class="row">
          <button class="btn" id="generate">Generate Preview</button>
          <button class="btn" id="save">Save JSON</button>
        </div>
        <div class="row" style="margin-top:8px;">
          <button class="btn" id="saveDraftRoute">Save Draft Route</button>
          <button class="btn" id="newRoute">New Route</button>
          <button class="btn" id="clearDraftRoutes">Clear Draft Routes</button>
          <span class="pill" id="draftCount">Draft routes: 0</span>
        </div>
        <div class="row" style="margin-top:8px;">
          <input type="file" id="fileJson" accept="application/json" multiple />
          <button class="btn" id="showJson">Import JSON</button>
          <button class="btn" id="clearImported">Clear Imported</button>
        </div>
      </div>

      <div class="card">
        <h3>Raster</h3>
        <input type="file" id="fileTiff" accept=".tif,.tiff,image/tiff" style="display:none"/>
        <div class="row">
          <button class="btn" id="chooseTiff">Choose GeoTIFF</button>
          <button class="btn" id="uploadOrtho">Upload Ortho</button>
          <button class="btn" id="uploadCost">Upload Cost</button>
          <button class="btn" id="clearAll">Clear All</button>
        </div>
        <div class="hint" style="margin-top:8px;">CostMap は Upload Cost で反映。Ortho が canonical grid です。</div>
      </div>

      <div class="card">
        <h3>Area Workspace</h3>
        <div class="row">
          <button class="btn" id="modePolygon" data-active="false">Polygon</button>
          <button class="btn" id="finishPolygon">Finish Polygon</button>
          <button class="btn" id="cancelPolygon">Cancel Polygon</button>
          <button class="btn" id="clearPolygons">Clear Polygons</button>
          <span class="pill" id="polygonCount">Polygons: 0</span>
        </div>
        <div class="row" style="margin-top:8px;">
          <label>Area preset:
            <select id="polygonNamePreset">
              <option value="">Custom</option>
              <option value="ParkingZone">ParkingZone</option>
              <option value="LoadingZone">LoadingZone</option>
              <option value="DumpingZone">DumpingZone</option>
              <option value="AutonomousOperationZone">AutonomousOperationZone</option>
              <option value="NoGoZone">NoGoZone</option>
            </select>
          </label>
          <label>Area name: <input id="polygonName" value="" placeholder="Area name (free text)" /></label>
        </div>
        <div class="hint" style="margin-top:8px;">Polygon modeで地図クリックして頂点追加。Finishで確定。</div>
      </div>

      <div class="card">
        <h3>Route Workspace</h3>
        <div class="row">
          <button class="btn" id="modeStart" data-active="true">Set Start</button>
          <button class="btn" id="modeGoal"  data-active="false">Set Goal</button>
          <button class="btn" id="modeVia"   data-active="false">Add Via</button>
          <button class="btn" id="modeInsertVia" data-active="false">Insert Via</button>
          <button class="btn" id="modeEdit"  data-active="false">Edit Points</button>
          <button class="btn" id="modePan"   data-active="false">Pan</button>
          <button class="btn" id="undo">Undo</button>
          <button class="btn" id="reset">Reset</button>
        </div>
        <div class="hint" style="margin-top:8px;">Insert Via: 線分途中に挿入。Edit Points: Start/Goal/Via をドラッグ編集。</div>

        <div class="field" style="margin-top:10px;">
          <strong>Route name:</strong>
          <input id="routeName" value="Load_to_Dump" />
          <strong>Working EPSG:</strong>
          <input id="workingEPSG" type="number" value="6677" />
        </div>
        <div class="grid2">
          <div>
            <div class="field">
              <strong>Start (EPSG)</strong>
              <label>X: <input id="startX" type="number" step="0.001"></label>
              <label>Y: <input id="startY" type="number" step="0.001"></label>
              <button class="btn" id="setStartXY">Set</button>
            </div>
            <div class="field">
              <strong>Start (Lon/Lat)</strong>
              <label>Lon: <input id="startLon" type="number" step="0.0000001"></label>
              <label>Lat: <input id="startLat" type="number" step="0.0000001"></label>
              <button class="btn" id="setStartLL">Set</button>
            </div>
          </div>
          <div>
            <div class="field">
              <strong>Goal (EPSG)</strong>
              <label>X: <input id="goalX" type="number" step="0.001"></label>
              <label>Y: <input id="goalY" type="number" step="0.001"></label>
              <button class="btn" id="setGoalXY">Set</button>
            </div>
            <div class="field">
              <strong>Goal (Lon/Lat)</strong>
              <label>Lon: <input id="goalLon" type="number" step="0.0000001"></label>
              <label>Lat: <input id="goalLat" type="number" step="0.0000001"></label>
              <button class="btn" id="setGoalLL">Set</button>
            </div>
          </div>
        </div>

        <div class="field">
          <label>Heading Start (deg): <input id="headStart" type="number" step="0.1" value="90"></label>
          <label>Heading Goal (deg): <input id="headGoal" type="number" step="0.1" value="90"></label>
          <span class="hint">0=East, 90=North (working CRS axis)</span>
        </div>

        <div class="field">
          <label>Constraint:
            <select id="constraintMode">
              <option value="strict_via" selected>Strict via-points</option>
              <option value="prefer_min_turn_radius">Prefer min turn radius</option>
            </select>
          </label>
          <label>Resample:
            <select id="resampleMode">
              <option value="curvature_based">Curvature-based</option>
              <option value="fixed" selected>Fixed spacing</option>
            </select>
          </label>
          <label>Spacing (m): <input id="fixedSpacing" type="number" step="0.1" value="2"></label>
          <label>Offset (m): <input id="offsetM" type="number" step="0.1" value="3"></label>
          <label>Min turn radius (m): <input id="minTurnR" type="number" step="0.1" value="2"></label>
        </div>
        <div class="hint">Strict via-points: 通過点厳守。最小旋回半径を満たせない場合はエラー。</div>
      </div>

      <div class="card">
        <h3>Saved Items</h3>
        <div class="hint">クリックでハイライト / Deleteで個別削除</div>
        <div class="row" style="margin-top:8px;">
          <button class="btn" id="focusSelected">Focus Selected</button>
          <button class="btn" id="clearSelection">Clear Selection</button>
        </div>
        <div style="margin-top:8px;">
          <strong style="font-size:12px;">Routes</strong>
          <div id="savedRoutesList" class="saved-list"></div>
        </div>
        <div style="margin-top:10px;">
          <strong style="font-size:12px;">Polygons</strong>
          <div id="savedPolygonsList" class="saved-list"></div>
        </div>
      </div>
    </aside>

    <main class="map-pane">
      <div class="card">
        <h3>Map View</h3>
        <div class="map-toolbar">
          <div class="row">
            <button class="btn" id="zoomIn">＋</button>
            <button class="btn" id="zoomOut">－</button>
            <button class="btn" id="zoomReset">Reset</button>
          </div>
          <div class="row" id="costControls">
            <label class="row" style="gap:8px; margin-right:8px;">
              <input type="checkbox" id="showCost" checked />
              <span>Show Cost</span>
            </label>
            <label class="row" style="gap:8px;">
              <span class="hint">Opacity</span>
              <input type="range" id="costOpacity" min="0" max="1" step="0.01" value="0.55" />
              <span class="pill" id="opVal">0.55</span>
            </label>
          </div>
        </div>
        <div class="hint" style="margin-top:8px;">Wheel zoom / Pan modeまたは中・右ドラッグで移動 / Double clickでリセット</div>

        <div class="legend">
          <span><span class="dot" style="background:#22c55e"></span>Start</span>
          <span><span class="dot" style="background:#ef4444"></span>Goal</span>
          <span><span class="dot" style="background:#0ea5e9"></span>Via</span>
          <span><span class="ln" style="border-color:#22d3ee"></span>Spline</span>
          <span><span class="ln" style="border-color:#f59e0b"></span>Final</span>
          <span><span class="ln" style="border-color:#10b981"></span>Left offset</span>
          <span><span class="ln" style="border-color:#8b5cf6"></span>Right offset</span>
          <span class="pill">Imported routes = dark red</span>
        </div>

        <div id="wrap">
          <div id="viewport">
            <img id="imgOrtho" src="" alt="ortho"/>
            <img id="imgCost" src="" alt="cost"/>
            <canvas id="overlay"></canvas>
          </div>
          <div class="hud" id="hud">Zoom 100%</div>
        </div>
        <p class="hint" id="statusMsg">Upload Ortho first.</p>
      </div>
    </main>
  </div>
</div>

<script>
(async function(){
  const $ = (id)=>document.getElementById(id);
  const viewport = $('viewport');
  const imgOrtho = $('imgOrtho');
  const imgCost  = $('imgCost');
  const overlay  = $('overlay');
  const ctx = overlay.getContext('2d');
  const hud = $('hud');
  const statusMsg = $('statusMsg');
  const appShell = $('appShell');
  const togglePaneBtn = $('togglePane');
  const savedRoutesList = $('savedRoutesList');
  const savedPolygonsList = $('savedPolygonsList');

  let meta = null;
  let versions = {ortho:0, cost:0};
  let loaded = {ortho:false, cost:false};
  let syncTimer = null;

  // ---------- transform (zoom/pan on viewport only) ----------
  let scale=1, panX=0, panY=0;
  function clamp(v,min,max){ return Math.max(min, Math.min(max, v)); }
  function applyTransform(){
    const t = `translate(${panX}px,${panY}px) scale(${scale})`;
    viewport.style.transform = t;
    hud.textContent = `Zoom ${Math.round(scale*100)}%`;
  }
  function resizeOverlay(){
    overlay.width  = imgOrtho.clientWidth;
    overlay.height = imgOrtho.clientHeight;
    overlay.style.width  = imgOrtho.clientWidth + 'px';
    overlay.style.height = imgOrtho.clientHeight + 'px';
    drawAll();
  }
  window.addEventListener('resize', resizeOverlay);

  $('zoomIn').onclick  = ()=>{ scale=clamp(scale*1.2, 0.2, 12); applyTransform(); };
  $('zoomOut').onclick = ()=>{ scale=clamp(scale/1.2, 0.2, 12); applyTransform(); };
  $('zoomReset').onclick=()=>{ scale=1; panX=0; panY=0; applyTransform(); };

  function setPaneCollapsed(collapsed){
    if(collapsed){
      appShell.classList.add('left-collapsed');
      togglePaneBtn.textContent = 'Show Panel';
      togglePaneBtn.dataset.active = 'true';
    }else{
      appShell.classList.remove('left-collapsed');
      togglePaneBtn.textContent = 'Hide Panel';
      togglePaneBtn.dataset.active = 'false';
    }
    try{
      localStorage.setItem('leftPaneCollapsed', collapsed ? '1' : '0');
    }catch(_e){}
    setTimeout(resizeOverlay, 220);
  }
  togglePaneBtn.onclick = ()=>{
    const collapsed = appShell.classList.contains('left-collapsed');
    setPaneCollapsed(!collapsed);
  };

  // ★追加: client座標 -> overlay(canvas)座標へ正規化（ズーム/縮尺に強くする）
  function eventToCanvas(ev){
    const rect = overlay.getBoundingClientRect();
    return {
      x: (ev.clientX - rect.left) * (overlay.width  / rect.width),
      y: (ev.clientY - rect.top ) * (overlay.height / rect.height)
    };
  }
  // ★追加: overlay(canvas)座標 -> raster px(meta.width/height)
  function canvasToPx(c){
    const rx = c.x / overlay.width;
    const ry = c.y / overlay.height;
    return { x: rx*meta.width, y: ry*meta.height };
  }

  overlay.addEventListener('wheel', (e)=>{
    if(!meta) return;
    e.preventDefault();

    // ★修正: wheel中心点もcanvas座標に正規化（ズーム中心のズレ低減）
    const c = eventToCanvas(e);
    const cx = c.x;
    const cy = c.y;

    const old = scale;
    const factor = (e.deltaY < 0) ? 1.1 : 1/1.1;
    scale = clamp(scale*factor, 0.2, 12);
    panX = panX - cx*(scale/old - 1);
    panY = panY - cy*(scale/old - 1);
    applyTransform();
  }, {passive:false});

  overlay.addEventListener('dblclick', ()=>{
    scale=1; panX=0; panY=0; applyTransform();
  });

  let spaceDown=false, isPanning=false, panStartX=0, panStartY=0, panMoved=false, suppressNextClick=false;
  function canStartPan(ev){
    return spaceDown || mode==='pan' || ev.button===1 || ev.button===2;
  }
  window.addEventListener('keydown', (e)=>{ if(e.code==='Space') spaceDown=true; });
  window.addEventListener('keyup', (e)=>{ if(e.code==='Space'){ spaceDown=false; isPanning=false; } });

  overlay.addEventListener('mousedown', (e)=>{
    if(canStartPan(e)){
      e.preventDefault();
      isPanning=true;
      panMoved=false;
      panStartX = e.clientX - panX;
      panStartY = e.clientY - panY;
      updateCanvasCursor();
    }
  });
  overlay.addEventListener('contextmenu', (e)=>{
    if(mode==='pan') e.preventDefault();
  });
  window.addEventListener('mousemove', (e)=>{
    if(!isPanning) return;
    panMoved = true;
    panX = e.clientX - panStartX;
    panY = e.clientY - panStartY;
    applyTransform();
  });
  window.addEventListener('mouseup', ()=>{
    if(isPanning && panMoved) suppressNextClick = true;
    isPanning=false;
    updateCanvasCursor();
  });

  applyTransform();

  // ---------- Cost overlay controls ----------
  function updateCostVisibility(){
    const show = $('showCost').checked;
    imgCost.style.display = show ? 'block' : 'none';
  }
  function updateOpacity(){
    const v = parseFloat($('costOpacity').value);
    imgCost.style.opacity = String(v);
    $('opVal').textContent = v.toFixed(2);
  }
  $('showCost').onchange = updateCostVisibility;
  $('costOpacity').oninput = updateOpacity;
  updateOpacity();

  // ---------- status/meta + set image src by version (cache-friendly) ----------
  async function refreshStatus(){
    try{
      const st = await fetch('/status?t='+Date.now()).then(r=>r.json());
      loaded = st.loaded;
      meta = st.canonical || null;
      versions = st.versions || {ortho:0, cost:0};

      $('canonInfo').textContent = meta ? `Canonical: EPSG ${meta.epsg} / ${meta.width}x${meta.height}` : 'Canonical: -';
      $('statusInfo').textContent = `Status: ortho=${loaded.ortho?'OK':'-'} cost=${loaded.cost?'OK':'-'}`;

      if(meta){
        statusMsg.textContent = `Ready. Canonical EPSG:${meta.epsg}`;
      }else{
        statusMsg.textContent = 'Upload Ortho first.';
      }
      return true;
    }catch(_e){
      $('statusInfo').textContent = 'Status: connection error';
      return false;
    }
  }

  function setImageSources(){
    imgOrtho.dataset.fallback = '0';
    imgCost.dataset.fallback = '0';
    if(!loaded.ortho){
      imgOrtho.src = '/raster.png?layer=ortho&v=0';
      imgCost.src  = '/raster.png?layer=cost&v=0';
      return;
    }
    imgOrtho.src = `/raster.png?layer=ortho&v=${versions.ortho}`;
    if(loaded.cost){
      imgCost.src = `/raster.png?layer=cost&v=${versions.cost}`;
    }else{
      imgCost.src = `/raster.png?layer=cost&v=0`;
    }
  }

  async function syncRasters(force=false){
    const prevV = {ortho: versions.ortho, cost: versions.cost};
    const prevLoaded = {ortho: loaded.ortho, cost: loaded.cost};
    const ok = await refreshStatus();
    if(!ok) return;
    const changed =
      force ||
      prevV.ortho !== versions.ortho ||
      prevV.cost !== versions.cost ||
      prevLoaded.ortho !== loaded.ortho ||
      prevLoaded.cost !== loaded.cost;
    if(changed){
      setImageSources();
      updateCostVisibility();
      updateOpacity();
      setTimeout(resizeOverlay, 60);
    }
  }

  imgOrtho.onload = ()=>{
    resizeOverlay();
    updateCostVisibility();
    updateOpacity();
  };
  imgOrtho.onerror = ()=>{
    if(imgOrtho.dataset.fallback !== '1'){
      imgOrtho.dataset.fallback = '1';
      imgOrtho.src = '/raster.png?layer=ortho&v=0&t=' + Date.now();
      statusMsg.textContent = 'Retrying Ortho image without version cache...';
      return;
    }
    statusMsg.textContent = 'Failed to load Ortho raster image.';
  };
  imgCost.onerror = ()=>{
    if(imgCost.dataset.fallback !== '1'){
      imgCost.dataset.fallback = '1';
      imgCost.src = '/raster.png?layer=cost&v=0&t=' + Date.now();
      statusMsg.textContent = 'Retrying Cost image without version cache...';
      return;
    }
    statusMsg.textContent = 'Failed to load Cost raster image.';
  };

  // ---------- upload/clear ----------
  $('chooseTiff').onclick = ()=> $('fileTiff').click();

  async function uploadTo(layer){
    const f = $('fileTiff').files[0];
    if(!f){ alert('Choose a GeoTIFF first.'); return; }
    const fd = new FormData(); fd.append('file', f);
    const res = await fetch('/upload_raster/'+layer, {method:'POST', body:fd});
    const js = await res.json();
    if(js.error){ alert(js.error); return; }

    await refreshStatus();
    setImageSources();
    updateCostVisibility();
    updateOpacity();
  }

  $('uploadOrtho').onclick = ()=>uploadTo('ortho');
  $('uploadCost').onclick  = ()=>uploadTo('cost');

  $('clearAll').onclick = async ()=>{
    await fetch('/clear_all', {method:'POST'});
    state.start=null; state.goal=null; state.mids=[];
    debugMap=null; lastResult=null;
    importedRoutes=[];
    draftRoutes=[];
    polygons=[];
    activePolygon=[];
    selectedDraftIndex = -1;
    selectedPolygonIndex = -1;
    scale=1; panX=0; panY=0; applyTransform();
    updateCounts();
    await refreshStatus();
    setImageSources();
    resizeOverlay();
  };

  // ---------- canonical affine helpers (px<->map) ----------
  function pxToMap(px){
    const a=meta.transform[0], b=meta.transform[1], c=meta.transform[2];
    const d=meta.transform[3], e=meta.transform[4], f=meta.transform[5];
    return { x: a*px.x + b*px.y + c, y: d*px.x + e*px.y + f };
  }
  function mapToPx(mp){
    const a=meta.transform[0], b=meta.transform[1], c=meta.transform[2];
    const d=meta.transform[3], e=meta.transform[4], f=meta.transform[5];
    const det = a*e - b*d;
    if(Math.abs(det) < 1e-12) return {x:0,y:0};
    const invA =  e/det, invB = -b/det;
    const invD = -d/det, invE =  a/det;
    const dx = mp.x - c, dy = mp.y - f;
    return { x: invA*dx + invB*dy, y: invD*dx + invE*dy };
  }
  function screenToPx(ev){
    const rect = overlay.getBoundingClientRect();
    const rx = (ev.clientX - rect.left) / rect.width;
    const ry = (ev.clientY - rect.top) / rect.height;
    return { x: rx*meta.width, y: ry*meta.height };
  }
  function pxToScreen(px){
    const rx = px.x/meta.width;
    const ry = px.y/meta.height;
    return { x: rx*overlay.width, y: ry*overlay.height };
  }

  function drawPolylineMap(pts, stroke, width, dash){
    if(!pts || pts.length < 2) return;
    ctx.beginPath();
    const p0 = pxToScreen(mapToPx(pts[0]));
    ctx.moveTo(p0.x, p0.y);
    for(let i=1;i<pts.length;i++){
      const s = pxToScreen(mapToPx(pts[i]));
      ctx.lineTo(s.x, s.y);
    }
    if(dash && dash.length) ctx.setLineDash(dash);
    ctx.strokeStyle = stroke;
    ctx.lineWidth = width;
    ctx.stroke();
    if(dash && dash.length) ctx.setLineDash([]);
  }

  function drawPointMap(mp, radius, fill, stroke){
    const s = pxToScreen(mapToPx(mp));
    ctx.beginPath();
    ctx.arc(s.x, s.y, radius, 0, Math.PI*2);
    ctx.fillStyle = fill;
    ctx.fill();
    if(stroke){
      ctx.strokeStyle = stroke;
      ctx.stroke();
    }
  }

  function drawLabelMap(mp, text, color){
    const s = pxToScreen(mapToPx(mp));
    ctx.fillStyle = color || "#f8fafc";
    ctx.font = "12px sans-serif";
    ctx.fillText(text, s.x + 6, s.y - 6);
  }

  function fitToMapPoints(points){
    if(!meta || !points || points.length === 0) return;
    const sc = points.map(p => pxToScreen(mapToPx(p)));
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for(const p of sc){
      minX = Math.min(minX, p.x); minY = Math.min(minY, p.y);
      maxX = Math.max(maxX, p.x); maxY = Math.max(maxY, p.y);
    }
    const w = Math.max(8, maxX - minX);
    const h = Math.max(8, maxY - minY);
    const target = Math.min((overlay.width * 0.85) / w, (overlay.height * 0.85) / h);
    scale = clamp(target, 0.2, 12);
    const cx = (minX + maxX) / 2;
    const cy = (minY + maxY) / 2;
    panX = overlay.width / 2 - scale * cx;
    panY = overlay.height / 2 - scale * cy;
    applyTransform();
  }

  // ---------- state ----------
  const state = { start:null, goal:null, mids:[] };
  let debugMap = null;
  let lastResult = null;
  let importedRoutes = [];
  let draftRoutes = [];
  let polygons = [];
  let activePolygon = [];
  let selectedDraftIndex = -1;
  let selectedPolygonIndex = -1;
  const DRAFT_COLORS = ["#f97316", "#fb7185", "#facc15", "#34d399", "#60a5fa", "#a78bfa", "#f43f5e"];
  const POLYGON_COLORS = ["#22c55e", "#14b8a6", "#38bdf8", "#eab308", "#f97316", "#ec4899"];

  function colorByIndex(colors, i){
    return colors[i % colors.length];
  }

  function updateCounts(){
    $('draftCount').textContent = `Draft routes: ${draftRoutes.length}`;
    const active = activePolygon.length ? ` (+${activePolygon.length} editing)` : "";
    $('polygonCount').textContent = `Polygons: ${polygons.length}${active}`;
    renderSavedLists();
  }

  function renderSavedLists(){
    if(savedRoutesList){
      savedRoutesList.innerHTML = '';
      draftRoutes.forEach((r, i)=>{
        const row = document.createElement('div');
        row.className = 'saved-item';
        row.dataset.active = (i===selectedDraftIndex) ? 'true' : 'false';
        const main = document.createElement('button');
        main.className = 'btn item-main';
        main.textContent = r.name || `Route_${i+1}`;
        main.onclick = ()=>{
          selectedDraftIndex = (selectedDraftIndex===i) ? -1 : i;
          drawAll();
          renderSavedLists();
        };
        const del = document.createElement('button');
        del.className = 'item-del';
        del.textContent = 'Delete';
        del.onclick = ()=>{
          draftRoutes.splice(i, 1);
          if(selectedDraftIndex === i) selectedDraftIndex = -1;
          if(selectedDraftIndex > i) selectedDraftIndex -= 1;
          drawAll();
          updateCounts();
        };
        row.appendChild(main);
        row.appendChild(del);
        savedRoutesList.appendChild(row);
      });
    }

    if(savedPolygonsList){
      savedPolygonsList.innerHTML = '';
      polygons.forEach((p, i)=>{
        const row = document.createElement('div');
        row.className = 'saved-item';
        row.dataset.active = (i===selectedPolygonIndex) ? 'true' : 'false';
        const main = document.createElement('button');
        main.className = 'btn item-main';
        main.textContent = p.name || `Area_${i+1}`;
        main.onclick = ()=>{
          selectedPolygonIndex = (selectedPolygonIndex===i) ? -1 : i;
          drawAll();
          renderSavedLists();
        };
        const del = document.createElement('button');
        del.className = 'item-del';
        del.textContent = 'Delete';
        del.onclick = ()=>{
          polygons.splice(i, 1);
          if(selectedPolygonIndex === i) selectedPolygonIndex = -1;
          if(selectedPolygonIndex > i) selectedPolygonIndex -= 1;
          drawAll();
          updateCounts();
        };
        row.appendChild(main);
        row.appendChild(del);
        savedPolygonsList.appendChild(row);
      });
    }
  }
  $('clearSelection').onclick = ()=>{
    selectedDraftIndex = -1;
    selectedPolygonIndex = -1;
    renderSavedLists();
    drawAll();
  };
  $('focusSelected').onclick = ()=>{
    if(selectedDraftIndex >= 0 && draftRoutes[selectedDraftIndex]){
      const r = draftRoutes[selectedDraftIndex];
      const pts = (r.thinned && r.thinned.length) ? r.thinned : (r.spline || []);
      fitToMapPoints(pts);
      return;
    }
    if(selectedPolygonIndex >= 0 && polygons[selectedPolygonIndex]){
      fitToMapPoints(polygons[selectedPolygonIndex].points || []);
      return;
    }
    alert('Select a route or polygon first.');
  };

  // ---------- modes ----------
  let mode='start';
  function updateCanvasCursor(){
    if(isPanning){
      overlay.style.cursor = 'grabbing';
      return;
    }
    if(mode==='pan'){
      overlay.style.cursor = 'grab';
      return;
    }
    if(mode==='edit'){
      overlay.style.cursor = 'move';
      return;
    }
    if(mode==='polygon'){
      overlay.style.cursor = 'crosshair';
      return;
    }
    overlay.style.cursor = 'crosshair';
  }
  function setMode(m){
    mode=m;
    $('modeStart').dataset.active=(m==='start');
    $('modeGoal').dataset.active =(m==='goal');
    $('modeVia').dataset.active  =(m==='via');
    $('modeInsertVia').dataset.active=(m==='insert_via');
    $('modeEdit').dataset.active =(m==='edit');
    $('modePolygon').dataset.active =(m==='polygon');
    $('modePan').dataset.active  =(m==='pan');
    updateCanvasCursor();
  }
  $('modeStart').onclick=()=>setMode('start');
  $('modeGoal').onclick =()=>setMode('goal');
  $('modeVia').onclick  =()=>setMode('via');
  $('modeInsertVia').onclick=()=>setMode('insert_via');
  $('modeEdit').onclick =()=>setMode('edit');
  $('modePolygon').onclick =()=>setMode('polygon');
  $('modePan').onclick =()=>setMode('pan');

  $('undo').onclick=()=>{
    if(mode==='polygon' && activePolygon.length){
      activePolygon.pop();
    }else if(state.mids.length) state.mids.pop();
    else if(state.goal) state.goal=null;
    else if(state.start) state.start=null;
    debugMap=null;
    drawAll();
    updateCounts();
  };
  $('reset').onclick=()=>{
    state.start=null; state.goal=null; state.mids=[];
    debugMap=null; lastResult=null;
    activePolygon=[];
    drawAll();
    updateCounts();
  };
  $('newRoute').onclick=()=>{
    state.start=null; state.goal=null; state.mids=[];
    debugMap=null; lastResult=null;
    activePolygon=[];
    setMode('start');
    statusMsg.textContent = 'New route started. Draft routes remain.';
    drawAll();
    updateCounts();
  };
  $('saveDraftRoute').onclick=()=>{
    if(!debugMap || !debugMap.thinned || debugMap.thinned.length < 2){
      alert('Generate route first.');
      return;
    }
    const idx = draftRoutes.length;
    const name = (($('routeName').value||'Route').trim() || 'Route') + `_${idx+1}`;
    draftRoutes.push({
      name,
      color: colorByIndex(DRAFT_COLORS, idx),
      haulRoute: (lastResult && lastResult.haulRoutes && lastResult.haulRoutes[0]) ? lastResult.haulRoutes[0].route : null,
      spline: (debugMap.spline || []).map(p=>({x:p.x, y:p.y})),
      thinned: (debugMap.thinned || []).map(p=>({x:p.x, y:p.y})),
      left: (debugMap.offset_left || []).map(p=>({x:p.x, y:p.y})),
      right: (debugMap.offset_right || []).map(p=>({x:p.x, y:p.y})),
    });
    statusMsg.textContent = `Draft route saved: ${name}`;
    updateCounts();
    drawAll();
  };
  $('clearDraftRoutes').onclick=()=>{
    draftRoutes = [];
    selectedDraftIndex = -1;
    updateCounts();
    drawAll();
  };
  $('polygonNamePreset').onchange = ()=>{
    const preset = (($('polygonNamePreset').value || '').trim());
    if(preset){
      $('polygonName').value = preset;
    }
  };
  $('finishPolygon').onclick=()=>{
    if(activePolygon.length < 3){
      alert('Polygon needs at least 3 points.');
      return;
    }
    const customName = (($('polygonName').value || '').trim());
    const presetName = (($('polygonNamePreset').value || '').trim());
    const polyName = customName || presetName || `Area_${polygons.length + 1}`;
    polygons.push({
      name: polyName,
      color: colorByIndex(POLYGON_COLORS, polygons.length),
      points: activePolygon.map(p=>({x:p.x, y:p.y})),
    });
    activePolygon = [];
    $('polygonName').value = '';
    $('polygonNamePreset').value = '';
    statusMsg.textContent = 'Polygon saved.';
    updateCounts();
    drawAll();
  };
  $('cancelPolygon').onclick=()=>{
    activePolygon = [];
    updateCounts();
    drawAll();
  };
  $('clearPolygons').onclick=()=>{
    polygons = [];
    activePolygon = [];
    selectedPolygonIndex = -1;
    updateCounts();
    drawAll();
  };

  function routePointsForEdit(){
    const pts = [];
    if(state.start) pts.push(state.start);
    pts.push(...state.mids);
    if(state.goal) pts.push(state.goal);
    return pts;
  }

  function dist2PointToSegment(p, a, b){
    const abx = b.x - a.x;
    const aby = b.y - a.y;
    const apx = p.x - a.x;
    const apy = p.y - a.y;
    const den = abx*abx + aby*aby;
    if(den < 1e-12){
      const dx = p.x - a.x;
      const dy = p.y - a.y;
      return dx*dx + dy*dy;
    }
    const t = Math.max(0, Math.min(1, (apx*abx + apy*aby) / den));
    const qx = a.x + t*abx;
    const qy = a.y + t*aby;
    const dx = p.x - qx;
    const dy = p.y - qy;
    return dx*dx + dy*dy;
  }

  function insertViaAtNearestSegment(mp){
    if(!state.start || !state.goal){
      state.mids.push(mp);
      return;
    }
    const pts = routePointsForEdit();
    if(pts.length < 2){
      state.mids.push(mp);
      return;
    }
    let bestSeg = 0;
    let bestD2 = Number.POSITIVE_INFINITY;
    for(let i=0;i<pts.length-1;i++){
      const d2 = dist2PointToSegment(mp, pts[i], pts[i+1]);
      if(d2 < bestD2){
        bestD2 = d2;
        bestSeg = i;
      }
    }
    const insertAt = Math.max(0, Math.min(state.mids.length, bestSeg));
    state.mids.splice(insertAt, 0, mp);
  }

  // click to set points
  overlay.addEventListener('click', (ev)=>{
    if(!meta){ alert('Upload Ortho first.'); return; }
    if(suppressNextClick){ suppressNextClick = false; return; }
    if(isPanning || spaceDown) return;
    if(mode==='edit' || mode==='pan') return;

    const mp = pxToMap(screenToPx(ev));
    if(mode==='start') state.start = mp;
    if(mode==='goal')  state.goal  = mp;
    if(mode==='via')   state.mids.push(mp);
    if(mode==='insert_via') insertViaAtNearestSegment(mp);
    if(mode==='polygon') activePolygon.push(mp);

    debugMap=null;
    drawAll();
    updateCounts();
  });

  // drag edit points
  let draggingTarget = null;
  const HIT=10; // canvas座標系(px)での当たり判定
  overlay.addEventListener('mousedown',(ev)=>{
    if(!meta) return;
    if(canStartPan(ev)) return;
    if(mode!=='edit') return;
    if(spaceDown) return;

    // ★修正: client座標をcanvas座標へ正規化して当たり判定
    const {x:sx, y:sy} = eventToCanvas(ev);

    draggingTarget = null;
    if(state.start){
      const p = pxToScreen(mapToPx(state.start));
      if(Math.hypot(p.x-sx, p.y-sy) < HIT){ draggingTarget = {kind:'start'}; return; }
    }
    if(state.goal){
      const p = pxToScreen(mapToPx(state.goal));
      if(Math.hypot(p.x-sx, p.y-sy) < HIT){ draggingTarget = {kind:'goal'}; return; }
    }
    for(let i=0;i<state.mids.length;i++){
      const sc = pxToScreen(mapToPx(state.mids[i])); // scはcanvas座標系
      if(Math.hypot(sc.x-sx, sc.y-sy) < HIT){ draggingTarget = {kind:'mid', index:i}; break; }
    }
  });

  window.addEventListener('mousemove',(ev)=>{
    if(!draggingTarget) return;
    if(!meta) return;
    if(spaceDown) return;

    // ★修正: event -> canvas -> raster px -> map
    const c = eventToCanvas(ev);
    const mp = pxToMap(canvasToPx(c));
    if(draggingTarget.kind === 'start') state.start = mp;
    if(draggingTarget.kind === 'goal') state.goal = mp;
    if(draggingTarget.kind === 'mid') state.mids[draggingTarget.index] = mp;
    debugMap=null;
    drawAll();
  });
  window.addEventListener('mouseup',()=>{ draggingTarget = null; });

  // ---------- set Start/Goal by inputs ----------
  async function xyToCanonical(x,y,src_epsg){
    const r = await fetch(`/xy_to_canonical?x=${encodeURIComponent(x)}&y=${encodeURIComponent(y)}&src_epsg=${encodeURIComponent(src_epsg)}&t=${Date.now()}`).then(r=>r.json());
    if(r.error) throw new Error(r.error);
    return {x:r.x, y:r.y};
  }
  async function lonlatToCanonical(lon,lat){
    const r = await fetch(`/lonlat_to_canonical?lon=${encodeURIComponent(lon)}&lat=${encodeURIComponent(lat)}&t=${Date.now()}`).then(r=>r.json());
    if(r.error) throw new Error(r.error);
    return {x:r.x, y:r.y};
  }

  $('setStartXY').onclick = async ()=>{
    if(!meta){ alert('Upload Ortho first.'); return; }
    const x=parseFloat($('startX').value||'');
    const y=parseFloat($('startY').value||'');
    const epsg=parseInt($('workingEPSG').value||'6677',10) || 6677;
    if(!Number.isFinite(x)||!Number.isFinite(y)) return;
    try{ state.start = await xyToCanonical(x,y,epsg); debugMap=null; drawAll(); setMode('goal'); }
    catch(e){ alert(String(e)); }
  };

  $('setGoalXY').onclick = async ()=>{
    if(!meta){ alert('Upload Ortho first.'); return; }
    const x=parseFloat($('goalX').value||'');
    const y=parseFloat($('goalY').value||'');
    const epsg=parseInt($('workingEPSG').value||'6677',10) || 6677;
    if(!Number.isFinite(x)||!Number.isFinite(y)) return;
    try{ state.goal = await xyToCanonical(x,y,epsg); debugMap=null; drawAll(); setMode('via'); }
    catch(e){ alert(String(e)); }
  };

  $('setStartLL').onclick = async ()=>{
    if(!meta){ alert('Upload Ortho first.'); return; }
    const lon=parseFloat($('startLon').value||'');
    const lat=parseFloat($('startLat').value||'');
    if(!Number.isFinite(lon)||!Number.isFinite(lat)) return;
    try{ state.start = await lonlatToCanonical(lon,lat); debugMap=null; drawAll(); setMode('goal'); }
    catch(e){ alert(String(e)); }
  };

  $('setGoalLL').onclick = async ()=>{
    if(!meta){ alert('Upload Ortho first.'); return; }
    const lon=parseFloat($('goalLon').value||'');
    const lat=parseFloat($('goalLat').value||'');
    if(!Number.isFinite(lon)||!Number.isFinite(lat)) return;
    try{ state.goal = await lonlatToCanonical(lon,lat); debugMap=null; drawAll(); setMode('via'); }
    catch(e){ alert(String(e)); }
  };

  // ---------- drawing ----------
  function drawUserBase(){
    const pts=[];
    if(state.start) pts.push(state.start);
    pts.push(...state.mids);
    if(state.goal) pts.push(state.goal);

    drawPolylineMap(pts, "#ffffffaa", 2);
    if(state.start) drawPointMap(state.start, 5, "#22c55e", "#0005");
    if(state.goal)  drawPointMap(state.goal, 5, "#ef4444", "#0005");
    state.mids.forEach(m=>drawPointMap(m, 5, "#0ea5e9", "#0005"));
  }

  function drawDebug(){
    if(!debugMap) return;
    drawPolylineMap(debugMap.spline, "#22d3ee", 3);
    drawPolylineMap(debugMap.thinned, "#f59e0b", 3);

    if(debugMap.thinned && debugMap.thinned.length){
      for(const mp of debugMap.thinned){
        drawPointMap(mp, 3, "#f59e0b", "#0004");
      }
    }

    drawPolylineMap(debugMap.offset_left,  "#10b981", 2);
    drawPolylineMap(debugMap.offset_right, "#8b5cf6", 2);
  }

  function drawDraftRoutes(){
    draftRoutes.forEach((r, i)=>{
      const selected = (i === selectedDraftIndex);
      drawPolylineMap(r.spline, `${r.color}88`, 2);
      drawPolylineMap(r.thinned, r.color, selected ? 5 : 3);
      drawPolylineMap(r.left, "#10b981", 2);
      drawPolylineMap(r.right, "#8b5cf6", 2);
      if(selected && r.thinned && r.thinned.length){
        drawPolylineMap(r.thinned, "#ffffffaa", 2);
      }
      if(r.thinned && r.thinned.length){
        drawLabelMap(r.thinned[0], r.name, "#f8fafc");
      }
    });
  }

  function drawPolygons(){
    polygons.forEach((poly, i)=>{
      const pts = poly.points || [];
      if(pts.length < 3) return;
      const selected = (i === selectedPolygonIndex);
      ctx.beginPath();
      let p0 = pxToScreen(mapToPx(pts[0]));
      ctx.moveTo(p0.x, p0.y);
      for(let i=1;i<pts.length;i++){
        const s = pxToScreen(mapToPx(pts[i]));
        ctx.lineTo(s.x, s.y);
      }
      ctx.closePath();
      ctx.fillStyle = `${poly.color}${selected ? "66" : "44"}`;
      ctx.strokeStyle = selected ? "#ffffff" : poly.color;
      ctx.lineWidth = selected ? 3 : 2;
      ctx.fill();
      ctx.stroke();

      const label = poly.name || "Area";
      drawLabelMap(pts[0], label, "#f8fafc");
    });

    if(activePolygon.length){
      drawPolylineMap(activePolygon, "#fbbf24", 2, [6, 4]);
      for(const mp of activePolygon){
        drawPointMap(mp, 4, "#fbbf24");
      }
    }
  }

  function drawImported(){
    for(const r of importedRoutes){
      if(!r.pts || r.pts.length<2) continue;

      drawPolylineMap(r.pts, "#b91c1c", 3);

      for(const mp of r.pts){
        drawPointMap(mp, 3, "#7f1d1d", "#0004");
      }

      drawPolylineMap(r.left,  "#10b981", 2);
      drawPolylineMap(r.right, "#8b5cf6", 2);
    }
  }

  function drawAll(){
    if(!meta){ ctx.clearRect(0,0,overlay.width,overlay.height); return; }
    ctx.clearRect(0,0,overlay.width,overlay.height);
    drawPolygons();
    drawDraftRoutes();
    drawUserBase();
    drawDebug();
    drawImported();
  }

  // ---------- generate/save ----------
  $('generate').onclick = async ()=>{
    if(!meta){ alert('Upload Ortho first.'); return; }
    if(!state.start || !state.goal){ alert('Please set Start and Goal.'); return; }
    statusMsg.textContent = 'Generating route...';

    const minRraw = ($('minTurnR').value||'').trim();
    const minR = minRraw ? parseFloat(minRraw) : null;

    const body = {
      route_name: ($('routeName').value||'Load_to_Dump').trim() || 'Load_to_Dump',
      canonical_epsg: meta.epsg,
      working_epsg: parseInt(($('workingEPSG').value||'6677'),10) || 6677,
      start: { point: state.start, heading_deg: parseFloat($('headStart').value||'90') },
      intermediates: state.mids,
      goal: { point: state.goal, heading_deg: parseFloat($('headGoal').value||'90') },
      constraint_mode: $('constraintMode').value,
      resample_mode: $('resampleMode').value,
      fixed_spacing_m: parseFloat($('fixedSpacing').value||'2'),
      offset_m: parseFloat($('offsetM').value||'0'),
      min_turn_radius_m: (Number.isFinite(minR) ? minR : null)
    };

    const res = await fetch('/waypoints_from_map', {
      method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body)
    });
    const js = await res.json();
    if(js.error){
      statusMsg.textContent = `Error: ${js.error}`;
      alert(js.error);
      return;
    }
    if(js.warning){ alert('⚠ ' + js.warning); }
    statusMsg.textContent = 'Route generated.';

    lastResult = { haulRoutes: js.haulRoutes };
    debugMap = js.debug_map || null;
    drawAll();
  };

  async function canonicalPointsToLonLat(points){
    if(!meta || !points || !points.length) return [];
    const body = { points: points.map(p=>({x:p.x, y:p.y})) };
    const res = await fetch('/canonical_to_lonlat_points', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(body),
    });
    const js = await res.json();
    if(js.error) throw new Error(js.error);
    return js.points || [];
  }

  $('save').onclick = async ()=>{
    try{
      if(!lastResult && !draftRoutes.length && !polygons.length){
        alert('Nothing to save. Generate route or create polygons first.');
        return;
      }

      const outRoutes = [];
      for(const r of draftRoutes){
        if(r.haulRoute && r.haulRoute.length){
          outRoutes.push({name: r.name, route: r.haulRoute});
        }
      }
      if(lastResult && lastResult.haulRoutes && lastResult.haulRoutes.length){
        for(const r of lastResult.haulRoutes){
          outRoutes.push(r);
        }
      }

      const polygonsOut = [];
      for(const p of polygons){
        const points_xy = (p.points || []).map(q=>({x:q.x, y:q.y}));
        const points = await canonicalPointsToLonLat(p.points || []);
        polygonsOut.push({
          name: p.name || `Area_${polygonsOut.length+1}`,
          points,
          points_xy,
        });
      }

      const payload = { haulRoutes: outRoutes, polygons: polygonsOut };
      const name = (($('routeName').value||'route').trim()||'route').replace(/[^-_.a-zA-Z0-9]/g,'_');
      const blob = new Blob([JSON.stringify(payload, null, 2)], {type:'application/json'});
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a'); a.href=url; a.download=name+'.json'; a.click();
      URL.revokeObjectURL(url);
    }catch(e){
      alert(`Save failed: ${String(e)}`);
    }
  };

  // ---------- import JSON multi ----------
  $('showJson').onclick = async ()=>{
    if(!meta){ alert('Upload Ortho first.'); return; }
    const files = Array.from($('fileJson').files||[]);
    if(!files.length){ alert('Select JSON files.'); return; }

    const offset_m = parseFloat($('offsetM').value||'0') || 0;
    let combined = { haulRoutes: [] };

    for(const f of files){
      try{
        const text = await f.text();
        const parsed = JSON.parse(text);
        if(parsed && parsed.haulRoutes && Array.isArray(parsed.haulRoutes)){
          combined.haulRoutes.push(...parsed.haulRoutes);
        }
      }catch(e){ console.warn('JSON parse error', f.name, e); }
    }
    if(!combined.haulRoutes.length){ alert('No valid haulRoutes found.'); return; }

    const res = await fetch('/json_to_canonical_with_offset', {
      method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({ payload: combined, offset_m })
    });
    const js = await res.json();
    if(js.error){ alert(js.error); return; }

    importedRoutes = (importedRoutes||[]).concat(js.routes||[]);
    drawAll();
  };
  $('clearImported').onclick = ()=>{ importedRoutes=[]; drawAll(); updateCounts(); };

  // ---------- init ----------
  let initiallyCollapsed = false;
  try{
    initiallyCollapsed = (localStorage.getItem('leftPaneCollapsed') === '1');
  }catch(_e){}
  setPaneCollapsed(initiallyCollapsed);
  await syncRasters(true);
  updateCanvasCursor();
  updateCounts();
  applyTransform();
  if(syncTimer) clearInterval(syncTimer);
  syncTimer = setInterval(()=>{ syncRasters(false); }, 3000);
})();
</script>
</body>
</html>
"""


# =========================
# Routes: UI
# =========================
@app.get("/", include_in_schema=False)
def index():
    resp = HTMLResponse(INDEX_HTML)
    resp.headers["Cache-Control"] = "no-store"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["Expires"] = "0"
    return resp

@app.get("/favicon.ico", include_in_schema=False)
def favicon():
    return Response(status_code=204)

@app.get("/whoami")
def whoami():
    return {"file": __file__, "cwd": os.getcwd(), "title": app.title}


# =========================
# Routes: status / upload / clear
# =========================
@app.get("/status")
def status():
    loaded = {"ortho": ORTHO_RASTER.exists(), "cost": COST_RASTER.exists()}
    canonical = None
    if loaded["ortho"]:
        try:
            canonical = canonical_meta()
        except Exception:
            canonical = None
    return {
        "loaded": loaded,
        "canonical": canonical,
        "versions": {
            "ortho": _version_of(ORTHO_RASTER),
            "cost": _version_of(COST_RASTER),
        }
    }

@app.post("/upload_raster/{layer}")
async def upload_raster(layer: Literal["ortho", "cost"], file: UploadFile = File(...)):
    try:
        content = await file.read()
        p = _raster_path(layer)
        with open(p, "wb") as f:
            f.write(content)
        return {"ok": True, "layer": layer, "path": str(p), "version": _version_of(p)}
    except Exception as e:
        return JSONResponse({"error": f"Upload failed: {e}"}, status_code=500)

@app.post("/clear_all")
def clear_all():
    try:
        if ORTHO_RASTER.exists():
            ORTHO_RASTER.unlink()
        if COST_RASTER.exists():
            COST_RASTER.unlink()
        return {"ok": True}
    except Exception as e:
        return JSONResponse({"error": f"Clear failed: {e}"}, status_code=500)


# =========================
# Routes: raster PNG (always canonical grid)
# =========================
@app.get("/raster.png")
def raster_png(layer: Literal["ortho", "cost"] = "ortho", v: int = 0):
    """
    Always returns image on canonical (Ortho) grid size.
    With v param, response is cache-friendly (immutable).
    """
    try:
        if not ORTHO_RASTER.exists():
            buf = io.BytesIO()
            Image.new("RGBA", (1, 1), (0, 0, 0, 0)).save(buf, format="PNG")
            buf.seek(0)
            return StreamingResponse(buf, media_type="image/png")

        if layer == "cost" and not COST_RASTER.exists():
            buf = io.BytesIO()
            Image.new("RGBA", (1, 1), (0, 0, 0, 0)).save(buf, format="PNG")
            buf.seek(0)
            return StreamingResponse(buf, media_type="image/png")

        rgb = _read_as_display_rgb_on_canonical(layer)
        buf = _png_bytes_from_rgb(rgb)

        resp = StreamingResponse(buf, media_type="image/png")
        if v:
            resp.headers["Cache-Control"] = "public, max-age=31536000, immutable"
        else:
            resp.headers["Cache-Control"] = "no-store"
        return resp

    except Exception:
        buf = io.BytesIO()
        Image.new("RGB", (1, 1), (255, 0, 0)).save(buf, format="PNG")
        buf.seek(0)
        return StreamingResponse(buf, media_type="image/png", status_code=500)


# =========================
# Routes: coordinate conversion for input helpers
# =========================
@app.get("/xy_to_canonical")
def xy_to_canonical(x: float, y: float, src_epsg: int):
    try:
        meta = canonical_meta()
        dst_epsg = int(meta["epsg"])
        arr = np.array([[float(x), float(y)]], dtype=float)
        out = proj_xy(arr, int(src_epsg), dst_epsg)
        return {"x": float(out[0, 0]), "y": float(out[0, 1]), "dst_epsg": dst_epsg}
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)

@app.get("/lonlat_to_canonical")
def lonlat_to_canonical(lon: float, lat: float):
    try:
        meta = canonical_meta()
        dst_epsg = int(meta["epsg"])
        arr = np.array([[float(lon), float(lat)]], dtype=float)
        out = proj_xy(arr, 4326, dst_epsg)
        return {"x": float(out[0, 0]), "y": float(out[0, 1]), "dst_epsg": dst_epsg}
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)

@app.post("/canonical_to_lonlat_points")
def canonical_to_lonlat_points(req: MapPointsRequest):
    try:
        meta = canonical_meta()
        src_epsg = int(meta["epsg"])
        if not req.points:
            return {"points": []}
        arr = np.array([[float(p.x), float(p.y)] for p in req.points], dtype=float)
        ll = proj_xy(arr, src_epsg, 4326)
        out = [{"lat": float(lat), "lng": float(lon)} for lon, lat in ll]
        return {"points": out, "src_epsg": src_epsg, "dst_epsg": 4326}
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)


# =========================
# Route: Waypoint generation (map-based)
# =========================
@app.post("/waypoints_from_map")
def waypoints_from_map(req: WaypointMapRequest):
    try:
        meta = canonical_meta()
        canonical_epsg = int(req.canonical_epsg or DEFAULT_WORKING_EPSG)
        canonical_epsg_server = int(meta["epsg"])
        if canonical_epsg != canonical_epsg_server:
            return JSONResponse(
                {
                    "error": (
                        f"canonical_epsg mismatch: request={canonical_epsg}, "
                        f"server={canonical_epsg_server}"
                    )
                },
                status_code=400,
            )
        working_epsg = int(req.working_epsg or DEFAULT_WORKING_EPSG)

        pts = [req.start.point] + req.intermediates + [req.goal.point]
        arr = np.array([[p.x, p.y] for p in pts], dtype=float)
        xy_work = proj_xy(arr, canonical_epsg, working_epsg)

        start_xy = xy_work[0].copy()
        goal_xy  = xy_work[-1].copy()

        xy_g = add_heading_ghost(xy_work, float(req.start.heading_deg), 20.0, at_start=True)
        xy_g = add_heading_ghost(xy_g, float(req.goal.heading_deg),  20.0, at_start=False)

        # ★最小旋回半径
        Rmin = float(req.min_turn_radius_m) if req.min_turn_radius_m else None

        warnR = None
        if req.constraint_mode == "strict_via":
            spline_xy = fit_spline(xy_g, s=0.0, n=5000)
            spline_xy = _crop_curve_between(spline_xy, start_xy, goal_xy)
            used_s = 0.0
            measured_minR = min_radius_on_polyline(spline_xy)
            if Rmin and measured_minR < Rmin:
                return JSONResponse(
                    {
                        "error": (
                            f"Min turn radius violated in strict_via mode: "
                            f"required={Rmin:.2f}m, measured={measured_minR:.2f}m"
                        )
                    },
                    status_code=400,
                )
        else:
            spline_xy, used_s, warnR, measured_minR = fit_spline_with_min_radius(
                xy_g, start_xy=start_xy, goal_xy=goal_xy, Rmin=Rmin, n=5000
            )

        if req.resample_mode == "curvature_based":
            final_xy = thin_by_curvature(spline_xy)
        else:
            final_xy = resample_by_spacing(spline_xy, float(req.fixed_spacing_m or 2.0))

        left_xy, right_xy = offset_lane(final_xy, float(req.offset_m or 0.0))

        lonlat = map_to_lonlat(final_xy, working_epsg)
        route = [{"lat": float(lat), "lng": float(lon)} for lon, lat in lonlat]

        def to_canon(arr_work):
            if arr_work is None:
                return None
            arr_c = proj_xy(arr_work, working_epsg, canonical_epsg)
            return [{"x": float(x), "y": float(y)} for x, y in arr_c]

        debug_map = {
            "spline": to_canon(spline_xy),
            "thinned": to_canon(final_xy),
            "offset_left": to_canon(left_xy),
            "offset_right": to_canon(right_xy),
        }

        name = (req.route_name or "Route").strip() or "Route"
        resp = {
            "haulRoutes": [{"name": name, "route": route}],
            "debug_map": debug_map,
            "debug_info": {
                "constraint_mode": req.constraint_mode,
                "output_epsg": 4326,
                "used_s": float(used_s),
                "requested_min_turn_radius_m": (float(Rmin) if Rmin else None),
                "measured_min_radius_m": float(measured_minR),
            }
        }
        if warnR:
            resp["warning"] = warnR

        return JSONResponse(resp)

    except Exception as e:
        return JSONResponse({"error": f"Generation failed: {e}"}, status_code=500)


# =========================
# Route: Import JSON -> canonical map points (with offsets)
# =========================
@app.post("/json_to_canonical_with_offset")
def json_to_canonical_with_offset(payload: Dict[str, Any]):
    try:
        meta = canonical_meta()
        canon_epsg = int(meta["epsg"])

        data = payload.get("payload", {})
        routes = data.get("haulRoutes", None)
        offset_m = float(payload.get("offset_m", 0.0) or 0.0)

        if not isinstance(routes, list) or not routes:
            return JSONResponse({"error": "haulRoutes missing or empty."}, status_code=400)

        out_routes = []
        for r in routes:
            name = str(r.get("name", "Route"))
            pts = r.get("route", [])
            if not isinstance(pts, list) or not pts:
                continue

            lonlat = []
            for p in pts:
                if not isinstance(p, dict) or "lat" not in p or "lng" not in p:
                    continue
                lonlat.append([float(p["lng"]), float(p["lat"])])
            if not lonlat:
                continue

            arr_ll = np.array(lonlat, dtype=float)
            xy = proj_xy(arr_ll, 4326, canon_epsg)

            left_xy, right_xy = (None, None)
            if abs(offset_m) > 1e-9:
                left_xy, right_xy = offset_lane(xy, offset_m)

            def to_list(arr):
                if arr is None: return None
                return [{"x": float(x), "y": float(y)} for x, y in arr]

            out_routes.append({
                "name": name,
                "pts": to_list(xy),
                "left": to_list(left_xy),
                "right": to_list(right_xy)
            })

        if not out_routes:
            return JSONResponse({"error": "No valid route found."}, status_code=400)

        return JSONResponse({"routes": out_routes})

    except Exception as e:
        return JSONResponse({"error": f"Conversion failed: {e}"}, status_code=500)


# =========================
# Local run
# =========================
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)
