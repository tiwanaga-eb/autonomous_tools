# 設計書 — FMS Route Studio

> **本書のステータス**: ドラフト 0.1（2026-07-01 初版）
> **位置づけ**: [要件定義書（REQUIREMENTS.md）](REQUIREMENTS.md) の SysRS（FRS-SysRS-*）を**どう実装するか**を定義する。コードに基づく現状設計のスナップショット。
> **対象コード**: `fms-route-studio/`（packages/planning_core, services/api, apps/web）

## 関連文書

- [要件定義書](REQUIREMENTS.md) — 何を満たすか（StRS/SysRS）
- [シナリオレポート](SCENARIO_REPORT.md) — 鉱山/土木シナリオの自動チューニング結果
- 上位: `unmanned-requirements/AutonomousDumptruck`（FMS 運航前工程 S-00 の上位概念）

---

## 1. アーキテクチャ概要

### 1.1 全体構成（3層モノレポ）

```
┌────────────────────────────────────────────────────────────┐
│  apps/web  (React + TS + Vite + Zustand)        :5173        │
│   2D 地図(OpenLayers, EPSG:6677 native) / 3D(three.js)        │
│   panels(data/map/route/vehicle/spotting/areas/fleet/ai/...)  │
│   store(Zustand) ── commandBus(Undo/Redo) ── api/client.ts    │
└───────────────────────────┬────────────────────────────────┘
                  /api, /health を :8077 へプロキシ (dev)
┌───────────────────────────┴────────────────────────────────┐
│  services/api  (FastAPI, API 専用)              :8077        │
│   routers: layers/tiles/costmap/drivable/geo/planning/        │
│            vehicles/simulate/projects/agent/fleet             │
│   store.py(レジストリ) / settings.py / _proj_fix.py / cogio   │
│   ── planning_core への薄いラッパ ──                           │
└───────────────────────────┬────────────────────────────────┘
┌───────────────────────────┴────────────────────────────────┐
│  packages/planning_core  (純 Python ライブラリ)              │
│   planners / analysis / costmap / drivable / simulator /      │
│   fleet / vehicle / geometry / io / footprint / models        │
└─────────────────────────────────────────────────────────────┘
            永続化: ~/.fms-route-studio/data (JSON レジストリ + COG)
```

**設計原則**:
- **本体ロジックは `planning_core` に集約**（純 Python、I/O・FastAPI 非依存）。API はその薄いラッパ。これにより CLI（`scripts/scenario_report.py`）からも同一ロジックを再利用できる。
- **作業座標系ネイティブ**: フロント・計算は JGD2011 平面直角（既定 EPSG:6677）で一貫。Web メルカトル再投影を避け 3cm 級精度を保持（FRS-SysRS-I-002）。
- **状態の単一ストア＋コマンドバス**: 全状態変更を `commandBus` 経由にし、Undo/Redo と AI 操作を同一経路に統一（FRS-SysRS-R-004）。
- **永続化は DB レス**: JSON レジストリ＋COG を原子的書込（FRS-SysRS-R-001）。

### 1.2 技術スタック

| 層 | 主要技術 |
|---|---|
| フロント | React 18 / TypeScript 5 / Vite 5 / Zustand 4 / OpenLayers 10 / three.js / proj4 / Vitest |
| API | FastAPI / uvicorn / rasterio / rio-tiler / rio-cogeo / pydantic 2 / anthropic / openai |
| コア | numpy / scipy / pyproj / rasterio / laspy / PyYAML / (任意) opencv |

### 1.3 標準パイプライン（データフロー）

```
LAS/LAZ ──[costmap]──► cost(float32)+DSM+RGB COG
                          │
                          ├─[drivable]──► 走行可能マスク (+ include/exclude 編集)
                          │
始点/経由/終点+方位 ──[plan]──► trajectory(s,x,y,θ,κ,dκ/ds,grade,steer,gear,v,t)
   (NoGoZone, 機種プロファイル)     │
                          ├─[analysis]──► κ/勾配/速度/操舵 プロファイル + 違反
                          ├─[safety]────► 配信可否(passed) + 不可理由   ← ルール固定
                          └─[export]────► CSV(6677) / GeoJSON(WGS84)

目標姿勢(start,target) ──[spotting]──► 接近/退出軌道(切返0/1) + メトリクス
複数ルート ──[fleet]──► 競合検出 / フリート走行 sim
```

---

## 2. コンポーネント設計 — planning_core

純 Python の本体ロジック。サブパッケージ単位の責務:

### 2.1 `costmap` — 点群→コストマップ（FRS-SysRS-F-001〜003）

- `build_costmap_arrays(x,y,z, CostmapParams)` → `{cost, dsm, transform, nodata_mask}`
- 処理: 点群を格子へビニング → 地物高（`ground_percentile`=5 でロバストに地面推定）→ `slope_grid`（勾配[deg]）・`roughness_grid`（局所標準偏差, 窓 `rough_window_m`）→ 重み付き `cost = slope_norm·w_slope + rough_norm·w_rough`。
- `slope_limit_deg`（既定15）超過は **`obstacle_value`（既定 1e9）** に倒し走行不能を表現（FRS-SysRS-F-003）。
- `cost_to_rgba` で表示用 RGBA（jet + nodata/障害物透過）。
- 既定 `grid_size_m`=0.3。点群密度から推奨格子サイズを算出して返す。
- 実装: `costmap/from_las.py`, `costmap/colorize.py`。

### 2.2 `drivable` — 走行可能領域（FRS-SysRS-F-004, F-005）

- `generate_drivable(cost, GenParams)`: 閾値（既定150）/Otsu/適応で2値化 → 形態学（close_m/open_m）→ 最小面積除去（min_area_m2）→ 穴埋め（max_hole_m2）→ 平滑（smooth_m, median）→ 最大領域抽出（keep_largest）→ クリアランス侵食（clearance_m）。統計（面積/島数/穴数/幅中央・最大）を返す。
- **非破壊編集**: `base + edits[]`（include/exclude ポリゴン）を毎回再適用する関数型パターン（`generate.py`）。編集の追加/削除/全消去でベースから再レンダリング。
- `segment_drivable_base`（OpenCV、Otsu/適応）は遅延 import の任意依存。
- 実装: `drivable/generate.py`, `drivable/segment.py`。

### 2.3 `planners` — 経路生成（FRS-SysRS-F-006〜010）

| 関数 | アルゴリズム | 用途 |
|---|---|---|
| `fit_spline_curvature_limited` | 曲率制限付き B-spline | 既定。端点方位を制御点で誘導 |
| `plan_dubins` | 6 word Dubins (LSL/RSR/LSR/RSL/RLR/LRL) | R_min 保証の経由点接続（前進のみ） |
| `plan_grid_astar` | 8近傍 A*（コスト格子） | R_min 未知時の自動モード。粗経路 |
| `hybrid_astar` | 運動学 A*（Dubins プリミティブ ±1/ρ,0 × 前/後進） | 障害物回避＋運動学拘束。ゴール近傍は解析 Dubins ハンドオフ |
| `sample_reeds_shepp` | 6 word Reeds-Shepp（後進込み） | 後進を許す機動 |
| `rrt_star` | サンプリング型（rewiring） | サンプリング型運動学計画 |
| `smooth_polyline_in_corridor` | taut + string-pull + spline | A* 出力の整形（境界尊重） |
| `smooth_kinematic_path` | Chaikin + 曲率制限 | hybrid A* の後平滑（R_min 保持） |
| `elastic_band` | 収束法（中央寄せ） | 任意の経路洗練（FRS-SysRS-F-010） |
| `limit_curvature_polyline` | 局所3次スプライン patch | 非運動学経路の R_min 保証後処理 |
| `resample_by_spacing` | 弧長再サンプリング | 点密度標準化（spacing_m） |

- **自動モード選択**（FRS-SysRS-F-007）: R_min 既知→hybrid A*、未知→grid A*。利用者が spline/dubins 指定でも auto では A* 系へ昇格。
- **衝突判定**: hybrid A* は `footprint`（車体ポリゴン）または円近似でマスク照合。

### 2.4 `analysis` — 解析・安全検証（FRS-SysRS-F-008, S-001〜004）

| 関数 | 責務 |
|---|---|
| `build_trajectory` | ポリライン+メタから `Trajectory` 構築。カスプ（ギア反転、cosθ<-0.7）検出、dκ/ds に任意 Savitzky-Golay 平滑 |
| `summarize` | `AnalysisResult`（min_radius, max_curvature, max_grade, feasible, violations[]）算出 |
| `verify_safety` | **`SafetyReport`（passed, checks[], reasons[日本語]）**。下記6チェックを統合 |
| `curvature_profile` | κ=1/外接円半径（3点法）、dκ/ds=np.gradient |
| `grade_profile` | DSM 双線形補間から縦断勾配[%]（nodata 考慮） |
| `velocity_profile` | κ・横加速度・降坂制限から v(s) |
| `stopping_distance` | 減速度から制動距離 |

**安全検証6チェック**（`safety.py`、FRS-SysRS-S-001）:

| チェック | 条件 | 非適用条件 |
|---|---|---|
| footprint | 車体ポリゴンが走行可能マスク内に収まる | 車両/マスク不在 |
| min_radius | 実測 min_radius ≥ 機種 R_min | クローラ（据切り可）/車両不在 |
| kappa_rate | max dκ/ds ≤ kappa_rate_max | 寄り付き時は advisory（低速で非拘束） |
| steer_angle | max 操舵角 ≤ max_steer_angle | 剛体ステア以外 |
| grade | \|max grade\| ≤ max_grade_pct | DSM 不在 |
| clearance | 最小離隔 ≥ min_clearance_m | マスク不在 |

→ `passed`=全適用チェック OK。不可時 `reasons[]` に日本語の不可理由。**この判定はサーバー側ルールで固定され、AI から上書き不可**（FRS-SysRS-S-005）。

### 2.5 `simulator` — 寄り付き（FRS-SysRS-F-012〜015）

- `plan_spotting(start, target, constraints, CostWeights)` → `SpottingResult`。
- 候補手法 auto/dubins/reeds_shepp/hybrid A* を**競合させ**、コスト関数（w_distance/time/reverse/switchback/costmap/turn/clearance/footprint）でスコア最小を採用。
- 制約: 切り返し最大数 0/1、必須後進、手動切り返し点、切り返しゾーン、包含ポリゴン、進入禁止、`cusp_margin_m`（切返し前後の操舵0ハンドオフ区間）、据切り（allow_stationary_steer）。
- 出力: points[]、switch_points[]、metrics（到達誤差・前進/後進別延長・切返数・最小離隔・スコア・footprint 内包率）、status（NO_PATH/COLLISION/FOOTPRINT_OUTSIDE/...）、退出軌道（with_exit）。
- 実装: `simulator/spotting.py`。

### 2.6 `fleet` — 複数台（進行中、FRS-SysRS-F-018〜020）

| 関数 | 責務 |
|---|---|
| `detect_conflicts` | ルート回廊（中心線±half_width）をラスタ化し重複検出。ペアごと区間（s 範囲）・面積・bbox |
| `simulate_fleet` | 離散時間ステップ。区間予約（Mutex）＋優先順位調停＋ギャップ確保 |
| `simulate_fleet_auto` | デッドロック/衝突時に待避所（bay）を自動挿入 |
| `simulate_fleet_sequential` | 逐次（相互作用なし） |
| `junction_pose` / `lateral_detour` / `auto_bay_center` | 分岐姿勢・待避迂回の幾何 |

- 実装: `fleet/conflict.py`, `fleet/sim.py`, `fleet/network.py`, `fleet/passing.py`。

### 2.7 `vehicle` / `footprint` — 機種（FRS-SysRS-F-016, F-017, S-003）

- 内蔵 `BUILTIN_IDS=["HD785","HD605","HM400","CD110R"]`、YAML（`vehicle/configs/*.yaml`）→ `VehicleProfile`。
- 運動学種別: **rigid_bicycle**（HD785/HD605, Ackermann, wheel_base/max_steer_angle, κ=tan(δ)/L）/ **articulated**（HM400, front/rear_length, max_articulation_angle）/ **tracked_skid**（CD110R, 据切り可, R_min 非適用）。
- `footprint.py`: `vehicle_footprint`（ポリゴン抽出/矩形算出）, `footprint_sample_points`, `path_min_clearance`, `outside_count`。hybrid A* 衝突・逸脱検出に使用。

### 2.8 `geometry` / `io` / `models`

- `geometry/projection.py`: pyproj ラッパ（`project`, `to_lonlat`, `from_lonlat`）。
- `io/las.py`: laspy で LAS→(x,y,z,RGB,epsg)、間引き読込。
- `models/`: pydantic データクラス（`vehicle.py`, `route.py`, `analysis.py`, `costmap.py`, `geo.py`）→ §4 参照。

---

## 3. コンポーネント設計 — services/api

`planning_core` の薄い FastAPI ラッパ。エントリ `app/main.py`、ルーターを include。

### 3.1 主要モジュール

| ファイル | 責務 |
|---|---|
| `main.py` | FastAPI 生成、CORS、`/health`・`/api/crs`、全ルーター include |
| `__init__.py` / `_proj_fix.py` | macOS の pyproj/rasterio 向け PROJ パッチ |
| `settings.py` | `FRS_DATA_DIR`（既定 `~/.fms-route-studio/data`）、CRS 既定、MAX_RASTER_DIM 等 |
| `store.py` | レイヤレジストリ（`registry.json`）。`threading.RLock` ＋原子的書込 |
| `cogio.py` | COG 入出力ヘルパ |
| `vehicle_overrides.py` | 機種オーバーライド（`vehicle_overrides.json`）の解決・保存 |

### 3.2 ルーター責務

| ルーター | prefix | 担当 SysRS |
|---|---|---|
| `layers` | /api/layers | レイヤ CRUD・アップロード・3D 点群・COG 配信（F-001 前提, P-001/002） |
| `tiles` | /api（直下） | XYZ タイル・プレビュー PNG（rio-tiler） |
| `costmap` | /api/costmap | コストマップ生成・DSM 格子（F-001, F-002） |
| `drivable` | /api/drivable | 走行可能領域生成/再生成/編集（F-004, F-005） |
| `geo` | /api/geo | 座標変換（I-004） |
| `planning` | /api（直下、plan/analyze/elevation） | 経路生成・解析・標高後付け。**業務ロジックは `planning_core.orchestrator`**（plan_route/analyze_polyline）へ委譲し、ルーターはリクエスト解釈＋レイヤIOのみ（F-006〜011, S-001〜004） |
| `vehicles` | /api/vehicles | 機種一覧/詳細/オーバーライド（F-016, F-017） |
| `simulate` | /api/simulate | 寄り付き（F-012〜015） |
| `projects` | /api/projects | プロジェクト CRUD（F-021） |
| `agent` | /api/agent | AI チャット/状態（F-023, I-005） |
| `fleet` | /api/fleet | 競合/分岐/フリート sim（F-018〜020） |

---

## 4. データモデル（models/）

主要 pydantic モデル（抜粋）:

### VehicleProfile（`vehicle.py`）
```
id, name, kinematic_type∈{rigid_bicycle, articulated, tracked_skid}
寸法: overall_length/width/height, footprint_polygon[], footprint_radius, road_width, track_width
運動学: min_turning_radius, kappa_max_fwd/rev, kappa_rate_max, max_steer_angle, wheel_base,
        max_articulation_angle, front_length, rear_length, can_turn_in_place
動力: max_speed_fwd/rev, max_accel/decel, max_lateral_accel, accel_start, decel_emergency,
       max_speed_downhill_empty/loaded, steer_rate_profile[(speed_kmh,max_rad_s)]
安全しきい: max_grade_pct, min_clearance_m
メタ: spec_status∈{measured, estimated}
```

### Trajectory / TrajPoint / Waypoint（`route.py`）
```
TrajPoint: s, x, y, z?(標高[m]・点群由来DSMサンプル), heading_deg(+East/CCW),
           curvature κ, curvature_rate dκ/ds,
           grade_pct?, steer_deg?(剛体ステアのみ), gear∈{F,R}?, speed_mps?, time_s?
Trajectory: points[], length_m, min_radius_m?, curvature_source∈{analytic,numeric}
Waypoint: id, role∈{start,via,goal}, xy, heading_deg?, gear?
```
z と grade は `analysis.grade.elevation_and_grade()` が DSM を1回サンプルして同時に算出
（grade は平滑化後 z の 100·dz/ds と厳密整合）。DSM 無し/範囲外は null。

### AnalysisResult / SafetyReport / Violation（`analysis.py`）
```
Violation: kind∈{min_radius,kappa_rate,steer_rate,road_width,grade,drivable_out,footprint},
           s_start, s_end, measured, limit
SafetyCheck: name, label(日本語), ok, applicable, measured?, limit?, detail
SafetyReport: passed, checks[], reasons[](日本語)
AnalysisResult: min_radius_m?, max_curvature, max_curvature_rate, max_steer_rate_required?,
                max_grade_pct?, feasible, not_applicable[], violations[],
                max_speed_mps?, time_total_s?, stopping_distance_m?
```

### CostmapParams（`costmap.py`）
```
grid_size_m=0.3, w_slope=500, w_rough=100, rough_window_m=1.0, canopy_ref_m=1.0,
slope_limit_deg=15, obstacle_value=1e9, display_vmax?, ground_percentile=5, min_points_per_cell=1
```

---

## 5. API 仕様（エンドポイント一覧）

> base `/api`（dev は :5173 → :8077 プロキシ）。詳細形は §4 モデル参照。

### メタ / 座標
- `GET /health` → {status, default_epsg, zones}
- `PUT /api/crs` → 作業 EPSG 設定（JGD2011 平面ゾーンのみ）
- `POST /api/geo/transform` {points, src_epsg, dst_epsg} → {points}

### レイヤ / タイル
- `GET /api/layers` / `GET|DELETE /api/layers/{id}`
- `POST /api/layers/{kind}`（multipart, kind=ortho|cost|las）→ Layer（CRS 検出/付与・再投影・間引き）
- `GET /api/layers/{id}/points?max=` → {n,x[],y[],z[],has_rgb,rgb[][],zmin,zmax}（≤~60万点）
- `GET /api/layers/{id}/cog.tif`（Range 配信）/ `GET /api/layers/{id}/preview.png`
- `GET /api/tiles/{id}/{z}/{x}/{y}.png`

### コストマップ / 走行可能領域
- `POST /api/costmap` {las_layer_id, params, src_epsg, target_epsg} → cost+dsm+RGB COG
- `GET /api/costmap/{id}/dsm_grid?max=` → {nx,ny,x0,y0,dx,dy,z[][],zmin,zmax}
- `POST /api/drivable` / `POST /api/drivable/{id}/regenerate`
- `PATCH /api/drivable/{id}` {op:include|exclude, polygon} / `DELETE .../edits/{i}` / `DELETE .../edits` / `GET .../analysis`

### 経路 / 解析
- `POST /api/plan`（PlanRequest）→ {trajectory, analysis, safety, measured_min_radius_m, min_clearance_m, warning, refined_elastic_band}
  - 主入力: waypoints[], mode, algorithm, vehicle_id, spacing_m, corridor_width_m, enforce_footprint, enforce_min_radius, allow_reverse, refine_elastic_band, min_turn_radius_m, planner_cell_m, limit_steer_rate, costmap_layer_id, drivable_layer_id, no_go_polygons[][]
- `POST /api/analyze` {points, vehicle_id, costmap_layer_id} → {trajectory, analysis}（安全レポートなし）
- `POST /api/elevation/sample` {points[], costmap_layer_id?, smooth_m?} → {z[], layer_id, n, n_missing}
  - 保存済みルート等の任意点列に点群由来 DSM の標高 z を後付け（layer 省略時は DSM 持ち最新 cost レイヤ）

### 寄り付き
- `POST /api/simulate/spotting`（SpottingRequest）→ {points[], switch_points[], metrics, trajectory, analysis, safety, feasible, status, reason, rho_m, method, exit?}

### 機種
- `GET /api/vehicles` / `GET /api/vehicles/{id}` / `GET /api/vehicles/{id}/detail`
- `PUT /api/vehicles/{id}` {fields} / `DELETE /api/vehicles/{id}/override`

### プロジェクト
- `GET|POST /api/projects` / `GET|PUT|DELETE /api/projects/{id}`（state は FE 不透明 JSON）

### フリート
- `POST /api/fleet/conflicts` {routes[], cell_m, clearance_m} → {conflicts[], n_routes, n_conflicts}
- `POST /api/fleet/junction` {points, s_frac} → {x, y, heading_deg}
- `POST /api/fleet/simulate`（SimRequest）→ {status, deadlock, collision, makespan_s, traces[], events[], auto_bays[], ...}

### AI
- `GET /api/agent/status` → {available, provider, model, detail}
- `POST /api/agent/chat` {messages[], context} → {reply, actions[], steps[], provider}

---

## 6. コンポーネント設計 — apps/web

### 6.1 構成
```
src/
  App.tsx              シェル(サイドバー/トップバー/機能パネル/中央地図3D/解析パネル)
  store/useStore.ts    Zustand 単一ストア（全状態）
  commandBus.ts        全状態変更の統一ディスパッチ（Undo/Redo + AI 連携）
  api/client.ts        fetch ベース API クライアント
  types/api.ts         API 型
  map/proj.ts          JGD2011 ゾーン登録 + WORKING_EPSG ランタイム束縛
  map/MapView.tsx      2D（OpenLayers, COG=GeoTIFF source）
  map/ThreeView.tsx    3D（three.js, 点群/DSM メッシュ）
  panels/*.tsx         機能パネル（下表）
  components/ProfileChart.tsx  解析チャート
  exporters.ts / io.ts / projectState.ts / layerSelect.ts / agentActions.ts
```

### 6.2 状態管理（Zustand 単一ストア）

- **コアデータ**: layers[], waypoints[], route, areas[], activePolygon, importedRoutes[], savedRoutes[]
- **計画パラメータ**: vehicleId, planMode(auto|waypoint_guided), algorithm(6種), routeSpacing, roadWidthM, enforceFootprint, enforceMinRadius, allowReverse, refineElasticBand, costLayerId, drivableLayerId
- **寄り付き**: spotStart/Target/SwitchPose, spotMethod, spotMaxSwitch(0|1), spotWeights, spotResult, spotIndex, with_exit 系
- **フリート**: fleetConflicts, fleetSim, fleetSimT, fleetBays
- **表示**: costVisible/Opacity, drivableVisible/Opacity, view3d, view3dMode(points|mesh), pointSize, vExag
- **UI**: activeFeature, mode（編集モード）, status/statusLevel, busy, hoverPointIndex, selectedId
- **履歴**: undoStack/redoStack（最大50, `commit()`/`undo()`/`redo()`）

### 6.3 コマンドバス（FRS-SysRS-R-004）

全状態変更を `commandBus` のコマンド（ADD_WAYPOINT/SET_ROUTE/FINISH_POLYGON/SELECT/UNDO/REDO 等）に統一。編集系コマンドはディスパッチ前にスナップショットを commit。**AI の操作（agentActions）も同じコマンド経由**なので等しく Undo 可能。mode/選択/履歴操作はスナップショット対象外。

### 6.4 パネル一覧

| パネル | 機能 | 対応 SysRS |
|---|---|---|
| LayerPanel | LAS/GeoTIFF アップロード・削除・ゾーン警告 | I-003, I-002 |
| CostmapPanel | コストマップ生成・表示制御 | F-001, F-002 |
| DrivableAreaPanel | 領域生成・include/exclude 編集・統計 | F-004, F-005 |
| RoutePanel | 経由点編集・アルゴリズム選択・生成・ルート保存・分岐 | F-006〜010 |
| SpottingPanel | 寄り付き姿勢/重み設定・生成・再生・退出 | F-012〜015 |
| VehiclePanel | 機種選択・諸元オーバーライド・リセット | F-016, F-017 |
| AreaPanel | ポリゴン作図/編集/削除 | F-009（NoGoZone 等） |
| FleetPanel | 競合検出・待避所・フリート sim・再生 | F-018〜020 |
| AnalysisPanel | κ/勾配/速度/操舵チャート・違反（地図と相互ハイライト） | F-008, S-004 |
| ChatPanel | AI チャット（文脈つき tool-use） | F-023 |
| ProjectPanel | プロジェクト保存/読込/削除 | F-021 |
| IOPanel | CSV/GeoJSON エクスポート・JSON 入出力 | F-022 |

### 6.5 座標・エクスポート

- `map/proj.ts`: 起動時に backend の `default_epsg` で `WORKING_EPSG` を束縛。JGD2011 全ゾーン（6669–6687）を proj4 登録。実行時切替は `setWorkingEpsg`（再読込）。
- `exporters.ts`: ルート/寄り付きの CSV（EPSG:6677 メートル, 速度プロファイル列含む）/ GeoJSON（`api.transform` で 6677→WGS84）。全ルート一括 FeatureCollection / 個別出力に対応。
- `projectState.ts`: v2 形式（route 幾何を含む）でシリアライズ。v1（route なし）は route=null で後方互換ロード。

---

## 7. 座標系・ラスタ処理（FRS-SysRS-I-002, P-001/002）

- **作業 CRS**: 既定 EPSG:6677（JGD2011 IX 系）。`FRS_DEFAULT_EPSG` で起動既定、`PUT /api/crs` で実行時変更。JGD2011 平面ゾーン 6669–6687 をサポート。
- **アップロード**: ラスタ `.crs` を検出、無ければ `assign_epsg` 付与。作業 CRS と異なれば WarpedVRT（bilinear）で再投影。
- **間引き**: 幅/高さ > MAX_RASTER_DIM(8192) で整数倍ダウンサンプル（mask=nearest, cost/dsm=average）。
- **COG**: 常にタイル化（deflate, 512×512）出力。`crs_source`（detected/assigned/computed）・`downsampled_from`・`reprojected_from` をメタに記録。
- **ラスタ規約**: north-up affine（transform.e<0）。world↔pixel は rasterio Affine。セル中心 = i+0.5。

---

## 8. 永続化・データ管理（FRS-SysRS-R-001〜003）

- **ルート**: `FRS_DATA_DIR`（既定 `~/.fms-route-studio/data`）。
- **レイヤ**: `layers/{layer_id}/`（cog.tif 表示用・source.* 原本・cost.tif・dsm.tif・mask.tif）＋ `registry.json`（メタ索引）。
- **プロジェクト**: `projects.json`（{id,name,state,created_at,updated_at}、state は FE 不透明）。
- **車両オーバーライド**: `vehicle_overrides.json`（{vehicle_id:{field:value}}、id/name/種別は不変）。
- **原子的書込**: 全 JSON は `.tmp` 書込→`os.replace()`。`store.py`/`projects.py`/`vehicle_overrides.py` で `RLock`。
- **版番号**: レイヤ編集/再生成で version++ → クライアントのタイルキャッシュ無効化（FRS-SysRS-R-003）。
- DB レス（FRS-StRS-D-002）。

---

## 9. AI アシスタント設計（FRS-SysRS-F-023, I-005, S-005）

### 9.1 プロバイダ抽象化
- **検出順**: 明示 `FRS_AI_PROVIDER` → 自動（`ANTHROPIC_API_KEY` → `GROQ_API_KEY` → `OPENROUTER_API_KEY` → `FRS_AI_BASE_URL`(Ollama 等) → `OPENAI_API_KEY`）→ 無ければ 503。
- **モデル**: `FRS_AI_MODEL`（既定 Anthropic=claude-opus-4-8）。
- Anthropic は thinking=adaptive + effort=medium。OpenAI 互換は `openai` SDK 1実装で Groq/OpenRouter/Ollama/OpenAI を網羅、tool-use を OpenAI tools 形式へ変換。Groq/Llama 等の `<function=...>` テキスト形式もパース。

### 9.2 tool-use（9ツール）
`get_app_state` / `list_vehicles` / `set_vehicle` / `set_plan_options` / `set_waypoints` / `plan_route` / `generate_costmap` / `generate_drivable` / `set_spotting` / `simulate_spotting`。
- **安全上限**: `MAX_TOOL_ITERS=8`（反復）、`MAX_GENERATE_CALLS=3`（ディスク書込生成）（FRS-SysRS-P-003）。
- **出力**: {reply, actions[](SET_VEHICLE/SET_PLAN_OPTIONS/SET_WAYPOINTS/SET_ROUTE/SET_SPOTTING/SET_SPOT_RESULT/REFRESH_LAYERS), steps[]}。FE はアクションを commandBus 経由でストアに適用 → Undo 可能。
- **安全判定は非介入**: AI は安全検証を「実行」できるが、配信可否はサーバー側 `verify_safety` のルールで固定（FRS-SysRS-S-005）。システムプロンプトで座標捏造禁止・ツール使用・違反説明を指示。

---

## 10. 非機能設計

| 区分 | 設計 |
|---|---|
| 性能 | 大容量ラスタ間引き（P-001）、COG タイル配信（P-002）、3D 点数上限（P-004）、AI 反復/生成上限（P-003） |
| 信頼性 | 原子的書込・RLock・版番号無効化・Undo/Redo（R-001〜004） |
| 保守性 | 本体ロジックを planning_core に集約し API/CLI/テストで再利用。機種は YAML 追加で拡張（D-001） |
| 移植性 | DB レス・ローカル単独起動（D-002）。macOS の PROJ パッチ（`_proj_fix.py`） |
| テスト | planning_core 単体 / api 結合（PROJ db 競合回避のため分離実行）/ web 単体（commandBus, projectState） |

---

## 11. 制約・既知の課題・今後

- **PROJ db 競合**: planning_core と api のテストは合同実行で PROJ db が競合するため分離実行（api の conftest は一時 `FRS_DATA_DIR` を設定）。
- **フリート機能（◐）**: 競合検出＋単純 sim まで。本番ディスパッチ最適化は未実装（[REQUIREMENTS.md](REQUIREMENTS.md) FRS-O-01）。
- **制限車速のエリア別 UI（未要求化）**: 速度プロファイル生成は実装済だがエリア別制限車速設定は未整備（FRS-O-02）。
- **性能 SLA 未定量**（FRS-O-04）。
- **大容量データは Git LFS 未導入**（リポジトリ直下 `.gitignore` で除外運用）。
- **マルチユーザー/サーバー集中運用は対象外**（現状ローカル単独）。

---

## 付録: 起動・テスト

詳細は [../README.md](../README.md) を参照。要点:
- セットアップ: `python -m venv .venv` → `pip install -e packages/planning_core[dev]` → `pip install -e services/api[dev]` → `apps/web && npm install`
- 起動: `uvicorn app.main:app --app-dir services/api --port 8077` ＋ `npm run dev`（:5173）
- テスト: `pytest packages/planning_core/tests` / `pytest services/api/tests`（分離実行）/ `npm run build && npm run test`

