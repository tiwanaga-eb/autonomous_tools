# Mining Dock Path Planner (F->R->F)

鉱山向け自動運転車両の切り返し経路プランナです。`start -> switch (F) -> dock (R) -> exit (F, optional)` を生成します。

## Project Structure

- `planner/models.py`: API相当データモデル
- `planner/geometry.py`: 幾何ユーティリティ（point-in-polygon、距離、角度正規化）
- `planner/vehicle_config.py`: `vehicle_configs/{vehicle_id}.yaml` ローダ
- `planner/dubins.py`: 曲率制限付き Dubins ソルバ（前進／後進変換）
- `planner/sampler.py`: 切り返し点サンプラ
- `planner/collision.py`: 外接円近似の衝突判定
- `planner/cost.py`: 重み付き評価関数
- `planner/planner.py`: 統括プランナ `plan_mining_dock`
- `planner/astar.py`: A* ユーティリティ
- `planner/primitives.py`: モーションプリミティブ生成・ロールアウト
- `planner/dynamic_planner.py`: Phase3 Motion Primitives Planner
- `planner/dubins_pp.py`: Dubins++ Planner（多曲率・バッファ・スイッチYawオフセット探索）
- `server/app.py`: FastAPI サーバ
- `server/static/index.html`: UI
- `server/static/main.js`: Canvas描画・操作・API呼び出し
- `server/static/styles.css`: スタイル
- `vehicle_configs/demo_truck.yaml`: テスト向け車両設定
- `vehicle_configs/HD785-7.yaml`: Komatsu HD785-7/8 想定設定
- `tests/test_planner.py`: pytest テスト
- `visualize_case.py`: matplotlib 可視化

## Setup

```bash
pip install -r requirements.txt
```

## Run Tests

```bash
pytest -q
```

## Offline Visualization

```bash
python visualize_case.py
```

出力:
- `out/case1.png`
- `out/case2.png`

## Web App Run

```bash
uvicorn server.app:app --reload
```

ブラウザで `http://127.0.0.1:8000/` を開きます。

## Web API

### `GET /api/vehicles`

利用可能な `vehicle_configs/*.yaml` を返します。

### `POST /api/plan`

- 入力: 車両ID、start/dock/exit、走行可能ポリゴン、障害物、プランナパラメータ
- `planner_params.algorithm`: `dubins` / `dubins_pp` / `primitives`
- `planner_params.primitives_params`: primitives探索パラメータ
- `planner_params.dubins_pp_params`: dubins_pp探索パラメータ
  - `max_reverse_length_ratio` (default: 1.5)
  - `switch_yaw_limit_deg` (default: 30)
  - `kappa_switch_limit_ratio` (default: 0.2)
- `planner_params.dubins_pp_behavioral_params`: dubins_pp行動制約
  - `reverse_arc_ratio_limit` (default: 0.3)
  - `switch_sector_d_min` / `switch_sector_d_max` / `switch_sector_angle_deg`
  - `delta_neutral_limit_deg`
  - `forward_length_ratio`
  - `entry_angle_limit_deg`
  - `switch_rank_min_obstacle_clearance` / `switch_rank_min_edge_margin` (switch候補の前処理ランク)
- 出力: `status`, `segments`, `switch_pose`, `metrics`, `debug`
- `segments[].states[].is_ramp` でスムージング区間を判別可能

### Example curl

```bash
curl -X POST http://127.0.0.1:8000/api/plan \
  -H "Content-Type: application/json" \
  -d '{
    "vehicle_id": "HD785-7",
    "start_pose": {"x": 10.0, "y": 8.0, "yaw": 0.0},
    "dock_pose": {"x": 58.0, "y": 30.0, "yaw": 3.1416},
    "exit_pose": null,
    "drivable_polygon": [[0,0],[80,0],[80,40],[0,40]],
    "obstacles": [[[32,14],[44,14],[44,26],[32,26]]],
    "planner_params": {
      "timeout_ms": 10000,
      "tolerances": {"pos": 0.1, "yaw": 0.0873},
      "weights": {"w_len": 1.0, "w_time": 1.0, "w_rev": 2.0, "w_goal": 5.0},
      "sampling_params": {"switch_sample_count": 1200, "yaw_bins": 32},
      "algorithm": "dubins",
      "primitives_params": {"xy_res": 0.5, "delta_bins": 21, "T": 0.5, "dt": 0.05},
      "seed": 42
    }
  }'
```

## Screenshot Explanation

`server/static/index.html` の UI では次を行えます。
- 左パネル: 車両選択、許容誤差、重み、サンプリング数の調整
- `Enable smoothing` で Phase2 の曲率連続化（クラソイド近似ランプ）をON/OFF
- `Algorithm` で `dubins / dubins_pp / primitives` を切替
- primitives選択時は `XY Res / Delta Bins / Primitive T / dt / A* Weight` を調整
- `Straight Margin [m]`: Dock直前に後退で直進させる最低距離（`sampling_params.straight_margin`）
- `Yaw` 入力は Degree 表示（送信時に内部で rad 変換）
- 背景切替: `Grid / OSM / Esri航空写真`、原点緯度経度でローカルXYを近似投影
- ズーム下限を緩和し、広域までZoom out可能
- キャンバス: Drivable/Obstacle 描画、Start/Dock/Exit 設定
- 経路表示: 前進は実線、後進は破線、切り返し点はオレンジ
- 右上: `total_length`, `total_time`, `compute_time_ms`, `candidate_generated`
- 経路の道幅（road_width）は半透明帯で可視化
- スムージングランプ区間は点線オーバーレイで可視化

## Vehicle Config

`vehicle_configs/{vehicle_id}.yaml` 必須キー:
- `wheel_base`
- `max_steer_angle`
- `max_speed_fwd`
- `max_speed_rev`
- `gear_switch_time_penalty`
- `max_steer_rate`
- `steer_ramp_length`
- `road_width`
- `kappa_rate_max`
- `smoothing_step`
- `footprint_polygon` または `footprint_radius`

`road_width` は経路中心線に対する道路幅チェックに使われ、左右端 (`road_width / 2`) が
drivable外へはみ出す、または障害物へ接近しすぎる候補は棄却されます。

`kappa_rate_max` と `smoothing_step` は Phase2 の曲率連続化に使用します。  
各曲率ジャンプに対して、`|dκ/ds| <= kappa_rate_max` を満たす線形ランプを挿入し、
挿入後の幾何に対して再度衝突判定を実施します。

Phase3 (`algorithm=primitives`) は状態 `(x, y, yaw, delta, gear, switch_used)` を離散化し、
Weighted A* + モーションプリミティブで探索します。  
Stage A で `start(F) -> dock(R)`（F→R 1回）、Stage B で `dock(F) -> exit(F)` を計画します。

任意キー:
- `min_turning_radius`
- `overall_length`, `overall_width`, `overall_height`
- `reverse_penalty_factor`

## Known Limitations

- 衝突判定は外接円近似（厳密な車体形状オフセット未実装）
- Drivable は単一ポリゴン前提（穴なし）
- 角度はUIで数値入力（マウス方向指定は未実装）
- 背景タイル利用時はネットワーク未接続だと表示不可
