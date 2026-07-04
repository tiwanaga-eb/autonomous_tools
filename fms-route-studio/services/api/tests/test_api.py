import os
from pathlib import Path

import laspy
import numpy as np
import pytest
from fastapi.testclient import TestClient

from app.main import app

client = TestClient(app)

# Phase 1 動作確認用の実ラスタ（あればアップロード→タイルまで検証）
COSTMAP = Path(
    "/Users/tosuke_iwanaga/Documents/GitHub/autonomous_tools/Map_Builder_Protoのコピー/costmap.tif"
)
PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def test_health():
    r = client.get("/health")
    assert r.status_code == 200
    assert r.json()["default_epsg"] == 6677


def test_geo_transform_roundtrip():
    pts = [[30465.1, 119228.8]]
    r1 = client.post("/api/geo/transform", json={"points": pts, "src_epsg": 6677, "dst_epsg": 4326})
    assert r1.status_code == 200
    ll = r1.json()["points"]
    assert 130.0 < ll[0][0] < 145.0 and 30.0 < ll[0][1] < 42.0
    r2 = client.post("/api/geo/transform", json={"points": ll, "src_epsg": 4326, "dst_epsg": 6677})
    back = r2.json()["points"][0]
    assert abs(back[0] - pts[0][0]) < 1e-2 and abs(back[1] - pts[0][1]) < 1e-2


def test_plan_spline_with_curvature():
    wp = [{"x": 0, "y": 0}, {"x": 20, "y": 10}, {"x": 40, "y": 0}, {"x": 60, "y": 15}]
    r = client.post("/api/plan", json={"waypoints": wp, "spacing_m": 2.0, "vehicle_id": "HD785"})
    assert r.status_code == 200, r.text
    j = r.json()
    pts = j["trajectory"]["points"]
    assert len(pts) >= 2
    assert "curvature" in pts[0] and "curvature_rate" in pts[0]
    # HD785 は rigid_bicycle -> steer_deg が入る
    assert any(p["steer_deg"] is not None for p in pts)
    assert j["analysis"]["max_curvature"] >= 0.0


def test_plan_min_radius_reported():
    wp = [{"x": 0, "y": 0}, {"x": 5, "y": 8}, {"x": 10, "y": 0}]
    r = client.post("/api/plan", json={"waypoints": wp, "min_turn_radius_m": 6.0})
    assert r.status_code == 200
    assert r.json()["measured_min_radius_m"] is not None


def test_vehicles_list_and_get():
    lst = client.get("/api/vehicles")
    assert lst.status_code == 200
    ids = [v["id"] for v in lst.json()]
    assert set(ids) == {"HD785", "HD605", "HM400", "CD110R"}
    one = client.get("/api/vehicles/HM400")
    assert one.status_code == 200 and one.json()["kinematic_type"] == "articulated"
    assert client.get("/api/vehicles/NOPE").status_code == 404


def test_plan_dubins_respects_radius():
    wp = [
        {"x": 0, "y": 0, "heading_deg": 0},
        {"x": 30, "y": 20, "heading_deg": 90},
    ]
    r = client.post("/api/plan", json={"waypoints": wp, "algorithm": "dubins", "min_turn_radius_m": 10.0, "spacing_m": 1.0})
    assert r.status_code == 200, r.text
    j = r.json()
    assert j["trajectory"]["curvature_source"] == "analytic"
    # R_min を大きく割り込まない
    assert j["measured_min_radius_m"] is None or j["measured_min_radius_m"] > 9.0


def test_plan_dubins_needs_radius():
    wp = [{"x": 0, "y": 0}, {"x": 10, "y": 0}]
    r = client.post("/api/plan", json={"waypoints": wp, "algorithm": "dubins"})
    assert r.status_code == 400


def test_plan_dubins_with_vehicle_is_steer_feasible():
    # 車両指定時は Dubins の曲率ステップ（瞬間操舵）をクロソイド近似で連続化 → numeric & 操舵レート違反なし
    wp = [{"x": 0, "y": 0}, {"x": 50, "y": 0}, {"x": 50, "y": 50}]
    r = client.post("/api/plan", json={"waypoints": wp, "algorithm": "dubins", "vehicle_id": "HD785", "spacing_m": 1.0})
    assert r.status_code == 200, r.text
    j = r.json()
    assert j["trajectory"]["curvature_source"] == "numeric"  # 平滑化済み
    kinds = [v["kind"] for v in j["analysis"]["violations"]]
    assert "kappa_rate" not in kinds  # 操舵レート(dκ/ds)は車両上限以下に整形済み


def test_plan_min_radius_violation_flags_infeasible():
    # R=5m の鋭いコーナーを HD785(min R=10.1m) で → infeasible & violation
    wp = [{"x": 0, "y": 0}, {"x": 5, "y": 6}, {"x": 10, "y": 0}]
    r = client.post("/api/plan", json={"waypoints": wp, "vehicle_id": "HD785", "min_turn_radius_m": 0.1, "spacing_m": 0.5})
    assert r.status_code == 200, r.text
    a = r.json()["analysis"]
    assert a["feasible"] is False
    assert any(v["kind"] == "min_radius" for v in a["violations"])


def test_plan_auto_grid_astar_and_grade(tmp_path):
    p = tmp_path / "c.las"
    _make_las(p)
    with open(p, "rb") as f:
        las_id = client.post("/api/layers/las", files={"file": ("c.las", f, "application/octet-stream")}).json()["id"]
    cost_id = client.post(
        "/api/costmap",
        json={"las_layer_id": las_id, "src_epsg": 6677, "target_epsg": 6677, "params": {"grid_size_m": 1.0}},
    ).json()["id"]
    dv_id = client.post(
        "/api/drivable",
        json={"cost_layer_id": cost_id, "params": {"threshold": 1e9, "close_m": 0, "open_m": 0, "min_area_m2": 0, "clearance_m": 0}},
    ).json()["id"]

    # auto モード: start/goal をエリア内に。grid_astar + drivable ハード制約 + DSM 勾配
    wp = [{"x": 30005, "y": 119005}, {"x": 30045, "y": 119035}]
    r = client.post(
        "/api/plan",
        json={
            "waypoints": wp,
            "mode": "auto",
            "algorithm": "grid_astar",
            "vehicle_id": "HD785",
            "costmap_layer_id": cost_id,
            "drivable_layer_id": dv_id,
            "spacing_m": 2.0,
        },
    )
    assert r.status_code == 200, r.text
    j = r.json()
    pts = j["trajectory"]["points"]
    assert len(pts) >= 2
    # ランプ DSM (z=0.1x) なので縦断勾配が入る
    assert any(p["grade_pct"] is not None for p in pts)
    assert j["analysis"]["max_grade_pct"] is not None
    # 標高 z も自動埋め込みされる（点群由来 DSM のサンプル値 ≒ 0.1*(x-30000)）
    zs = [p for p in pts if p.get("z") is not None]
    assert zs, "trajectory points should carry z when DSM is present"
    for p in zs:
        assert abs(p["z"] - 0.1 * (p["x"] - 30000.0)) < 1.0

    # 後付けサンプリング API（保存済みルート等への z 付与）
    er = client.post(
        "/api/elevation/sample",
        json={"points": [{"x": 30010, "y": 119010}, {"x": 30040, "y": 119030}, {"x": -9999, "y": -9999}],
              "costmap_layer_id": cost_id},
    )
    assert er.status_code == 200, er.text
    ej = er.json()
    assert ej["n"] == 3 and ej["n_missing"] == 1
    assert ej["z"][2] is None
    assert abs(ej["z"][0] - 1.0) < 1.0 and abs(ej["z"][1] - 4.0) < 1.0
    # costmap_layer_id 省略時は DSM を持つ最新 cost レイヤへフォールバック
    er2 = client.post("/api/elevation/sample", json={"points": [{"x": 30010, "y": 119010}]})
    assert er2.status_code == 200 and er2.json()["layer_id"] == cost_id

    client.delete(f"/api/layers/{dv_id}")
    client.delete(f"/api/layers/{cost_id}")
    client.delete(f"/api/layers/{las_id}")


def test_plan_hybrid_astar(tmp_path):
    p = tmp_path / "c.las"
    _make_las(p)
    with open(p, "rb") as f:
        las_id = client.post("/api/layers/las", files={"file": ("c.las", f, "application/octet-stream")}).json()["id"]
    cost_id = client.post(
        "/api/costmap",
        json={"las_layer_id": las_id, "src_epsg": 6677, "target_epsg": 6677, "params": {"grid_size_m": 1.0}},
    ).json()["id"]
    dv_id = client.post(
        "/api/drivable",
        json={"cost_layer_id": cost_id, "params": {"threshold": 1e9, "close_m": 0, "open_m": 0, "min_area_m2": 0, "clearance_m": 0}},
    ).json()["id"]

    r = client.post(
        "/api/plan",
        json={
            "waypoints": [{"x": 30005, "y": 119005, "heading_deg": 45}, {"x": 30045, "y": 119035, "heading_deg": 45}],
            "mode": "auto", "algorithm": "hybrid_astar", "vehicle_id": "HD785",
            "costmap_layer_id": cost_id, "drivable_layer_id": dv_id, "spacing_m": 2.0,
            # この合成領域(50×40m)は端まで使うため、HD785(10×5.5m)の実フットプリントは端で領域外へ出る。
            # 本テストの主眼は運動学(R_min)なので footprint 厳密化はオフ（包含は test_plan_hybrid_footprint で検証）。
            "enforce_footprint": False,
        },
    )
    assert r.status_code == 200, r.text
    j = r.json()
    pts = j["trajectory"]["points"]
    assert len(pts) >= 2
    # R_min(HD785=10.1) を大きく割り込まない（運動学的に生成）
    mr = j["measured_min_radius_m"]
    assert mr is None or mr > 8.0

    client.delete(f"/api/layers/{dv_id}")
    client.delete(f"/api/layers/{cost_id}")
    client.delete(f"/api/layers/{las_id}")


def test_plan_hybrid_footprint(tmp_path):
    """フットプリント包含: 端まで使う経路は HD785 の車体が領域外へ出るため、
    enforce_footprint=True なら 422、False（中心点判定）なら 200。"""
    p = tmp_path / "c.las"
    _make_las(p)
    with open(p, "rb") as f:
        las_id = client.post("/api/layers/las", files={"file": ("c.las", f, "application/octet-stream")}).json()["id"]
    cost_id = client.post(
        "/api/costmap",
        json={"las_layer_id": las_id, "src_epsg": 6677, "target_epsg": 6677, "params": {"grid_size_m": 1.0}},
    ).json()["id"]
    dv_id = client.post(
        "/api/drivable",
        json={"cost_layer_id": cost_id, "params": {"threshold": 1e9, "close_m": 0, "open_m": 0, "min_area_m2": 0, "clearance_m": 0}},
    ).json()["id"]

    body = {
        "waypoints": [{"x": 30002, "y": 119002, "heading_deg": 45}, {"x": 30048, "y": 119038, "heading_deg": 45}],
        "mode": "auto", "algorithm": "hybrid_astar", "vehicle_id": "HD785",
        "costmap_layer_id": cost_id, "drivable_layer_id": dv_id, "spacing_m": 2.0,
    }
    # 既定(enforce True): 端で車体が領域外 → 422
    r_on = client.post("/api/plan", json={**body, "enforce_footprint": True})
    assert r_on.status_code == 422, r_on.text
    # 中心点判定(enforce False): 経路あり
    r_off = client.post("/api/plan", json={**body, "enforce_footprint": False})
    assert r_off.status_code == 200, r_off.text

    client.delete(f"/api/layers/{dv_id}")
    client.delete(f"/api/layers/{cost_id}")
    client.delete(f"/api/layers/{las_id}")


def test_plan_enforce_min_radius_hm400():
    """R_min を持つ車種(HM400)で、タイトな経由点でも R_min を保証して feasible になる。
    enforce_min_radius=False ならコーナーが R_min を割り、min_radius violation が出る。"""
    wps = [(0, 0), (20, 0), (30, 8), (20, 16), (0, 16), (0, 30), (25, 30)]
    body = {
        "waypoints": [{"x": x, "y": y} for x, y in wps],
        "mode": "waypoint_guided", "algorithm": "spline", "vehicle_id": "HM400", "spacing_m": 2.0,
    }
    on = client.post("/api/plan", json={**body, "enforce_min_radius": True}).json()
    assert on["measured_min_radius_m"] is None or on["measured_min_radius_m"] >= 8.885 * 0.97
    assert not [v for v in on["analysis"]["violations"] if v["kind"] == "min_radius"]

    off = client.post("/api/plan", json={**body, "enforce_min_radius": False}).json()
    assert [v for v in off["analysis"]["violations"] if v["kind"] == "min_radius"]


def test_plan_no_go_zone_blocks(tmp_path):
    """進入禁止領域が start→goal の直線コリドーを塞ぐと auto/grid で経路が見つからない(422)。"""
    p = tmp_path / "c.las"
    _make_las(p)
    with open(p, "rb") as f:
        las_id = client.post("/api/layers/las", files={"file": ("c.las", f, "application/octet-stream")}).json()["id"]
    cost_id = client.post(
        "/api/costmap",
        json={"las_layer_id": las_id, "src_epsg": 6677, "target_epsg": 6677, "params": {"grid_size_m": 1.0}},
    ).json()["id"]
    dv_id = client.post(
        "/api/drivable",
        json={"cost_layer_id": cost_id, "params": {"threshold": 1e9, "close_m": 0, "open_m": 0, "min_area_m2": 0, "clearance_m": 0}},
    ).json()["id"]
    base = {
        "waypoints": [{"x": 30010, "y": 119020}, {"x": 30040, "y": 119020}],
        "mode": "auto", "algorithm": "grid_astar",
        "costmap_layer_id": cost_id, "drivable_layer_id": dv_id, "spacing_m": 2.0,
    }
    # 障害なし → 経路あり
    assert client.post("/api/plan", json=base).status_code == 200
    # y=119020 の通路を横断する帯を NoGo に → 経路なし(422)
    wall = [[30024, 119000], [30026, 119000], [30026, 119040], [30024, 119040]]
    blocked = client.post("/api/plan", json={**base, "no_go_polygons": [wall]})
    assert blocked.status_code == 422, blocked.text

    client.delete(f"/api/layers/{dv_id}")
    client.delete(f"/api/layers/{cost_id}")
    client.delete(f"/api/layers/{las_id}")


def test_spotting_includes_analysis_and_safety():
    """寄り付き応答に経路と同じ軌跡解析＋安全検証が付く。"""
    body = {
        "start": {"x": 0, "y": 0, "heading_deg": 0},
        "target": {"x": 25, "y": 8, "heading_deg": 0},
        "max_switchbacks": 0, "vehicle_id": "HM400",
    }
    r = client.post("/api/simulate/spotting", json=body)
    assert r.status_code == 200, r.text
    j = r.json()
    assert "analysis" in j and "trajectory" in j and "safety" in j
    a = j["analysis"]
    assert "max_curvature" in a and "feasible" in a
    assert j["trajectory"]["points"]  # 軌跡点がある
    assert "passed" in j["safety"] and isinstance(j["safety"]["checks"], list)


def test_spotting_manual_switch_pose_api():
    """手動切り返し点モード: manual_switch_pose 指定で前進→S→後進の1切り返しが返る。"""
    body = {
        "start": {"x": 0, "y": 0, "heading_deg": 0},
        "target": {"x": 10, "y": 0, "heading_deg": 180},
        "max_switchbacks": 1, "vehicle_id": "HD785",
        "manual_switch_pose": {"x": 22, "y": 6, "heading_deg": 200},
    }
    r = client.post("/api/simulate/spotting", json=body)
    assert r.status_code == 200, r.text
    j = r.json()
    assert j["metrics"]["n_switchbacks"] == 1
    assert any(p["gear"] == "R" for p in j["points"])


def test_spotting_exit_goal_api():
    """退出Goal指定: with_exit + exit_goal で退出軌道が exit_goal 近傍へ到達する。"""
    body = {
        "start": {"x": 0, "y": 0, "heading_deg": 0},
        "target": {"x": 20, "y": 0, "heading_deg": 0},
        "max_switchbacks": 0, "vehicle_id": "HD785",
        "with_exit": True,
        "exit_goal": {"x": 40, "y": 10, "heading_deg": 0},
    }
    r = client.post("/api/simulate/spotting", json=body)
    assert r.status_code == 200, r.text
    j = r.json()
    assert "exit" in j and j["exit"]["points"]
    last = j["exit"]["points"][-1]
    assert abs(last["x"] - 40) < 2.0 and abs(last["y"] - 10) < 2.0


def test_spotting_switchback_zone_api():
    """切り返し可能エリア: zone を渡すと切り返し点(cusp)は必ずその中に入る（200を返す）。"""
    body = {
        "start": {"x": 0, "y": 0, "heading_deg": 0},
        "target": {"x": 30, "y": 0, "heading_deg": 0},
        "max_switchbacks": 1, "require_switchback": True, "vehicle_id": "HD785",
        # ターゲット背後を広く覆うゾーン（HD785 の R_min でも S が収まる）
        "switchback_zone": [[10, -50], [120, -50], [120, 50], [10, 50]],
    }
    r = client.post("/api/simulate/spotting", json=body)
    assert r.status_code == 200, r.text
    j = r.json()
    for sp in j["switch_points"]:
        assert 10 <= sp["x"] <= 120 and -50 <= sp["y"] <= 50


def test_spotting_containment_polygon_api():
    """走行を収めるエリア: containment_polygon を渡すと経路（＋車体）がその多角形内に収まる。
    走行可能レイヤ無しでも多角形からローカルラスタを生成して封じ込めが効く。"""
    band = [[-5, -35], [70, -35], [70, 40], [-5, 40]]  # world 矩形（y[-35,40] の帯）
    body = {
        "start": {"x": 5, "y": 0, "heading_deg": 0},
        "target": {"x": 50, "y": 0, "heading_deg": 180},
        "max_switchbacks": 1, "require_switchback": True, "vehicle_id": "HD785",
        "containment_polygon": band,
    }
    r = client.post("/api/simulate/spotting", json=body)
    assert r.status_code == 200, r.text
    j = r.json()
    assert j["points"], j
    # 経路点が containment 多角形（矩形）内に収まる（外側は走行不可化されるため）。
    for p in j["points"]:
        assert -5.5 <= p["x"] <= 70.5 and -35.5 <= p["y"] <= 40.5, p


def test_spotting_containment_with_cost_no_drivable(tmp_path):
    """走行可能レイヤ無し・cost レイヤあり・封じ込めエリアあり（旧: hybrid で mask/cost 形状不一致→500）。
    cost と同一グリッド上に封じ込めマスクを作るので 200 を返し、経路は多角形内に収まる。"""
    p = tmp_path / "c.las"
    _make_las(p)
    with open(p, "rb") as f:
        las_id = client.post("/api/layers/las", files={"file": ("c.las", f, "application/octet-stream")}).json()["id"]
    cost_id = client.post(
        "/api/costmap",
        json={"las_layer_id": las_id, "src_epsg": 6677, "target_epsg": 6677, "params": {"grid_size_m": 1.0}},
    ).json()["id"]
    band = [[30000, 119000], [30050, 119000], [30050, 119040], [30000, 119040]]
    body = {
        "start": {"x": 30008, "y": 119020, "heading_deg": 0},
        "target": {"x": 30040, "y": 119020, "heading_deg": 180},
        "max_switchbacks": 1, "require_switchback": True, "vehicle_id": "HD785",
        "costmap_layer_id": cost_id, "containment_polygon": band, "method": "auto",
    }
    r = client.post("/api/simulate/spotting", json=body)
    assert r.status_code == 200, r.text  # 旧コードは hybrid の mask/cost 形状不一致で 500 だった
    j = r.json()
    assert j["points"], j
    for pt in j["points"]:  # 封じ込め多角形（cost グリッド上）の内側に収まる
        assert 29999 <= pt["x"] <= 30051 and 118999 <= pt["y"] <= 119041, pt
    client.delete(f"/api/layers/{cost_id}")
    client.delete(f"/api/layers/{las_id}")


def test_plan_reeds_shepp_algorithm():
    """Reeds-Shepp 単体アルゴリズム: 後方への寄り付き(U姿勢)でも前進＋後進で接続して 200。"""
    body = {
        "waypoints": [{"x": 0, "y": 0, "heading_deg": 0}, {"x": 12, "y": 4, "heading_deg": 180}],
        "mode": "waypoint_guided", "algorithm": "reeds_shepp", "vehicle_id": "HD785", "spacing_m": 2.0,
    }
    r = client.post("/api/plan", json=body)
    assert r.status_code == 200, r.text
    pts = r.json()["trajectory"]["points"]
    assert len(pts) >= 2
    last = pts[-1]
    assert abs(last["x"] - 12) < 1.5 and abs(last["y"] - 4) < 1.5  # 終点付近に到達


def test_plan_rejects_nonpositive_spacing():
    """入力バリデーション: spacing_m<=0 は 422（下流の数値エラーにせず弾く）。"""
    body = {"waypoints": [{"x": 0, "y": 0}, {"x": 10, "y": 0}], "spacing_m": 0}
    r = client.post("/api/plan", json=body)
    assert r.status_code == 422


def test_plan_reeds_shepp_preserves_reverse_gear():
    """後進つきプランナ(RS)の経路は、解析軌跡に後進(gear=R)区間を保持する（gear貫通）。"""
    body = {
        "waypoints": [{"x": 0, "y": 0, "heading_deg": 0}, {"x": 12, "y": 4, "heading_deg": 180}],
        "algorithm": "reeds_shepp", "vehicle_id": "HD785", "spacing_m": 1.0,
    }
    r = client.post("/api/plan", json=body)
    assert r.status_code == 200, r.text
    gears = [p.get("gear") for p in r.json()["trajectory"]["points"]]
    assert "R" in gears  # 後進セグメントが解析まで届いている


def test_plan_rrt_star_algorithm():
    """RRT* 単体アルゴリズム: 自由空間でも start→goal に到達して 200。"""
    body = {
        "waypoints": [{"x": 0, "y": 0, "heading_deg": 0}, {"x": 25, "y": 8, "heading_deg": 0}],
        "mode": "waypoint_guided", "algorithm": "rrt_star", "vehicle_id": "HD785", "spacing_m": 2.0,
    }
    r = client.post("/api/plan", json=body)
    assert r.status_code == 200, r.text
    pts = r.json()["trajectory"]["points"]
    assert len(pts) >= 2
    last = pts[-1]
    assert abs(last["x"] - 25) < 2.0 and abs(last["y"] - 8) < 2.0


def test_plan_velocity_profile():
    """車両指定の経路は速度プロファイル(speed_mps/time_s)と要約(max_speed/time/停止距離)を返す。"""
    body = {
        "waypoints": [{"x": 0, "y": 0}, {"x": 40, "y": 10}, {"x": 80, "y": 0}],
        "mode": "waypoint_guided", "algorithm": "spline", "vehicle_id": "HD785", "spacing_m": 2.0,
    }
    j = client.post("/api/plan", json=body).json()
    pts = j["trajectory"]["points"]
    assert pts[0].get("speed_mps") is not None and pts[0]["speed_mps"] == 0.0  # 始点停止
    assert any(p["speed_mps"] > 0 for p in pts)
    a = j["analysis"]
    assert a["max_speed_mps"] and a["time_total_s"] and a["stopping_distance_m"] is not None


def test_spotting_with_exit():
    body = {
        "start": {"x": 0, "y": 0, "heading_deg": 0}, "target": {"x": 25, "y": 8, "heading_deg": 0},
        "max_switchbacks": 0, "vehicle_id": "HD785", "with_exit": True,
    }
    j = client.post("/api/simulate/spotting", json=body).json()
    assert "reason" in j
    assert j.get("exit") and len(j["exit"]["points"]) >= 2


def test_drivable_edit_delete(tmp_path):
    p = tmp_path / "c.las"
    _make_las(p)
    with open(p, "rb") as f:
        las_id = client.post("/api/layers/las", files={"file": ("c.las", f, "application/octet-stream")}).json()["id"]
    cost_id = client.post("/api/costmap", json={"las_layer_id": las_id, "src_epsg": 6677, "target_epsg": 6677, "params": {"grid_size_m": 1.0}}).json()["id"]
    dv = client.post("/api/drivable", json={"cost_layer_id": cost_id, "params": {"threshold": 1e9, "close_m": 0, "open_m": 0, "min_area_m2": 0, "clearance_m": 0}}).json()["id"]
    poly1 = [[30005, 119005], [30012, 119005], [30012, 119012], [30005, 119012]]
    poly2 = [[30020, 119020], [30028, 119020], [30028, 119028], [30020, 119028]]
    client.patch(f"/api/drivable/{dv}", json={"op": "exclude", "polygon": poly1})
    m = client.patch(f"/api/drivable/{dv}", json={"op": "include", "polygon": poly2}).json()
    assert len(m["edits"]) == 2
    m = client.delete(f"/api/drivable/{dv}/edits/0").json()
    assert len(m["edits"]) == 1 and m["edits"][0]["op"] == "include"
    m = client.delete(f"/api/drivable/{dv}/edits").json()
    assert len(m.get("edits", [])) == 0
    client.delete(f"/api/layers/{dv}")
    client.delete(f"/api/layers/{cost_id}")
    client.delete(f"/api/layers/{las_id}")


def test_plan_safety_report():
    """plan 応答に safety(配信可否＋不可理由) が含まれ、R_min違反で NG＋理由になる。"""
    wps = [(0, 0), (20, 0), (30, 8), (20, 16), (0, 16), (0, 30), (25, 30)]
    body = {
        "waypoints": [{"x": x, "y": y} for x, y in wps],
        "mode": "waypoint_guided", "algorithm": "spline", "vehicle_id": "HM400", "spacing_m": 2.0,
    }
    on = client.post("/api/plan", json={**body, "enforce_min_radius": True}).json()
    assert "safety" in on and on["safety"]["passed"] is True

    off = client.post("/api/plan", json={**body, "enforce_min_radius": False}).json()
    assert off["safety"]["passed"] is False
    assert any("旋回半径" in r for r in off["safety"]["reasons"])
    names = {c["name"] for c in on["safety"]["checks"]}
    assert {"footprint", "min_radius"} <= names


def test_plan_refine_elastic_band(tmp_path):
    """Elastic Band 洗練: drivable mask があれば spline 経路を洗練し refined_elastic_band=True を返す。"""
    p = tmp_path / "c.las"
    _make_las(p)
    with open(p, "rb") as f:
        las_id = client.post("/api/layers/las", files={"file": ("c.las", f, "application/octet-stream")}).json()["id"]
    cost_id = client.post(
        "/api/costmap",
        json={"las_layer_id": las_id, "src_epsg": 6677, "target_epsg": 6677, "params": {"grid_size_m": 1.0}},
    ).json()["id"]
    dv_id = client.post(
        "/api/drivable",
        json={"cost_layer_id": cost_id, "params": {"threshold": 1e9, "close_m": 0, "open_m": 0, "min_area_m2": 0, "clearance_m": 0}},
    ).json()["id"]

    r = client.post(
        "/api/plan",
        json={
            "waypoints": [{"x": 30015, "y": 119015}, {"x": 30035, "y": 119025}],
            "mode": "waypoint_guided", "algorithm": "spline", "vehicle_id": "HD785",
            "drivable_layer_id": dv_id, "costmap_layer_id": cost_id,
            "spacing_m": 2.0, "refine_elastic_band": True,
        },
    )
    assert r.status_code == 200, r.text
    j = r.json()
    assert j["refined_elastic_band"] is True
    assert len(j["trajectory"]["points"]) >= 2

    client.delete(f"/api/layers/{dv_id}")
    client.delete(f"/api/layers/{cost_id}")
    client.delete(f"/api/layers/{las_id}")


def test_plan_auto_corridor_width(tmp_path):
    p = tmp_path / "c.las"
    _make_las(p)
    with open(p, "rb") as f:
        las_id = client.post("/api/layers/las", files={"file": ("c.las", f, "application/octet-stream")}).json()["id"]
    cost_id = client.post(
        "/api/costmap",
        json={"las_layer_id": las_id, "src_epsg": 6677, "target_epsg": 6677, "params": {"grid_size_m": 1.0}},
    ).json()["id"]
    dv_id = client.post(
        "/api/drivable",
        json={"cost_layer_id": cost_id, "params": {"threshold": 1e9, "close_m": 0, "open_m": 0, "min_area_m2": 0, "clearance_m": 0}},
    ).json()["id"]
    wp = [{"x": 30005, "y": 119005}, {"x": 30045, "y": 119035}]
    base = {"waypoints": wp, "mode": "auto", "algorithm": "grid_astar", "costmap_layer_id": cost_id, "drivable_layer_id": dv_id}

    # 妥当な道幅(4m)は通る
    ok = client.post("/api/plan", json={**base, "corridor_width_m": 4.0})
    assert ok.status_code == 200, ok.text
    # 過大な道幅(200m)は領域に収まらず 422
    too_wide = client.post("/api/plan", json={**base, "corridor_width_m": 200.0})
    assert too_wide.status_code == 422

    client.delete(f"/api/layers/{dv_id}")
    client.delete(f"/api/layers/{cost_id}")
    client.delete(f"/api/layers/{las_id}")


def test_large_raster_downsampled_on_upload(tmp_path):
    # MAX_RASTER_DIM(8192) を超える ortho はアップロード時にダウンサンプルされる
    import rasterio
    from affine import Affine

    p = tmp_path / "big.tif"
    W, H = 9000, 64
    prof = dict(driver="GTiff", width=W, height=H, count=3, dtype="uint8",
                crs=rasterio.crs.CRS.from_epsg(6677), transform=Affine(0.05, 0, 30000, 0, -0.05, 119000))
    with rasterio.open(p, "w", **prof) as ds:
        ds.write(np.full((3, H, W), 120, dtype="uint8"))
    with open(p, "rb") as f:
        r = client.post("/api/layers/ortho", files={"file": ("big.tif", f, "image/tiff")})
    assert r.status_code == 200, r.text
    m = r.json()
    assert m["downsampled_from"] == [W, H]
    assert m["width"] <= 8192 and m["width"] < W
    client.delete(f"/api/layers/{m['id']}")


def test_raster_reprojected_to_working_crs(tmp_path):
    # 4326 の ortho をアップロード → 作業CRS(6677) へ再投影される（ビュー整合のため）
    import rasterio
    from affine import Affine

    p = tmp_path / "wgs.tif"
    W, H = 256, 256
    # 福島付近(140.17E,37.07N)の小さな 4326 ラスタ
    prof = dict(driver="GTiff", width=W, height=H, count=3, dtype="uint8",
                crs=rasterio.crs.CRS.from_epsg(4326),
                transform=Affine(0.00001, 0, 140.176, 0, -0.00001, 37.075))
    with rasterio.open(p, "w", **prof) as ds:
        ds.write(np.full((3, H, W), 100, dtype="uint8"))
    with open(p, "rb") as f:
        r = client.post("/api/layers/ortho", files={"file": ("wgs.tif", f, "image/tiff")})
    assert r.status_code == 200, r.text
    m = r.json()
    assert m["epsg"] == 6677
    assert m["reprojected_from"] == 4326
    client.delete(f"/api/layers/{m['id']}")


def test_projects_crud():
    # 作成→一覧→読込→更新→削除
    cr = client.post("/api/projects", json={"name": "proj A", "state": {"waypoints": [1, 2], "vehicleId": "HD785"}, "updated_at": "2026-06-09T00:00:00Z"})
    assert cr.status_code == 200, cr.text
    pid = cr.json()["id"]
    assert any(p["id"] == pid for p in client.get("/api/projects").json())
    got = client.get(f"/api/projects/{pid}")
    assert got.status_code == 200 and got.json()["state"]["vehicleId"] == "HD785"
    up = client.put(f"/api/projects/{pid}", json={"name": "proj A2", "state": {"waypoints": []}, "updated_at": "2026-06-09T01:00:00Z"})
    assert up.status_code == 200 and up.json()["name"] == "proj A2"
    assert client.get(f"/api/projects/{pid}").json()["name"] == "proj A2"
    assert client.delete(f"/api/projects/{pid}").status_code == 200
    assert client.get(f"/api/projects/{pid}").status_code == 404


def test_spotting_forward():
    r = client.post(
        "/api/simulate/spotting",
        json={"start": {"x": 0, "y": 0, "heading_deg": 0}, "target": {"x": 25, "y": 8, "heading_deg": 0},
              "max_switchbacks": 0, "vehicle_id": "HD785"},
    )
    assert r.status_code == 200, r.text
    j = r.json()
    assert j["feasible"] and j["status"] == "OK"
    assert j["metrics"]["n_switchbacks"] == 0
    assert all(p["gear"] == "F" for p in j["points"])
    assert j["metrics"]["approach_error_m"] < 0.6


def test_spotting_needs_radius():
    r = client.post(
        "/api/simulate/spotting",
        json={"start": {"x": 0, "y": 0}, "target": {"x": 10, "y": 0}},  # 車両も半径も無し
    )
    assert r.status_code == 400


def test_spotting_switchback_reaches_target():
    r = client.post(
        "/api/simulate/spotting",
        json={"start": {"x": 0, "y": 0, "heading_deg": 0}, "target": {"x": 10, "y": 0, "heading_deg": 180},
              "max_switchbacks": 1, "min_turn_radius_m": 5.0},
    )
    assert r.status_code == 200, r.text
    j = r.json()
    assert j["feasible"]
    assert j["metrics"]["approach_error_m"] < 0.8
    assert j["metrics"]["time_total_s"] > 0


def test_plan_spline_uses_endpoint_headings():
    # 始終点の方位ベクトルを指定 → 端点の進行方位がそれに概ね一致
    wp = [
        {"x": 0, "y": 0, "heading_deg": 90},   # 北向き発進
        {"x": 40, "y": 40, "heading_deg": 0},  # 東向き到着
    ]
    r = client.post("/api/plan", json={"waypoints": wp, "algorithm": "spline", "spacing_m": 1.0})
    assert r.status_code == 200, r.text
    pts = r.json()["trajectory"]["points"]
    assert pts[0]["heading_deg"] > 45      # 発進は北寄り(>45°)
    assert abs(pts[-1]["heading_deg"]) < 45  # 到着は東寄り(<45°)


def test_analyze_straight_line():
    pts = [{"x": x, "y": 0} for x in range(0, 11)]
    r = client.post("/api/analyze", json={"points": pts})
    assert r.status_code == 200
    # 直線 -> 最小旋回半径は None（無限大）
    assert r.json()["analysis"]["min_radius_m"] is None


def test_unknown_vehicle_404():
    r = client.post("/api/analyze", json={"points": [{"x": 0, "y": 0}, {"x": 1, "y": 0}], "vehicle_id": "NOPE"})
    assert r.status_code == 404


@pytest.mark.skipif(not COSTMAP.exists(), reason="test raster not available")
def test_layer_upload_preview_and_tile():
    with open(COSTMAP, "rb") as f:
        r = client.post("/api/layers/ortho", files={"file": ("costmap.tif", f, "image/tiff")})
    assert r.status_code == 200, r.text
    meta = r.json()
    lid = meta["id"]
    assert meta["epsg"] == 6677
    assert meta["crs_source"] == "detected"
    assert meta["bands"] == 3

    # 一覧に出る
    assert any(m["id"] == lid for m in client.get("/api/layers").json()["layers"])

    # preview PNG
    pv = client.get(f"/api/layers/{lid}/preview.png")
    assert pv.status_code == 200 and pv.content[:8] == PNG_MAGIC

    # 中心のタイルを取得（WebMercatorQuad）
    import morecantile

    tms = morecantile.tms.get("WebMercatorQuad")
    w, s, e, n = meta["geographic_bounds"]
    lng, lat = (w + e) / 2, (s + n) / 2
    got = False
    for z in (20, 19, 18, 17, 16, 15):
        t = tms.tile(lng, lat, z)
        tr = client.get(f"/api/tiles/{lid}/{z}/{t.x}/{t.y}.png")
        if tr.status_code == 200:
            assert tr.content[:8] == PNG_MAGIC
            got = True
            break
    assert got, "no tile returned in z15-20"

    # COG 直配信（OL ol/source/GeoTIFF 用。Range 対応）
    full = client.get(f"/api/layers/{lid}/cog.tif")
    assert full.status_code == 200
    rng = client.get(f"/api/layers/{lid}/cog.tif", headers={"Range": "bytes=0-99"})
    assert rng.status_code == 206 and len(rng.content) == 100

    # 後始末
    assert client.delete(f"/api/layers/{lid}").status_code == 200


def _make_las(path, x0=30000.0, y0=119000.0):
    xs = np.linspace(x0, x0 + 50.0, 60)
    ys = np.linspace(y0, y0 + 40.0, 50)
    X, Y = np.meshgrid(xs, ys)
    Z = 0.1 * (X - x0)
    x, y, z = X.ravel(), Y.ravel(), Z.ravel()
    h = laspy.LasHeader(point_format=3, version="1.4")
    h.offsets = [x.min(), y.min(), z.min()]
    h.scales = [0.001, 0.001, 0.001]
    las = laspy.LasData(h)
    las.x, las.y, las.z = x, y, z
    las.write(str(path))


def test_plan_rejects_misregistered_layers(tmp_path):
    """B3: costmap A と、別位置の costmap B から生成した drivable の組合せは
    shape が同じでも transform が違う → co-registration 検証で 422 になる。"""
    pa, pb = tmp_path / "a.las", tmp_path / "b.las"
    _make_las(pa)
    _make_las(pb, x0=40000.0, y0=120000.0)  # 同サイズ・別位置
    ids = []
    for p in (pa, pb):
        with open(p, "rb") as f:
            las_id = client.post("/api/layers/las", files={"file": (p.name, f, "application/octet-stream")}).json()["id"]
        cost_id = client.post(
            "/api/costmap",
            json={"las_layer_id": las_id, "src_epsg": 6677, "target_epsg": 6677, "params": {"grid_size_m": 1.0}},
        ).json()["id"]
        ids.append((las_id, cost_id))
    dv_b = client.post(
        "/api/drivable",
        json={"cost_layer_id": ids[1][1], "params": {"threshold": 1e9, "close_m": 0, "open_m": 0, "min_area_m2": 0, "clearance_m": 0}},
    ).json()["id"]

    r = client.post(
        "/api/plan",
        json={
            "waypoints": [{"x": 30005, "y": 119005}, {"x": 30045, "y": 119035}],
            "mode": "auto", "algorithm": "grid_astar", "vehicle_id": "HD785",
            "costmap_layer_id": ids[0][1], "drivable_layer_id": dv_b, "spacing_m": 2.0,
        },
    )
    assert r.status_code == 422, r.text
    assert "グリッド" in r.json()["detail"]

    client.delete(f"/api/layers/{dv_b}")
    for las_id, cost_id in ids:
        client.delete(f"/api/layers/{cost_id}")
        client.delete(f"/api/layers/{las_id}")


def test_earthworks_pile_plan():
    """パイル配置: 撒き出しモード（V=24m³, t=0.5m, 60×30mエリア）→ 推奨間隔√48m・理論数37。"""
    r = client.post(
        "/api/earthworks/piles",
        json={"polygon": [[0, 0], [60, 0], [60, 30], [0, 30]],
              "repose_deg": 37.0, "volume_m3": 24.0, "spread_thickness_m": 0.5},
    )
    assert r.status_code == 200, r.text
    j = r.json()
    assert j["n_theory"] == 37
    assert 0 < j["count"] <= 37 and len(j["centers"]) == j["count"]
    assert j["pile"]["height_m"] > 0 and j["pile"]["radius_m"] > 0
    assert abs(j["suggested_spacing_m"] ** 2 - 48.0) < 0.1

    # 間隔指定モード（高さ指定・千鳥）
    r2 = client.post(
        "/api/earthworks/piles",
        json={"polygon": [[0, 0], [40, 0], [40, 20], [0, 20]],
              "repose_deg": 35.0, "height_m": 1.5, "dx_m": 6.0, "stagger": True},
    )
    assert r2.status_code == 200 and r2.json()["count"] > 0

    # 入力不備（体積も高さも無し）→ 400
    r3 = client.post("/api/earthworks/piles", json={"polygon": [[0, 0], [10, 0], [10, 10]], "dx_m": 5.0})
    assert r3.status_code == 400


def test_las_points_bin_roundtrip(tmp_path):
    """バイナリ点群: ヘッダ解析→origin相対f32から絶対座標を復元し、JSON版と一致（±2cm）。"""
    import struct

    p = tmp_path / "c.las"
    _make_las(p)
    with open(p, "rb") as f:
        las_id = client.post("/api/layers/las", files={"file": ("c.las", f, "application/octet-stream")}).json()["id"]

    rb = client.get(f"/api/layers/{las_id}/points.bin", params={"max_points": 5000})
    assert rb.status_code == 200 and rb.headers["content-type"].startswith("application/octet-stream")
    buf = rb.content
    magic, ver, has_rgb, _pad, n, ox, oy, oz, zmin, zmax = struct.unpack_from("<4sBBHI5d", buf, 0)
    assert magic == b"FRSP" and ver == 1 and n > 0
    off = struct.calcsize("<4sBBHI5d")
    xs = np.frombuffer(buf, dtype="<f4", count=n, offset=off)
    ys = np.frombuffer(buf, dtype="<f4", count=n, offset=off + 4 * n)
    zs = np.frombuffer(buf, dtype="<f4", count=n, offset=off + 8 * n)
    assert len(buf) == off + 12 * n + (3 * n if has_rgb else 0)
    ax, ay = xs + ox, ys + oy
    assert 29999.0 <= ax.min() and ax.max() <= 30051.0
    assert 118999.0 <= ay.min() and ay.max() <= 119041.0
    assert zmin <= float(zs.min() + oz) + 1e-3 and float(zs.max() + oz) <= zmax + 1e-3

    rj = client.get(f"/api/layers/{las_id}/points", params={"max_points": 5000}).json()
    assert rj["n"] == n
    assert abs(rj["x"][0] - float(ax[0])) < 0.02 and abs(rj["y"][0] - float(ay[0])) < 0.02

    client.delete(f"/api/layers/{las_id}")


def test_costmap_from_wgs84_las_auto_reprojects(tmp_path):
    """WGS84（ヘッダCRS無し・経緯度座標）の LAS が自動で作業ゾーンへ再投影される回帰。

    従来は src_epsg 未指定＋ヘッダ CRS 無しだと度をメートル扱いして壊れていた。
    経緯度らしき範囲（|x|<=180, |y|<=90）は WGS84 と推定して再投影する。
    """
    import rasterio

    from planning_core.geometry import project

    xs = np.linspace(30000.0, 30050.0, 40)
    ys = np.linspace(119000.0, 119040.0, 30)
    X, Y = np.meshgrid(xs, ys)
    ll = project(np.column_stack([X.ravel(), Y.ravel()]), 6677, 4326)  # lon/lat（度）
    h = laspy.LasHeader(point_format=3, version="1.4")  # CRS はあえて付けない
    h.offsets = [float(ll[:, 0].min()), float(ll[:, 1].min()), 0.0]
    h.scales = [1e-7, 1e-7, 0.001]
    las = laspy.LasData(h)
    las.x, las.y, las.z = ll[:, 0], ll[:, 1], np.zeros(len(ll))
    p = tmp_path / "wgs84.las"
    las.write(str(p))

    with open(p, "rb") as f:
        las_id = client.post("/api/layers/las", files={"file": ("wgs84.las", f, "application/octet-stream")}).json()["id"]
    r = client.post("/api/costmap", json={"las_layer_id": las_id, "target_epsg": 6677,
                                          "params": {"grid_size_m": 2.0}})
    assert r.status_code == 200, r.text
    m = r.json()
    assert m["las_crs_source"] == "assumed_wgs84"
    assert "推定" in (m.get("warning") or "")
    with rasterio.open(m["cost_cog"]) as ds:
        b = ds.bounds
    # 度をメートル扱いしていれば範囲は ~1e2 の度数域。作業ゾーンの元座標(±5m)に一致すること。
    assert abs(b.left - 30000.0) < 5.0 and abs(b.top - 119040.0) < 5.0

    client.delete(f"/api/layers/{m['id']}")
    client.delete(f"/api/layers/{las_id}")


def test_costmap_from_las(tmp_path):
    p = tmp_path / "c.las"
    _make_las(p)
    with open(p, "rb") as f:
        up = client.post("/api/layers/las", files={"file": ("c.las", f, "application/octet-stream")})
    assert up.status_code == 200, up.text
    las_id = up.json()["id"]
    assert up.json()["kind"] == "las"

    r = client.post(
        "/api/costmap",
        json={
            "las_layer_id": las_id,
            "src_epsg": 6677,
            "target_epsg": 6677,
            "params": {"grid_size_m": 0.5, "color_threshold": 300},
        },
    )
    assert r.status_code == 200, r.text
    m = r.json()
    assert m["kind"] == "cost" and m["epsg"] == 6677
    # 生cost / DSM / 表示RGB の3 COG が生成される（B1/B3）
    assert os.path.exists(m["cost_cog"]) and os.path.exists(m["dsm_cog"]) and os.path.exists(m["cog"])

    pv = client.get(f"/api/layers/{m['id']}/preview.png")
    assert pv.status_code == 200 and pv.content[:8] == PNG_MAGIC

    # 3D用 DSM グリッド（ダウンサンプル）
    grid = client.get(f"/api/costmap/{m['id']}/dsm_grid", params={"max_size": 32})
    assert grid.status_code == 200, grid.text
    g = grid.json()
    assert g["nx"] >= 2 and g["ny"] >= 2 and len(g["z"]) == g["ny"] and len(g["z"][0]) == g["nx"]
    assert g["zmax"] >= g["zmin"]

    client.delete(f"/api/layers/{m['id']}")
    client.delete(f"/api/layers/{las_id}")


def test_las_points_endpoint(tmp_path):
    p = tmp_path / "c.las"
    _make_las(p)
    with open(p, "rb") as f:
        las_id = client.post("/api/layers/las", files={"file": ("c.las", f, "application/octet-stream")}).json()["id"]
    r = client.get(f"/api/layers/{las_id}/points", params={"max_points": 1000})
    assert r.status_code == 200, r.text
    j = r.json()
    assert j["n"] >= 2 and len(j["x"]) == j["n"] == len(j["z"])
    assert j["zmax"] >= j["zmin"]
    assert "has_rgb" in j
    client.delete(f"/api/layers/{las_id}")


def test_drivable_from_costmap(tmp_path):
    p = tmp_path / "c.las"
    _make_las(p)
    with open(p, "rb") as f:
        las_id = client.post("/api/layers/las", files={"file": ("c.las", f, "application/octet-stream")}).json()["id"]
    cost_id = client.post(
        "/api/costmap",
        json={"las_layer_id": las_id, "src_epsg": 6677, "target_epsg": 6677, "params": {"grid_size_m": 1.0}},
    ).json()["id"]

    dv = client.post(
        "/api/drivable",
        json={"cost_layer_id": cost_id, "params": {"threshold": 1e9, "close_m": 0, "open_m": 0, "min_area_m2": 0, "clearance_m": 0}},
    )
    assert dv.status_code == 200, dv.text
    m = dv.json()
    assert m["kind"] == "drivable" and m["stats"]["area_m2"] > 0
    did, area0, ver0 = m["id"], m["stats"]["area_m2"], m["version"]
    assert client.get(f"/api/layers/{did}/preview.png").status_code == 200
    assert client.get(f"/api/drivable/{did}/analysis").json()["area_m2"] == area0

    # exclude 20x20m -> 面積減・version up（非破壊編集）
    box = [[30010, 119010], [30030, 119010], [30030, 119030], [30010, 119030]]
    ed = client.patch(f"/api/drivable/{did}", json={"op": "exclude", "polygon": box})
    assert ed.status_code == 200, ed.text
    m2 = ed.json()
    assert m2["version"] > ver0
    assert m2["stats"]["area_m2"] < area0

    client.delete(f"/api/layers/{did}")
    client.delete(f"/api/layers/{cost_id}")
    client.delete(f"/api/layers/{las_id}")


def test_fleet_conflicts_api():
    """複数経路の重なり判定 API: 直交2経路は交差で1件、車幅で重なる併走は shared。"""
    a = [[x, 0.0] for x in range(0, 51, 2)]
    b = [[25.0, y] for y in range(-20, 21, 2)]
    body = {"routes": [
        {"name": "A", "points": a, "vehicle_id": "HD785"},
        {"name": "B", "points": b, "vehicle_id": "HD785"},
    ], "cell_m": 0.5}
    r = client.post("/api/fleet/conflicts", json=body)
    assert r.status_code == 200, r.text
    j = r.json()
    assert j["n_conflicts"] == 1
    c = j["conflicts"][0]
    assert (c["a"], c["b"]) == (0, 1) and c["kind"] == "crossing"
    assert c["a_intervals"] and c["a_intervals"][0]["s_start"] < 25 < c["a_intervals"][0]["s_end"]


def test_fleet_conflicts_disjoint_api():
    """離れた経路は競合 0 件。"""
    a = [[x, 0.0] for x in range(0, 51, 2)]
    b = [[x, 60.0] for x in range(0, 51, 2)]
    body = {"routes": [{"points": a, "half_width_m": 1.7}, {"points": b, "half_width_m": 1.7}]}
    r = client.post("/api/fleet/conflicts", json=body)
    assert r.status_code == 200, r.text
    assert r.json()["n_conflicts"] == 0


def test_fleet_simulate_api():
    """複数台 簡易sim API: 直交2経路で両車到達、低優先が待機、デッドロックなし。"""
    a = [[x, 0.0] for x in range(0, 81, 2)]
    b = [[40.0, y] for y in range(-40, 41, 2)]
    body = {"routes": [
        {"name": "A", "points": a, "vehicle_id": "HD785", "priority": 0},
        {"name": "B", "points": b, "vehicle_id": "HD785", "priority": 1},
    ], "dt_s": 0.2, "gap_m": 2.0}
    r = client.post("/api/fleet/simulate", json=body)
    assert r.status_code == 200, r.text
    j = r.json()
    assert j["status"] == "OK" and j["deadlock"] is False
    assert len(j["traces"]) == 2 and j["traces"][0][-1]["s"] > 78
    assert any(tr["state"] == "wait" for tr in j["traces"][1])  # B が待機


def test_fleet_simulate_passing_bay_resolves_head_on():
    """対向(head-on)単線: bay 無しは接触検出、譲る側に bay を設定すると衝突せず両者到達。"""
    a = [[x, 0.0] for x in range(0, 121, 2)]
    b = [[120 - x, 0.0] for x in range(0, 121, 2)]
    base = {"routes": [
        {"name": "A", "points": a, "half_width_m": 1.7, "v_max_mps": 6.0, "priority": 0},
        {"name": "B", "points": b, "half_width_m": 1.7, "v_max_mps": 6.0, "priority": 1},
    ], "dt_s": 0.2, "gap_m": 2.0, "max_time_s": 200}
    bad = client.post("/api/fleet/simulate", json=base).json()
    assert bad["collision"] is False and bad["deadlock"] is True  # 待避所なしは追従で停止＝膠着（衝突しない）

    withbay = {**base, "routes": [
        base["routes"][0],
        {**base["routes"][1], "bay": {"s_frac": 0.5, "offset_m": 8.0, "side": 1, "ramp_m": 8.0, "hold_m": 20.0}},
    ]}
    good = client.post("/api/fleet/simulate", json=withbay).json()
    assert good["collision"] is False and good["status"] == "OK"
    assert good["traces"][0][-1]["s"] > 118 and good["traces"][1][-1]["s"] > 118


def test_fleet_simulate_auto_passing():
    """auto_passing=True: 対向(衝突)を自動待避所で解決し、auto_bays が返る。"""
    a = [[x, 0.0] for x in range(0, 121, 2)]
    b = [[120 - x, 0.0] for x in range(0, 121, 2)]
    body = {"routes": [
        {"name": "A", "points": a, "half_width_m": 1.7, "v_max_mps": 6.0, "priority": 0},
        {"name": "B", "points": b, "half_width_m": 1.7, "v_max_mps": 6.0, "priority": 1},
    ], "dt_s": 0.2, "gap_m": 2.0, "max_time_s": 240, "auto_passing": True}
    j = client.post("/api/fleet/simulate", json=body).json()
    assert j["collision"] is False and j["status"] == "OK"
    assert len(j["auto_bays"]) >= 1 and j["auto_bays"][0]["vehicle"] == 1


def test_fleet_junction_api():
    """分岐起点姿勢 API: 東向き直線の中点は heading≈0、位置は中央。"""
    pts = [[float(x), 0.0] for x in range(0, 101, 5)]
    j = client.post("/api/fleet/junction", json={"points": pts, "s_frac": 0.5}).json()
    assert abs(j["x"] - 50.0) < 2.0 and abs(j["y"]) < 1e-6
    assert abs(((j["heading_deg"] + 180) % 360) - 180) < 1e-6
