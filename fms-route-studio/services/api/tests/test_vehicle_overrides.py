"""車種ごとパラメータチューニング（オーバーライド）のテスト。"""
from fastapi.testclient import TestClient

from app.main import app

client = TestClient(app)


def _reset(vid):
    client.delete(f"/api/vehicles/{vid}/override")


def test_detail_shape():
    d = client.get("/api/vehicles/HD785/detail").json()
    assert set(d.keys()) == {"effective", "default", "override", "editable_fields"}
    assert "min_turning_radius" in d["editable_fields"]
    assert d["override"] == {}


def test_put_get_reset_and_overridden_flag():
    try:
        r = client.put("/api/vehicles/HD785", json={"fields": {"min_turning_radius": 14.0, "overall_width": 6.0}})
        assert r.status_code == 200, r.text
        eff = r.json()["effective"]
        assert eff["min_turning_radius"] == 14.0 and eff["overall_width"] == 6.0
        # 幅変更で footprint_polygon が追従（半幅3.0）＋後輪軸基準の前後非対称は保持される
        poly = eff["footprint_polygon"]
        assert abs(abs(poly[0][1]) - 3.0) < 1e-6
        xs = [p[0] for p in poly]
        assert max(xs) > -min(xs)  # 前端>後端＝後輪軸基準を維持（中心矩形に戻っていない）
        assert abs(max(xs) - 7.825) < 1e-6  # 幅変更では前後端(x)は不変
        # 一覧に overridden フラグ
        lst = {v["id"]: v for v in client.get("/api/vehicles").json()}
        assert lst["HD785"]["overridden"] is True and lst["HD605"]["overridden"] is False
        # 単体GETも resolve 済み
        assert client.get("/api/vehicles/HD785").json()["min_turning_radius"] == 14.0
    finally:
        _reset("HD785")
    # reset 後は既定へ
    assert client.get("/api/vehicles/HD785").json()["min_turning_radius"] == 10.1
    assert client.get("/api/vehicles").json()[0]["overridden"] is False


def test_put_partial_merge():
    try:
        client.put("/api/vehicles/HD605", json={"fields": {"min_turning_radius": 11.0}})
        client.put("/api/vehicles/HD605", json={"fields": {"road_width": 6.0}})
        d = client.get("/api/vehicles/HD605/detail").json()
        assert d["override"]["min_turning_radius"] == 11.0 and d["override"]["road_width"] == 6.0
    finally:
        _reset("HD605")


def test_invalid_override_returns_400():
    # tracked_skid に wheel_base は付けられない（運動学と矛盾）
    r = client.put("/api/vehicles/CD110R", json={"fields": {"wheel_base": 3.0}})
    assert r.status_code == 400
    _reset("CD110R")


def test_override_propagates_to_planning():
    """min_turning_radius を上げると dubins 経路の実測最小半径も増える（resolver 反映）。"""
    body = {
        "waypoints": [{"x": 0, "y": 0, "heading_deg": 90}, {"x": 12, "y": 0, "heading_deg": 270}],
        "algorithm": "dubins", "vehicle_id": "HD785", "spacing_m": 1.0,
    }
    r_def = client.post("/api/plan", json=body).json()["measured_min_radius_m"]
    try:
        client.put("/api/vehicles/HD785", json={"fields": {"min_turning_radius": 25.0}})
        r_ovr = client.post("/api/plan", json=body).json()["measured_min_radius_m"]
    finally:
        _reset("HD785")
    assert r_def is not None and r_ovr is not None
    assert r_ovr > r_def + 3.0  # 半径上限を上げた分、経路の旋回半径が大きくなる


def test_fleet_sim_vehicle_uses_loaded_decel():
    """fleet sim: 制動カーブは通常減速度、予約距離は保守側（積載時）減速度で構築される。

    予約側が空車値だと積載車の制動が伸びた際に Mutex ゾーンが過小になる。
    逆に制動カーブまで積載値だと停止点の数百m手前から徐行になり非現実的（M2）。"""
    from app.routers.fleet import SimRouteIn, _sim_vehicle
    from planning_core.vehicle import load_builtin

    prof = load_builtin("HM400")
    sv = _sim_vehicle(SimRouteIn(id="r1", points=[[0.0, 0.0], [50.0, 0.0]], vehicle_id="HM400"))
    assert sv.decel == prof.max_decel                  # 制動カーブ=通常値
    assert sv.reserve_decel == prof.max_decel_loaded   # 予約距離=積載値（保守側）
    assert sv.reserve_decel < sv.decel
