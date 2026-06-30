import numpy as np

from planning_core.fleet import SimVehicle, simulate_fleet


def _line(p0, p1, n=80):
    return np.column_stack([np.linspace(p0[0], p1[0], n), np.linspace(p0[1], p1[1], n)])


def _veh(p0, p1, **kw):
    kw.setdefault("v_max", 6.0)
    kw.setdefault("accel", 0.8)
    kw.setdefault("decel", 1.2)
    kw.setdefault("half_width", 1.7)
    return SimVehicle(points=_line(p0, p1), **kw)


def test_crossing_lower_priority_yields():
    """直交2経路: 低優先(B)が交差手前で待ち、両車とも到達。衝突せず直列化。"""
    a = _veh((0, 0), (80, 0), priority=0, name="A")
    b = _veh((40, -40), (40, 40), priority=1, name="B")
    r = simulate_fleet([a, b], dt=0.2, gap_m=2.0)
    assert r.status == "OK" and not r.deadlock and not r.collision
    assert r.traces[0][-1]["s"] > 79 and r.traces[1][-1]["s"] > 79  # 両者到達
    assert any(tr["state"] == "wait" for tr in r.traces[1])         # B は待機した
    # 区間Mutexにより車体が重ならない（半幅和=3.4m を下回らない）。
    assert r.min_separation_m is not None and r.min_separation_m >= a.half_width + b.half_width


def test_opposite_single_lane_deadlocks_without_bay():
    """同一単線を同時に逆走: 前方車追従で両者が正面手前で停止し**衝突せず膠着(DEADLOCK)**になる。
    解消には待避所(Phase C)が必要。衝突しない＝安全側であることを確認。"""
    c = _veh((0, 0), (60, 0), priority=0, name="C")
    d = _veh((60, 0), (0, 0), priority=1, name="D")
    r = simulate_fleet([c, d], dt=0.2, gap_m=2.0, max_time=120)
    assert not r.collision  # 衝突しない（追従で停止）
    assert r.deadlock and r.status == "DEADLOCK"
    assert r.min_separation_m is not None and r.min_separation_m >= c.half_width + d.half_width


def test_staggered_start_both_complete():
    """時差発進: 先行が抜けてから後発が出発、両者到達。"""
    c = _veh((0, 0), (60, 0), priority=0, name="C")
    d = _veh((60, 0), (0, 0), priority=1, start_time=20.0, name="D")
    r = simulate_fleet([c, d], dt=0.2, max_time=120)
    assert r.status == "OK" and not r.deadlock
    assert r.traces[0][-1]["s"] > 59 and r.traces[1][-1]["s"] > 59


def test_no_conflict_runs_free():
    """競合の無い2経路は互いに減速せず最大速度近くまで出る。"""
    a = _veh((0, 0), (60, 0), priority=0)
    b = _veh((0, 50), (60, 50), priority=1)
    r = simulate_fleet([a, b], dt=0.2)
    assert r.status == "OK"
    assert max(tr["v"] for tr in r.traces[0]) > 5.0  # 自由走行で v_max(6) 近く


def test_three_vehicles_crossing_no_collision():
    """3台が1点で交差（直交＋平行、head-onなし）。Mutex で順番待ち、全車到達・衝突なし。"""
    a = _veh((0, 0), (80, 0), priority=0, name="A")        # 東進 y=0
    b = _veh((30, -40), (30, 40), priority=1, name="B")    # 北進 x=30
    c = _veh((55, -40), (55, 40), priority=2, name="C")    # 北進 x=55（B と平行＝非競合）
    r = simulate_fleet([a, b, c], dt=0.2, gap_m=2.0, max_time=120)
    assert r.status == "OK" and not r.collision and not r.deadlock
    assert all(r.traces[i][-1]["s"] > 78 for i in range(3))
    assert r.min_separation_m is not None and r.min_separation_m >= 3.4


def test_decel_to_route_end():
    """経路末端で停止する（規定減速）。最終速度0かつ末端到達。"""
    a = _veh((0, 0), (40, 0), priority=0)
    r = simulate_fleet([a], dt=0.2)
    assert r.traces[0][-1]["s"] > 39
    assert r.traces[0][-1]["v"] < 0.6  # ほぼ停止


def test_sim_reports_travel_and_wait_metrics():
    """各車の所要時間(travel_time_s)と待機時間(total_wait_s)が返る。所要は0<…<=makespan。"""
    a = _veh((0, 0), (80, 0), priority=0)
    b = _veh((40, -40), (40, 40), priority=1)
    r = simulate_fleet([a, b], dt=0.2, gap_m=2.0)
    assert len(r.travel_time_s) == 2 and all(0 < tt <= r.makespan + 1e-6 for tt in r.travel_time_s)
    assert isinstance(r.total_wait_s, float) and r.total_wait_s >= 0.0
    assert len(r.wait_time_s) == 2


def test_sequential_rotation_dispatch():
    """逐次ローテーション: 1台ずつ走り(同時runは常に1台以下)、各車 loops 周、Goal到達で次が発進。"""
    from planning_core.fleet import simulate_fleet_sequential
    vs = [_veh((0, 0), (60, 0)), _veh((0, 10), (40, 10)), _veh((0, -10), (50, -10))]
    r = simulate_fleet_sequential(vs, loops=2, dt=0.2, max_time=300)
    assert r.status == "OK"
    nf = len(r.traces[0])
    for k in range(nf):  # 同時に run は1台以下
        assert sum(1 for i in range(3) if r.traces[i][k]["state"] == "run") <= 1
    # 各車2周＝dispatch が車両ごと2回
    disp = [e for e in r.events if e["type"] == "dispatch"]
    assert sum(1 for e in disp if e["vehicle"] == 0) == 2
    assert all(tt > 0 for tt in r.travel_time_s)
