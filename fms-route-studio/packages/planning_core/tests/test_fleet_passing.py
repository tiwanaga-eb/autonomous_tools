import numpy as np

from planning_core.fleet import SimVehicle, auto_bay_center, detect_conflicts, lateral_detour, simulate_fleet


def _line(p0, p1, n=120):
    return np.column_stack([np.linspace(p0[0], p1[0], n), np.linspace(p0[1], p1[1], n)])


def test_lateral_detour_offsets_and_rejoins():
    """待避所: 中央で横へ offset し、端点は不変・本線へ戻る。"""
    pts = _line((0, 0), (100, 0))
    det = lateral_detour(pts, s_center=50.0, offset=5.0, side=1, ramp=8.0, hold=8.0)
    # 端点不変
    assert abs(det[0, 1]) < 1e-6 and abs(det[-1, 1]) < 1e-6
    # 中央付近で左(+y)へ ~5m 退避
    mid = det[np.argmin(np.abs(det[:, 0] - 50.0))]
    assert 4.5 < mid[1] < 5.5
    # 本線部（s<30, s>70）はほぼ不変
    assert max(abs(det[i, 1]) for i in range(len(det)) if det[i, 0] < 30) < 0.6


def test_head_on_without_bay_deadlocks():
    """対向(head-on)単線は待避所なしでは捌けない＝前方車追従で**衝突せず膠着(DEADLOCK)**。"""
    a = _line((0, 0), (120, 0))
    b = _line((120, 0), (0, 0))
    va = SimVehicle(points=a, v_max=6, accel=0.8, decel=1.2, half_width=1.7, half_length=3.0, priority=0)
    vb = SimVehicle(points=b, v_max=6, accel=0.8, decel=1.2, half_width=1.7, half_length=3.0, priority=1)
    r = simulate_fleet([va, vb], dt=0.2, gap_m=2.0, max_time=120)
    assert not r.collision and r.deadlock  # 衝突せず膠着（待避所が必要）


def test_passing_bay_resolves_head_on():
    """待避所(横退避)を譲る側に入れると、競合が2領域に割れて区間Mutexが交互通行を成立させ、
    対向(head-on)が**衝突せず両者到達**する（Phase C の核）。"""
    a = _line((0, 0), (120, 0))
    b = _line((120, 0), (0, 0))
    b_det = lateral_detour(b, s_center=auto_bay_center(b, 0.5), offset=8.0, side=1, ramp=8.0, hold=20.0)
    va = SimVehicle(points=a, v_max=6, accel=0.8, decel=1.2, half_width=1.7, half_length=3.0, priority=0)
    vb = SimVehicle(points=b_det, v_max=6, accel=0.8, decel=1.2, half_width=1.7, half_length=3.0, priority=1)
    r = simulate_fleet([va, vb], dt=0.2, gap_m=2.0, max_time=200)
    assert not r.collision and not r.deadlock and r.status == "OK"
    assert r.traces[0][-1]["s"] > 119 and r.traces[1][-1]["s"] > 119


def test_detour_reduces_conflict_overlap():
    """待避所で退避させると、本線との競合（重なり面積）が減る。"""
    a = _line((0, 0), (100, 0))
    b = _line((100, 0), (0, 0))
    base = detect_conflicts([{"points": a, "half_width": 1.7}, {"points": b, "half_width": 1.7}], cell=0.5)
    b_det = lateral_detour(b, s_center=auto_bay_center(b, 0.5), offset=6.0, side=1, ramp=10.0, hold=10.0)
    after = detect_conflicts([{"points": a, "half_width": 1.7}, {"points": b_det, "half_width": 1.7}], cell=0.5)
    area_base = sum(c.overlap_area_m2 for c in base)
    area_after = sum(c.overlap_area_m2 for c in after)
    assert area_after < area_base * 0.8  # 退避ぶん重なりが減る


def test_auto_passing_resolves_head_on():
    """自動トラフィック管理: 対向(衝突)を simulate_fleet_auto が低優先側へ自動待避所を入れて解決。"""
    from planning_core.fleet import simulate_fleet_auto
    a = _line((0, 0), (120, 0))
    b = _line((120, 0), (0, 0))
    va = SimVehicle(points=a, v_max=6, accel=0.8, decel=1.2, half_width=1.7, half_length=3.0, priority=0)
    vb = SimVehicle(points=b, v_max=6, accel=0.8, decel=1.2, half_width=1.7, half_length=3.0, priority=1)
    r = simulate_fleet_auto([va, vb], dt=0.2, gap_m=2.0, max_time=240)
    assert not r.collision and r.status == "OK"
    assert len(r.auto_bays) >= 1 and r.auto_bays[0]["vehicle"] == 1  # 低優先(B)へ配置
    assert r.traces[0][-1]["s"] > 119 and r.traces[1][-1]["s"] > 119


def test_auto_passing_no_bay_when_no_collision():
    """衝突の無い交差では自動待避所は配置されない。"""
    from planning_core.fleet import simulate_fleet_auto
    a = _line((0, 0), (80, 0))
    b = _line((40, -40), (40, 40))
    va = SimVehicle(points=a, v_max=6, accel=0.8, decel=1.2, half_width=1.7, half_length=3.0, priority=0)
    vb = SimVehicle(points=b, v_max=6, accel=0.8, decel=1.2, half_width=1.7, half_length=3.0, priority=1)
    r = simulate_fleet_auto([va, vb], dt=0.2, gap_m=2.0)
    assert not r.collision and len(r.auto_bays) == 0
