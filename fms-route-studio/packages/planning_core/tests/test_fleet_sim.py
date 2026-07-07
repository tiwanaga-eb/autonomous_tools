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


def test_crossing_60deg_creates_mutex_and_no_body_overlap():
    """60°交差は Mutex で直列化される（H2 回帰）。

    旧実装は中点1点の cos>0.3（±72°）で「同方向」と誤判定し Mutex を作らず、
    車追従だけで捌いた結果 10m 級車体が重なっていた。区間全体平均 cos>0.85 に
    厳格化後は片方が待ち、車体接触なしで両車到達する。
    """
    import math

    L = 100.0
    th = math.radians(60)
    a = SimVehicle(points=_line((0, 0), (L, 0), n=200), v_max=8.0, accel=1.0, decel=0.5,
                   half_width=1.7, half_length=5.0)
    b = SimVehicle(points=_line((50 - 50 * math.cos(th), -50 * math.sin(th)),
                                (50 + 50 * math.cos(th), 50 * math.sin(th)), n=200),
                   v_max=8.0, accel=1.0, decel=0.5, half_width=1.7, half_length=5.0, priority=1)
    res = simulate_fleet([a, b], dt=0.2)
    assert res.status == "OK", res.status
    assert not res.collision
    assert any(e["type"] == "reserve" for e in res.events)  # Mutex が機能している
    # 車体対角の和より十分離れている（車体が触れ得ない）
    assert res.min_separation_m is not None and res.min_separation_m > 2 * math.hypot(5.0, 1.7)


def test_bodies_overlap_detects_longitudinal_contact():
    """車体重なり判定は車長を考慮する（H1 回帰: 旧の幅円判定は縦方向の接触を見逃す）。"""
    from planning_core.fleet.sim import _bodies_overlap

    vi = SimVehicle(points=_line((0, 0), (1, 0)), half_width=1.7, half_length=5.0)
    # 同方位で中心間 6m（車長10m どうし → 4m 重なる。幅円判定なら 6 > 1.7+1.7 で見逃し）
    f1 = {"x": 0.0, "y": 0.0, "heading_deg": 0.0}
    f2 = {"x": 6.0, "y": 0.0, "heading_deg": 0.0}
    assert _bodies_overlap(f1, f2, vi, vi)
    # 中心間 12m（車長和 10m 超）→ 非接触
    f3 = {"x": 12.0, "y": 0.0, "heading_deg": 0.0}
    assert not _bodies_overlap(f1, f3, vi, vi)
    # T字（直交）: 相手の側面へ 5m → 自車前端(5m)+相手半幅(1.7m) > 5m で接触
    f4 = {"x": 5.0, "y": 0.0, "heading_deg": 90.0}
    assert _bodies_overlap(f1, f4, vi, vi)
    # 横に十分ずれた並走 → 非接触
    f5 = {"x": 0.0, "y": 4.0, "heading_deg": 0.0}
    assert not _bodies_overlap(f1, f5, vi, vi)


def test_reserve_decel_extends_reservation_only():
    """M2: reserve_decel（積載）は予約距離のみに効き、走行の制動カーブは decel（通常）のまま。

    同一条件で reserve_decel を保守化しても所要時間（自由走行部）は変わらず、
    予約イベントはより手前（早い時刻）で発生する。
    """
    def run(reserve_decel):
        a = SimVehicle(points=_line((0, 0), (200, 0), n=200), v_max=10.0, accel=1.0, decel=1.0,
                       reserve_decel=reserve_decel, half_width=1.7, half_length=5.0)
        b = SimVehicle(points=_line((150, -80), (150, 80), n=200), v_max=10.0, accel=1.0, decel=1.0,
                       reserve_decel=reserve_decel, half_width=1.7, half_length=5.0,
                       priority=1, start_time=8.0)
        return simulate_fleet([a, b], dt=0.2)

    base = run(None)          # 予約距離 = v²/2·1.0 + 3 = 53m
    conservative = run(0.25)  # 予約距離 = v²/2·0.25 + 3 = 203m
    t_res = {tuple(sorted(e["zone"])): e["t"] for e in base.events if e["type"] == "reserve"}
    t_res_c = {tuple(sorted(e["zone"])): e["t"] for e in conservative.events if e["type"] == "reserve"}
    assert base.status == "OK" and conservative.status == "OK"
    # 予約はより早く（手前で）発生する
    common = set(t_res) & set(t_res_c)
    assert common and all(t_res_c[z] <= t_res[z] for z in common)
    # 制動カーブは同じ decel → 先行車Aの所要時間は不変（±1ステップ）
    assert abs(base.travel_time_s[0] - conservative.travel_time_s[0]) <= 0.4


def test_shallow_merge_flows_with_priority_tiebreak():
    """M1: 20°の浅い合流（同方向扱い=Mutexなし）は相互追従の偽デッドロックにならず、
    高優先が先行・低優先が譲って両車到達する。車体接触なし。"""
    import math

    th = math.radians(20)
    a = SimVehicle(points=_line((0, 0), (120, 0), n=200), v_max=6.0, accel=1.0, decel=0.8,
                   half_width=1.7, half_length=5.0)
    b = SimVehicle(points=_line((60 - 60 * math.cos(th), -60 * math.sin(th)),
                                (60 + 60 * math.cos(th), 60 * math.sin(th)), n=200),
                   v_max=6.0, accel=1.0, decel=0.8, half_width=1.7, half_length=5.0, priority=1)
    res = simulate_fleet([a, b], dt=0.2)
    assert res.status == "OK", res.status  # 旧実装: DEADLOCK（相互に前方車と認識して両者停止）
    assert not res.collision


def test_park_at_goal_keeps_done_vehicle_as_obstacle():
    """M3: 既定は到達車=退場（後続がすり抜け可・端点共有の誤検出回避）。
    park_at_goal=True で駐機扱いになり、後続は駐機車の手前で停止する（接触なし）。"""
    a = _veh((0, 0), (50, 0), priority=0, name="A")      # 50m 先で停車（B の経路上）
    b = _veh((12, 0), (100, 0), priority=1, name="B", start_time=25.0)  # A 駐機後に出発して通過を試みる
    # 既定: A は到達後に退場 → B は通過して到達
    res = simulate_fleet([a, b], dt=0.2, max_time=120)
    assert res.status == "OK" and not res.collision
    assert res.traces[1][-1]["s"] >= 84.0  # B の経路長 88m をほぼ走破
    # 駐機: B は A の手前で停止（進めない=膠着として報告されるが接触はしない）
    res2 = simulate_fleet([a, b], dt=0.2, max_time=120, park_at_goal=True)
    assert not res2.collision
    assert res2.status in ("DEADLOCK", "TIMEOUT")
    assert res2.traces[1][-1]["s"] < 38.0  # A(Bの弧長38m地点) の手前で停止


def test_asymmetric_extents_shift_collision_envelope():
    """基準点=後輪軸（前後非対称 extent）で車体矩形が前方へ伸びる（HD系の後輪軸基準）。

    後輪軸を経路点に置く車は、車体が前方に大きく張り出す。front_ext を大きく取ると
    前方の相手との接触が正しく検出される（対称 half_length では見逃す配置）。
    """
    from planning_core.fleet.sim import _bodies_overlap

    # 自車: 後輪軸基準、前 8m / 後 3m（x=-3〜+8 を占有）。相手は小さなプローブ（点近似）。
    vi = SimVehicle(points=_line((0, 0), (1, 0)), half_width=1.7, front_ext=8.0, rear_ext=3.0)
    probe = SimVehicle(points=_line((0, 0), (1, 0)), half_width=0.1, front_ext=0.1, rear_ext=0.1)
    f_self = {"x": 0.0, "y": 0.0, "heading_deg": 0.0}
    # 前方 7m: 前端 8m 以内 → 接触（対称 half_length では基準点=中心と誤認し見逃す配置）
    assert _bodies_overlap(f_self, {"x": 7.0, "y": 0.0, "heading_deg": 0.0}, vi, probe)
    # 後方 4m: 後端 3m を超える → 非接触
    assert not _bodies_overlap(f_self, {"x": -4.0, "y": 0.0, "heading_deg": 0.0}, vi, probe)


def test_fext_rext_default_to_half_length():
    """front_ext/rear_ext 未指定は half_length（対称=旧挙動フォールバック）。"""
    v = SimVehicle(points=_line((0, 0), (1, 0)), half_length=4.0)
    assert v.fext == 4.0 and v.rext == 4.0
