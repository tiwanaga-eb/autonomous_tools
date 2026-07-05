import math

import numpy as np
from affine import Affine

from planning_core.analysis import min_turning_radius
from planning_core.planners.dubins import reverse_dubins
from planning_core.simulator import CostWeights, plan_spotting


def test_reverse_dubins_reaches_goal_and_respects_radius():
    rho = 6.0
    start = (0.0, 0.0, 0.0)
    goal = (12.0, -4.0, 0.0)
    pts = reverse_dubins(start, goal, rho, step=0.3)
    assert pts is not None
    assert np.allclose(pts[0], [0.0, 0.0], atol=1e-6)
    assert np.allclose(pts[-1], [12.0, -4.0], atol=1e-6)
    assert min_turning_radius(pts) > rho - 0.8


def test_spotting_forward_only_reaches_target():
    res = plan_spotting((0.0, 0.0, 0.0), (25.0, 8.0, 0.0), rho=8.0, max_switchbacks=0)
    assert res.status == "OK" and res.feasible
    assert res.n_switchbacks == 0
    assert all(p["gear"] == "F" for p in res.points)
    assert res.approach_error_m < 0.5
    assert res.length_total > 0 and res.time_total > 0
    # 一発到達精度の方位側（P-008 ±5° 判定の入力）も算出される
    assert res.approach_error_deg is not None and res.approach_error_deg < 5.0


def test_spotting_with_switchback_reaches_target():
    # 1切り返し許可。自由空間では最短(前進)が選ばれることもあるが、必ずターゲットに到達。
    res = plan_spotting((0.0, 0.0, 0.0), (10.0, 0.0, math.pi), rho=5.0, max_switchbacks=1)
    assert res.feasible
    assert res.approach_error_m < 0.6
    assert res.n_switchbacks in (0, 1)


def test_spotting_clearance_with_mask():
    # 全面走行可能 mask → feasible かつ min_clearance が報告される
    n = 80
    mask = np.ones((n, n), np.uint8)
    t = Affine(0.5, 0, 0, 0, -0.5, float(n) * 0.5)
    res = plan_spotting((2.0, 2.0, 0.0), (18.0, 6.0, 0.0), rho=6.0, max_switchbacks=1,
                        drivable_mask=mask, transform=t, footprint_radius=1.0)
    assert res.feasible
    assert res.min_clearance_m is not None and res.min_clearance_m >= 1.0


def test_spotting_require_switchback_forces_reverse():
    # require_switchback=True → 前進だけで届く配置でも必ず後進(切り返し1回)を含む
    res = plan_spotting((0.0, 0.0, 0.0), (25.0, 8.0, 0.0), rho=6.0, max_switchbacks=1,
                        require_switchback=True)
    assert res.n_switchbacks == 1
    assert res.length_rev > 0.0
    assert any(p["gear"] == "R" for p in res.points)
    assert res.approach_error_m < 0.8


def test_spotting_cost_map_avoidance():
    # 中央に高コストの塊。w_costmap を上げると、それを避ける（低コスト積分の）経路を選ぶ。
    n = 50
    cost = np.ones((n, n), float)
    cost[18:32, 20:30] = 400.0  # 高コスト塊
    t = Affine(1.0, 0, 0, 0, -1.0, float(n))
    start = (5.0, 25.0, 0.0)
    goal = (45.0, 25.0, 0.0)
    lo = plan_spotting(start, goal, rho=6.0, max_switchbacks=0, cost=cost, transform=t,
                       weights=CostWeights(w_costmap=0.0))
    hi = plan_spotting(start, goal, rho=6.0, max_switchbacks=0, cost=cost, transform=t,
                       weights=CostWeights(w_costmap=80.0))
    assert hi.cost_integral < lo.cost_integral  # 高weightは高コスト塊を避ける


def test_spotting_infeasible_when_outside_mask():
    # 経路が通る領域に mask が無い → 非到達(best-effort) 扱い
    n = 20
    mask = np.zeros((n, n), np.uint8)
    mask[0:2, 0:2] = 1  # ごく一部だけ走行可能
    t = Affine(1.0, 0, 0, 0, -1.0, float(n))
    res = plan_spotting((5.0, 5.0, 0.0), (15.0, 15.0, 0.0), rho=5.0, max_switchbacks=1,
                        drivable_mask=mask, transform=t, footprint_radius=0.5)
    assert not res.feasible
    assert res.status in ("INFEASIBLE_BEST_EFFORT", "COLLISION", "NO_PATH")


def test_spotting_manual_switch_pose():
    """手動切り返し点: 指定姿勢 S を経由する前進→後進の1切り返し経路を必ず生成。"""
    target = (10.0, 0.0, math.pi)
    S = (20.0, 6.0, math.radians(200))  # ドック背後・側方の手動ステージ姿勢
    res = plan_spotting((0.0, 0.0, 0.0), target, rho=5.0, max_switchbacks=1, manual_switch_pose=S)
    assert res.n_switchbacks == 1
    assert any(p["gear"] == "R" for p in res.points)
    # 切り返し点が指定 S 近傍にある
    assert res.switch_points
    sx, sy = res.switch_points[0]
    assert math.hypot(sx - S[0], sy - S[1]) < 1.5


def test_spotting_cusp_has_straight_margin():
    """切り返し点の前後が同一直線（ステア0°）= 追従可能。手動Sで margin を指定して検証。"""
    target = (10.0, 0.0, math.pi)
    S = (24.0, 5.0, math.radians(190))
    margin = 5.0
    res = plan_spotting((0.0, 0.0, 0.0), target, rho=6.0, max_switchbacks=1,
                        manual_switch_pose=S, cusp_margin_m=margin)
    assert res.n_switchbacks == 1
    pts = res.points
    # cusp（gear が F→R に変わる）index を特定
    ci = next(i for i in range(1, len(pts)) if pts[i]["gear"] != pts[i - 1]["gear"])
    syaw = math.degrees(S[2])

    def straight_run(idxs):
        """cusp から経路に沿って累積距離 margin*0.7 まで、heading が Syaw に一致するか。"""
        acc, n = 0.0, 0
        prev = None
        for i in idxs:
            if prev is not None:
                acc += math.hypot(pts[i]["x"] - pts[prev]["x"], pts[i]["y"] - pts[prev]["y"])
            prev = i
            if acc > margin * 0.7:
                break
            dh = abs((pts[i]["heading_deg"] - syaw + 180) % 360 - 180)
            assert dh < 5.0, f"point {i} heading {pts[i]['heading_deg']} not straight (expect {syaw})"
            n += 1
        return n

    before = straight_run(range(ci - 1, -1, -1))   # cusp から前方(forward側)へ遡る
    after = straight_run(range(ci, len(pts)))       # cusp から後方(reverse側)へ進む
    assert before >= 2 and after >= 2  # 切り返し点の前後とも直線マージンがある
    # 診断値: 実効 cusp マージンが指定 margin に対して妥当な範囲で報告される
    assert res.min_cusp_margin_m is not None
    assert res.min_cusp_margin_m >= margin * 0.5


def test_spotting_min_cusp_margin_none_when_forward_only():
    """内部 cusp が無い（前進のみ）経路では min_cusp_margin_m=None（診断対象外）。"""
    res = plan_spotting((0.0, 0.0, 0.0), (40.0, 5.0, 0.0), rho=6.0, max_switchbacks=0)
    assert res.n_switchbacks == 0
    assert res.min_cusp_margin_m is None


def test_spotting_switchback_zone_filters_cusp():
    """切り返し可能エリア: cusp がゾーン外の候補を除外。ゾーンを切り返し点付近に置くと到達。"""
    start = (0.0, 0.0, 0.0)
    target = (10.0, 0.0, math.pi)
    # ターゲット背後に十分広いゾーン（自動候補のSはこのあたりに出る）
    zone = [(12.0, -12.0), (40.0, -12.0), (40.0, 12.0), (12.0, 12.0)]
    res = plan_spotting(start, target, rho=5.0, max_switchbacks=1, require_switchback=True,
                        switchback_zone=zone)
    assert res.n_switchbacks == 1
    for sx, sy in res.switch_points:
        assert 12.0 <= sx <= 40.0 and -12.0 <= sy <= 12.0


def test_spotting_switchback_zone_unreachable_falls_back_forward():
    """切り返し点を置けない極小ゾーン → 切り返し候補が全滅し、前進のみ候補へフォールバック。"""
    start = (0.0, 0.0, 0.0)
    target = (25.0, 8.0, 0.0)  # 前進で届く配置
    zone = [(-100.0, -100.0), (-99.0, -100.0), (-99.0, -99.0), (-100.0, -99.0)]  # 経路から遠い
    res = plan_spotting(start, target, rho=8.0, max_switchbacks=1, switchback_zone=zone)
    assert res.n_switchbacks == 0  # 切り返し不可 → 前進のみ
    assert res.approach_error_m < 0.6


def test_spotting_corridor_width_via_clearance():
    """道幅(=要求クリアランス)の挙動: 半幅3mの帯で、要求1m→feasible / 要求4m→infeasible。"""
    n = 120
    mask = np.zeros((n, n), np.uint8)
    t = Affine(0.5, 0, 0, 0, -0.5, float(n) * 0.5)  # col->x=0.5col, row->y=60-0.5row
    for r in range(n):
        y = n * 0.5 - (r + 0.5) * 0.5
        if abs(y - 15.0) <= 3.0:
            mask[r, :] = 1
    start, target = (2.0, 15.0, 0.0), (40.0, 15.0, 0.0)
    ok = plan_spotting(start, target, rho=6.0, max_switchbacks=0,
                       drivable_mask=mask, transform=t, footprint_radius=1.0)
    ng = plan_spotting(start, target, rho=6.0, max_switchbacks=0,
                       drivable_mask=mask, transform=t, footprint_radius=4.0)
    assert ok.feasible and not ng.feasible


class _Veh:
    """テスト用の最小車両（向き付きフットプリント=全長×全幅の矩形）。"""
    def __init__(self, length, width):
        self.overall_length = length
        self.overall_width = width
        self.footprint_polygon = None
        self.footprint_radius = width / 2.0


def _wide_area_mask():
    cell = 0.5
    W, H = 120, 60
    t = Affine(cell, 0, 0, 0, -cell, 30.0)
    cc = (np.arange(W) * cell + cell / 2)[None, :]
    rr = (30.0 - np.arange(H) * cell - cell / 2)[:, None]
    X = np.broadcast_to(cc, (H, W))
    Y = np.broadcast_to(rr, (H, W))
    mask = np.zeros((H, W), np.uint8)
    mask[(X >= 3) & (X <= 57) & (Y >= 2) & (Y <= 26)] = 1  # 幅24mの帯
    return mask, t


def test_spotting_oriented_footprint_small_vehicle_stays_inside():
    """1切り返し（180°反転）。小型車はフットプリント全体がエリア内に収まり feasible。"""
    mask, t = _wide_area_mask()
    veh = _Veh(6.0, 3.0)
    res = plan_spotting((6.0, 14.0, 0.0), (45.0, 14.0, math.pi), rho=5.0, max_switchbacks=1,
                        require_switchback=True, drivable_mask=mask, transform=t,
                        footprint_radius=veh.footprint_radius, vehicle=veh, step=0.4)
    assert res.status == "OK" and res.feasible
    assert res.footprint_inside is True
    assert res.fp_max_frac == 0.0
    assert res.n_switchbacks == 1
    assert res.approach_error_m < 0.6


def test_spotting_oriented_footprint_big_vehicle_detected_outside():
    """同じ狭めエリアで大型車は切り返し時に車体がはみ出す → FOOTPRINT_OUTSIDE で検出。"""
    mask, t = _wide_area_mask()
    veh = _Veh(12.0, 5.0)
    res = plan_spotting((6.0, 14.0, 0.0), (45.0, 14.0, math.pi), rho=5.0, max_switchbacks=1,
                        require_switchback=True, drivable_mask=mask, transform=t,
                        footprint_radius=veh.footprint_radius, vehicle=veh, step=0.4)
    assert not res.feasible
    assert res.footprint_inside is False
    assert res.status in ("FOOTPRINT_OUTSIDE", "INFEASIBLE_BEST_EFFORT")
    assert res.fp_max_frac > 0.0  # はみ出し率が報告される


def test_spotting_oriented_footprint_off_keeps_circular_clearance():
    """vehicle 未指定なら従来の円近似クリアランス判定（後方互換、footprint_inside は None）。"""
    mask, t = _wide_area_mask()
    res = plan_spotting((6.0, 14.0, 0.0), (20.0, 14.0, 0.0), rho=5.0, max_switchbacks=0,
                        drivable_mask=mask, transform=t, footprint_radius=1.5)
    assert res.footprint_inside is None
    assert res.feasible  # 中央を直進するので円近似でも余裕


def test_spotting_method_selection():
    """method 選択: 各手法が到達 or 適切な状態を返す。reeds_shepp は切り返しを出せる。"""
    S, T = (0.0, 0.0, 0.0), (10.0, 0.0, math.pi)  # 180度反転＝切り返し向き
    auto = plan_spotting(S, T, rho=5.0, max_switchbacks=1, method="auto")
    rs = plan_spotting(S, T, rho=5.0, max_switchbacks=1, method="reeds_shepp")
    dub = plan_spotting(S, T, rho=5.0, max_switchbacks=1, method="dubins")
    assert auto.feasible and rs.feasible and dub.feasible
    # RS は曲がりながらの後進で切り返す。据え切り回避（既定 deny）が両端直線化できる
    # 2カスプ RS 変種を選ぶことがあるため 1〜2 を許容。
    assert rs.n_switchbacks in (1, 2)
    # hybrid はマップ無しでは生成不可（明示状態）。
    hyb = plan_spotting(S, T, rho=5.0, max_switchbacks=1, method="hybrid_astar")
    assert hyb.status == "NO_HYBRID_MAP" and not hyb.feasible


def test_spotting_method_forward_only_all_reach():
    """前進のみ(0切り返し)では auto/dubins/reeds_shepp いずれも前進経路で到達。"""
    S, T = (0.0, 0.0, 0.0), (25.0, 8.0, 0.0)
    for m in ("auto", "dubins", "reeds_shepp"):
        r = plan_spotting(S, T, rho=8.0, max_switchbacks=0, method=m)
        assert r.feasible and r.n_switchbacks == 0, m


def test_spotting_endpoint_overhang_ignored():
    """固定の始点姿勢で車体が境界に少しかかるだけのケースは、端点除外で feasible になる
    （マニューバ自体は領域内＝計画の不備ではない）。"""
    cell = 0.5
    W, H = 120, 60
    t = Affine(cell, 0, 0, 0, -cell, 30.0)
    cc = (np.arange(W) * cell + cell / 2)[None, :]
    rr = (30.0 - np.arange(H) * cell - cell / 2)[:, None]
    X = np.broadcast_to(cc, (H, W))
    Y = np.broadcast_to(rr, (H, W))
    mask = np.zeros((H, W), np.uint8)
    mask[(X >= 3) & (X <= 57) & (Y >= 2) & (Y <= 24)] = 1  # 幅22m
    veh = _Veh(8.0, 3.5)
    # 始点を境界(y=2)ギリギリに置く → 車体が下にわずかにはみ出すが、それは固定姿勢由来。
    start, target = (8.0, 3.0, 0.0), (40.0, 13.0, math.pi)
    strict = plan_spotting(start, target, rho=6.0, max_switchbacks=1, require_switchback=True,
                           drivable_mask=mask, transform=t, vehicle=veh, step=0.4,
                           footprint_ignore_ends_m=0.0)
    lenient = plan_spotting(start, target, rho=6.0, max_switchbacks=1, require_switchback=True,
                            drivable_mask=mask, transform=t, vehicle=veh, step=0.4,
                            footprint_ignore_ends_m=0.6 * 8.0)
    # 端点除外ありの方が、はみ出し率は小さい（端点の固定オーバーハングを数えない）。
    assert lenient.fp_max_frac <= strict.fp_max_frac


def test_spotting_best_effort_minimizes_overhang():
    """完全に収まる候補が無いとき、best-effort は『はみ出し率最小』の候補を返す。

    注: 幅13mの旧シナリオは hybrid の footprint 対応（多点ターン）で解けるようになったため、
    車体対角(8.7m)に対し回転余地の無い幅9.5mへ狭めて「真に不可」を維持する。
    """
    cell = 0.5
    W, H = 100, 36
    t = Affine(cell, 0, 0, 0, -cell, 18.0)
    cc = (np.arange(W) * cell + cell / 2)[None, :]
    rr = (18.0 - np.arange(H) * cell - cell / 2)[:, None]
    X = np.broadcast_to(cc, (H, W))
    Y = np.broadcast_to(rr, (H, W))
    mask = np.zeros((H, W), np.uint8)
    mask[(X >= 2) & (X <= 38) & (Y >= 2) & (Y <= 11.5)] = 1  # 幅9.5m: 8×3.5m車は反転不可
    veh = _Veh(8.0, 3.5)
    r = plan_spotting((6.0, 6.75, 0.0), (33.0, 6.75, math.pi), rho=6.0, max_switchbacks=1,
                      require_switchback=True, drivable_mask=mask, transform=t, vehicle=veh, step=0.4,
                      footprint_ignore_ends_m=0.6 * 8.0)
    assert not r.feasible
    assert r.status == "FOOTPRINT_OUTSIDE"
    assert 0.0 < r.fp_max_frac < 1.0  # 最小限のはみ出しに抑えられている


def test_spotting_smoothing_preserves_endpoints_and_feasibility():
    """平滑化ON/OFFで feasible・到達精度・はみ出しが悪化しない（安全側＝悪化時は不採用）。"""
    cell = 0.5
    W, H = 160, 160
    t = Affine(cell, 0, -20.0, 0, -cell, 40.0)
    mask = np.ones((H, W), np.uint8)
    veh = _Veh(8.0, 3.5)
    start, target = (0.0, 0.0, 0.0), (10.0, 4.0, math.pi)
    off = plan_spotting(start, target, rho=8.0, max_switchbacks=1, drivable_mask=mask, transform=t,
                        vehicle=veh, step=0.4, smooth_path=False)
    on = plan_spotting(start, target, rho=8.0, max_switchbacks=1, drivable_mask=mask, transform=t,
                       vehicle=veh, step=0.4, smooth_path=True)
    assert on.feasible == off.feasible
    assert on.approach_error_m <= off.approach_error_m + 0.05
    assert on.fp_max_frac <= off.fp_max_frac + 1e-6
    # 端点（始点・終点）は不変（固定）。
    assert abs(on.points[0]["x"] - off.points[0]["x"]) < 0.2
    assert abs(on.points[-1]["x"] - off.points[-1]["x"]) < 0.2


def test_spotting_smoothing_reduces_curvature_rate():
    """平滑化で gear 区間内のピーク dκ/ds（速度律速点）が悪化しない（多くは低下）。"""
    from planning_core.analysis.curvature import curvature_profile
    from planning_core.analysis.trajectory import _robust_dkappa

    def peak_dk(res):
        xy = np.array([[p["x"], p["y"]] for p in res.points], float)
        pr = curvature_profile(xy)
        # cusp 反転点の生スパイクを避けるため、95パーセンタイルで評価。
        dk = np.abs(_robust_dkappa(pr["kappa"], pr["s"]))
        return float(np.percentile(dk, 90)) if len(dk) else 0.0

    off = plan_spotting((0, 0, 0), (12, 5, math.pi / 2), rho=6.0, max_switchbacks=1, smooth_path=False)
    on = plan_spotting((0, 0, 0), (12, 5, math.pi / 2), rho=6.0, max_switchbacks=1, smooth_path=True)
    assert peak_dk(on) <= peak_dk(off) + 1e-6


def test_spotting_cusp_yaw_continuous_with_smoothing():
    """cusp(前進終了→後進開始)で車体ヨー角が連続すること（直進マージンをロックして保持）。
    平滑化が cusp 直前の直進区間を曲げると前後でヨーが食い違って飛ぶ＝それを防ぐ回帰テスト。"""
    for tgt in [(10.0, 4.0, math.pi), (8.0, 0.0, math.pi)]:
        res = plan_spotting((0, 0, 0), tgt, rho=5.0, max_switchbacks=1,
                            require_switchback=True, smooth_path=True)
        pts = res.points
        cusps = [i for i in range(1, len(pts)) if pts[i]["gear"] != pts[i - 1]["gear"]]
        assert cusps, "切り返しが生成されていない"
        for c in cusps:
            a, b = pts[c - 1]["heading_deg"], pts[c]["heading_deg"]
            jump = abs((b - a + 180) % 360 - 180)
            # 直線マージンが入った cusp は連続（<1°）。狭所でマージン0の cusp は円弧接合の
            # 離散接線差（~κ·step ≈ 2〜4°、停止点なので無害）が残るため 5° まで許容。
            # 平滑化がマージンを曲げる回帰（~19°ジャンプ）はこの閾値でも検出できる。
            assert jump < 5.0, f"cusp idx={c} でヨー角が {jump:.1f}deg 飛んでいる"


def _endpoint_curvature(res):
    """経路の始端・終端近傍の曲率（折れ角/距離）。据え切り判定用。"""
    P = np.array([(p["x"], p["y"]) for p in res.points], float)

    def k(i):
        a, b, c = P[i], P[i + 1], P[i + 2]
        v1, v2 = b - a, c - b
        ang = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])
        ang = (ang + math.pi) % (2 * math.pi) - math.pi
        return abs(ang) / max((np.hypot(*v1) + np.hypot(*v2)) / 2, 1e-6)

    return k(0), k(len(P) - 3)


def test_endpoint_margins_avoid_stationary_steer_when_disallowed():
    """据え切り禁止: 出発・到着の端点曲率がほぼ0（直線リードイン/アウト）。"""
    start, target, rho = (0.0, 0.0, 0.0), (18.0, 10.0, math.radians(90)), 8.0
    res = plan_spotting(start, target, rho, max_switchbacks=1, allow_stationary_steer=False)
    assert res.points
    ks, kg = _endpoint_curvature(res)
    assert ks < 0.02 and kg < 0.02, (ks, kg)


def test_endpoint_margins_apply_on_reverse_approach_switchback():
    """切り返し必須(後進差し込み)でも出発・到着とも端点曲率≈0（短い後進区間はマージン自動短縮）。"""
    start, target, rho = (0.0, 0.0, 0.0), (14.0, 6.0, math.radians(90)), 8.885
    res = plan_spotting(start, target, rho, max_switchbacks=1, require_switchback=True,
                        allow_stationary_steer=False)
    assert res.points and res.n_switchbacks >= 1
    ks, kg = _endpoint_curvature(res)
    assert ks < 0.02 and kg < 0.02, (ks, kg)


def test_endpoint_stationary_steer_allowed_option():
    """据え切り許可(オプション): 端点に円弧が残る（曲率≈1/R）。"""
    start, target, rho = (0.0, 0.0, 0.0), (18.0, 10.0, math.radians(90)), 8.0
    res = plan_spotting(start, target, rho, max_switchbacks=1, allow_stationary_steer=True)
    assert res.points
    ks, kg = _endpoint_curvature(res)
    assert ks > 0.5 / rho or kg > 0.5 / rho, (ks, kg)


def test_spotting_narrow_corridor_multipoint_turn():
    """狭いコリドー（帯）でも解ける: HM400 相当が幅18mの帯内で180°反転して寄り付く。

    hybrid A* を footprint 対応＋細格子リトライ化し、カスプ直線マージンを「収まらないなら挿入
    しない」へ倒した回帰。従来は幅32mでも FOOTPRINT_OUTSIDE で全滅していた。
    """
    from rasterio.transform import from_origin

    from planning_core.vehicle.profiles import load_builtin

    veh = load_builtin("HM400")
    W, L, cell, pad = 18.0, 80.0, 0.5, 8.0
    h = int(round((W + 2 * pad) / cell))
    wpx = int(round((L + 2 * pad) / cell))
    t = from_origin(-pad, W + pad, cell, cell)
    mask = np.zeros((h, wpx), np.uint8)
    r0, r1 = int(round(pad / cell)), int(round((pad + W) / cell))
    c0, c1 = int(round(pad / cell)), int(round((pad + L) / cell))
    mask[r0:r1, c0:c1] = 1

    res = plan_spotting(
        (10.0, W * 0.5, 0.0), (18.0, W * 0.5, math.pi), rho=veh.min_turning_radius,
        max_switchbacks=1, drivable_mask=mask, transform=t, vehicle=veh,
        footprint_ignore_ends_m=veh.overall_length * 0.6,
    )
    assert res.feasible and res.status == "OK", (res.status, res.fp_max_frac)
    assert res.footprint_inside is True
    assert res.n_switchbacks >= 2  # 狭所は多点ターン（複数切り返し）で解く
    assert res.approach_error_m < 0.5


def test_spotting_stationary_deny_differs_and_has_no_loops():
    """据え切り 禁止/許可 で経路が変わる回帰（ユーザー報告「どちらでも同じパス」対応）。

    横向きターゲット＝端点で円弧が要る配置。禁止では両端に直線リードが入り、
    リード区間の方位ドリフトは ≤2°。旧実装の後付けスプライスはρループを作り
    経路長が ~4.7倍に膨れていた → 1.6倍以内であることも検証。
    """
    from planning_core.vehicle.profiles import load_builtin

    veh = load_builtin("HM400")
    start, target = (0.0, 0.0, 0.0), (25.0, 12.0, math.radians(90))
    allow = plan_spotting(start, target, rho=veh.min_turning_radius, max_switchbacks=1,
                          vehicle=veh, allow_stationary_steer=True)
    deny = plan_spotting(start, target, rho=veh.min_turning_radius, max_switchbacks=1,
                         vehicle=veh, allow_stationary_steer=False)
    assert allow.feasible and deny.feasible
    assert deny.allow_stationary is False and allow.allow_stationary is True
    assert deny.endpoint_margin_start_m > 0 and deny.endpoint_margin_goal_m > 0
    assert deny.length_total < allow.length_total * 1.6  # ρループの大回りを作らない

    # リード区間（端点直線）内は方位一定 = 停止中の据え切り不要
    pts = deny.points
    s = [p["s"] for p in pts]

    def max_drift(idxs, lead):
        h0 = pts[idxs[0]]["heading_deg"]
        base = s[idxs[0]]
        mx = 0.0
        for i in idxs:
            if abs(s[i] - base) > lead - 0.2:
                break
            mx = max(mx, abs((pts[i]["heading_deg"] - h0 + 180) % 360 - 180))
        return mx

    assert max_drift(range(len(pts)), deny.endpoint_margin_start_m) <= 2.0
    assert max_drift(range(len(pts) - 1, -1, -1), deny.endpoint_margin_goal_m) <= 2.0


def test_spotting_stage_candidates_reach_far_lateral_zone():
    """横・遠方に離れた切り返しゾーンにも S 候補が置かれる（遠方粗シェルの検証）。

    ゾーン中心は目標前方 6ρ × 横 2.5ρ ＝ 旧候補域（密格子 横±1.5ρ／リング距離4ρ）の外側。
    旧実装ではゾーン内に cusp を持つ候補が生成されず NO_PATH になっていた。
    """
    rho = 6.0
    start = (0.0, 0.0, 0.0)
    target = (10.0, 0.0, 0.0)
    cx, cy = 10.0 + 6.0 * rho, 2.5 * rho  # (46, 15)
    zone = [(cx - 4.0, cy - 4.0), (cx + 5.0, cy - 4.0), (cx + 5.0, cy + 5.0), (cx - 4.0, cy + 5.0)]
    res = plan_spotting(start, target, rho=rho, max_switchbacks=1, require_switchback=True,
                        switchback_zone=zone)
    assert res.n_switchbacks == 1, f"far-lateral zone should be reachable (status={res.status})"
    for sx, sy in res.switch_points:
        assert cx - 4.0 <= sx <= cx + 5.0 and cy - 4.0 <= sy <= cy + 5.0
