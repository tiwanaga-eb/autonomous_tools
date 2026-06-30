"""フットプリント包含のテスト（実車体矩形 vs 円近似）。"""
import numpy as np
from affine import Affine

from planning_core.analysis import build_trajectory, summarize
from planning_core.footprint import (
    footprint_clear,
    footprint_sample_points,
    trajectory_footprint_violations,
    vehicle_footprint,
)
from planning_core.models.vehicle import VehicleProfile
from planning_core.planners import hybrid_astar


# col→x, row→y=H-row（北上）。cell=1m。
H, W = 20, 60
TRANSFORM = Affine(1.0, 0.0, 0.0, 0.0, -1.0, float(H))


def _corridor_mask(half_width: float) -> np.ndarray:
    """y=10 を中心に半幅 half_width の水平コリドーを 1、外を 0 とする mask。"""
    m = np.zeros((H, W), dtype=np.uint8)
    for r in range(H):
        y = H - r - 0.5
        if abs(y - 10.0) <= half_width:
            m[r, :] = 1
    return m


def _veh(width: float, length: float = 4.0) -> VehicleProfile:
    hl, hw = length / 2.0, width / 2.0
    return VehicleProfile(
        id="T", name="test", kinematic_type="tracked_skid",
        overall_length=length, overall_width=width, overall_height=2.0,
        min_turning_radius=0.0,
        footprint_polygon=[(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)],
    )


def test_vehicle_footprint_uses_polygon_else_rectangle():
    v = _veh(4.0)
    fp = vehicle_footprint(v)
    assert fp.shape == (4, 2)
    # polygon 未定義なら overall から矩形
    v2 = VehicleProfile(id="R", name="r", kinematic_type="tracked_skid",
                        overall_length=6.0, overall_width=3.0, overall_height=2.0)
    fp2 = vehicle_footprint(v2)
    assert fp2.shape == (4, 2)
    assert np.isclose(np.abs(fp2[:, 0]).max(), 3.0) and np.isclose(np.abs(fp2[:, 1]).max(), 1.5)


def test_footprint_clear_inside_vs_edge():
    mask = _corridor_mask(4.0)  # y in [6,14]
    inv = (~TRANSFORM).a, (~TRANSFORM).b, (~TRANSFORM).c, (~TRANSFORM).d, (~TRANSFORM).e, (~TRANSFORM).f
    samp = footprint_sample_points(vehicle_footprint(_veh(4.0)), 1.0)
    # 中心 y=10, yaw=0 → 車体 y∈[8,12] ⊂ コリドー → clear
    assert footprint_clear(samp, 30.0, 10.0, 0.0, mask, inv)
    # 中心 y=13 → 車体 y∈[11,15], 15 はコリドー外(>14) → not clear
    assert not footprint_clear(samp, 30.0, 13.0, 0.0, mask, inv)


def test_trajectory_footprint_violation_reported():
    mask = _corridor_mask(4.0)
    veh = _veh(4.0)
    # コリドー中心を通る軌跡 → 包含 violation 無し
    pts_in = np.array([[x, 10.0] for x in range(5, 56, 2)], float)
    res_in = summarize(build_trajectory(pts_in, vehicle=veh), vehicle=veh, drivable_mask=mask, transform=TRANSFORM)
    assert not any(v.kind == "footprint" for v in res_in.violations)
    # コリドー端 y=13 を通る軌跡 → 車体が外へはみ出す → footprint violation
    pts_edge = np.array([[x, 13.0] for x in range(5, 56, 2)], float)
    res_edge = summarize(build_trajectory(pts_edge, vehicle=veh), vehicle=veh, drivable_mask=mask, transform=TRANSFORM)
    fvs = [v for v in res_edge.violations if v.kind == "footprint"]
    assert fvs and not res_edge.feasible
    assert fvs[0].measured > 0.0


def test_trajectory_violations_helper_direct():
    mask = _corridor_mask(1.0)  # 幅2 の細いコリドー
    veh = _veh(4.0)             # 車体幅4 → どこでもはみ出す
    pts = np.array([[x, 10.0] for x in range(5, 56, 2)], float)
    traj = build_trajectory(pts, vehicle=veh)
    vios = trajectory_footprint_violations(traj, veh, mask, TRANSFORM)
    assert vios and vios[0].kind == "footprint"


def test_hybrid_astar_footprint_blocks_narrow_corridor():
    """車体幅より細いコリドー: footprint 有効なら経路なし、中心点判定なら経路あり。"""
    start = (5.0, 10.0, 0.0)
    goal = (54.0, 10.0, 0.0)
    rho = 2.0
    samp = footprint_sample_points(vehicle_footprint(_veh(4.0)), 1.0)

    # 幅8コリドー(半幅4): 車体幅4 は収まる → footprint 有効でも経路あり
    wide = _corridor_mask(4.0)
    r_wide = hybrid_astar(start, goal, rho=rho, mask=wide, transform=TRANSFORM,
                          footprint=samp, xy_res=1.0, pos_tol=2.5, analytic_radius=14.0)
    assert r_wide is not None and r_wide["status"] == "OK"

    # 幅2コリドー(半幅1): 車体幅4 ははみ出す → footprint 有効なら経路なし
    narrow = _corridor_mask(1.0)
    r_fp = hybrid_astar(start, goal, rho=rho, mask=narrow, transform=TRANSFORM,
                        footprint=samp, xy_res=1.0, pos_tol=2.5, analytic_radius=14.0)
    assert r_fp is None

    # 同じ細コリドーでも footprint 無し(中心点判定)なら経路あり（=円/点近似の限界）
    r_pt = hybrid_astar(start, goal, rho=rho, mask=narrow, transform=TRANSFORM,
                        xy_res=1.0, pos_tol=2.5, analytic_radius=14.0)
    assert r_pt is not None


def test_footprint_ignores_endpoint_overhang_but_flags_midroute():
    from planning_core.analysis import build_trajectory, summarize
    mask = _corridor_mask(6.0)            # y∈[4,16]
    veh = _veh(4.0, 12.0)                 # 全長12 → 半長6ぶん端点オーバーハング
    pts = np.array([[x, 10.0] for x in range(2, 59, 2)], float)  # 端 x=2/58 で車体が x<0 / >60 へはみ出す
    res = summarize(build_trajectory(pts, vehicle=veh), vehicle=veh, drivable_mask=mask, transform=TRANSFORM)
    assert not any(v.kind == "footprint" for v in res.violations)  # 端点オーバーハングは除外
    # 中間に走行不可の穴 → そこは検出される
    mask2 = mask.copy()
    mask2[7:13, 28:33] = 0
    res2 = summarize(build_trajectory(pts, vehicle=veh), vehicle=veh, drivable_mask=mask2, transform=TRANSFORM)
    assert any(v.kind == "footprint" for v in res2.violations)
