"""orchestrator.plan_route / analyze_polyline の HTTP 非依存テスト（A1 コア抽出の検証）。

FastAPI/レイヤ IO を介さず、配列＋transform を直接渡して経路生成パイプライン全体
（探索→R_min 保証→勾配/標高→解析→安全検証）を回せることを確認する。
"""
import numpy as np
import pytest
from affine import Affine

from planning_core.orchestrator import PlanError, PlanSpec, analyze_polyline, plan_route


def _ramp(n=60, slope=0.2):
    t = Affine(1.0, 0.0, 0.0, 0.0, -1.0, float(n))
    z = np.zeros((n, n), float)
    for r in range(n):
        for c in range(n):
            x, _y = t * (c + 0.5, r + 0.5)
            z[r, c] = slope * x
    return z, t


def test_plan_route_spline_smoke():
    spec = PlanSpec(
        waypoints=np.array([[0.0, 0.0], [40.0, 10.0], [80.0, 0.0]]),
        headings_deg=[None, None, None],
        algorithm="spline", r_min=10.0, spacing_m=2.0,
    )
    out = plan_route(spec)
    assert out.algorithm == "spline"
    assert len(out.trajectory.points) >= 2
    assert out.measured_min_radius_m is None or out.measured_min_radius_m > 0
    # 安全検証は fail-closed（車両・走行可能領域なし → 実評価チェック皆無 → passed=False）
    assert out.safety.passed is False


def test_plan_route_needs_two_waypoints():
    with pytest.raises(PlanError) as ei:
        plan_route(PlanSpec(waypoints=np.zeros((1, 2)), headings_deg=[None]))
    assert ei.value.status == 400


def test_plan_route_grid_astar_requires_cost():
    with pytest.raises(PlanError) as ei:
        plan_route(PlanSpec(
            waypoints=np.array([[0.0, 0.0], [10.0, 0.0]]),
            headings_deg=[None, None], algorithm="grid_astar",
        ))
    assert ei.value.status == 400


def test_plan_route_grid_astar_embeds_z_and_grade():
    dsm, t = _ramp(slope=0.2)
    cost = np.zeros(dsm.shape, float)
    spec = PlanSpec(
        waypoints=np.array([[5.0, 30.0], [50.0, 30.0]]),
        headings_deg=[None, None],
        mode="auto", algorithm="grid_astar",
        cost=cost, cost_transform=t, obstacle_value=1e9,
        dsm=dsm, dsm_transform=t, spacing_m=2.0,
    )
    out = plan_route(spec)
    withz = [p for p in out.trajectory.points if p.z is not None]
    assert withz, "DSM 供給時は z が埋め込まれる"
    for p in withz:
        assert abs(p.z - 0.2 * p.x) < 1.0
    assert out.analysis.max_grade_pct is not None


def test_plan_route_misregistered_mask_rejected():
    dsm, t1 = _ramp()
    t2 = Affine(1.0, 0.0, 500.0, 0.0, -1.0, 560.0)  # 同サイズ・別位置
    cost = np.zeros(dsm.shape, float)
    mask = np.ones(dsm.shape, np.uint8)
    with pytest.raises(PlanError) as ei:
        plan_route(PlanSpec(
            waypoints=np.array([[5.0, 30.0], [50.0, 30.0]]),
            headings_deg=[None, None], mode="auto", algorithm="grid_astar",
            cost=cost, cost_transform=t1, mask=mask, mask_transform=t2,
        ))
    assert ei.value.status == 422


def test_analyze_polyline_attaches_z_grade_and_failclosed_safety():
    dsm, t = _ramp(slope=0.1)
    xs = np.linspace(5.0, 50.0, 24)
    pts = np.column_stack([xs, np.full_like(xs, 30.0)])
    traj, analysis, safety, clearance, warn = analyze_polyline(pts, dsm=dsm, dsm_transform=t)
    assert warn is None and clearance is None
    assert all(p.z is not None for p in traj.points)
    assert analysis.max_grade_pct == pytest.approx(10.0, abs=0.5)
    assert safety.passed is False  # 車両なし・走行可能領域なし → fail-closed


def test_plan_route_skid_vehicle_gets_approximation_note():
    """スキッドステア車（CD110R）は Ackermann 近似で計画される旨を warning に明示する。"""
    from planning_core.vehicle import load_builtin

    spec = PlanSpec(
        waypoints=np.array([[0.0, 0.0], [40.0, 10.0], [80.0, 0.0]]),
        headings_deg=[None, None, None],
        algorithm="spline", spacing_m=2.0,
        vehicle=load_builtin("CD110R"),
    )
    out = plan_route(spec)
    assert out.warning is not None and "スキッドステア" in out.warning
    # 通常車（HD785）には付かない
    spec2 = PlanSpec(
        waypoints=np.array([[0.0, 0.0], [40.0, 10.0], [80.0, 0.0]]),
        headings_deg=[None, None, None],
        algorithm="spline", r_min=15.0, spacing_m=2.0,
        vehicle=load_builtin("HD785"),
    )
    out2 = plan_route(spec2)
    assert out2.warning is None or "スキッドステア" not in out2.warning
