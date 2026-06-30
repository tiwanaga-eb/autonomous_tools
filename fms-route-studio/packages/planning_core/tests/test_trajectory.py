import numpy as np

from planning_core.analysis import build_trajectory, summarize
from planning_core.vehicle import load_builtin


def test_build_trajectory_circle():
    R = 25.0
    th = np.linspace(0.0, np.pi, 60)
    pts = np.column_stack([R * np.cos(th), R * np.sin(th)])
    traj = build_trajectory(pts)
    assert len(traj.points) == 60
    assert traj.length_m > 0
    assert abs(traj.min_radius_m - R) < 1.0
    # 各点に κ と dκ/ds が入る
    assert all(p.curvature >= 0 for p in traj.points)


def test_steer_only_for_rigid_bicycle():
    pts = np.column_stack([np.linspace(0, 30, 40), 5 * np.sin(np.linspace(0, 3, 40))])
    hd785 = load_builtin("HD785")
    cd110 = load_builtin("CD110R")
    traj_b = build_trajectory(pts, vehicle=hd785)
    traj_s = build_trajectory(pts, vehicle=cd110)
    assert any(p.steer_deg is not None for p in traj_b.points)   # 自転車 -> 操舵角あり
    assert all(p.steer_deg is None for p in traj_s.points)       # スキッド -> None


def test_summarize_skid_marks_not_applicable():
    pts = np.column_stack([np.linspace(0, 10, 20), np.zeros(20)])
    cd110 = load_builtin("CD110R")
    res = summarize(build_trajectory(pts, vehicle=cd110), vehicle=cd110)
    assert "min_radius" in res.not_applicable


def test_summarize_min_radius_violation():
    # HD785: min_turning_radius=10.1m。R=6m の円弧は κ 上限超過 → violation & infeasible。
    R = 6.0
    th = np.linspace(0.0, np.pi / 2, 80)
    pts = np.column_stack([R * np.cos(th), R * np.sin(th)])
    hd785 = load_builtin("HD785")
    res = summarize(build_trajectory(pts, vehicle=hd785), vehicle=hd785)
    assert not res.feasible
    assert any(v.kind == "min_radius" for v in res.violations)


def test_summarize_feasible_straight():
    pts = np.column_stack([np.linspace(0, 100, 50), np.zeros(50)])
    hd785 = load_builtin("HD785")
    res = summarize(build_trajectory(pts, vehicle=hd785), vehicle=hd785)
    assert res.feasible
    assert res.violations == []


def test_grade_flows_into_analysis():
    pts = np.column_stack([np.linspace(0, 50, 40), np.zeros(40)])
    grade = np.full(40, 7.5)
    traj = build_trajectory(pts, grade_pct=grade)
    assert all(p.grade_pct is not None for p in traj.points)
    res = summarize(traj)
    assert res.max_grade_pct is not None and abs(res.max_grade_pct - 7.5) < 1e-6


def test_cusp_heading_uses_body_yaw_not_degenerate():
    """切り返し点(cusp)では前後の隣接点が重複し中心差分が(0,0)に退化して見かけ上 0°(東)を
    向く回帰があった。ギア区間ごとの片側差分で、cusp 点でも進入方位を保つことを確認する。"""
    import math
    ux, uy = math.cos(math.radians(60)), math.sin(math.radians(60))  # 進入方位60°（東でない）
    step = 0.5
    fwd = [(d * ux, d * uy) for d in np.arange(0.0, 10.0 + 1e-9, step)]
    rev = [(d * ux, d * uy) for d in np.arange(10.0 - step, 5.0 - 1e-9, -step)]
    pts = fwd + rev
    gears = ["F"] * len(fwd) + ["R"] * len(rev)
    cusp_i = len(fwd) - 1
    traj = build_trajectory(pts, gears=gears)
    # cusp 点の車体方位は進入方位(60°)であるべき（退化 0° ではない）
    assert abs(traj.points[cusp_i].heading_deg - 60.0) < 1e-6
    # 後進区間も車体方位は前進と同じ向き（180°反転して進行方向の逆＝60°）
    assert abs(traj.points[cusp_i + 1].heading_deg - 60.0) < 1e-6
