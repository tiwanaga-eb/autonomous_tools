import numpy as np

from planning_core.analysis import build_trajectory, summarize
from planning_core.analysis.curvature import cusp_mask
from planning_core.models.vehicle import VehicleProfile


def test_cusp_mask_detects_reversal_and_not_straight():
    # 前進(+x)→折返して後進(-x,+y) の V 字
    pts = np.array([[i, 0.0] for i in range(0, 6)] + [[5 - i, i * 0.3] for i in range(1, 6)], float)
    m = cusp_mask(pts)
    assert m.any() and m[5]  # 折返点(idx5)を検出
    assert not cusp_mask(np.array([[0, 0], [1, 0], [2, 0], [3, 0]], float)).any()  # 直線はなし


def test_summarize_excludes_cusp_from_min_radius():
    veh = VehicleProfile(id="V", name="v", kinematic_type="rigid_bicycle", overall_length=10.0,
                         overall_width=5.0, overall_height=4.0, wheel_base=4.0, max_steer_angle=0.7,
                         min_turning_radius=10.0, kappa_max_fwd=0.1)
    fwd = [[i * 1.0, 0.0] for i in range(0, 12)]
    rev = [[11 - i, i * 0.3] for i in range(1, 12)]  # 折返して後進（角度つき）
    traj = build_trajectory(np.array(fwd + rev, float), vehicle=veh)
    res = summarize(traj, vehicle=veh)
    # cusp 除外で min_radius が見かけ上ゼロにならず、min_radius 違反も出ない
    assert traj.min_radius_m is None or traj.min_radius_m > 2.0
    assert not any(v.kind == "min_radius" for v in res.violations)
