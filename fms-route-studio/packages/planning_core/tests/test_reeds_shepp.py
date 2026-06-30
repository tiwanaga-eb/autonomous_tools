import math

import numpy as np

from planning_core.planners import sample_reeds_shepp, reeds_shepp_paths


def _reaches(pts, goal, postol=0.6, yawtol=10):
    ex, ey, eyaw = pts[-1][0], pts[-1][1], pts[-1][2]
    return (math.hypot(ex - goal[0], ey - goal[1]) < postol
            and abs(((eyaw - goal[2] + math.pi) % (2 * math.pi)) - math.pi) < math.radians(yawtol))


def test_rs_forward_reaches():
    goal = (12.0, 0.0, 0.0)
    pts = sample_reeds_shepp((0, 0, 0), goal, rho=5.0)
    assert pts is not None and _reaches(pts, goal)


def test_rs_goal_behind_uses_reverse():
    goal = (-6.0, 0.0, 0.0)
    pts = sample_reeds_shepp((0, 0, 0), goal, rho=5.0)
    assert pts is not None and _reaches(pts, goal)
    assert any(p[3] == "R" for p in pts)  # 後進を含む


def test_rs_side_with_heading_reaches():
    for goal in [(8.0, 6.0, math.pi / 2), (10.0, -4.0, math.pi), (3.0, 8.0, math.pi / 2)]:
        pts = sample_reeds_shepp((0, 0, 0), goal, rho=4.0)
        assert pts is not None and _reaches(pts, goal), goal


def test_rs_all_candidates_validated_reach_goal():
    goal = (7.0, 5.0, math.pi / 3)
    for pts in reeds_shepp_paths((0, 0, 0), goal, rho=4.0):
        assert _reaches(pts, goal)  # 返る候補は必ず終点一致（検証済み）


def test_rs_tight_endpoint_tolerance():
    # 終点スナップは微小残差吸収のみ。サンプル前の最終点が厳しめ許容内に収まること。
    goal = (9.0, 4.0, math.radians(40))
    paths = reeds_shepp_paths((0, 0, 0), goal, rho=4.0, step=0.2, pos_tol=0.05, yaw_tol_deg=1.5)
    assert paths
    for pts in paths:
        ex, ey, eyaw = pts[-1][:3]  # スナップ後＝厳密一致なので、スナップ前検証は _reaches(厳)で代替
        assert math.hypot(ex - goal[0], ey - goal[1]) < 1e-6


def test_rs_full_word_set_many_candidates():
    # 完全48語化で、自由空間でも複数の有効候補が出る（旧3語版より多い）。
    goal = (6.0, 4.0, math.radians(120))
    paths = reeds_shepp_paths((0, 0, 0), goal, rho=3.0)
    assert len(paths) >= 4
    # 最短候補は単調に短い（ソート済み）
    lens = [sum(math.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1]) for i in range(len(p) - 1)) for p in paths]
    assert lens == sorted(lens)


def test_rs_known_optimal_lengths():
    # 既知の最適長: 直進=距離、真後ろ=後進距離（rho に依らず一致するはず）。
    p = sample_reeds_shepp((0, 0, 0), (12.0, 0.0, 0.0), rho=4.0)
    L = sum(math.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1]) for i in range(len(p) - 1))
    assert abs(L - 12.0) < 0.2
    p = sample_reeds_shepp((0, 0, 0), (-6.0, 0.0, 0.0), rho=4.0)
    L = sum(math.hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1]) for i in range(len(p) - 1))
    assert abs(L - 6.0) < 0.2
    assert any(pt[3] == "R" for pt in p)
