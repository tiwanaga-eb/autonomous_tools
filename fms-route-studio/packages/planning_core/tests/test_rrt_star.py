import math

import numpy as np
from affine import Affine

from planning_core.planners.rrt_star import rrt_star, sample_rrt_star


def _reaches(pts, goal, postol=1.0, yawtol_deg=15.0):
    ex, ey, eyaw = pts[-1][0], pts[-1][1], pts[-1][2]
    dyaw = abs((eyaw - goal[2] + math.pi) % (2 * math.pi) - math.pi)
    return math.hypot(ex - goal[0], ey - goal[1]) <= postol and dyaw <= math.radians(yawtol_deg)


def test_rrt_star_reaches_goal_open_space():
    start, goal = (0.0, 0.0, 0.0), (20.0, 6.0, 0.0)
    r = rrt_star(start, goal, rho=5.0, max_iters=500, seed=1)
    assert r is not None and r["status"] == "OK"
    assert _reaches(r["points"], goal)
    assert r["length"] > 0


def test_rrt_star_uturn_with_reverse():
    # 背面ゴール（U字）。前進＋後進の切り返しで到達できる。
    start, goal = (0.0, 0.0, 0.0), (6.0, 0.0, math.pi)
    pts = sample_rrt_star(start, goal, rho=4.0, max_iters=800, seed=3)
    assert pts is not None
    assert _reaches(pts, goal)


def test_rrt_star_deterministic_same_seed():
    start, goal = (0.0, 0.0, 0.0), (18.0, -5.0, math.radians(30))
    a = rrt_star(start, goal, rho=5.0, max_iters=400, seed=7)
    b = rrt_star(start, goal, rho=5.0, max_iters=400, seed=7)
    assert a is not None and b is not None
    assert abs(a["length"] - b["length"]) < 1e-9
    assert a["n_nodes"] == b["n_nodes"]


def test_rrt_star_respects_mask_corridor():
    # 横帯のみ走行可能。経路は帯内に収まる（mask外を踏まない）。
    n = 120
    mask = np.zeros((n, n), np.uint8)
    t = Affine(0.5, 0, 0, 0, -0.5, float(n) * 0.5)
    for r in range(n):
        y = n * 0.5 - (r + 0.5) * 0.5
        if abs(y - 15.0) <= 4.0:
            mask[r, :] = 1
    start, goal = (3.0, 15.0, 0.0), (45.0, 15.0, 0.0)
    res = rrt_star(start, goal, rho=5.0, mask=mask, transform=t, max_iters=600, seed=2)
    assert res is not None
    # 全サンプルが mask 内
    for x, y, _yaw, _g in res["points"]:
        col, row = ~t * (x, y)
        rr, cc = int(math.floor(row)), int(math.floor(col))
        assert 0 <= rr < n and 0 <= cc < n and mask[rr, cc] == 1
