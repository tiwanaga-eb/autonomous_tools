import math

import numpy as np
from affine import Affine

from planning_core.analysis import min_turning_radius
from planning_core.planners import hybrid_astar


def _curv_max(points):
    xy = np.array([[p[0], p[1]] for p in points], float)
    r = min_turning_radius(xy)
    return 1.0 / r if r and math.isfinite(r) else 0.0


def test_hybrid_freespace_straight():
    res = hybrid_astar((0.0, 0.0, 0.0), (30.0, 0.0, 0.0), rho=8.0, xy_res=1.0)
    assert res is not None and res["status"] == "OK"
    p = res["points"]
    assert abs(p[-1][0] - 30.0) < 2.0 and abs(p[-1][1] - 0.0) < 2.0
    assert abs(math.degrees(p[-1][2])) < 20  # ゴール方位~0
    assert all(g[3] == "F" for g in p)


def test_hybrid_respects_min_radius_on_turn():
    rho = 8.0
    res = hybrid_astar((0.0, 0.0, 0.0), (25.0, 20.0, math.pi / 2), rho=rho, xy_res=1.0)
    assert res is not None
    p = res["points"]
    # 終端姿勢が概ね北向き
    assert abs(_wrap(p[-1][2] - math.pi / 2)) < math.radians(25)
    # R_min を大きく割り込まない（解析接続込みで余裕）
    assert _curv_max(p) <= 1.0 / rho * 1.3


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def test_hybrid_routes_in_mask():
    # 縦壁を避けて通る（mask で通行可否）
    n = 60
    mask = np.ones((n, n), np.uint8)
    mask[10:50, 28:32] = 0  # 壁
    mask[28:32, 28:32] = 1  # 隙間
    t = Affine(1.0, 0, 0, 0, -1.0, float(n))
    # world: x=col, y=n-row。start左、goal右、同じ y
    start = (5.0, float(n) - 30.0, 0.0)
    goal = (55.0, float(n) - 30.0, 0.0)
    res = hybrid_astar(start, goal, rho=5.0, mask=mask, transform=t, xy_res=1.0, pos_tol=2.5, analytic_radius=6.0)
    assert res is not None
    # 全点が mask 内
    for x, y, _, _ in res["points"]:
        c = int(math.floor(x))
        r = int(math.floor(float(n) - y))
        assert 0 <= r < n and 0 <= c < n and mask[r, c] == 1


def test_hybrid_returns_none_when_blocked():
    n = 30
    mask = np.zeros((n, n), np.uint8)
    mask[0:3, 0:3] = 1  # 行き止まり小領域
    t = Affine(1.0, 0, 0, 0, -1.0, float(n))
    res = hybrid_astar((1.0, float(n) - 1.0, 0.0), (25.0, 5.0, 0.0), rho=5.0, mask=mask, transform=t,
                       xy_res=1.0, max_iters=20000)
    assert res is None
