import math

import numpy as np

from planning_core.analysis import min_turning_radius
from planning_core.planners import plan_dubins, sample_dubins, shortest_dubins


def test_shortest_dubins_straight():
    # 同一方位・直線上 → S 区間で距離≈ユークリッド距離
    best = shortest_dubins((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), rho=5.0)
    assert best is not None
    _, _, length = best
    assert abs(length - 10.0) < 0.5


def test_sample_dubins_endpoints_and_radius():
    rho = 8.0
    start = (0.0, 0.0, 0.0)
    goal = (10.0, 10.0, math.pi / 2)
    path = sample_dubins(start, goal, rho, step=0.3)
    assert path is not None
    # 端点一致
    assert np.allclose(path[0], [0.0, 0.0], atol=1e-6)
    assert np.allclose(path[-1], [10.0, 10.0], atol=1e-6)
    # 曲率半径が rho を大きく下回らない（数値誤差込みで余裕）
    r = min_turning_radius(path)
    assert r > rho - 0.8


def test_plan_dubins_through_waypoints():
    wps = [(0.0, 0.0), (20.0, 5.0), (40.0, -5.0), (60.0, 0.0)]
    path = plan_dubins(wps, rho=10.0, step=0.5)
    assert len(path) > 10
    # 始点・終点がウェイポイントに一致
    assert np.allclose(path[0], wps[0], atol=1e-6)
    assert np.allclose(path[-1], wps[-1], atol=1e-6)
    # R_min が保証される（rho を割り込まない、誤差余裕あり）
    assert min_turning_radius(path) > 10.0 - 1.0
