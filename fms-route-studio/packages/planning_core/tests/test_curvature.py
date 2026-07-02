import numpy as np

from planning_core.analysis import circumradius, curvature_profile, min_turning_radius


def test_circle_curvature_and_radius():
    R = 20.0
    th = np.linspace(0.0, np.pi, 80)
    pts = np.column_stack([R * np.cos(th), R * np.sin(th)])
    prof = curvature_profile(pts)
    k = prof["kappa"][2:-2]
    assert np.allclose(k, 1.0 / R, rtol=0.02)
    assert abs(min_turning_radius(pts) - R) < 0.5
    # 円弧は曲率一定 -> dκ/ds ≈ 0
    assert np.max(np.abs(prof["dkappa_ds"][2:-2])) < 5e-3


def test_straight_line():
    pts = np.column_stack([np.linspace(0.0, 10.0, 20), np.zeros(20)])
    prof = curvature_profile(pts)
    assert np.allclose(prof["kappa"], 0.0, atol=1e-6)
    assert np.isinf(min_turning_radius(pts))


def test_circumradius_right_triangle():
    # 直角三角形: 斜辺が直径 -> 外接半径 = 斜辺/2
    r = circumradius([0.0, 0.0], [2.0, 0.0], [0.0, 2.0])
    assert abs(r - np.hypot(2.0, 2.0) / 2.0) < 1e-9


def test_min_turning_radius_excludes_cusp():
    """後進を伴う切り返し経路: cusp を除外しないと見かけの半径が極小に化ける。"""
    from planning_core.analysis.curvature import cusp_mask

    fwd = np.column_stack([np.linspace(0.0, 6.0, 7), np.zeros(7)])           # +x 直進
    rev = np.column_stack([np.linspace(6.0, 0.0, 7)[1:], np.full(6, 0.02)])  # -x 直進（微小オフセット）
    pts = np.vstack([fwd, rev])
    assert cusp_mask(pts).any()  # 反転点を検出している
    # cusp を除外すれば直線同士なので最小旋回半径は非常に大きい（ほぼ∞）
    assert min_turning_radius(pts) > 100.0
