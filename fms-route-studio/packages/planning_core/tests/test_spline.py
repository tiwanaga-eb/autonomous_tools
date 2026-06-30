import numpy as np

from planning_core.planners import (
    fit_spline,
    fit_spline_with_min_radius,
    resample_by_spacing,
)


def test_fit_spline_through_endpoints():
    pts = np.array([[0, 0], [5, 2], [10, 0], [15, 3]], float)
    curve = fit_spline(pts, s=0.0, n=500)
    assert curve.shape == (500, 2)
    assert np.allclose(curve[0], pts[0], atol=1e-3)
    assert np.allclose(curve[-1], pts[-1], atol=1e-3)


def test_resample_uniform_spacing():
    line = np.column_stack([np.linspace(0.0, 10.0, 50), np.zeros(50)])
    rs = resample_by_spacing(line, 1.0)
    d = np.linalg.norm(np.diff(rs, axis=0), axis=1)
    assert np.all(d[:-1] > 0.8) and np.all(d[:-1] < 1.2)


def test_min_radius_none_returns_zero_smoothing():
    pts = np.array([[0, 0], [5, 1], [10, 0]], float)
    curve, used_s, warn, r = fit_spline_with_min_radius(pts, None)
    assert used_s == 0.0 and curve is not None


def test_min_radius_smoothing_runs():
    pts = np.array([[0, 0], [1, 3], [2, 0], [3, 3], [4, 0]], float)  # zigzag
    curve, used_s, warn, r = fit_spline_with_min_radius(pts, 3.0, n=400)
    assert curve is not None
    assert used_s >= 0.0
    assert r >= 0.0
