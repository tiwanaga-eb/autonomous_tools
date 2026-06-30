import numpy as np
from affine import Affine

from planning_core.analysis import grade_profile, sample_bilinear


def _ramp_dsm(n=50, slope=0.1):
    # z = slope * x（東向きに一定勾配）。transform は north-up。
    t = Affine(1.0, 0.0, 0.0, 0.0, -1.0, float(n))
    z = np.zeros((n, n), float)
    for r in range(n):
        for c in range(n):
            x, y = t * (c + 0.5, r + 0.5)
            z[r, c] = slope * x
    return z, t


def test_sample_bilinear_matches_plane():
    dsm, t = _ramp_dsm(slope=0.2)
    xy = np.array([[10.0, 20.0], [25.5, 30.0]])
    z = sample_bilinear(dsm, t, xy)
    assert np.isfinite(z).all()
    assert abs(z[0] - 0.2 * 10.0) < 1e-6
    assert abs(z[1] - 0.2 * 25.5) < 1e-6


def test_grade_profile_constant_slope():
    dsm, t = _ramp_dsm(slope=0.1)
    xs = np.linspace(5.0, 40.0, 36)
    xy = np.column_stack([xs, np.full_like(xs, 25.0)])
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))])
    g = grade_profile(xy, s, dsm, t)
    # 進行方向が東で勾配0.1 → 10%
    assert np.all(np.abs(g - 10.0) < 0.5)


def test_grade_nan_outside():
    dsm, t = _ramp_dsm()
    xy = np.array([[-100.0, -100.0]])  # 範囲外
    z = sample_bilinear(dsm, t, xy)
    assert np.isnan(z[0])
