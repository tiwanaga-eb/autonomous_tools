import numpy as np

from planning_core.geometry import from_lonlat, project, to_lonlat


def test_roundtrip_6677():
    pts = np.array([[30465.1, 119228.8], [31000.0, 120000.0]])
    ll = to_lonlat(pts, 6677)
    back = from_lonlat(ll, 6677)
    assert np.allclose(back, pts, atol=1e-3)


def test_identity_same_epsg():
    pts = np.array([[1.0, 2.0], [3.0, 4.0]])
    out = project(pts, 6677, 6677)
    assert np.allclose(out, pts)


def test_single_point_shape_and_range():
    p = [30465.1, 119228.8]
    ll = to_lonlat(p, 6677)
    assert ll.shape == (2,)
    assert 130.0 < ll[0] < 145.0   # lon (Japan)
    assert 30.0 < ll[1] < 42.0     # lat (Japan)
