"""最小旋回半径(R_min)保証の曲率制限テスト。"""
import numpy as np
from affine import Affine

from planning_core.analysis import min_turning_radius
from planning_core.planners import fit_spline, limit_curvature_polyline, resample_by_spacing


def _tight():
    wps = np.array([(0, 0), (20, 0), (30, 8), (20, 16), (0, 16), (0, 30), (25, 30)], float)
    return resample_by_spacing(fit_spline(wps, s=0.0, n=400), 2.0)


def test_opens_tight_corner_to_rmin():
    curve = _tight()
    assert min_turning_radius(curve) < 8.885  # 元はタイト
    for rmin in (8.885, 10.1):
        out, meas = limit_curvature_polyline(curve, rmin)
        assert min_turning_radius(out) >= rmin * 0.99  # R_min をほぼ達成
        assert np.allclose(out[0], curve[0]) and np.allclose(out[-1], curve[-1])  # 端点固定


def test_compliant_curve_untouched():
    # ゆるい正弦波（R が十分大きい）→ ほぼ無加工
    wps = np.array([(i * 5.0, 3.0 * np.sin(i * 0.3)) for i in range(40)], float)
    curve = resample_by_spacing(fit_spline(wps, s=0.0, n=400), 2.0)
    r0 = min_turning_radius(curve)
    assert r0 > 8.885
    out, _ = limit_curvature_polyline(curve, 8.885)
    assert np.allclose(out, curve)  # compliant 区間は触らない


def test_noop_when_no_rmin():
    curve = _tight()
    out, meas = limit_curvature_polyline(curve, None)
    assert np.allclose(out, curve)
    out2, _ = limit_curvature_polyline(curve, 0.0)
    assert np.allclose(out2, curve)


def test_corridor_keeps_inside_mask():
    # 幅の広い水平コリドー内なら、開いた経路も領域内に留まる
    H, W = 40, 80
    T = Affine(1.0, 0.0, 0.0, 0.0, -1.0, float(H))
    mask = np.zeros((H, W), np.uint8)
    for r in range(H):
        y = H - r - 0.5
        if 5 <= y <= 35:
            mask[r, :] = 1
    wps = np.array([(2, 20), (20, 8), (30, 32), (45, 10), (70, 20)], float)
    curve = resample_by_spacing(fit_spline(wps, s=0.0, n=400), 2.0)
    out, _ = limit_curvature_polyline(curve, 9.0, mask=mask, transform=T)

    inv = ~T
    def inside(x, y):
        c = int(np.floor(inv.a * x + inv.b * y + inv.c))
        r = int(np.floor(inv.d * x + inv.e * y + inv.f))
        return 0 <= r < H and 0 <= c < W and mask[r, c] != 0
    # コリドー制約: 元が領域内の点を領域外へ押し出さない（外→内化は EB の責務で別）。
    before = [inside(x, y) for x, y in curve]
    after = [inside(x, y) for x, y in out]
    assert all((not b) or a for b, a in zip(before, after))
