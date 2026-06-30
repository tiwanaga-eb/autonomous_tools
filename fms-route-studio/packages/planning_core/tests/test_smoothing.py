import itertools

import numpy as np

from planning_core.analysis import curvature_profile, min_turning_radius
from planning_core.planners import (
    fit_spline_curvature_limited,
    plan_dubins,
    resample_by_spacing,
    smooth_kinematic_path,
)


def _zigzag_path(n=60, seed=1):
    """緩いカーブに離散プリミティブ風の小刻みな折れ（クネクネ）を載せた擬似 hybrid A* 出力。"""
    rng = np.random.default_rng(seed)
    x = y = th = 0.0
    pts = []
    for _ in range(n):
        th += 0.03 + rng.choice([-0.18, 0.18, 0.0])
        x += 2.0 * np.cos(th)
        y += 2.0 * np.sin(th)
        pts.append((x, y))
    return np.array(pts, float)


def _tortuosity(p):
    p = resample_by_spacing(np.asarray(p, float), 0.5)
    d = np.diff(p, axis=0)
    a = np.arctan2(d[:, 1], d[:, 0])
    return float(np.sum(np.abs(np.diff(a))))


def test_smooth_kinematic_path_reduces_winding_and_keeps_endpoints():
    xy = _zigzag_path()
    gears = ["D"] * len(xy)
    curve, g = smooth_kinematic_path(xy, gears, None, None, r_min=8.885, kappa_rate_max=0.12)
    # クネクネ（折れ角の総和）が明確に減る
    assert _tortuosity(curve) < _tortuosity(xy) * 0.6
    # 最小旋回半径が R_min をほぼ満たす（生の鋭い折れが解消）
    assert min_turning_radius(resample_by_spacing(curve, 0.5)) >= 8.885 * 0.9
    # 始終点は厳密保持
    assert np.allclose(curve[0], xy[0]) and np.allclose(curve[-1], xy[-1])


def test_smooth_kinematic_path_preserves_cusp():
    xy = _zigzag_path()
    gears = ["D"] * 30 + ["R"] * 30  # 中央に切り返し(cusp)
    curve, g = smooth_kinematic_path(xy, gears, None, None, r_min=8.885, kappa_rate_max=0.12)
    # gear の並び（D→R の1回反転）が保たれる
    assert [k for k, _ in itertools.groupby(g)] == ["D", "R"]
    assert len(curve) == len(g)


def _max_dk(curve):
    prof = curvature_profile(curve)
    return float(np.max(np.abs(prof["dkappa_ds"])))


def test_curvature_limited_enforces_min_radius():
    # 90°/40m脚 → R_min=10m は平滑化で達成可能（警告なし）
    pts = np.array([[0, 0], [40, 0], [40, 40]], float)
    curve, used_s, warn, r, dk = fit_spline_curvature_limited(pts, 10.0, kappa_rate_max=None, n=1500)
    assert warn is None
    assert r >= 10.0 - 0.5
    assert min_turning_radius(curve) >= 10.0 - 0.5


def test_curvature_limited_limits_steer_rate():
    # 鋭い 90°/5m脚: dκ/ds 上限を課すと、課さない場合より dκ/ds が下がる（操舵レート制限＝クロソイド近似）
    pts = np.array([[0, 0], [5, 0], [5, 5]], float)
    _, _, _, _, dk_unlimited = fit_spline_curvature_limited(pts, 0.0, kappa_rate_max=None, n=1500)
    curve, _, warn, r, dk = fit_spline_curvature_limited(pts, 0.0, kappa_rate_max=0.08, n=1500)
    assert dk < dk_unlimited            # より平滑化される
    assert dk <= 0.08 + 1e-2            # 概ね上限以下


def test_sharp_corner_is_best_effort_with_warning():
    # 90°/2m脚は平滑化しても R_min=10m に到底届かない → best-effort＋警告（限界を明示）
    pts = np.array([[0, 0], [2, 0], [2, 2]], float)
    _, _, warn, r, _ = fit_spline_curvature_limited(pts, 10.0, kappa_rate_max=None, n=1500)
    assert warn is not None
    assert r < 10.0


def test_spline_anchors_endpoints():
    # 平滑化(s>0)でも、生成曲線の始点・終点は入力の Start/Goal に一致する（ドリフトしない）
    pts = np.array([[0, 0], [40, 0], [40, 40], [80, 40]], float)
    curve, _, _, _, _ = fit_spline_curvature_limited(pts, 10.0, kappa_rate_max=0.05, n=1500)
    assert np.allclose(curve[0], pts[0], atol=1e-6)
    assert np.allclose(curve[-1], pts[-1], atol=1e-6)


def test_dubins_smoothing_removes_instant_steering():
    # Dubins L字（直線→R=10アーク→直線, 曲率ステップ）を平滑化すると dκ/ds が大幅低下
    wps = [(0.0, 0.0), (50.0, 0.0), (50.0, 50.0)]
    dub = plan_dubins(wps, rho=10.0, step=0.4)
    dub_rs = resample_by_spacing(dub, 1.0)
    raw_dk = _max_dk(resample_by_spacing(dub, 2.0))
    curve, _, _, r, dk = fit_spline_curvature_limited(dub_rs, 10.0, kappa_rate_max=0.05, n=1500)
    assert dk < raw_dk          # 瞬間操舵が緩和される
    assert r >= 10.0 - 1.5      # 最小半径は維持
