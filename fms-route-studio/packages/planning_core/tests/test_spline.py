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


def test_curvature_limited_respects_via_deviation_cap():
    """max_dev_m: R_min 充足のための平滑化が経由点から離れすぎない（Via無視の回帰）。

    旧実装は R_min を満たすまで s を無制限（最大1e7）に上げたため、鋭角の経由点列では
    Via から数十m 離れた大回り経路になっていた。上限付きでは逸脱 ≤ max_dev_m を保証し、
    満たせない R_min は warning で下流（局所平滑化）へ委ねる。
    """
    import numpy as np

    from planning_core.planners.spline import fit_spline_curvature_limited

    # ジグザグ5点＋R_min=15 → 補間(s=0)では R_min を満たせず、旧実装は s=19683 まで
    # 平滑化して Via から ~15m 逸脱していた構成（実測）
    wps = np.array([[0.0, 0.0], [40.0, 0.0], [60.0, 25.0], [80.0, 0.0], [120.0, 0.0]])

    def max_dev(curve):
        return max(float(np.hypot(curve[:, 0] - w[0], curve[:, 1] - w[1]).min()) for w in wps)

    capped, _s1, warn1, _r1, _ = fit_spline_curvature_limited(wps, r_min=15.0, max_dev_m=3.0)
    uncapped, _s2, _warn2, _r2, _ = fit_spline_curvature_limited(wps, r_min=15.0)
    assert max_dev(capped) <= 3.0 + 1e-6
    assert max_dev(uncapped) > 5.0  # 旧挙動: Via から大きく離れる（このための上限）
    assert warn1 is not None  # R_min は満たせない旨を警告（下流の局所平滑化で仕上げ）


def test_plan_route_spline_passes_near_vias():
    """plan_route(spline): 経由点の近くを通る（R_min起因の内回り分を含め ≤3.5m）。"""
    import numpy as np

    from planning_core.orchestrator import PlanSpec, plan_route

    # 135°の緩いジグザグ（R_min=10 の内回り量 ~1m）→ Via 追従と R_min を両立できる形状
    wps = np.array([[0.0, 0.0], [60.0, 0.0], [120.0, 40.0], [180.0, 40.0]])
    spec = PlanSpec(waypoints=wps, headings_deg=[None] * 4, algorithm="spline",
                    r_min=10.0, spacing_m=1.0)
    out = plan_route(spec)
    pts = np.array([[p.x, p.y] for p in out.trajectory.points])
    for w in wps:
        d = float(np.hypot(pts[:, 0] - w[0], pts[:, 1] - w[1]).min())
        assert d <= 3.5, f"via {w} から {d:.2f}m 離れている"
