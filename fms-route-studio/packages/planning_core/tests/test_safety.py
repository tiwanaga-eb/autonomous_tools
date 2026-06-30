"""安全検証(Stage 5) verify_safety のテスト。"""
from planning_core.analysis import verify_safety
from planning_core.models.analysis import AnalysisResult, Violation
from planning_core.models.route import TrajPoint, Trajectory
from planning_core.models.vehicle import VehicleProfile


def _traj():
    p = TrajPoint(s=0.0, x=0.0, y=0.0, heading_deg=0.0, curvature=0.0, curvature_rate=0.0, grade_pct=None, steer_deg=None)
    return Trajectory(points=[p, p], length_m=1.0, min_radius_m=12.0, curvature_source="numeric")


def _veh(**kw):
    base = dict(
        id="V", name="v", kinematic_type="rigid_bicycle", overall_length=10.0, overall_width=5.0,
        overall_height=4.0, wheel_base=4.0, max_steer_angle=0.7, min_turning_radius=10.0,
        kappa_rate_max=0.08, max_grade_pct=20.0, min_clearance_m=0.5,
    )
    base.update(kw)
    return VehicleProfile(**base)


def _an(violations=None, max_grade=5.0, min_r=12.0):
    return AnalysisResult(
        min_radius_m=min_r, max_curvature=0.08, max_curvature_rate=0.05,
        max_steer_rate_required=20.0, max_grade_pct=max_grade, feasible=not violations,
        not_applicable=[], violations=violations or [],
    )


def test_all_clear_passes():
    rep = verify_safety(_traj(), _an(), _veh(), clearance_m=1.2)
    assert rep.passed and rep.reasons == []
    names = {c.name for c in rep.checks}
    assert {"footprint", "min_radius", "kappa_rate", "steer", "grade", "clearance"} <= names


def test_min_radius_violation_fails_with_reason():
    vio = [Violation(kind="min_radius", s_start=10, s_end=14, measured=0.12, limit=0.099)]
    rep = verify_safety(_traj(), _an(violations=vio), _veh(), clearance_m=1.0)
    assert not rep.passed
    assert any("旋回半径" in r for r in rep.reasons)


def test_grade_limit_fails():
    rep = verify_safety(_traj(), _an(max_grade=30.0), _veh(max_grade_pct=20.0), clearance_m=1.0)
    grade = next(c for c in rep.checks if c.name == "grade")
    assert grade.applicable and not grade.ok and not rep.passed


def test_clearance_below_min_fails():
    rep = verify_safety(_traj(), _an(), _veh(min_clearance_m=0.5), clearance_m=0.3)
    cl = next(c for c in rep.checks if c.name == "clearance")
    assert not cl.ok and not rep.passed


def test_skid_min_radius_not_applicable():
    skid = VehicleProfile(id="S", name="s", kinematic_type="tracked_skid", overall_length=5.0,
                          overall_width=2.5, overall_height=2.5, min_turning_radius=0.0, kappa_rate_max=0.3)
    rep = verify_safety(_traj(), _an(min_r=1.0), skid, clearance_m=2.0)
    mr = next(c for c in rep.checks if c.name == "min_radius")
    assert not mr.applicable and rep.passed  # 半径は評価対象外なので合否に影響しない
