import math

from planner.models import Segment, State
from planner.smoother import smooth_path_with_clothoid
from planner.vehicle_config import VehicleConfig


def make_vehicle(kappa_rate_max=0.2, smoothing_step=0.2):
    return VehicleConfig(
        wheel_base=2.5,
        max_steer_angle=0.6,
        max_speed_fwd=4.0,
        max_speed_rev=2.0,
        footprint_polygon=[(1.0, 0.5), (1.0, -0.5), (-1.0, -0.5), (-1.0, 0.5)],
        gear_switch_time_penalty=1.0,
        r_min=4.0,
        kappa_max=0.25,
        bounding_radius=1.2,
        max_steer_rate=1.0,
        steer_ramp_length=1.0,
        road_width=3.0,
        kappa_rate_max=kappa_rate_max,
        smoothing_step=smoothing_step,
        enable_smoothing=True,
    )


def make_segment(start_x, end_x, curvature):
    states = []
    x = start_x
    t = 0.0
    while x <= end_x + 1e-9:
        states.append(
            State(
                x=x,
                y=0.0,
                yaw=0.0,
                curvature=curvature,
                gear="F",
                v=2.0,
                t=t,
            )
        )
        x += 0.5
        t += 0.25
    return Segment(gear="F", states=states)


def collect_states(segments):
    out = []
    for seg in segments:
        if not out:
            out.extend(seg.states)
        else:
            out.extend(seg.states[1:])
    return out


def test_curvature_continuity():
    seg1 = make_segment(0.0, 8.0, 0.0)
    seg2 = make_segment(8.0, 16.0, 0.2)
    vehicle = make_vehicle(kappa_rate_max=0.1, smoothing_step=0.2)

    res = smooth_path_with_clothoid([seg1, seg2], vehicle=vehicle, enable_smoothing=True)
    assert res.success is True
    states = collect_states(res.segments)

    for i in range(1, len(states)):
        ds = math.hypot(states[i].x - states[i - 1].x, states[i].y - states[i - 1].y)
        if ds < 1e-6:
            continue
        dk_ds = abs(states[i].curvature - states[i - 1].curvature) / ds
        assert dk_ds <= vehicle.kappa_rate_max + 1e-2


def test_ramp_inserted():
    seg1 = make_segment(0.0, 10.0, 0.0)
    seg2 = make_segment(10.0, 20.0, -0.2)
    vehicle = make_vehicle(kappa_rate_max=0.1, smoothing_step=0.2)

    res = smooth_path_with_clothoid([seg1, seg2], vehicle=vehicle, enable_smoothing=True)
    assert res.success is True
    assert res.smoothing_applied is True
    assert res.added_smoothing_length > 0.0

    ramp_count = sum(1 for s in collect_states(res.segments) if s.is_ramp)
    assert ramp_count > 0


def test_insufficient_length_rejected():
    seg1 = make_segment(0.0, 0.6, 0.0)
    seg2 = make_segment(0.6, 1.2, 0.3)
    vehicle = make_vehicle(kappa_rate_max=0.05, smoothing_step=0.2)

    res = smooth_path_with_clothoid([seg1, seg2], vehicle=vehicle, enable_smoothing=True)
    assert res.success is False
    assert res.reason == "insufficient_segment_length_for_smoothing"
