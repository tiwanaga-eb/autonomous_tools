from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

from .models import Segment
from .vehicle_config import VehicleConfig


@dataclass
class DynamicsStats:
    dynamic_feasible: bool
    total_time: float
    max_steering_change: float
    max_steering_rate_required: float
    max_steer_time: float
    sum_abs_delta_steer: float


def _segment_length(segment: Segment) -> float:
    length = 0.0
    for i in range(1, len(segment.states)):
        dx = segment.states[i].x - segment.states[i - 1].x
        dy = segment.states[i].y - segment.states[i - 1].y
        length += math.hypot(dx, dy)
    return length


def _segment_nominal_speed(segment: Segment, vehicle: VehicleConfig) -> float:
    return vehicle.max_speed_fwd if segment.gear == "F" else vehicle.max_speed_rev


def evaluate_steering_feasibility(
    segments: List[Segment],
    vehicle: VehicleConfig,
    min_speed_threshold: float,
) -> DynamicsStats:
    base_time = 0.0
    for seg in segments:
        length = _segment_length(seg)
        v_nominal = max(_segment_nominal_speed(seg, vehicle), 1e-6)
        base_time += length / v_nominal

    max_delta = 0.0
    max_steer_time = 0.0
    max_rate_required = 0.0
    sum_abs_delta = 0.0
    steer_time_sum = 0.0

    all_states = []
    for seg in segments:
        if not seg.states:
            continue
        if not all_states:
            all_states.extend(seg.states)
        else:
            all_states.extend(seg.states[1:])

    for i in range(1, len(all_states)):
        k_prev = all_states[i - 1].curvature
        k_next = all_states[i].curvature
        if abs(k_next - k_prev) < 1e-9:
            continue

        delta_prev = math.atan(vehicle.wheel_base * k_prev)
        delta_next = math.atan(vehicle.wheel_base * k_next)
        delta_steer = delta_next - delta_prev
        abs_delta = abs(delta_steer)

        t_steer = abs_delta / max(vehicle.max_steer_rate, 1e-9)
        v_max_steer = vehicle.steer_ramp_length / max(t_steer, 1e-9)

        local_v_nominal = max(abs(all_states[i].v), abs(all_states[i - 1].v), 1e-6)
        rate_required = abs_delta * local_v_nominal / max(vehicle.steer_ramp_length, 1e-9)

        max_delta = max(max_delta, abs_delta)
        max_steer_time = max(max_steer_time, t_steer)
        max_rate_required = max(max_rate_required, rate_required)
        sum_abs_delta += abs_delta
        steer_time_sum += t_steer

        if v_max_steer < min_speed_threshold:
            return DynamicsStats(
                dynamic_feasible=False,
                total_time=base_time + steer_time_sum + vehicle.gear_switch_time_penalty,
                max_steering_change=max_delta,
                max_steering_rate_required=max_rate_required,
                max_steer_time=max_steer_time,
                sum_abs_delta_steer=sum_abs_delta,
            )

    return DynamicsStats(
        dynamic_feasible=True,
        total_time=base_time + steer_time_sum + vehicle.gear_switch_time_penalty,
        max_steering_change=max_delta,
        max_steering_rate_required=max_rate_required,
        max_steer_time=max_steer_time,
        sum_abs_delta_steer=sum_abs_delta,
    )
