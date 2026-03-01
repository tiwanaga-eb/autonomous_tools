from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

from .collision import CollisionChecker
from .cost import CostEvaluator
from .dynamics import DynamicsStats, evaluate_steering_feasibility
from .dubins import DubinsSolver
from .geometry import point_in_any_polygon, wrap_angle
from .models import (
    PlanMetrics,
    PlanRequest,
    PlanResponse,
    Pose,
    Segment,
    TrajectoryPlan,
)
from .sampler import SwitchSampler
from .smoother import SmoothingResult, smooth_path_with_clothoid
from .vehicle_config import VehicleConfig, load_vehicle_config


def _segment_length(segment: Segment) -> float:
    length = 0.0
    states = segment.states
    for i in range(1, len(states)):
        dx = states[i].x - states[i - 1].x
        dy = states[i].y - states[i - 1].y
        length += math.hypot(dx, dy)
    return length


def _concat_segments_same_gear(first: Segment, second: Segment) -> Segment:
    if first.gear != second.gear:
        raise ValueError("Cannot concatenate segments with different gears")
    if not first.states:
        return second
    if not second.states:
        return first

    out = list(first.states)
    t_offset = out[-1].t
    for i, s in enumerate(second.states):
        if i == 0:
            continue
        out.append(
            type(s)(
                x=s.x,
                y=s.y,
                yaw=s.yaw,
                curvature=s.curvature,
                gear=s.gear,
                v=s.v,
                t=t_offset + s.t,
            )
        )
    return Segment(gear=first.gear, states=out)


class Planner:
    def __init__(self, request: PlanRequest):
        self.request = request
        self.vehicle: VehicleConfig = load_vehicle_config(request.vehicle_id)
        self.dubins = DubinsSolver(min_turn_radius=self.vehicle.r_min)
        self.sampler = SwitchSampler(request.map.drivable_polygons, seed=request.planner_params.seed)
        self.collision = CollisionChecker(
            drivable_polygons=request.map.drivable_polygons,
            obstacle_polygons=request.map.obstacle_polygons,
            safety_margin=request.map.safety_margin,
            vehicle_radius=self.vehicle.bounding_radius,
            road_width=self.vehicle.road_width,
        )
        self.cost_eval = CostEvaluator(request.planner_params.weights)
        self.debug: Dict[str, float] = {
            "candidate_generated": 0,
            "candidate_smoothing_collision_rejected": 0,
            "candidate_collision_rejected": 0,
            "candidate_goal_rejected": 0,
            "candidate_dynamic_rejected": 0,
            "max_delta_steer": 0.0,
            "max_steer_time": 0.0,
            "best_cost": float("inf"),
            "compute_time_ms": 0.0,
        }

    def _validate(self) -> Optional[PlanResponse]:
        req = self.request
        if not req.map.drivable_polygons:
            return PlanResponse(status="INVALID_INPUT", plan=None, reason="No drivable polygon", debug=self.debug)
        if req.tolerances.pos <= 0.0 or req.tolerances.yaw <= 0.0:
            return PlanResponse(status="INVALID_INPUT", plan=None, reason="Invalid tolerances", debug=self.debug)
        if not point_in_any_polygon((req.start_pose.x, req.start_pose.y), req.map.drivable_polygons):
            return PlanResponse(status="INVALID_INPUT", plan=None, reason="start_pose outside drivable", debug=self.debug)
        if not point_in_any_polygon((req.dock_pose.x, req.dock_pose.y), req.map.drivable_polygons):
            return PlanResponse(status="INVALID_INPUT", plan=None, reason="dock_pose outside drivable", debug=self.debug)
        if req.exit_pose and not point_in_any_polygon((req.exit_pose.x, req.exit_pose.y), req.map.drivable_polygons):
            return PlanResponse(status="INVALID_INPUT", plan=None, reason="exit_pose outside drivable", debug=self.debug)
        return None

    def _build_metrics(
        self,
        segments: List[Segment],
        switch_pose: Pose,
        dock_error_pos: float,
        dock_error_yaw: float,
        compute_time_ms: float,
        dynamics: DynamicsStats,
        smoothing: SmoothingResult,
    ) -> PlanMetrics:
        forward_length = sum(_segment_length(s) for s in segments if s.gear == "F")
        reverse_length = sum(_segment_length(s) for s in segments if s.gear == "R")
        total_length = forward_length + reverse_length

        total_time = dynamics.total_time
        collision_free = self.collision.is_plan_collision_free(segments)
        return PlanMetrics(
            total_length=total_length,
            total_time=total_time,
            forward_length=forward_length,
            reverse_length=reverse_length,
            switch_yaw=switch_pose.yaw,
            dock_error_pos=dock_error_pos,
            dock_error_yaw=dock_error_yaw,
            collision_free=collision_free,
            dynamic_feasible=dynamics.dynamic_feasible,
            max_steering_change=dynamics.max_steering_change,
            max_steering_rate_required=dynamics.max_steering_rate_required,
            max_kappa_rate=smoothing.max_kappa_rate,
            smoothing_applied=smoothing.smoothing_applied,
            added_smoothing_length=smoothing.added_smoothing_length,
            compute_time_ms=compute_time_ms,
        )

    def _goal_error(self, dock_end: Pose) -> Tuple[float, float, float]:
        pos_err = math.hypot(dock_end.x - self.request.dock_pose.x, dock_end.y - self.request.dock_pose.y)
        yaw_err = abs(wrap_angle(dock_end.yaw - self.request.dock_pose.yaw))
        goal_error = pos_err + yaw_err
        return pos_err, yaw_err, goal_error

    def plan(self) -> PlanResponse:
        t_start = time.perf_counter()
        invalid = self._validate()
        if invalid:
            return invalid

        params = self.request.planner_params.sampling_params
        step_size = float(params.get("step_size", 0.5))
        switch_sample_count = int(params.get("switch_sample_count", 160))
        yaw_bins = int(params.get("yaw_bins", 16))
        collision_stride = int(params.get("collision_stride", 1))
        straight_margin = max(0.0, float(params.get("straight_margin", 0.0)))
        min_speed_threshold = max(0.0, float(params.get("min_speed_threshold", 0.2)))
        enable_smoothing = bool(params.get("enable_smoothing", self.vehicle.enable_smoothing))
        timeout_ms = int(self.request.planner_params.timeout_ms)

        # Always include a deterministic near-dock candidate to improve robustness.
        sampled_switches = [
            Pose(
                x=(self.request.start_pose.x + self.request.dock_pose.x) / 2.0,
                y=(self.request.start_pose.y + self.request.dock_pose.y) / 2.0,
                yaw=self.request.dock_pose.yaw,
            )
        ]
        sampled_switches.extend(self.sampler.sample(switch_sample_count, yaw_bins))

        best_plan: Optional[TrajectoryPlan] = None
        timed_out = False

        for switch_pose in sampled_switches:
            elapsed_ms = (time.perf_counter() - t_start) * 1000.0
            if elapsed_ms > timeout_ms:
                timed_out = True
                break

            self.debug["candidate_generated"] += 1

            seg1 = self.dubins.sample_segment(
                self.request.start_pose,
                switch_pose,
                gear="F",
                speed=self.vehicle.max_speed_fwd,
                step_size=step_size,
            )
            if seg1 is None:
                continue

            if straight_margin > 1e-6:
                dock_pre = Pose(
                    x=self.request.dock_pose.x + straight_margin * math.cos(self.request.dock_pose.yaw),
                    y=self.request.dock_pose.y + straight_margin * math.sin(self.request.dock_pose.yaw),
                    yaw=self.request.dock_pose.yaw,
                )
                seg2_curve = self.dubins.sample_reverse_segment(
                    switch_pose,
                    dock_pre,
                    speed=self.vehicle.max_speed_rev,
                    step_size=step_size,
                )
                if seg2_curve is None:
                    continue
                seg2_straight = self.dubins.sample_straight_segment(
                    dock_pre,
                    self.request.dock_pose,
                    gear="R",
                    speed=self.vehicle.max_speed_rev,
                    step_size=step_size,
                )
                seg2 = _concat_segments_same_gear(seg2_curve, seg2_straight)
            else:
                seg2 = self.dubins.sample_reverse_segment(
                    switch_pose,
                    self.request.dock_pose,
                    speed=self.vehicle.max_speed_rev,
                    step_size=step_size,
                )
                if seg2 is None:
                    continue

            raw_segments: List[Segment] = [seg1, seg2]
            if self.request.exit_pose is not None:
                seg3 = self.dubins.sample_segment(
                    self.request.dock_pose,
                    self.request.exit_pose,
                    gear="F",
                    speed=self.vehicle.max_speed_fwd,
                    step_size=step_size,
                )
                if seg3 is None:
                    continue
                raw_segments.append(seg3)

            smoothing = smooth_path_with_clothoid(
                segments=raw_segments,
                vehicle=self.vehicle,
                enable_smoothing=enable_smoothing,
            )
            if not smoothing.success:
                self.debug["candidate_smoothing_collision_rejected"] += 1
                continue
            segments = smoothing.segments

            if not self.collision.is_plan_collision_free(segments, stride=collision_stride):
                self.debug["candidate_smoothing_collision_rejected"] += 1
                continue

            dock_end_state = segments[1].states[-1]
            pos_err, yaw_err, goal_error = self._goal_error(Pose(dock_end_state.x, dock_end_state.y, dock_end_state.yaw))
            if pos_err > self.request.tolerances.pos or yaw_err > self.request.tolerances.yaw:
                self.debug["candidate_goal_rejected"] += 1
                continue

            dyn_stats = evaluate_steering_feasibility(
                segments=segments,
                vehicle=self.vehicle,
                min_speed_threshold=min_speed_threshold,
            )
            self.debug["max_delta_steer"] = max(self.debug["max_delta_steer"], dyn_stats.max_steering_change)
            self.debug["max_steer_time"] = max(self.debug["max_steer_time"], dyn_stats.max_steer_time)
            if not dyn_stats.dynamic_feasible:
                self.debug["candidate_dynamic_rejected"] += 1
                continue

            metrics = self._build_metrics(
                segments=segments,
                switch_pose=switch_pose,
                dock_error_pos=pos_err,
                dock_error_yaw=yaw_err,
                compute_time_ms=elapsed_ms,
                dynamics=dyn_stats,
                smoothing=smoothing,
            )
            cost = self.cost_eval.compute(
                total_length=metrics.total_length,
                total_time=metrics.total_time,
                reverse_length=metrics.reverse_length,
                switch_yaw=switch_pose.yaw,
                yaw_pref=self.request.planner_params.yaw_pref,
                goal_error=goal_error,
                steer_change_sum=dyn_stats.sum_abs_delta_steer,
                kappa_change_sum=smoothing.kappa_change_sum,
            )

            if cost < self.debug["best_cost"]:
                self.debug["best_cost"] = cost
                best_plan = TrajectoryPlan(segments=segments, switch_pose=switch_pose, metrics=metrics)

        compute_time_ms = (time.perf_counter() - t_start) * 1000.0
        self.debug["compute_time_ms"] = compute_time_ms

        if best_plan is not None:
            best_plan.metrics.compute_time_ms = compute_time_ms
            if timed_out:
                return PlanResponse(status="TIMEOUT", plan=best_plan, reason="timeout with best effort", debug=self.debug)
            return PlanResponse(status="OK", plan=best_plan, debug=self.debug)

        if timed_out:
            return PlanResponse(status="TIMEOUT", plan=None, reason="timeout without feasible path", debug=self.debug)
        return PlanResponse(status="NO_FEASIBLE_PATH", plan=None, reason="No collision-free candidate", debug=self.debug)


def plan_mining_dock(request: PlanRequest) -> PlanResponse:
    algorithm = (request.planner_params.algorithm or "").strip().lower()
    if not algorithm:
        algorithm = str(request.planner_params.sampling_params.get("algorithm", "dubins")).lower()
    if algorithm == "dubins_pp":
        from .dubins_pp import DubinsPlusPlusPlanner

        planner = DubinsPlusPlusPlanner(request)
        return planner.plan()
    if algorithm == "primitives":
        from .dynamic_planner import MotionPrimitivesPlanner
        from .vehicle_config import load_vehicle_config

        vehicle = load_vehicle_config(request.vehicle_id)
        planner = MotionPrimitivesPlanner(request, vehicle)
        return planner.plan()

    planner = Planner(request)
    return planner.plan()
