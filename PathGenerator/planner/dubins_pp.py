from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

from .collision import CollisionChecker
from .cost import CostEvaluator
from .dynamics import DynamicsStats, evaluate_steering_feasibility
from .dubins import DubinsSolver
from .geometry import point_in_any_polygon, point_to_polygon_distance, point_to_segment_distance, wrap_angle
from .models import PlanMetrics, PlanRequest, PlanResponse, Pose, Segment, State, TrajectoryPlan
from .sampler import SwitchSampler
from .smoother import SmoothingResult, smooth_path_with_clothoid
from .vehicle_config import VehicleConfig, load_vehicle_config


def _segment_length(segment: Segment) -> float:
    length = 0.0
    for i in range(1, len(segment.states)):
        dx = segment.states[i].x - segment.states[i - 1].x
        dy = segment.states[i].y - segment.states[i - 1].y
        length += math.hypot(dx, dy)
    return length


def _concat_same_gear(first: Segment, second: Segment) -> Segment:
    if first.gear != second.gear:
        raise ValueError("gear mismatch")
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
            State(
                x=s.x,
                y=s.y,
                yaw=s.yaw,
                curvature=s.curvature,
                gear=s.gear,
                v=s.v,
                t=t_offset + s.t,
                is_ramp=s.is_ramp,
            )
        )
    return Segment(gear=first.gear, states=out)


def _to_pose(st: State) -> Pose:
    return Pose(x=st.x, y=st.y, yaw=st.yaw)


class DubinsPlusPlusPlanner:
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
            "candidate_generated": 0.0,
            "candidate_collision_rejected": 0.0,
            "candidate_goal_rejected": 0.0,
            "candidate_dynamic_rejected": 0.0,
            "candidate_reverse_rejected": 0.0,
            "candidate_switch_rejected": 0.0,
            "candidate_direction_rejected": 0.0,
            "candidate_behavior_rejected": 0.0,
            "candidate_cost_pruned": 0.0,
            "best_cost": float("inf"),
            "best_reverse_length": float("inf"),
            "compute_time_ms": 0.0,
            "algorithm": 2.0,
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

    def _parse_array(self, raw: object, default: Sequence[float]) -> List[float]:
        if isinstance(raw, list):
            out: List[float] = []
            for v in raw:
                try:
                    out.append(float(v))
                except Exception:
                    continue
            return out or list(default)
        return list(default)

    def _parse_positive_array(self, raw: object, default: Sequence[float], min_value: float = 1e-6) -> List[float]:
        out = self._parse_array(raw, default)
        filtered = [v for v in out if math.isfinite(v) and v > min_value]
        return filtered or [float(v) for v in default if v > min_value]

    def _parse_nonnegative_array(self, raw: object, default: Sequence[float]) -> List[float]:
        out = self._parse_array(raw, default)
        filtered = [v for v in out if math.isfinite(v) and v >= 0.0]
        return filtered or [float(v) for v in default if v >= 0.0]

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
        return PlanMetrics(
            total_length=total_length,
            total_time=dynamics.total_time,
            forward_length=forward_length,
            reverse_length=reverse_length,
            switch_yaw=switch_pose.yaw,
            dock_error_pos=dock_error_pos,
            dock_error_yaw=dock_error_yaw,
            collision_free=self.collision.is_plan_collision_free(segments),
            dynamic_feasible=dynamics.dynamic_feasible,
            max_steering_change=dynamics.max_steering_change,
            max_steering_rate_required=dynamics.max_steering_rate_required,
            max_kappa_rate=smoothing.max_kappa_rate,
            smoothing_applied=smoothing.smoothing_applied,
            added_smoothing_length=smoothing.added_smoothing_length,
            compute_time_ms=compute_time_ms,
        )

    def _build_candidate(
        self,
        switch_xy: Pose,
        switch_yaw: float,
        kappa2: float,
        s_buffer: float,
        step_size: float,
    ) -> Optional[Tuple[List[Segment], Pose]]:
        """Build forward-reverse candidate using Dubins paths for both legs.

        Forward segment: optimal Dubins path (start → switch_pose).
          - Fixes the original heading-discontinuity bug caused by constant-arc +
            forced straight-link (the straight could point in any direction regardless
            of the arc's final heading, producing kinematically invalid paths).
          - All six Dubins path types (LSL/RSR/LSR/RSL/RLR/LRL) are tried; the
            shortest feasible one is selected.

        Reverse segment: Dubins reverse path (switch_pose → dock_pose).
          - kappa2 controls the minimum turning radius for the reverse leg so that
            different approach curvatures can be explored.
          - An optional straight buffer is prepended to the dock to ensure a clean
            straight-in final approach.
        """
        req = self.request
        switch_pose = Pose(x=switch_xy.x, y=switch_xy.y, yaw=wrap_angle(switch_yaw))

        # ── Forward segment: Dubins optimal path ──────────────────────────────
        seg1 = self.dubins.sample_segment(
            req.start_pose,
            switch_pose,
            gear="F",
            speed=self.vehicle.max_speed_fwd,
            step_size=step_size,
        )
        if seg1 is None:
            return None

        # ── Reverse segment with configurable turning radius ──────────────────
        r_rev = 1.0 / max(kappa2, 1e-6)
        dubins_rev = (
            DubinsSolver(min_turn_radius=r_rev)
            if abs(r_rev - self.vehicle.r_min) > 0.01
            else self.dubins
        )

        if s_buffer > 1e-6:
            # Straight buffer before the final dock approach
            dock_pre = Pose(
                x=req.dock_pose.x + s_buffer * math.cos(req.dock_pose.yaw),
                y=req.dock_pose.y + s_buffer * math.sin(req.dock_pose.yaw),
                yaw=req.dock_pose.yaw,
            )
            seg2 = dubins_rev.sample_reverse_segment(
                switch_pose,
                dock_pre,
                speed=self.vehicle.max_speed_rev,
                step_size=step_size,
            )
            if seg2 is None:
                return None
            seg2_straight = self.dubins.sample_straight_segment(
                dock_pre,
                req.dock_pose,
                gear="R",
                speed=self.vehicle.max_speed_rev,
                step_size=step_size,
            )
            seg2 = _concat_same_gear(seg2, seg2_straight)
        else:
            seg2 = dubins_rev.sample_reverse_segment(
                switch_pose,
                req.dock_pose,
                speed=self.vehicle.max_speed_rev,
                step_size=step_size,
            )
            if seg2 is None:
                return None

        segments = [seg1, seg2]
        if req.exit_pose is not None:
            seg3 = self.dubins.sample_segment(
                req.dock_pose,
                req.exit_pose,
                gear="F",
                speed=self.vehicle.max_speed_fwd,
                step_size=step_size,
            )
            if seg3 is None:
                return None
            segments.append(seg3)
        return segments, switch_pose

    def _reverse_direction_valid(self, reverse_seg: Segment, stride: int = 3) -> bool:
        if len(reverse_seg.states) < 2:
            return True
        dock = self.request.dock_pose
        tol = max(self.request.tolerances.pos, 1e-3)
        step = max(1, stride)
        for i in range(1, len(reverse_seg.states), step):
            prev = reverse_seg.states[i - 1]
            cur = reverse_seg.states[i]
            mx = cur.x - prev.x
            my = cur.y - prev.y
            norm = math.hypot(mx, my)
            if norm < 1e-8:
                continue
            to_dock_x = dock.x - cur.x
            to_dock_y = dock.y - cur.y
            if math.hypot(to_dock_x, to_dock_y) <= tol:
                continue
            if mx * to_dock_x + my * to_dock_y <= 0.0:
                return False
        return True

    def _reverse_monotonic_approach(self, reverse_seg: Segment, stride: int = 2) -> bool:
        if len(reverse_seg.states) < 2:
            return True
        dock = self.request.dock_pose
        step = max(1, stride)
        prev = math.hypot(reverse_seg.states[0].x - dock.x, reverse_seg.states[0].y - dock.y)
        for i in range(1, len(reverse_seg.states), step):
            cur = math.hypot(reverse_seg.states[i].x - dock.x, reverse_seg.states[i].y - dock.y)
            if cur > prev + 1e-3:
                return False
            prev = cur
        cur_last = math.hypot(reverse_seg.states[-1].x - dock.x, reverse_seg.states[-1].y - dock.y)
        return cur_last <= prev + 1e-3

    def _switch_points_in_sector(
        self,
        requested_count: int,
        d_min: float,
        d_max: float,
        sector_angle: float,
    ) -> List[Pose]:
        dock = self.request.dock_pose
        out: List[Pose] = []
        seen: set[tuple[int, int]] = set()

        def _push(x: float, y: float) -> None:
            key = (round(x * 10), round(y * 10))
            if key in seen:
                return
            if not point_in_any_polygon((x, y), self.request.map.drivable_polygons):
                return
            seen.add(key)
            out.append(Pose(x=x, y=y, yaw=0.0))

        mid_x = (self.request.start_pose.x + dock.x) * 0.5
        mid_y = (self.request.start_pose.y + dock.y) * 0.5
        _push(mid_x, mid_y)

        d_min = max(0.5, d_min)
        d_max = max(d_min + 0.5, d_max)
        dist_samples = [d_min + (d_max - d_min) * (i / 5.0) for i in range(6)]
        angle_samples = [-sector_angle, -0.5 * sector_angle, 0.0, 0.5 * sector_angle, sector_angle]
        for d in dist_samples:
            for a in angle_samples:
                th = dock.yaw + a
                _push(dock.x + d * math.cos(th), dock.y + d * math.sin(th))

        # Fill remaining with random samples, then sector-filter them.
        if len(out) < requested_count:
            random_samples = self.sampler.sample(max(0, requested_count * 2), yaw_bins=8)
            for p in random_samples:
                dx = p.x - dock.x
                dy = p.y - dock.y
                dist = math.hypot(dx, dy)
                if dist < d_min or dist > d_max:
                    continue
                ang = abs(wrap_angle(math.atan2(dy, dx) - dock.yaw))
                if ang > sector_angle:
                    continue
                _push(p.x, p.y)
                if len(out) >= requested_count:
                    break
        return out[: max(1, requested_count)]

    def _switch_points_basic(self, requested_count: int) -> List[Pose]:
        out: List[Pose] = [
            Pose(
                x=(self.request.start_pose.x + self.request.dock_pose.x) * 0.5,
                y=(self.request.start_pose.y + self.request.dock_pose.y) * 0.5,
                yaw=0.0,
            )
        ]
        extra = self.sampler.sample(max(0, requested_count - 1), yaw_bins=8)
        out.extend(extra)
        return out[: max(1, requested_count)]

    def _point_min_obstacle_clearance(self, x: float, y: float) -> float:
        if not self.request.map.obstacle_polygons:
            return 1e9
        p = (x, y)
        return min(point_to_polygon_distance(p, obs) for obs in self.request.map.obstacle_polygons)

    def _point_min_drivable_edge_distance(self, x: float, y: float) -> float:
        p = (x, y)
        d_min = float("inf")
        for poly in self.request.map.drivable_polygons:
            n = len(poly)
            for i in range(n):
                d = point_to_segment_distance(p, poly[i], poly[(i + 1) % n])
                d_min = min(d_min, d)
        return d_min if d_min < float("inf") else 0.0

    def _rank_switch_points(
        self,
        points: List[Pose],
        desired_count: int,
        min_obstacle_clearance: float,
        min_edge_margin: float,
    ) -> List[Pose]:
        dock = self.request.dock_pose
        start = self.request.start_pose
        scored: List[tuple[float, Pose]] = []
        for p in points:
            obs_clear = self._point_min_obstacle_clearance(p.x, p.y)
            edge_clear = self._point_min_drivable_edge_distance(p.x, p.y)
            if obs_clear < min_obstacle_clearance:
                continue
            if edge_clear < min_edge_margin:
                continue
            d_mid = abs(
                math.hypot(p.x - dock.x, p.y - dock.y)
                - 0.5 * math.hypot(start.x - dock.x, start.y - dock.y)
            )
            score = 2.0 * min(obs_clear, 10.0) + 0.8 * min(edge_clear, 10.0) - 0.1 * d_mid
            scored.append((score, p))
        scored.sort(key=lambda t: t[0], reverse=True)
        ranked = [p for _, p in scored]
        if not ranked:
            return points[: max(1, desired_count)]
        return ranked[: max(1, desired_count)]

    def _reverse_arc_length(self, reverse_seg: Segment) -> float:
        out = 0.0
        for i in range(1, len(reverse_seg.states)):
            c = abs(reverse_seg.states[i].curvature)
            if c < 1e-6:
                continue
            dx = reverse_seg.states[i].x - reverse_seg.states[i - 1].x
            dy = reverse_seg.states[i].y - reverse_seg.states[i - 1].y
            out += math.hypot(dx, dy)
        return out

    def _vehicle_length(self) -> float:
        if self.vehicle.overall_length is not None:
            return max(0.1, float(self.vehicle.overall_length))
        if not self.vehicle.footprint_polygon:
            return 1.0
        xs = [p[0] for p in self.vehicle.footprint_polygon]
        return max(0.1, max(xs) - min(xs))

    def plan(self) -> PlanResponse:
        t_start = time.perf_counter()
        invalid = self._validate()
        if invalid:
            return invalid

        sampling = self.request.planner_params.sampling_params
        pp = self.request.planner_params.dubins_pp_params
        timeout_ms = max(50, int(self.request.planner_params.timeout_ms))
        enable_dubins_fallback = bool(pp.get("enable_dubins_fallback", True))
        step_size = max(0.05, float(pp.get("arc_sample_step", 0.2)))
        collision_stride = max(1, int(sampling.get("collision_stride", 1)))
        min_speed_threshold = max(0.0, float(sampling.get("min_speed_threshold", 0.2)))
        enable_smoothing = bool(sampling.get("enable_smoothing", self.vehicle.enable_smoothing))
        candidate_cap = max(60, int(pp.get("candidate_limit", 1200)))

        # kappa1_ratios removed: forward segment now uses Dubins (always kinematically
        # valid, explores all 6 path types automatically).  kappa2_ratios still varies
        # the reverse turning radius so different dock-approach curvatures are explored.
        kappa2_ratios = self._parse_positive_array(pp.get("kappa2_ratios"), [1.0], min_value=1e-4)
        switch_buffers = self._parse_nonnegative_array(pp.get("switch_buffer_lengths"), [0.0, 2.0, 4.0])
        yaw_offsets_deg = self._parse_array(pp.get("switch_yaw_offsets_deg"), [-25.0, -10.0, 0.0, 10.0, 25.0])
        yaw_offsets_deg = [v for v in yaw_offsets_deg if math.isfinite(v)]
        if not yaw_offsets_deg:
            yaw_offsets_deg = [-25.0, -10.0, 0.0, 10.0, 25.0]
        yaw_offsets = [math.radians(v) for v in yaw_offsets_deg]
        max_reverse_length_ratio = max(0.1, float(pp.get("max_reverse_length_ratio", 1.5)))
        switch_yaw_limit_deg = float(pp.get("switch_yaw_limit_deg", 60.0))
        switch_yaw_limit = math.radians(min(180.0, max(1.0, switch_yaw_limit_deg)))
        kappa_switch_limit_ratio = max(0.0, float(pp.get("kappa_switch_limit_ratio", 1.0)))
        w_reverse_excess = float(self.cost_eval.weights.get("w_reverse_excess", 0.0))
        behavior = self.request.planner_params.dubins_pp_behavioral_params
        reverse_arc_ratio_limit = float(behavior.get("reverse_arc_ratio_limit", 0.70))
        switch_sector_d_min = max(0.0, float(behavior.get("switch_sector_d_min", 3.0)))
        switch_sector_d_max = max(switch_sector_d_min + 0.5, float(behavior.get("switch_sector_d_max", 30.0)))
        switch_sector_angle = math.radians(float(behavior.get("switch_sector_angle_deg", 90.0)))
        delta_neutral_limit = math.radians(float(behavior.get("delta_neutral_limit_deg", 90.0)))
        forward_length_ratio = float(behavior.get("forward_length_ratio", 0.8))
        entry_angle_limit = math.radians(float(behavior.get("entry_angle_limit_deg", 60.0)))
        switch_rank_min_obstacle_clearance = float(behavior.get("switch_rank_min_obstacle_clearance", 0.0))
        switch_rank_min_edge_margin = float(behavior.get("switch_rank_min_edge_margin", 0.0))

        best_plan: Optional[TrajectoryPlan] = None
        timed_out = False
        best_reverse_so_far = float("inf")
        vehicle_length = self._vehicle_length()
        direct_distance = math.hypot(
            self.request.dock_pose.x - self.request.start_pose.x,
            self.request.dock_pose.y - self.request.start_pose.y,
        )
        geometric_yaw = math.atan2(
            self.request.dock_pose.y - self.request.start_pose.y,
            self.request.dock_pose.x - self.request.start_pose.x,
        )
        stages = [
            {
                "name": "strict",
                "use_sector_points": True,
                "switch_yaw_limit": switch_yaw_limit,
                "switch_sector_d_min": switch_sector_d_min,
                "switch_sector_d_max": switch_sector_d_max,
                "switch_sector_angle": switch_sector_angle,
                "reverse_arc_ratio_limit": reverse_arc_ratio_limit,
                "delta_neutral_limit": delta_neutral_limit,
                "forward_length_ratio": forward_length_ratio,
                "entry_angle_limit": entry_angle_limit,
            },
            {
                "name": "balanced",
                "use_sector_points": True,
                "switch_yaw_limit": max(switch_yaw_limit, math.radians(45.0)),
                "switch_sector_d_min": max(1.0, switch_sector_d_min * 0.7),
                "switch_sector_d_max": max(switch_sector_d_max, switch_sector_d_max * 1.2),
                "switch_sector_angle": max(switch_sector_angle, math.radians(60.0)),
                "reverse_arc_ratio_limit": max(reverse_arc_ratio_limit, 0.6),
                "delta_neutral_limit": max(delta_neutral_limit, math.radians(25.0)),
                "forward_length_ratio": min(forward_length_ratio, 0.5),
                "entry_angle_limit": max(entry_angle_limit, math.radians(50.0)),
            },
            {
                "name": "fallback",
                "use_sector_points": False,
                "switch_yaw_limit": max(switch_yaw_limit, math.radians(90.0)),
                "switch_sector_d_min": 0.0,
                "switch_sector_d_max": 1e9,
                "switch_sector_angle": math.pi,
                "reverse_arc_ratio_limit": 1.0,
                "delta_neutral_limit": math.radians(89.0),
                "forward_length_ratio": 0.0,
                "entry_angle_limit": math.pi,
            },
        ]
        stage_cap = max(80, candidate_cap // max(1, len(stages)))
        # combo_count no longer includes kappa1_ratios (removed — Dubins handles it)
        combo_count = max(1, len(kappa2_ratios) * len(switch_buffers) * len(yaw_offsets))
        requested_switch_count = int(sampling.get("switch_sample_count", 40))
        stage_stats = {
            "strict": {"generated": 0.0, "accepted": 0.0},
            "balanced": {"generated": 0.0, "accepted": 0.0},
            "fallback": {"generated": 0.0, "accepted": 0.0},
        }

        for stage in stages:
            stage_generated = 0
            max_switch_from_budget = max(1, stage_cap // combo_count)
            switch_count = max(max_switch_from_budget, min(requested_switch_count, 64))
            if stage["use_sector_points"]:
                switch_points = self._switch_points_in_sector(
                    requested_count=switch_count,
                    d_min=stage["switch_sector_d_min"],
                    d_max=stage["switch_sector_d_max"],
                    sector_angle=stage["switch_sector_angle"],
                )
            else:
                switch_points = self._switch_points_basic(switch_count)
            switch_points = self._rank_switch_points(
                points=switch_points,
                desired_count=switch_count,
                min_obstacle_clearance=switch_rank_min_obstacle_clearance,
                min_edge_margin=switch_rank_min_edge_margin,
            )

            # Iterate r2/buffer/yaw_off in the outer loops and switch_points in the
            # innermost loop.  This guarantees that ALL switch points are visited for
            # each parameter combination rather than exhausting the stage budget on
            # the first 1-2 switch points (which happened when switch_points was the
            # outer loop with combo_count≈75 and stage_cap≈100).
            for r2 in kappa2_ratios:
                for s_buffer in switch_buffers:
                    for yaw_off in yaw_offsets:
                        for sw in switch_points:
                            elapsed_ms = (time.perf_counter() - t_start) * 1000.0
                            if elapsed_ms > timeout_ms:
                                timed_out = True
                                break
                            if stage_generated >= stage_cap:
                                break
                            self.debug["candidate_generated"] += 1.0
                            stage_generated += 1
                            stage_stats[stage["name"]]["generated"] += 1.0

                            kappa2 = max(1e-4, abs(self.vehicle.kappa_max_rev * r2))
                            switch_yaw = wrap_angle(geometric_yaw + yaw_off)
                            if abs(wrap_angle(switch_yaw - self.request.dock_pose.yaw)) > stage["switch_yaw_limit"]:
                                self.debug["candidate_switch_rejected"] += 1.0
                                continue
                            dock_to_switch_x = sw.x - self.request.dock_pose.x
                            dock_to_switch_y = sw.y - self.request.dock_pose.y
                            dock_to_switch_d = math.hypot(dock_to_switch_x, dock_to_switch_y)
                            dock_to_switch_ang = abs(
                                wrap_angle(math.atan2(dock_to_switch_y, dock_to_switch_x) - self.request.dock_pose.yaw)
                            )
                            if (
                                dock_to_switch_d < stage["switch_sector_d_min"]
                                or dock_to_switch_d > stage["switch_sector_d_max"]
                                or dock_to_switch_ang > stage["switch_sector_angle"]
                            ):
                                self.debug["candidate_behavior_rejected"] += 1.0
                                continue

                            d2 = math.hypot(sw.x - self.request.dock_pose.x, sw.y - self.request.dock_pose.y)
                            approx_reverse = d2 + max(0.0, s_buffer)
                            if approx_reverse > max_reverse_length_ratio * max(direct_distance, 1e-6):
                                self.debug["candidate_reverse_rejected"] += 1.0
                                continue
                            if approx_reverse > best_reverse_so_far:
                                self.debug["candidate_cost_pruned"] += 1.0
                                continue

                            d1 = math.hypot(sw.x - self.request.start_pose.x, sw.y - self.request.start_pose.y)
                            quick_fwd = d1 + max(0.0, s_buffer)
                            quick_rev = d2 + max(0.0, s_buffer)
                            quick_len = quick_fwd + quick_rev
                            quick_time = (
                                quick_fwd / max(self.vehicle.max_speed_fwd, 1e-6)
                                + quick_rev / max(self.vehicle.max_speed_rev, 1e-6)
                                + self.vehicle.gear_switch_time_penalty
                            )
                            quick_cost = (
                                self.cost_eval.weights["w_len"] * quick_len
                                + self.cost_eval.weights["w_time"] * quick_time
                                + self.cost_eval.weights["w_rev"] * quick_rev
                            )
                            if quick_cost > self.debug["best_cost"] * 1.4:
                                self.debug["candidate_cost_pruned"] += 1.0
                                continue

                            built = self._build_candidate(
                                switch_xy=sw,
                                switch_yaw=switch_yaw,
                                kappa2=kappa2,
                                s_buffer=max(0.0, s_buffer),
                                step_size=step_size,
                            )
                            if built is None:
                                continue
                            raw_segments, switch_pose = built
                            forward_length = _segment_length(raw_segments[0])
                            if forward_length < stage["forward_length_ratio"] * vehicle_length:
                                self.debug["candidate_behavior_rejected"] += 1.0
                                continue
                            kappa_switch = max(
                                abs(raw_segments[0].states[-1].curvature),
                                abs(raw_segments[1].states[0].curvature),
                            )
                            if kappa_switch > kappa_switch_limit_ratio * max(self.vehicle.kappa_max, 1e-6):
                                self.debug["candidate_switch_rejected"] += 1.0
                                continue
                            delta_switch = abs(math.atan(self.vehicle.wheel_base * kappa_switch))
                            if delta_switch > stage["delta_neutral_limit"]:
                                self.debug["candidate_behavior_rejected"] += 1.0
                                continue

                            reverse_length = _segment_length(raw_segments[1])
                            if reverse_length > max_reverse_length_ratio * max(direct_distance, 1e-6):
                                self.debug["candidate_reverse_rejected"] += 1.0
                                continue
                            if reverse_length > best_reverse_so_far:
                                self.debug["candidate_cost_pruned"] += 1.0
                                continue
                            if not self._reverse_direction_valid(raw_segments[1], stride=3):
                                self.debug["candidate_direction_rejected"] += 1.0
                                continue
                            reverse_arc_length = self._reverse_arc_length(raw_segments[1])
                            if reverse_arc_length > stage["reverse_arc_ratio_limit"] * max(reverse_length, 1e-6):
                                self.debug["candidate_behavior_rejected"] += 1.0
                                continue
                            reverse_start_yaw = raw_segments[1].states[0].yaw
                            yaw_error_to_dock = abs(wrap_angle(reverse_start_yaw - self.request.dock_pose.yaw))
                            if yaw_error_to_dock > stage["entry_angle_limit"]:
                                self.debug["candidate_behavior_rejected"] += 1.0
                                continue
                            # Monotonic approach is a strict behavioral constraint; relax
                            # it in "balanced" and "fallback" stages so that Dubins reverse
                            # paths that curve away before approaching are not rejected.
                            if stage["name"] == "strict" and not self._reverse_monotonic_approach(raw_segments[1], stride=3):
                                self.debug["candidate_behavior_rejected"] += 1.0
                                continue

                            smoothing = smooth_path_with_clothoid(
                                segments=raw_segments,
                                vehicle=self.vehicle,
                                enable_smoothing=enable_smoothing,
                            )
                            if not smoothing.success:
                                self.debug["candidate_collision_rejected"] += 1.0
                                continue
                            segments = smoothing.segments

                            if not self.collision.is_plan_collision_free(segments, stride=collision_stride):
                                self.debug["candidate_collision_rejected"] += 1.0
                                continue

                            dock_end = segments[1].states[-1]
                            pos_err = math.hypot(dock_end.x - self.request.dock_pose.x, dock_end.y - self.request.dock_pose.y)
                            yaw_err = abs(wrap_angle(dock_end.yaw - self.request.dock_pose.yaw))
                            if pos_err > self.request.tolerances.pos or yaw_err > self.request.tolerances.yaw:
                                self.debug["candidate_goal_rejected"] += 1.0
                                continue

                            dyn_stats = evaluate_steering_feasibility(
                                segments=segments,
                                vehicle=self.vehicle,
                                min_speed_threshold=min_speed_threshold,
                            )
                            if not dyn_stats.dynamic_feasible:
                                self.debug["candidate_dynamic_rejected"] += 1.0
                                continue

                            goal_error = pos_err + yaw_err
                            metrics = self._build_metrics(
                                segments=segments,
                                switch_pose=switch_pose,
                                dock_error_pos=pos_err,
                                dock_error_yaw=yaw_err,
                                compute_time_ms=elapsed_ms,
                                dynamics=dyn_stats,
                                smoothing=smoothing,
                            )
                            reverse_excess = max(0.0, metrics.reverse_length - direct_distance)
                            cost = self.cost_eval.compute(
                                total_length=metrics.total_length,
                                total_time=metrics.total_time,
                                reverse_length=metrics.reverse_length,
                                switch_yaw=switch_pose.yaw,
                                yaw_pref=self.request.planner_params.yaw_pref,
                                goal_error=goal_error,
                                steer_change_sum=dyn_stats.sum_abs_delta_steer,
                                kappa_change_sum=smoothing.kappa_change_sum,
                            ) + w_reverse_excess * reverse_excess
                            if cost < self.debug["best_cost"]:
                                self.debug["best_cost"] = cost
                                best_reverse_so_far = min(best_reverse_so_far, metrics.reverse_length)
                                self.debug["best_reverse_length"] = best_reverse_so_far
                                best_plan = TrajectoryPlan(segments=segments, switch_pose=switch_pose, metrics=metrics)
                                stage_stats[stage["name"]]["accepted"] += 1.0
                        if timed_out or stage_generated >= stage_cap:
                            break
                    if timed_out or stage_generated >= stage_cap:
                        break
                if timed_out or stage_generated >= stage_cap:
                    break
            if timed_out:
                break
            if best_plan is not None:
                self.debug["stage_used"] = {"strict": 1.0, "balanced": 2.0, "fallback": 3.0}[stage["name"]]
                break

        compute_time_ms = (time.perf_counter() - t_start) * 1000.0
        self.debug["compute_time_ms"] = compute_time_ms
        total_generated = max(1.0, self.debug["candidate_generated"])
        self.debug["ratio_collision_reject"] = self.debug["candidate_collision_rejected"] / total_generated
        self.debug["ratio_goal_reject"] = self.debug["candidate_goal_rejected"] / total_generated
        self.debug["ratio_dynamic_reject"] = self.debug["candidate_dynamic_rejected"] / total_generated
        self.debug["ratio_reverse_reject"] = self.debug["candidate_reverse_rejected"] / total_generated
        self.debug["ratio_switch_reject"] = self.debug["candidate_switch_rejected"] / total_generated
        self.debug["ratio_behavior_reject"] = self.debug["candidate_behavior_rejected"] / total_generated
        self.debug["ratio_direction_reject"] = self.debug["candidate_direction_rejected"] / total_generated
        self.debug["ratio_cost_pruned"] = self.debug["candidate_cost_pruned"] / total_generated
        self.debug["strict_generated"] = stage_stats["strict"]["generated"]
        self.debug["balanced_generated"] = stage_stats["balanced"]["generated"]
        self.debug["fallback_generated"] = stage_stats["fallback"]["generated"]
        self.debug["strict_accepted"] = stage_stats["strict"]["accepted"]
        self.debug["balanced_accepted"] = stage_stats["balanced"]["accepted"]
        self.debug["fallback_accepted"] = stage_stats["fallback"]["accepted"]

        if best_plan is not None:
            best_plan.metrics.compute_time_ms = compute_time_ms
            if timed_out:
                return PlanResponse(status="TIMEOUT", plan=best_plan, reason="timeout with best effort", debug=self.debug)
            return PlanResponse(status="OK", plan=best_plan, debug=self.debug)

        if enable_dubins_fallback:
            try:
                from .planner import Planner as ClassicDubinsPlanner

                classic_resp = ClassicDubinsPlanner(self.request).plan()
                if classic_resp.debug is None:
                    classic_resp.debug = {}
                classic_resp.debug["dubins_pp_fallback_used"] = 1.0
                classic_resp.debug["dubins_pp_compute_time_ms"] = compute_time_ms
                classic_resp.debug["dubins_pp_candidate_generated"] = self.debug.get("candidate_generated", 0.0)
                return classic_resp
            except Exception:
                # Fallback failure should not mask original dubins++ outcome.
                self.debug["dubins_pp_fallback_used"] = -1.0

        if timed_out:
            return PlanResponse(status="TIMEOUT", plan=None, reason="timeout without feasible path", debug=self.debug)
        return PlanResponse(status="NO_FEASIBLE_PATH", plan=None, reason="No collision-free candidate", debug=self.debug)
