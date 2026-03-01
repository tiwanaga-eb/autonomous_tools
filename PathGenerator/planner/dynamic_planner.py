from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .astar import PriorityOpenSet
from .collision import CollisionChecker
from .geometry import wrap_angle
from .models import PlanMetrics, PlanRequest, PlanResponse, Pose, Segment, State, TrajectoryPlan
from .primitives import generate_actions, rollout_primitive
from .vehicle_config import VehicleConfig


@dataclass
class MPNode:
    x: float
    y: float
    yaw: float
    delta: float
    gear: str
    switch_used: int
    t: float


@dataclass
class SearchStats:
    nodes_expanded: int = 0
    primitives_tested: int = 0
    collision_rejected: int = 0
    heuristic_calls: int = 0
    pruned_by_corridor: int = 0
    pruned_by_dominance: int = 0
    pruned_by_distance_increase: int = 0


def _segment_length(segment: Segment) -> float:
    out = 0.0
    for i in range(1, len(segment.states)):
        dx = segment.states[i].x - segment.states[i - 1].x
        dy = segment.states[i].y - segment.states[i - 1].y
        out += math.hypot(dx, dy)
    return out


def _states_length(states: List[State]) -> float:
    out = 0.0
    for i in range(1, len(states)):
        dx = states[i].x - states[i - 1].x
        dy = states[i].y - states[i - 1].y
        out += math.hypot(dx, dy)
    return out


def _count_gear_switches(segments: List[Segment]) -> int:
    gears = [s.gear for s in segments if s.states]
    if len(gears) < 2:
        return 0
    return sum(1 for i in range(1, len(gears)) if gears[i] != gears[i - 1])


def _angle_bin(yaw: float, bins: int) -> int:
    y = wrap_angle(yaw)
    return int(round(((y + math.pi) / (2.0 * math.pi)) * bins)) % bins


def _discrete_key(node: MPNode, xy_res: float, yaw_bins: int, delta_bins: int, max_steer: float) -> Tuple[int, int, int, int, str, int]:
    xk = int(round(node.x / xy_res))
    yk = int(round(node.y / xy_res))
    yawk = _angle_bin(node.yaw, yaw_bins)
    if max_steer <= 1e-9:
        deltak = 0
    else:
        norm = (node.delta + max_steer) / (2.0 * max_steer)
        deltak = max(0, min(delta_bins - 1, int(round(norm * (delta_bins - 1)))))
    return (xk, yk, yawk, deltak, node.gear, node.switch_used)


def _euclid(x: float, y: float, goal: Pose) -> float:
    return math.hypot(goal.x - x, goal.y - y)


def _approx_dubins_lb(node: MPNode, goal: Pose, vehicle: VehicleConfig) -> float:
    # Fast lower-bound proxy (no heavy solver call): straight + heading mismatch penalty.
    dist = _euclid(node.x, node.y, goal)
    heading = abs(wrap_angle(node.yaw - goal.yaw))
    return dist + vehicle.r_min * heading


def _heuristic(
    node: MPNode,
    goal: Pose,
    vehicle: VehicleConfig,
    params: Dict[str, float],
    stats: SearchStats,
) -> float:
    stats.heuristic_calls += 1
    w_pos = float(params.get("h_w_pos", 1.0))
    w_yaw = float(params.get("h_w_yaw", 0.6))
    w_delta = float(params.get("h_w_delta", 0.3))
    w_dubins = float(params.get("h_w_dubins", 0.0))

    h = 0.0
    h += w_pos * _euclid(node.x, node.y, goal)
    h += w_yaw * abs(wrap_angle(node.yaw - goal.yaw))
    h += w_delta * abs(node.delta)
    if w_dubins > 0.0:
        h += w_dubins * _approx_dubins_lb(node, goal, vehicle)
    return h


def _point_line_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    vv = vx * vx + vy * vy
    if vv < 1e-12:
        return math.hypot(px - ax, py - ay)
    # infinite line perpendicular distance
    cross = abs(vx * wy - vy * wx)
    return cross / math.sqrt(vv)


def _goal_reached(node: MPNode, goal: Pose, pos_tol: float, yaw_tol: float, require_gear: Optional[str]) -> bool:
    if require_gear and node.gear != require_gear:
        return False
    if math.hypot(node.x - goal.x, node.y - goal.y) > pos_tol:
        return False
    if abs(wrap_angle(node.yaw - goal.yaw)) > yaw_tol:
        return False
    return True


def _states_to_segments(states: List[State]) -> List[Segment]:
    if not states:
        return []
    segments: List[Segment] = []
    cur = [states[0]]
    cur_gear = states[0].gear
    for s in states[1:]:
        if s.gear == cur_gear:
            cur.append(s)
        else:
            segments.append(Segment(gear=cur_gear, states=cur))
            cur = [s]
            cur_gear = s.gear
    segments.append(Segment(gear=cur_gear, states=cur))
    return segments


def _concat_stage_states(stage_a: List[State], stage_b: List[State]) -> List[State]:
    if not stage_a:
        return stage_b
    if not stage_b:
        return stage_a
    out = list(stage_a)
    t_offset = out[-1].t
    for i, s in enumerate(stage_b):
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
    return out


def _reconstruct_states(
    start: MPNode,
    key: Tuple,
    parent: Dict[Tuple, Optional[Tuple]],
    traj: Dict[Tuple, List[State]],
    vehicle: VehicleConfig,
) -> List[State]:
    path_states: List[State] = []
    chain: List[Tuple] = []
    k = key
    while k is not None:
        chain.append(k)
        k = parent[k]
    chain.reverse()

    start_state = State(
        x=start.x,
        y=start.y,
        yaw=start.yaw,
        curvature=math.tan(start.delta) / max(vehicle.wheel_base, 1e-6),
        gear=start.gear,
        v=0.0,
        t=start.t,
    )
    path_states.append(start_state)
    for ck in chain[1:]:
        path_states.extend(traj[ck])
    return path_states


def _search(
    start: MPNode,
    goal: Pose,
    vehicle: VehicleConfig,
    collision: CollisionChecker,
    timeout_s: float,
    params: Dict[str, float],
    require_gear: Optional[str],
    forward_only: bool,
    pos_tol: float,
    yaw_tol: float,
    stage_mode: str = "generic",
    dock_pose: Optional[Pose] = None,
    corridor_line: Optional[Tuple[Pose, Pose]] = None,
) -> Tuple[Optional[List[State]], SearchStats, bool]:
    xy_res = max(1e-3, float(params.get("xy_res", 0.5)))
    yaw_bins = max(8, int(params.get("yaw_bins", 36)))
    delta_bins = max(3, int(params.get("delta_bins", 21)))
    T = max(1e-3, float(params.get("T", 0.5)))
    dt = max(1e-3, float(params.get("dt", 0.05)))
    astar_weight = max(0.1, float(params.get("astar_weight", 1.5)))
    speed_levels = max(1, int(params.get("speed_levels", 2)))
    delta_dot_levels = max(1, int(params.get("delta_dot_levels", 5)))
    collision_stride = max(1, int(params.get("collision_check_stride", 1)))
    v_fwd = abs(float(params.get("v_fwd", vehicle.max_speed_fwd)))
    v_rev = abs(float(params.get("v_rev", vehicle.max_speed_rev)))
    timeout_close_factor = max(1.0, float(params.get("timeout_close_factor", 3.0)))
    switch_distance_threshold = max(0.0, float(params.get("switch_distance_threshold", 18.0)))
    max_nodes_expanded = max(500, int(params.get("max_nodes_expanded", 18000)))
    corridor_width = max(0.0, float(params.get("corridor_width", 12.0)))
    distance_increase_limit = max(0.0, float(params.get("distance_increase_limit", 3.0)))
    near_goal_threshold = max(0.0, float(params.get("near_goal_threshold", 6.0)))
    near_goal_T = min(T, max(1e-3, float(params.get("near_goal_T", 0.2))))
    relaxed_tol_pos = max(pos_tol, float(params.get("relaxed_tol_pos", pos_tol * 1.5)))
    relaxed_tol_yaw = max(yaw_tol, float(params.get("relaxed_tol_yaw", yaw_tol * 1.5)))

    t0 = time.perf_counter()
    open_set = PriorityOpenSet()
    stats = SearchStats()

    root_key = _discrete_key(start, xy_res, yaw_bins, delta_bins, vehicle.max_steer_angle)
    best_g: Dict[Tuple, float] = {root_key: 0.0}
    best_key_info: Dict[Tuple, Tuple[float, float]] = {root_key: (0.0, abs(wrap_angle(start.yaw - goal.yaw)))}
    parent: Dict[Tuple, Optional[Tuple]] = {root_key: None}
    end_state: Dict[Tuple, MPNode] = {root_key: start}
    traj: Dict[Tuple, List[State]] = {root_key: []}

    h0 = _heuristic(start, goal, vehicle, params, stats)
    open_set.push(h0, 0.0, root_key)

    best_close_key = root_key
    best_close_score = math.inf

    while len(open_set) > 0:
        if time.perf_counter() - t0 > timeout_s:
            best_node = end_state[best_close_key]
            can_return = (
                best_close_score <= max(pos_tol * timeout_close_factor, pos_tol)
                and (require_gear is None or best_node.gear == require_gear)
            )
            if can_return:
                return _reconstruct_states(start, best_close_key, parent, traj, vehicle), stats, True
            return None, stats, True

        item = open_set.pop()
        if item is None:
            break
        cur_key = item.key
        if item.g > best_g.get(cur_key, float("inf")) + 1e-9:
            continue
        cur_node = end_state[cur_key]
        cur_g = best_g[cur_key]

        close_score = math.hypot(cur_node.x - goal.x, cur_node.y - goal.y)
        if close_score < best_close_score:
            best_close_score = close_score
            best_close_key = cur_key

        if _goal_reached(cur_node, goal, pos_tol, yaw_tol, require_gear):
            return _reconstruct_states(start, cur_key, parent, traj, vehicle), stats, False
        if _goal_reached(cur_node, goal, relaxed_tol_pos, relaxed_tol_yaw, require_gear):
            return _reconstruct_states(start, cur_key, parent, traj, vehicle), stats, False

        stats.nodes_expanded += 1
        if stats.nodes_expanded >= max_nodes_expanded:
            best_node = end_state[best_close_key]
            can_return = (
                best_close_score <= max(pos_tol * timeout_close_factor, pos_tol)
                and (require_gear is None or best_node.gear == require_gear)
            )
            if can_return:
                return _reconstruct_states(start, best_close_key, parent, traj, vehicle), stats, True
            return None, stats, True

        dist_to_goal = _euclid(cur_node.x, cur_node.y, goal)
        allow_reverse = (not forward_only) and (
            cur_node.gear == "R" or cur_node.switch_used == 1 or dist_to_goal <= switch_distance_threshold
        )

        actions = generate_actions(
            vehicle=vehicle,
            current_gear=cur_node.gear,
            switch_used=cur_node.switch_used,
            forward_only=forward_only,
            allow_reverse=allow_reverse,
            speed_levels=speed_levels,
            delta_dot_levels=delta_dot_levels,
            v_fwd=v_fwd,
            v_rev=v_rev,
        )

        # Steering bias toward reducing yaw error.
        if actions:
            scored_actions = []
            for a in actions:
                if abs(cur_node.delta) > vehicle.max_steer_angle * 0.95 and abs(a.delta_dot) > 1e-9:
                    continue
                delta_pred = max(
                    -vehicle.max_steer_angle,
                    min(vehicle.max_steer_angle, cur_node.delta + a.delta_dot * T * 0.5),
                )
                yaw_pred = wrap_angle(cur_node.yaw + a.v * math.tan(delta_pred) / max(vehicle.wheel_base, 1e-6) * T)
                yaw_err_pred = abs(wrap_angle(goal.yaw - yaw_pred))
                scored_actions.append((yaw_err_pred, abs(a.delta_dot), a))
            scored_actions.sort(key=lambda x: (x[0], x[1]))
            actions = [x[2] for x in scored_actions]

        for a in actions:
            stats.primitives_tested += 1
            next_switch = cur_node.switch_used
            if cur_node.gear == "F" and a.target_gear == "R":
                next_switch = 1

            local_T = near_goal_T if dist_to_goal < near_goal_threshold else T

            r = rollout_primitive(
                x=cur_node.x,
                y=cur_node.y,
                yaw=cur_node.yaw,
                delta=cur_node.delta,
                t0=cur_node.t,
                action=a,
                vehicle=vehicle,
                T=local_T,
                dt=dt,
            )

            seg = Segment(gear=a.target_gear, states=r.states)
            if not collision.is_segment_collision_free(seg, stride=collision_stride):
                stats.collision_rejected += 1
                continue

            nnode = MPNode(
                x=r.end_x,
                y=r.end_y,
                yaw=r.end_yaw,
                delta=r.end_delta,
                gear=r.gear,
                switch_used=next_switch,
                t=r.states[-1].t if r.states else cur_node.t,
            )

            if stage_mode == "A2" and corridor_line is not None:
                p0, p1 = corridor_line
                d_perp = _point_line_distance(nnode.x, nnode.y, p0.x, p0.y, p1.x, p1.y)
                if d_perp > corridor_width:
                    stats.pruned_by_corridor += 1
                    continue

            if stage_mode == "A1" and dock_pose is not None:
                d_cur = _euclid(cur_node.x, cur_node.y, dock_pose)
                d_new = _euclid(nnode.x, nnode.y, dock_pose)
                if d_new > d_cur + distance_increase_limit:
                    stats.pruned_by_distance_increase += 1
                    continue

            nkey = _discrete_key(nnode, xy_res, yaw_bins, delta_bins, vehicle.max_steer_angle)

            g_new = cur_g
            g_new += r.dist
            g_new += local_T
            if r.gear == "R":
                g_new += float(params.get("reverse_penalty", 1.5)) * r.dist
            if a.target_gear != cur_node.gear:
                g_new += max(0.0, vehicle.gear_switch_time_penalty)
            g_new += float(params.get("steer_effort_penalty", 0.2)) * r.steer_effort

            yaw_err_new = abs(wrap_angle(nnode.yaw - goal.yaw))
            old = best_key_info.get(nkey)
            if old is not None:
                old_g, old_yaw = old
                if old_g <= g_new and old_yaw <= yaw_err_new:
                    stats.pruned_by_dominance += 1
                    continue

            prev_g = best_g.get(nkey)
            if prev_g is not None and prev_g <= g_new:
                stats.pruned_by_dominance += 1
                continue
            best_key_info[nkey] = (g_new, yaw_err_new)

            best_g[nkey] = g_new
            parent[nkey] = cur_key
            end_state[nkey] = nnode
            traj[nkey] = r.states

            h = _heuristic(nnode, goal, vehicle, params, stats)
            f = g_new + astar_weight * h
            open_set.push(f, g_new, nkey)

    # no solution
    return None, stats, False


class MotionPrimitivesPlanner:
    def __init__(self, request: PlanRequest, vehicle: VehicleConfig):
        self.request = request
        self.vehicle = vehicle
        self.params = {
            "xy_res": 0.5,          # finer grid for better path quality (was 1.0)
            "yaw_bins": 24,
            "delta_bins": 11,
            "T": 0.6,
            "dt": 0.1,
            "v_fwd": vehicle.max_speed_fwd,
            "v_rev": vehicle.max_speed_rev,
            "delta_dot_levels": 5,  # more steering resolution (was 3)
            "speed_levels": 1,
            "astar_weight": 1.8,
            "collision_check_stride": 3,
            "h_w_pos": 1.0,
            "h_w_yaw": 0.6,
            "h_w_delta": 0.3,
            "h_w_dubins": 0.5,      # stronger Dubins lower-bound heuristic (was 0.2)
            "timeout_close_factor": 3.0,
            "switch_distance_threshold": 18.0,
            "max_nodes_expanded": 25000,  # more budget for complex scenes (was 18000)
            "corridor_width": 12.0,
            "distance_increase_limit": 3.0,
            "near_goal_threshold": 6.0,
            "near_goal_T": 0.2,
            **request.planner_params.primitives_params,
        }

    def _build_metrics(self, segments: List[Segment], switch_pose: Pose, compute_time_ms: float, stats: SearchStats) -> PlanMetrics:
        forward_length = sum(_segment_length(s) for s in segments if s.gear == "F")
        reverse_length = sum(_segment_length(s) for s in segments if s.gear == "R")
        total_length = forward_length + reverse_length
        total_time = 0.0
        for s in segments:
            if not s.states:
                continue
            dt = s.states[-1].t - s.states[0].t
            total_time += max(0.0, dt)
        total_time += _count_gear_switches(segments) * self.vehicle.gear_switch_time_penalty

        dock_end = None
        for seg in segments:
            if seg.gear == "R":
                dock_end = seg.states[-1]
        if dock_end is None:
            dock_end = segments[-1].states[-1]

        dock_error_pos = math.hypot(dock_end.x - self.request.dock_pose.x, dock_end.y - self.request.dock_pose.y)
        dock_error_yaw = abs(wrap_angle(dock_end.yaw - self.request.dock_pose.yaw))

        return PlanMetrics(
            total_length=total_length,
            total_time=total_time,
            forward_length=forward_length,
            reverse_length=reverse_length,
            switch_yaw=switch_pose.yaw,
            dock_error_pos=dock_error_pos,
            dock_error_yaw=dock_error_yaw,
            collision_free=True,
            dynamic_feasible=True,
            max_steering_change=0.0,
            max_steering_rate_required=0.0,
            max_kappa_rate=0.0,
            smoothing_applied=False,
            added_smoothing_length=0.0,
            compute_time_ms=compute_time_ms,
        )

    def plan(self) -> PlanResponse:
        start_t = time.perf_counter()
        timeout_s = float(self.request.planner_params.timeout_ms) / 1000.0

        collision = CollisionChecker(
            drivable_polygons=self.request.map.drivable_polygons,
            obstacle_polygons=self.request.map.obstacle_polygons,
            safety_margin=self.request.map.safety_margin,
            vehicle_radius=self.vehicle.bounding_radius,
            road_width=self.vehicle.road_width,
        )

        start_node = MPNode(
            x=self.request.start_pose.x,
            y=self.request.start_pose.y,
            yaw=self.request.start_pose.yaw,
            delta=0.0,
            gear="F",
            switch_used=0,
            t=0.0,
        )

        # Decomposed stage A:
        #   A1) forward-only to a switch candidate near dock
        #   A2) reverse-only from switch candidate to dock
        switch_backoffs = self.params.get("switch_backoffs", [8.0, 12.0, 16.0, 20.0])
        if not isinstance(switch_backoffs, list) or not switch_backoffs:
            switch_backoffs = [8.0, 12.0, 16.0, 20.0]
        switch_pos_tol = float(self.params.get("switch_pos_tol", max(1.5, self.request.tolerances.pos * 4.0)))
        switch_yaw_tol = float(self.params.get("switch_yaw_tol", max(0.5, self.request.tolerances.yaw * 3.0)))

        best_stage_a_states: Optional[List[State]] = None
        best_stage_a_stats = SearchStats()
        best_stage_a_score = float("inf")
        best_stage_a_backoff = -1.0
        stage_a_candidate_count = 0
        timed_out_a = False

        for backoff in switch_backoffs:
            elapsed = time.perf_counter() - start_t
            remain = max(0.01, timeout_s - elapsed)
            if remain <= 0.01:
                timed_out_a = True
                break

            switch_goal = Pose(
                x=self.request.dock_pose.x + backoff * math.cos(self.request.dock_pose.yaw),
                y=self.request.dock_pose.y + backoff * math.sin(self.request.dock_pose.yaw),
                yaw=self.request.dock_pose.yaw,
            )

            stage_f_states, stats_f, timed_out_f = _search(
                start=start_node,
                goal=switch_goal,
                vehicle=self.vehicle,
                collision=collision,
                timeout_s=remain,
                params=self.params,
                require_gear="F",
                forward_only=True,
                pos_tol=switch_pos_tol,
                yaw_tol=switch_yaw_tol,
                stage_mode="A1",
                dock_pose=self.request.dock_pose,
            )
            best_stage_a_stats.nodes_expanded += stats_f.nodes_expanded
            best_stage_a_stats.primitives_tested += stats_f.primitives_tested
            best_stage_a_stats.collision_rejected += stats_f.collision_rejected
            best_stage_a_stats.heuristic_calls += stats_f.heuristic_calls
            best_stage_a_stats.pruned_by_corridor += stats_f.pruned_by_corridor
            best_stage_a_stats.pruned_by_dominance += stats_f.pruned_by_dominance
            best_stage_a_stats.pruned_by_distance_increase += stats_f.pruned_by_distance_increase
            timed_out_a = timed_out_a or timed_out_f
            if stage_f_states is None:
                continue

            sw_last = stage_f_states[-1]
            stage_rev_start = MPNode(
                x=sw_last.x,
                y=sw_last.y,
                yaw=sw_last.yaw,
                delta=math.atan(self.vehicle.wheel_base * sw_last.curvature),
                gear="R",
                switch_used=1,
                t=0.0,
            )
            elapsed = time.perf_counter() - start_t
            remain = max(0.01, timeout_s - elapsed)
            stage_r_states, stats_r, timed_out_r = _search(
                start=stage_rev_start,
                goal=self.request.dock_pose,
                vehicle=self.vehicle,
                collision=collision,
                timeout_s=remain,
                params=self.params,
                require_gear="R",
                forward_only=True,
                pos_tol=self.request.tolerances.pos,
                yaw_tol=self.request.tolerances.yaw,
                stage_mode="A2",
                corridor_line=(self.request.start_pose, self.request.dock_pose),
            )
            best_stage_a_stats.nodes_expanded += stats_r.nodes_expanded
            best_stage_a_stats.primitives_tested += stats_r.primitives_tested
            best_stage_a_stats.collision_rejected += stats_r.collision_rejected
            best_stage_a_stats.heuristic_calls += stats_r.heuristic_calls
            best_stage_a_stats.pruned_by_corridor += stats_r.pruned_by_corridor
            best_stage_a_stats.pruned_by_dominance += stats_r.pruned_by_dominance
            best_stage_a_stats.pruned_by_distance_increase += stats_r.pruned_by_distance_increase
            timed_out_a = timed_out_a or timed_out_r
            if stage_r_states is None:
                continue

            candidate_states = _concat_stage_states(stage_f_states, stage_r_states)
            stage_a_candidate_count += 1
            total_len = _states_length(candidate_states)
            dock_last = stage_r_states[-1]
            dock_pos_err = math.hypot(dock_last.x - self.request.dock_pose.x, dock_last.y - self.request.dock_pose.y)
            dock_yaw_err = abs(wrap_angle(dock_last.yaw - self.request.dock_pose.yaw))
            candidate_score = total_len + 4.0 * dock_pos_err + self.vehicle.r_min * dock_yaw_err
            if candidate_score < best_stage_a_score:
                best_stage_a_score = candidate_score
                best_stage_a_backoff = float(backoff)
                best_stage_a_states = candidate_states

        stage_a_states = best_stage_a_states
        stats_a = best_stage_a_stats

        if stage_a_states is None:
            debug = {
                "candidate_generated": float(stats_a.nodes_expanded),
                "nodes_expanded": float(stats_a.nodes_expanded),
                "primitives_tested": float(stats_a.primitives_tested),
                "collision_rejected": float(stats_a.collision_rejected),
                "heuristic_calls": float(stats_a.heuristic_calls),
                "pruned_by_corridor": float(stats_a.pruned_by_corridor),
                "pruned_by_dominance": float(stats_a.pruned_by_dominance),
                "pruned_by_distance_increase": float(stats_a.pruned_by_distance_increase),
                "average_branching_factor": float(stats_a.primitives_tested / max(stats_a.nodes_expanded, 1)),
                "stage_a_candidates_found": float(stage_a_candidate_count),
                "stage_a_best_score": float(best_stage_a_score if best_stage_a_score < float("inf") else -1.0),
                "stage_a_best_backoff": float(best_stage_a_backoff),
                "compute_time_ms": (time.perf_counter() - start_t) * 1000.0,
            }
            if timed_out_a:
                return PlanResponse(status="TIMEOUT", plan=None, reason="primitives stage A timeout", debug=debug)
            return PlanResponse(status="NO_FEASIBLE_PATH", plan=None, reason="primitives stage A failed", debug=debug)

        combined_states = stage_a_states
        timed_out_any = timed_out_a
        stats_total = SearchStats(
            nodes_expanded=stats_a.nodes_expanded,
            primitives_tested=stats_a.primitives_tested,
            collision_rejected=stats_a.collision_rejected,
            heuristic_calls=stats_a.heuristic_calls,
            pruned_by_corridor=stats_a.pruned_by_corridor,
            pruned_by_dominance=stats_a.pruned_by_dominance,
            pruned_by_distance_increase=stats_a.pruned_by_distance_increase,
        )

        if self.request.exit_pose is not None:
            dock_last = stage_a_states[-1]
            stage_b_start = MPNode(
                x=dock_last.x,
                y=dock_last.y,
                yaw=dock_last.yaw,
                delta=math.atan(self.vehicle.wheel_base * dock_last.curvature),
                gear="F",
                switch_used=1,
                t=0.0,
            )
            elapsed = time.perf_counter() - start_t
            remain = max(0.01, timeout_s - elapsed)
            stage_b_states, stats_b, timed_out_b = _search(
                start=stage_b_start,
                goal=self.request.exit_pose,
                vehicle=self.vehicle,
                collision=collision,
                timeout_s=remain,
                params=self.params,
                require_gear="F",
                forward_only=True,
                pos_tol=self.request.tolerances.pos,
                yaw_tol=self.request.tolerances.yaw,
                stage_mode="B",
            )
            stats_total.nodes_expanded += stats_b.nodes_expanded
            stats_total.primitives_tested += stats_b.primitives_tested
            stats_total.collision_rejected += stats_b.collision_rejected
            stats_total.heuristic_calls += stats_b.heuristic_calls
            stats_total.pruned_by_corridor += stats_b.pruned_by_corridor
            stats_total.pruned_by_dominance += stats_b.pruned_by_dominance
            stats_total.pruned_by_distance_increase += stats_b.pruned_by_distance_increase
            timed_out_any = timed_out_any or timed_out_b

            if stage_b_states is None:
                debug = {
                    "candidate_generated": float(stats_total.nodes_expanded),
                    "nodes_expanded": float(stats_total.nodes_expanded),
                    "primitives_tested": float(stats_total.primitives_tested),
                    "collision_rejected": float(stats_total.collision_rejected),
                    "heuristic_calls": float(stats_total.heuristic_calls),
                    "pruned_by_corridor": float(stats_total.pruned_by_corridor),
                    "pruned_by_dominance": float(stats_total.pruned_by_dominance),
                    "pruned_by_distance_increase": float(stats_total.pruned_by_distance_increase),
                    "average_branching_factor": float(stats_total.primitives_tested / max(stats_total.nodes_expanded, 1)),
                    "stage_a_candidates_found": float(stage_a_candidate_count),
                    "stage_a_best_score": float(best_stage_a_score if best_stage_a_score < float("inf") else -1.0),
                    "stage_a_best_backoff": float(best_stage_a_backoff),
                    "compute_time_ms": (time.perf_counter() - start_t) * 1000.0,
                }
                if timed_out_b:
                    return PlanResponse(status="TIMEOUT", plan=None, reason="primitives stage B timeout", debug=debug)
                return PlanResponse(status="NO_FEASIBLE_PATH", plan=None, reason="primitives stage B failed", debug=debug)

            combined_states = _concat_stage_states(stage_a_states, stage_b_states)

        segments = _states_to_segments(combined_states)
        if not segments:
            return PlanResponse(status="NO_FEASIBLE_PATH", plan=None, reason="no segments")

        switch_pose = Pose(x=segments[0].states[0].x, y=segments[0].states[0].y, yaw=segments[0].states[0].yaw)
        found_switch = False
        for seg in segments:
            if seg.gear == "R":
                first = seg.states[0]
                switch_pose = Pose(x=first.x, y=first.y, yaw=first.yaw)
                found_switch = True
                break
        if not found_switch and len(segments) > 1:
            st = segments[1].states[0]
            switch_pose = Pose(x=st.x, y=st.y, yaw=st.yaw)

        compute_time_ms = (time.perf_counter() - start_t) * 1000.0
        metrics = self._build_metrics(segments, switch_pose, compute_time_ms, stats_total)

        debug = {
            "candidate_generated": float(stats_total.nodes_expanded),
            "nodes_expanded": float(stats_total.nodes_expanded),
            "primitives_tested": float(stats_total.primitives_tested),
            "collision_rejected": float(stats_total.collision_rejected),
            "heuristic_calls": float(stats_total.heuristic_calls),
            "pruned_by_corridor": float(stats_total.pruned_by_corridor),
            "pruned_by_dominance": float(stats_total.pruned_by_dominance),
            "pruned_by_distance_increase": float(stats_total.pruned_by_distance_increase),
            "average_branching_factor": float(stats_total.primitives_tested / max(stats_total.nodes_expanded, 1)),
            "stage_a_candidates_found": float(stage_a_candidate_count),
            "stage_a_best_score": float(best_stage_a_score if best_stage_a_score < float("inf") else -1.0),
            "stage_a_best_backoff": float(best_stage_a_backoff),
            "compute_time_ms": compute_time_ms,
        }
        status = "TIMEOUT" if timed_out_any else "OK"
        reason = "primitives best-effort under timeout" if timed_out_any else None
        return PlanResponse(
            status=status,
            plan=TrajectoryPlan(segments=segments, switch_pose=switch_pose, metrics=metrics),
            reason=reason,
            debug=debug,
        )
