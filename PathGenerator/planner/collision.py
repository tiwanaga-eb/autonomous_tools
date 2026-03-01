from __future__ import annotations

import math
from typing import Iterable, List

from .geometry import point_in_any_polygon, point_to_polygon_distance
from .models import Segment, State


class CollisionChecker:
    def __init__(
        self,
        drivable_polygons,
        obstacle_polygons,
        safety_margin: float,
        vehicle_radius: float,
        road_width: float = 0.0,
    ):
        self.drivable_polygons = drivable_polygons
        self.obstacle_polygons = obstacle_polygons
        self.clearance = safety_margin + vehicle_radius
        self.road_half_width = max(0.0, road_width) * 0.5
        self.road_edge_clearance = max(0.0, safety_margin)

    def _point_collision(self, p, clearance: float) -> bool:
        if not point_in_any_polygon(p, self.drivable_polygons):
            return True
        for obs in self.obstacle_polygons:
            if point_to_polygon_distance(p, obs) < clearance:
                return True
        return False

    def _state_collision(self, state: State) -> bool:
        p = (state.x, state.y)
        if self._point_collision(p, self.clearance):
            return True

        # Ensure the requested road width envelope also stays in drivable and avoids obstacles.
        if self.road_half_width > 1e-6:
            nx = -math.sin(state.yaw)
            ny = math.cos(state.yaw)
            left = (state.x + nx * self.road_half_width, state.y + ny * self.road_half_width)
            right = (state.x - nx * self.road_half_width, state.y - ny * self.road_half_width)
            if self._point_collision(left, self.road_edge_clearance):
                return True
            if self._point_collision(right, self.road_edge_clearance):
                return True
        return False

    def is_segment_collision_free(self, segment: Segment, stride: int = 1) -> bool:
        step = max(1, stride)
        for i, s in enumerate(segment.states):
            if i % step != 0 and i != len(segment.states) - 1:
                continue
            if self._state_collision(s):
                return False
        return True

    def is_plan_collision_free(self, segments: Iterable[Segment], stride: int = 1) -> bool:
        return all(self.is_segment_collision_free(seg, stride=stride) for seg in segments)
