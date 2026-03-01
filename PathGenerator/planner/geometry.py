from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

Point = Tuple[float, float]
Polygon = List[Point]


_EPS = 1e-9


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def mod2pi(angle: float) -> float:
    return angle % (2.0 * math.pi)


def point_on_segment(p: Point, a: Point, b: Point) -> bool:
    (px, py), (ax, ay), (bx, by) = p, a, b
    cross = (px - ax) * (by - ay) - (py - ay) * (bx - ax)
    if abs(cross) > 1e-8:
        return False
    dot = (px - ax) * (bx - ax) + (py - ay) * (by - ay)
    if dot < -_EPS:
        return False
    sq_len = (bx - ax) ** 2 + (by - ay) ** 2
    return dot <= sq_len + _EPS


def point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    x, y = point
    inside = False
    n = len(polygon)
    if n < 3:
        return False
    for i in range(n):
        a = polygon[i]
        b = polygon[(i + 1) % n]
        if point_on_segment(point, a, b):
            return True
        x1, y1 = a
        x2, y2 = b
        cond = (y1 > y) != (y2 > y)
        if cond:
            x_cross = (x2 - x1) * (y - y1) / (y2 - y1 + _EPS) + x1
            if x < x_cross:
                inside = not inside
    return inside


def point_to_segment_distance(point: Point, a: Point, b: Point) -> float:
    px, py = point
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    vv = vx * vx + vy * vy
    if vv <= _EPS:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / vv))
    cx, cy = ax + t * vx, ay + t * vy
    return math.hypot(px - cx, py - cy)


def point_to_polygon_distance(point: Point, polygon: Sequence[Point]) -> float:
    if point_in_polygon(point, polygon):
        return 0.0
    d_min = float("inf")
    n = len(polygon)
    for i in range(n):
        d = point_to_segment_distance(point, polygon[i], polygon[(i + 1) % n])
        d_min = min(d_min, d)
    return d_min


def point_in_any_polygon(point: Point, polygons: Iterable[Sequence[Point]]) -> bool:
    return any(point_in_polygon(point, poly) for poly in polygons)


def bbox_of_polygons(polygons: Iterable[Sequence[Point]]) -> Tuple[float, float, float, float]:
    xs: List[float] = []
    ys: List[float] = []
    for poly in polygons:
        for x, y in poly:
            xs.append(x)
            ys.append(y)
    if not xs:
        raise ValueError("No polygon points given")
    return min(xs), min(ys), max(xs), max(ys)
