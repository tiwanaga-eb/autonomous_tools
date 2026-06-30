"""Coarse path planning for the swing / approach.

Two-tier with the CBF: this provides *global* avoidance (pick a swing direction
that clears the truck), while the CBF is the online insurance during tracking.

Minimal implementation: scan the swing angle in 1-D and pick a clear interval
toward the goal, treating the truck footprint as a forbidden swing sector.
"""
from __future__ import annotations

import numpy as np


def truck_swing_sector(cfg) -> tuple[float, float]:
    """Angular sector (min,max swing angle, rad) occupied by the truck as seen
    from the swing axis, padded by a margin."""
    t = cfg["safety"]["truck"]
    pos = np.array(t["pos"], dtype=float)
    vc = pos + np.array(t["vessel_offset"])
    cab = pos + np.array(t["cab_offset"])
    angs = [np.arctan2(p[1], p[0]) for p in (vc, cab)]
    half = np.arctan2(max(t["vessel_half"][1], t["cab_half"][1]) + t["margin"],
                      np.linalg.norm(vc[:2]) + 1e-6)
    return (min(angs) - half, max(angs) + half)


def plan_swing(cfg, current: float, goal: float, avoid_truck: bool = False):
    """Return a sequence of swing waypoints from current to goal that, if
    ``avoid_truck``, does not sweep the bucket through the truck sector."""
    if not avoid_truck:
        return [goal]
    lo, hi = truck_swing_sector(cfg)
    # if direct path doesn't cross the sector, go direct
    a, b = sorted((current, goal))
    crosses = not (b < lo or a > hi)
    if not crosses:
        return [goal]
    # route the long way around (away from the sector midpoint)
    mid = 0.5 * (lo + hi)
    direction = 1.0 if goal > current else -1.0
    if (mid - current) * direction > 0:      # goal is past the sector -> reverse
        direction *= -1.0
    return [current + direction * np.pi * 0.5, goal]
