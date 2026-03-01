from __future__ import annotations

import math
import random
from typing import List, Optional

from .geometry import bbox_of_polygons, point_in_any_polygon
from .models import Pose


class SwitchSampler:
    def __init__(self, drivable_polygons, seed: Optional[int] = None):
        self.drivable_polygons = drivable_polygons
        self.rng = random.Random(seed)
        self.bbox = bbox_of_polygons(drivable_polygons)

    def sample(self, switch_sample_count: int, yaw_bins: int) -> List[Pose]:
        min_x, min_y, max_x, max_y = self.bbox
        yaw_bins = max(1, yaw_bins)
        samples: List[Pose] = []
        attempts = 0
        max_attempts = max(100, switch_sample_count * 20)

        while len(samples) < switch_sample_count and attempts < max_attempts:
            attempts += 1
            x = self.rng.uniform(min_x, max_x)
            y = self.rng.uniform(min_y, max_y)
            if not point_in_any_polygon((x, y), self.drivable_polygons):
                continue
            b = len(samples) % yaw_bins
            yaw = -math.pi + (2.0 * math.pi) * (b / yaw_bins)
            samples.append(Pose(x=x, y=y, yaw=yaw))
        return samples
