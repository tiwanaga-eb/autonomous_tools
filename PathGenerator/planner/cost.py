from __future__ import annotations

import math
from typing import Dict, Optional

from .geometry import wrap_angle


class CostEvaluator:
    def __init__(self, weights: Dict[str, float]):
        self.weights = {
            "w_len": 1.0,
            "w_time": 1.0,
            "w_rev": 0.5,
            "w_switch_yaw": 0.2,
            "w_goal": 50.0,
            "w_steer": 0.0,
            "w_reverse_excess": 0.0,
            **weights,
        }

    def compute(
        self,
        total_length: float,
        total_time: float,
        reverse_length: float,
        switch_yaw: float,
        yaw_pref: Optional[float],
        goal_error: float,
        steer_change_sum: float = 0.0,
        kappa_change_sum: float = 0.0,
    ) -> float:
        yaw_term = 0.0
        if yaw_pref is not None:
            yaw_term = abs(wrap_angle(switch_yaw - yaw_pref))

        return (
            self.weights["w_len"] * total_length
            + self.weights["w_time"] * total_time
            + self.weights["w_rev"] * reverse_length
            + self.weights["w_switch_yaw"] * yaw_term
            + self.weights["w_goal"] * goal_error
            + self.weights["w_steer"] * steer_change_sum
            + self.weights.get("w_kappa_rate", 0.0) * kappa_change_sum
        )
