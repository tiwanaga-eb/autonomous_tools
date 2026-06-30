"""Manual joystick / gamepad handling + shared-control mode management.

Stick values in [-1,1] are interpreted as RATE commands (joint angular rate),
integrated into joint targets — steadier than position-direct mapping.  The
resulting nominal joint-velocity goes through the SAME safety filter as the
autonomous path (see engine), so the design-surface and truck barriers also
intervene on manual input.

Loss-of-comm safety stop: if no joystick packet arrives within ``comm_timeout``
the rate command is forced to zero.
"""
from __future__ import annotations

import numpy as np

from config import JOINT_NAMES

# Axis -> joint mapping per ISO/SAE pattern.  Value = (axis_index, sign).
# axes layout assumed: [leftX, leftY, rightX, rightY] like a standard gamepad.
PATTERNS = {
    "ISO": {  # right Y=arm, right X=swing, left Y=boom, left X=bucket
        "swing": (2, 1.0), "arm": (3, -1.0), "boom": (1, -1.0), "bucket": (0, 1.0),
    },
    "SAE": {  # right Y=boom, right X=swing, left Y=arm, left X=bucket
        "swing": (2, 1.0), "boom": (3, -1.0), "arm": (1, -1.0), "bucket": (0, 1.0),
    },
}


def _expo_deadzone(v: float, dz: float, expo: float) -> float:
    if abs(v) < dz:
        return 0.0
    s = (abs(v) - dz) / (1.0 - dz)
    return np.sign(v) * (s ** expo)


class Teleop:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.axes = [0.0, 0.0, 0.0, 0.0]
        self.buttons: list = []
        self.last_packet_t = -1e9
        self.travel = [0.0, 0.0]      # left/right track rate

    def update_input(self, axes, buttons, t: float) -> None:
        self.axes = list(axes) + [0.0] * max(0, 4 - len(axes))
        self.buttons = list(buttons)
        self.last_packet_t = t

    def rate_command(self, t: float) -> np.ndarray:
        """Return desired joint angular-rate command (rad/s), canonical order.

        Zeroed on comm loss (Loss-of-Comm safe stop)."""
        tc = self.cfg["teleop"]
        if t - self.last_packet_t > tc["comm_timeout"]:
            return np.zeros(len(JOINT_NAMES))
        pat = PATTERNS.get(tc["pattern"], PATTERNS["ISO"])
        out = np.zeros(len(JOINT_NAMES))
        for i, name in enumerate(JOINT_NAMES):
            ax, sgn = pat[name]
            raw = self.axes[ax] if ax < len(self.axes) else 0.0
            v = _expo_deadzone(sgn * raw, tc["deadzone"], tc["expo"])
            out[i] = v * tc["max_rate"][name]
        return out

    def active(self, t: float) -> bool:
        """True if the operator is currently commanding motion (for OVERRIDE)."""
        return float(np.linalg.norm(self.rate_command(t))) > 1e-3
