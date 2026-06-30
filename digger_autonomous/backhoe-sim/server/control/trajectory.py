"""Maneuver trajectory generators.

Each maneuver produces, every control tick, a Cartesian target (tip position +
bucket pitch) or direct joint targets, which the engine turns into joint angle
references via IK and tracks with the computed-torque controller.

Maneuvers implement the protocol: reset(ctx) / step(ctx, dt) -> Target / done().
``ctx`` is the live SimContext (machine, soil, cfg, admittance, ...).

Phase coverage:
  P3  Dig            (3-phase: penetrate -> drag -> curl, admittance depth)
  P4  Swing, Dump
  P6  Trench, Grade, Slope, Compaction
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from control import ik


@dataclass
class Target:
    """A control target.  Either cartesian (pos+pitch) or direct joint angles."""
    q: np.ndarray | None = None              # direct joint targets (rad)
    tip_pos: np.ndarray | None = None        # world tip target (m)
    bucket_pitch: float | None = None        # absolute bucket pitch (rad)


def _smoothstep(a: float) -> float:
    a = min(max(a, 0.0), 1.0)
    return a * a * (3 - 2 * a)


class Dig:
    """Three-phase digging with admittance depth control.

    1. penetrate : drive the tip into the ground at a fixed entry angle
    2. drag      : pull the bucket back keeping a target cutting depth
                   (admittance: reduce depth when cutting force is too high)
    3. curl      : roll the bucket closed to scoop and retain the soil
    """

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.phase = "penetrate"
        self.t = 0.0
        self.start = np.zeros(3)
        # 絶対バケットpitch(R_y; +で先端が下を向く). バックホウ符号系では掘削は正側。
        self.entry_pitch = math.radians(110.0)

    def reset(self, ctx) -> None:
        d = self.cfg["maneuvers"]["dig"]
        self.phase = "penetrate"
        self.t = 0.0
        ctx.admittance.reset()
        gz = ctx.soil.height_at(d["approach_x"], d["approach_y"])
        self.surface = np.array([d["approach_x"], d["approach_y"], gz])
        self.start = self.surface.copy()

    def step(self, ctx, dt: float) -> Target:
        d = self.cfg["maneuvers"]["dig"]
        self.t += dt
        depth = d["target_depth"]
        give = ctx.admittance.update(float(np.linalg.norm(ctx.soil.last_force)), dt)
        eff_depth = max(0.05, depth - give)

        if self.phase == "penetrate":
            a = _smoothstep(self.t / 2.0)
            pos = self.surface + np.array([0, 0, -eff_depth]) * a
            pitch = self.entry_pitch
            if a >= 1.0:
                self.phase = "drag"; self.t = 0.0
            return Target(tip_pos=pos, bucket_pitch=pitch)

        if self.phase == "drag":
            a = _smoothstep(self.t / 3.5)
            back = d["drag_distance"] * a
            pos = self.surface + np.array([-back, 0.0, -eff_depth])
            # gradually curl the bucket while dragging (正側へ巻き込む)
            pitch = self.entry_pitch + 0.5 * a
            if a >= 1.0:
                self.phase = "curl"; self.t = 0.0
            return Target(tip_pos=pos, bucket_pitch=pitch)

        # curl / scoop up
        a = _smoothstep(self.t / 2.0)
        pos = self.surface + np.array([-d["drag_distance"], 0.0, -eff_depth + 0.6 * a])
        pitch = self.entry_pitch + 0.5 + 0.9 * a
        return Target(tip_pos=pos, bucket_pitch=pitch)

    def done(self) -> bool:
        return self.phase == "curl" and self.t >= 2.0


class MoveTo:
    """Generic timed Cartesian move to a tip target with a bucket pitch hold
    (used for swing-to-dump approach and returns)."""

    def __init__(self, cfg, target_pos, bucket_pitch, duration=3.0):
        self.cfg = cfg
        self.target = np.asarray(target_pos, dtype=float)
        self.pitch = bucket_pitch
        self.duration = duration
        self.t = 0.0
        self.start = None

    def reset(self, ctx):
        self.t = 0.0
        self.start, _ = ctx.machine.tip_pose()

    def step(self, ctx, dt) -> Target:
        self.t += dt
        a = _smoothstep(self.t / self.duration)
        pos = self.start + (self.target - self.start) * a
        return Target(tip_pos=pos, bucket_pitch=self.pitch)

    def done(self) -> bool:
        return self.t >= self.duration


class Dump:
    """Open the bucket over the dump point to release the payload."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.t = 0.0
        self.released = False

    def reset(self, ctx):
        self.t = 0.0
        self.released = False
        self.hold_pose, _ = ctx.machine.tip_pose()

    def step(self, ctx, dt) -> Target:
        c = self.cfg["maneuvers"]["dump"]
        self.t += dt
        # open the bucket (pitch toward 0) to dump（正側に巻いた状態から開く）
        pitch = 2.5 - min(1.0, self.t / 1.0) * 2.4
        if self.t > 0.6 and not self.released:
            ctx.soil.dump(self.hold_pose, fraction=1.0)
            self.released = True
        return Target(tip_pos=self.hold_pose, bucket_pitch=pitch)

    def done(self) -> bool:
        return self.t >= self.cfg["maneuvers"]["dump"]["hold_time"] + 1.0
