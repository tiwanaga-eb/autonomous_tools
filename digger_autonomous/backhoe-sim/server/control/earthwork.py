"""P6 earthwork maneuvers (trench / grading / slope shaping / compaction).

These are functional skeletons: they generate sensible tip trajectories and use
the soil mass-balance + design-surface safety so the qualitative behaviour and
completion criteria work.  Depth tuning is left to the GUI.
"""
from __future__ import annotations

import math

import numpy as np

from control.trajectory import Dig, Dump, MoveTo, Target, _smoothstep


def make(name: str, cfg: dict, ctx):
    if name == "trench":
        return Trench(cfg)
    if name in ("grade", "grading"):
        return Grading(cfg)
    if name == "slope":
        return Slope(cfg)
    if name in ("compact", "compaction"):
        return Compaction(cfg)
    raise ValueError(f"unknown maneuver {name!r}")


class Trench:
    """Repeated dig along a line, advancing the dig point by dx each pass until
    the target cross-section (depth) is reached."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.pass_i = 0
        self.sub = None
        self._done = False

    def reset(self, ctx):
        self.pass_i = 0
        self._done = False
        self._new_dig(ctx)

    def _new_dig(self, ctx):
        c = self.cfg
        tr = c["maneuvers"]["trench"]
        x = c["maneuvers"]["dig"]["approach_x"] + self.pass_i * tr["dx"]
        self.cfg["maneuvers"]["dig"]["approach_x"] = x
        self.sub = Dig(c)
        self.sub.reset(ctx)

    def step(self, ctx, dt) -> Target:
        tr = self.cfg["maneuvers"]["trench"]
        tgt = self.sub.step(ctx, dt)
        if self.sub.done():
            ctx.soil.dump(np.array([0.0, -3.0, ctx.soil.height_at(0, -3)]))
            self.pass_i += 1
            # completion: dug deep enough over the trench length, or max passes
            depth_now = -ctx.soil.height_diff().min()
            if depth_now >= tr["depth"] - tr["tol"] or self.pass_i >= 6:
                self._done = True
            else:
                self._new_dig(ctx)
        return tgt

    def done(self):
        return self._done


class Grading:
    """Hold the cutting edge at the target plane height and sweep forward,
    levelling the terrain over multiple overlapping passes."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.t = 0.0
        self.x = None

    def reset(self, ctx):
        self.t = 0.0
        self.x0 = 3.0
        self.x = self.x0

    def step(self, ctx, dt) -> Target:
        g = self.cfg["maneuvers"]["grade"]
        self.t += dt
        self.x = self.x0 + g["forward_speed"] * self.t
        z = g["target_z"]
        return Target(tip_pos=np.array([self.x, 0.0, z]), bucket_pitch=-1.2)

    def done(self):
        return self.x is not None and self.x > 8.0


class Slope:
    """Move the edge along an inclined straight line, bucket parallel to slope."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.t = 0.0

    def reset(self, ctx):
        self.t = 0.0
        self.start = np.array([7.0, 0.0, 0.0])

    def step(self, ctx, dt) -> Target:
        sp = self.cfg["maneuvers"]["slope"]
        self.t += dt
        s = sp["speed"] * self.t
        grade = sp["grade"]
        pos = self.start + s * np.array([-math.cos(grade), 0.0, math.sin(grade)])
        pitch = -1.0 - grade
        return Target(tip_pos=pos, bucket_pitch=pitch)

    def done(self):
        return self.t > 6.0


class Compaction:
    """Press the bucket back onto the ground at grid points to a target force."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.idx = 0
        self.t = 0.0

    def reset(self, ctx):
        c = self.cfg["maneuvers"]["compaction"]
        nx, ny = c["grid"]
        xs = np.linspace(3.5, 6.0, nx)
        ys = np.linspace(-1.0, 1.0, ny)
        self.points = [np.array([x, y]) for x in xs for y in ys]
        self.idx = 0
        self.t = 0.0

    def step(self, ctx, dt) -> Target:
        self.t += dt
        p = self.points[self.idx]
        gz = ctx.soil.height_at(p[0], p[1])
        a = _smoothstep((self.t % 2.0) / 1.0)
        z = gz + 0.8 * (1 - a)        # descend onto surface
        if self.t % 2.0 > 1.0:
            ctx.soil.compact(np.array([p[0], p[1], gz]))
        if self.t > (self.idx + 1) * 2.0 and self.idx < len(self.points) - 1:
            self.idx += 1
        return Target(tip_pos=np.array([p[0], p[1], z]), bucket_pitch=-0.3)

    def done(self):
        return self.idx >= len(self.points) - 1 and self.t > len(self.points) * 2.0
