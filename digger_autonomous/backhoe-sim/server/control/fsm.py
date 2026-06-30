"""Top-level maneuver FSM.

Sequences primitive maneuvers into the canonical truck-loading cycle
(dig -> swing-to-dump -> dump -> swing-back -> repeat) and also runs single
maneuvers selected directly from the GUI.

The FSM only decides *which* maneuver is active and hands its Cartesian/joint
target to the engine; the engine does IK + control + safety filtering.
"""
from __future__ import annotations

import math

import numpy as np

from control.trajectory import Dig, Dump, MoveTo, Target


class ManeuverFSM:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.state = "idle"
        self.cycle = False            # loop the full dig/dump cycle
        self.active = None
        self._seq: list = []
        self._seq_i = 0
        self.selected = "dig"

    # ----------------------------------------------------------------- control
    def select(self, name: str, cycle: bool = False) -> None:
        self.selected = name
        self.cycle = cycle
        self.state = "run"
        self._seq = []
        self._seq_i = 0
        self.active = None

    def stop(self) -> None:
        self.state = "idle"
        self.active = None

    # ------------------------------------------------------------------- build
    def _build_cycle(self, ctx):
        c = self.cfg
        sw = c["maneuvers"]["swing"]
        dig = Dig(c)
        # dump point: above the truck vessel centre
        t = c["safety"]["truck"]
        vessel = np.array(t["pos"]) + np.array(t["vessel_offset"])
        dump_pos = vessel + np.array([0, 0, 1.0])
        to_dump = MoveTo(c, dump_pos, bucket_pitch=2.5, duration=3.0)
        dump = Dump(c)
        # return to dig approach
        d = c["maneuvers"]["dig"]
        gz = ctx.soil.height_at(d["approach_x"], d["approach_y"])
        ret = MoveTo(c, np.array([d["approach_x"], d["approach_y"], gz + 1.0]),
                     bucket_pitch=2.2, duration=3.0)
        return [dig, to_dump, dump, ret]

    def _build_single(self, ctx, name):
        c = self.cfg
        if name == "dig":
            return [Dig(c)]
        if name in ("swing", "load", "dump"):
            return self._build_cycle(ctx)
        # earthwork maneuvers (P6)
        from control import earthwork
        return [earthwork.make(name, c, ctx)]

    # -------------------------------------------------------------------- step
    def step(self, ctx, dt) -> Target | None:
        if self.state != "run":
            return None
        if not self._seq:
            self._seq = (self._build_cycle(ctx) if self.cycle
                         else self._build_single(ctx, self.selected))
            self._seq_i = 0
            self.active = self._seq[0]
            self.active.reset(ctx)

        tgt = self.active.step(ctx, dt)
        if self.active.done():
            self._seq_i += 1
            if self._seq_i >= len(self._seq):
                if self.cycle:
                    self._seq = []          # rebuild & loop
                    return tgt
                self.state = "idle"
                self.active = None
                return tgt
            self.active = self._seq[self._seq_i]
            self.active.reset(ctx)
        return tgt

    @property
    def phase_name(self) -> str:
        if self.state != "run" or self.active is None:
            return "idle"
        nm = type(self.active).__name__
        sub = getattr(self.active, "phase", "")
        return f"{nm}:{sub}" if sub else nm
