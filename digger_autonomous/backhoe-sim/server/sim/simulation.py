"""SimEngine — the integrated control loop.

Runs at the physics rate (500 Hz).  Every tick:

  1. pick a NOMINAL joint-velocity command from the active source
       MANUAL / MANUAL_ASSIST  -> teleop rate command
       AUTO                    -> FSM maneuver -> IK -> velocity toward targets
       AUTO_OVERRIDE           -> blend(AUTO, teleop) by beta when stick active
       joint slider mode (P1)  -> velocity toward GUI joint targets
  2. pass v_nom through the SHARED SafetyFilter (GeoFence + CBF-QP) -> v_safe
  3. integrate v_safe into joint position targets
  4. computed-torque controller -> valve command u -> hydraulics -> joint torque
  5. soil dig force + mass balance injected at the bucket tip
  6. mj_step

This is the single point where manual and autonomous inputs merge and are made
safe (the design hinge from the spec).
"""
from __future__ import annotations

import numpy as np
import mujoco

from config import JOINT_NAMES, deep_copy
from sim.machine import Machine
from sim.hydraulics import Hydraulics
from sim.soil import Soil
from control.joint_ctrl import JointController, Admittance
from control.teleop import Teleop
from control.fsm import ManeuverFSM
from control import ik
from planning.safety_filter import SafetyFilter


class SimContext:
    """Bundle passed to maneuvers."""
    def __init__(self, engine):
        self.machine = engine.machine
        self.soil = engine.soil
        self.cfg = engine.cfg
        self.admittance = engine.admittance


class SimEngine:
    def __init__(self, cfg: dict | None = None):
        self.cfg = cfg or deep_copy()
        self.machine = Machine.load()
        self.hyd = Hydraulics(self.cfg)
        self.soil = Soil(self.cfg)
        self.jc = JointController(self.cfg)
        self.admittance = Admittance(self.cfg)
        self.teleop = Teleop(self.cfg)
        self.fsm = ManeuverFSM(self.cfg)
        self.safety = SafetyFilter(self.cfg)
        self.ctx = SimContext(self)

        self.t = 0.0
        self.paused = False
        self.q_des = self.machine.q.copy()
        self.slider_targets = self.machine.q.copy()   # P1 joint-slider mode
        self.last_status = {}
        self.last_target = None
        self._payload_mass = 0.0

    # ----------------------------------------------------------------- helpers
    def _mode(self) -> str:
        return self.cfg["teleop"]["mode"]

    def _is_digging(self) -> bool:
        return self.soil.last_depth > 0.01

    def _auto_velocity(self, dt: float) -> tuple[np.ndarray, bool]:
        """Velocity command from the FSM maneuver via IK.  Returns (v, active)."""
        tgt = self.fsm.step(self.ctx, dt)
        if tgt is None:
            return np.zeros(len(JOINT_NAMES)), False
        self.last_target = tgt
        if tgt.q is not None:
            q_ref = np.asarray(tgt.q)
        else:
            q_ref, ok = ik.solve(tgt.tip_pos, tgt.bucket_pitch)
            if not ok or q_ref is None:
                q_ref = self.q_des
        # velocity that moves current target toward the IK reference
        v = (q_ref - self.q_des) / max(dt, 1e-3)
        return v, True

    # -------------------------------------------------------------------- step
    def step(self) -> None:
        if self.paused:
            return
        dt = self.cfg["sim"]["dt"]
        mode = self._mode()
        n = len(JOINT_NAMES)

        # 1) nominal velocity command by source
        if mode in ("MANUAL", "MANUAL_ASSIST"):
            v_nom = self.teleop.rate_command(self.t)
            self.travel = self.teleop.travel
        elif mode == "AUTO":
            v_nom, _ = self._auto_velocity(dt)
        elif mode == "AUTO_OVERRIDE":
            v_auto, _ = self._auto_velocity(dt)
            v_man = self.teleop.rate_command(self.t)
            if self.teleop.active(self.t):
                beta = self.cfg["teleop"]["override_blend"]
                v_nom = beta * v_man + (1 - beta) * v_auto
            else:
                v_nom = v_auto
        else:  # P1 joint-slider mode (mode == 'JOINT')
            v_nom = (self.slider_targets - self.q_des) / max(dt, 1e-3)

        # 2) shared safety filter (GeoFence + CBF) — manual AND auto
        digging = self._is_digging()
        v_safe, status = self.safety.apply(self.machine, v_nom, digging)
        self.last_status = status

        # 3) integrate to joint position targets (clamped to limits)
        self.q_des = self.machine.clamp_q(self.q_des + v_safe * dt)

        # 4) computed-torque -> valve -> hydraulics -> joint torque
        q, qd = self.machine.q, self.machine.qd
        u = self.jc.u_from_targets(self.q_des, q, qd, dt, bias=self.machine.qfrc_bias)
        tau = self.hyd.step(u, q, qd, dt)
        self.machine.data.qfrc_applied[:] = 0.0
        self.machine.apply_joint_torque(tau)

        # 5) soil interaction at the bucket tip
        self._apply_soil(dt)

        # 6) advance physics
        mujoco.mj_step(self.machine.model, self.machine.data)
        self.machine.data.qfrc_applied[:] = 0.0
        self.t += dt

    def _apply_soil(self, dt: float) -> None:
        tip, R = self.machine.tip_pose()
        tip_vel = self.machine.tip_velocity()
        cut_dir = R @ np.array([1.0, 0.0, 0.0])
        force = self.soil.dig_force(tip, tip_vel, cut_dir)
        if np.any(force):
            self.machine.add_site_force(self.machine.tip_site, force)
        self.soil.excavate(tip, tip_vel, dt)
        # reflect payload as added bucket mass (equivalent payload)
        target_mass = self.soil.payload * self.cfg["soil"]["gamma"] / 9.81
        if abs(target_mass - self._payload_mass) > 1.0:
            self._set_bucket_payload(target_mass)
            self._payload_mass = target_mass

    def _set_bucket_payload(self, mass: float) -> None:
        bid = mujoco.mj_name2id(self.machine.model, mujoco.mjtObj.mjOBJ_BODY, "bucket")
        base = 600.0
        self.machine.model.body_mass[bid] = base + max(0.0, mass)

    # ------------------------------------------------------------------ control
    def set_mode(self, mode: str) -> None:
        self.cfg["teleop"]["mode"] = mode
        # If switching to AUTO without active maneuver, start dig
        if mode == "AUTO" and self.fsm.state != "run":
            self.fsm.select("dig", cycle=False)

    def select_maneuver(self, name: str, cycle: bool = False) -> None:
        self.set_mode("AUTO")
        self.fsm.select(name, cycle)

    def set_joint_target(self, name: str, value: float) -> None:
        self.set_mode("JOINT")
        idx = JOINT_NAMES.index(name)
        self.slider_targets[idx] = value

    def reset(self) -> None:
        self.machine.reset_home()
        self.hyd.reset()
        self.soil.reset()
        self.jc.reset()
        self.admittance.reset()
        self.fsm.stop()
        self.q_des = self.machine.q.copy()
        self.slider_targets = self.machine.q.copy()
        self._payload_mass = 0.0
        self._set_bucket_payload(0.0)
        self.t = 0.0
