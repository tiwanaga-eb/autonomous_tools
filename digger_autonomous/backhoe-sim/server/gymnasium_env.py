"""
Gymnasium-compatible RL environment wrapping the backhoe SimEngine.

Observation space (25-dim):
  [0:4]   joint angles q  (normalized to [-1, 1] by joint limits)
  [4:8]   joint velocities qd (clipped+normalized by max_qd)
  [8:11]  bucket tip position (x, y, z) in MuJoCo world frame
  [11:13] dig-zone target relative position (dx, dz) — swing is pre-set
  [13]    soil depth at tip (m, clipped to [0, 0.5])
  [14]    payload volume (m³, normalized by bucket capacity)
  [15:19] last applied torque τ[4] (normalized by max_tau)
  [19:23] safety status: geofence, cbf, active flags + min_h (norm.)
  [23:25] tip velocity magnitude, vertical component

Action space (4-dim continuous, [-1, 1]):
  Joint velocity commands mapped to ± max_v_rad_s per joint.

Reward (dense):
  +  dig_reward     : payload increment gained this step
  -  energy_penalty : |τ·qd| * dt * coeff
  -  boundary_pen   : triggered when safety filter activates
  +  task_complete  : large bonus when payload ≥ target_vol

Episode ends when:
  - payload ≥ target_vol (success)
  - max_steps reached
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

# Allow running from repo root or server/
sys.path.insert(0, str(Path(__file__).parent))

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as exc:
    raise ImportError("Install gymnasium: pip install gymnasium") from exc

from config import JOINT_LIMITS, JOINT_NAMES
from sim.simulation import SimEngine

_JNAMES = JOINT_NAMES  # ["swing", "boom", "arm", "bucket"]

# Joint limits as arrays
_Q_LO = np.array([JOINT_LIMITS[n][0] for n in _JNAMES], dtype=np.float32)
_Q_HI = np.array([JOINT_LIMITS[n][1] for n in _JNAMES], dtype=np.float32)
_Q_MID = (_Q_LO + _Q_HI) / 2.0
_Q_RNG = (_Q_HI - _Q_LO) / 2.0

_MAX_QD   = np.array([0.8, 0.6, 0.8, 1.0], dtype=np.float32)  # rad/s per joint
_MAX_TAU  = np.array([6e4, 9e4, 6e4, 4e4], dtype=np.float32)   # Nm (approx)
_MAX_V_CMD = _MAX_QD  # action → velocity


class BackhoeDigEnv(gym.Env):
    """Single-dig episode: fill the bucket from a target soil patch."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        target_vol: float = 0.15,       # m³ to collect (≈ 2/3 bucket)
        max_steps: int = 2000,
        dig_x: float = 3.5,             # target patch centre x (MuJoCo)
        dig_y: float = 0.0,             # target patch centre y
        reward_weights: dict | None = None,
        seed: int | None = None,
    ):
        super().__init__()
        self.target_vol = target_vol
        self.max_steps = max_steps
        self.dig_x = dig_x
        self.dig_y = dig_y

        rw = reward_weights or {}
        self._w_dig    = float(rw.get("dig",      500.0))
        self._w_energy = float(rw.get("energy",     0.01))
        self._w_bound  = float(rw.get("boundary",   2.0))
        self._w_done   = float(rw.get("task_done", 20.0))

        # 24-dim observation, 4-dim action
        self.observation_space = spaces.Box(
            low=-np.ones(25, dtype=np.float32),
            high=np.ones(25, dtype=np.float32),
            dtype=np.float32,
        )
        self.action_space = spaces.Box(
            low=-np.ones(4, dtype=np.float32),
            high=np.ones(4, dtype=np.float32),
            dtype=np.float32,
        )

        self._engine: SimEngine | None = None
        self._step_count = 0
        self._prev_payload = 0.0
        self._rng = np.random.default_rng(seed)

    # ── internal helpers ──────────────────────────────────────────────────────

    def _make_engine(self) -> SimEngine:
        eng = SimEngine()
        eng.set_mode("MANUAL")
        return eng

    def _obs(self) -> np.ndarray:
        eng = self._engine
        assert eng is not None
        q   = eng.machine.q.astype(np.float32)
        qd  = eng.machine.qd.astype(np.float32)
        tip, _ = eng.machine.tip_pose()
        tip = tip.astype(np.float32)

        # relative target position in (x, z) — swing pre-aligns y
        dx = float(self.dig_x) - float(tip[0])
        dz = float(self.dig_y) - float(tip[2])  # MuJoCo z = depth

        soil_depth = float(np.clip(eng.soil.last_depth, 0.0, 0.5))
        cap = eng.cfg["soil"]["bucket_capacity"]
        payload_norm = float(np.clip(eng.soil.payload / cap, 0.0, 1.0))

        tau = np.array(eng.hyd.last_tau, dtype=np.float32)
        status = eng.last_status
        # safety flags
        active_raw = status.get("active", [])
        safety_active_flag = float(bool(active_raw))
        geofence_flag = float(bool(status.get("geofence", False)))
        cbf_flag = float(bool(status.get("cbf", False)))
        min_h = float(status.get("min_h", 10.0))

        tip_vel = eng.machine.tip_velocity().astype(np.float32)
        tip_speed = float(np.linalg.norm(tip_vel))
        tip_vz = float(tip_vel[2])

        obs = np.array([
            # q normalised
            *((q - _Q_MID) / _Q_RNG).clip(-1, 1),
            # qd normalised
            *(qd / _MAX_QD).clip(-1, 1),
            # tip position (keep raw but clip)
            float(np.clip(tip[0], -10, 10)) / 10.0,
            float(np.clip(tip[1], -5,  5))  /  5.0,
            float(np.clip(tip[2], -3,  3))  /  3.0,
            # target relative
            float(np.clip(dx, -10, 10)) / 10.0,
            float(np.clip(dz, -10, 10)) / 10.0,
            # soil
            soil_depth / 0.5,
            payload_norm,
            # torque
            *(tau / _MAX_TAU).clip(-1, 1),
            # safety
            geofence_flag,
            cbf_flag,
            safety_active_flag,
            float(np.clip(min_h, -2, 10)) / 10.0,
            # tip kinematics
            float(np.clip(tip_speed, 0, 3)) / 3.0,
            float(np.clip(tip_vz, -2, 2)) / 2.0,
        ], dtype=np.float32)
        assert obs.shape == (25,), obs.shape
        return obs

    def _apply_action(self, action: np.ndarray) -> None:
        """Map [-1,1] action to velocity commands sent as MANUAL joystick."""
        eng = self._engine
        assert eng is not None
        v_cmd = (action * _MAX_V_CMD).astype(float)
        # map 4 joint velocities → q_des increment (teleop expects rate commands)
        dt = eng.cfg["sim"]["dt"]
        # directly set q_des for fine-grained RL control
        new_q_des = eng.machine.clamp_q(eng.q_des + v_cmd * dt)
        eng.q_des = new_q_des

    # ── gym interface ─────────────────────────────────────────────────────────

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict | None = None,
    ) -> tuple[np.ndarray, dict]:
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        self._engine = self._make_engine()
        self._step_count = 0
        self._prev_payload = 0.0

        # Optional: randomise target x within ±1 m of default
        if options and options.get("randomise_target", False):
            self.dig_x = float(self._rng.uniform(2.0, 5.0))

        obs = self._obs()
        return obs, {}

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict]:
        eng = self._engine
        assert eng is not None, "Call reset() before step()"

        action = np.clip(action, -1.0, 1.0).astype(np.float32)
        self._apply_action(action)

        # advance one physics step
        eng.step()
        self._step_count += 1

        obs = self._obs()

        # reward
        payload = float(eng.soil.payload)
        dig_gain = payload - self._prev_payload
        self._prev_payload = payload

        tau = np.array(eng.hyd.last_tau, dtype=np.float32)
        qd  = eng.machine.qd.astype(np.float32)
        dt  = float(eng.cfg["sim"]["dt"])
        energy = float(np.abs(tau @ qd)) * dt

        status = eng.last_status
        safety_active = bool(status.get("active", []))

        reward = (
            self._w_dig    * dig_gain
            - self._w_energy * energy
            - self._w_bound  * float(safety_active)
        )

        terminated = payload >= self.target_vol
        truncated  = self._step_count >= self.max_steps

        if terminated:
            reward += self._w_done

        info: dict[str, Any] = {
            "payload": payload,
            "step": self._step_count,
            "safety_active": safety_active,
        }
        return obs, float(reward), terminated, truncated, info

    def close(self) -> None:
        self._engine = None
