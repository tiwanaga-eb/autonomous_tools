"""MuJoCo model loader + the single source of truth for joint/actuator indices
and the analytic forward kinematics used by IK and the safety layer.

The analytic FK here MUST reproduce MuJoCo's frame composition exactly so that
joint targets produced by ik.py land where the trajectory wants them.  This is
verified in tests/test_kinematics.py against ``mj_kinematics``.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

import mujoco
import numpy as np

from config import GEOM, HOME_POSE, JOINT_LIMITS, JOINT_NAMES

_HERE = os.path.dirname(__file__)
MODEL_PATH = os.path.join(_HERE, "model.xml")


def _rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@dataclass
class Machine:
    """Wraps mjModel/mjData and exposes named indices + kinematics helpers."""

    model: mujoco.MjModel
    data: mujoco.MjData
    jnt_id: dict[str, int] = field(default_factory=dict)
    jnt_qpos: dict[str, int] = field(default_factory=dict)   # qpos address
    jnt_dof: dict[str, int] = field(default_factory=dict)    # dof / qvel address
    tip_site: int = -1
    back_site: int = -1

    @classmethod
    def load(cls, path: str = MODEL_PATH) -> "Machine":
        model = mujoco.MjModel.from_xml_path(path)
        data = mujoco.MjData(model)
        m = cls(model=model, data=data)
        for name in JOINT_NAMES:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"joint {name!r} missing from model.xml")
            m.jnt_id[name] = jid
            m.jnt_qpos[name] = int(model.jnt_qposadr[jid])
            m.jnt_dof[name] = int(model.jnt_dofadr[jid])
        m.tip_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "bucket_tip")
        m.back_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "bucket_back_site")
        m.reset_home()
        return m

    # ------------------------------------------------------------------ state
    def reset_home(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        for name, ang in HOME_POSE.items():
            self.data.qpos[self.jnt_qpos[name]] = ang
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    @property
    def q(self) -> np.ndarray:
        """Joint angles in canonical JOINT_NAMES order."""
        return np.array([self.data.qpos[self.jnt_qpos[n]] for n in JOINT_NAMES])

    @property
    def qd(self) -> np.ndarray:
        return np.array([self.data.qvel[self.jnt_dof[n]] for n in JOINT_NAMES])

    def set_q(self, q: np.ndarray) -> None:
        for i, n in enumerate(JOINT_NAMES):
            self.data.qpos[self.jnt_qpos[n]] = q[i]

    def clamp_q(self, q: np.ndarray) -> np.ndarray:
        out = q.copy()
        for i, n in enumerate(JOINT_NAMES):
            lo, hi = JOINT_LIMITS[n]
            out[i] = min(max(out[i], lo), hi)
        return out

    def apply_joint_torque(self, tau: np.ndarray) -> None:
        """Inject torque (canonical order) into qfrc_applied."""
        for i, n in enumerate(JOINT_NAMES):
            self.data.qfrc_applied[self.jnt_dof[n]] = tau[i]

    def add_site_force(self, site_id: int, force_world: np.ndarray) -> None:
        """Map a Cartesian force at a site into generalized forces (added to
        qfrc_applied) via the site translational Jacobian."""
        jacp = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, None, site_id)
        self.data.qfrc_applied[:] += jacp.T @ force_world

    # -------------------------------------------------------------- kinematics
    def fk_analytic(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return (tip_pos_world, R_bucket) from joint angles using CONFIG GEOM.

        Mirrors model.xml's nested frames exactly.
        """
        T = np.eye(4)
        T = T @ _homog(_rot_z(0), GEOM["swing_pivot"])      # base->swing origin
        T = T @ _homog(_rot_z(q[0]), [0, 0, 0])             # swing joint
        T = T @ _homog(np.eye(3), GEOM["boom_pivot"])
        T = T @ _homog(_rot_y(q[1]), [0, 0, 0])
        T = T @ _homog(np.eye(3), GEOM["arm_pivot"])
        T = T @ _homog(_rot_y(q[2]), [0, 0, 0])
        T = T @ _homog(np.eye(3), GEOM["bucket_pivot"])
        T = T @ _homog(_rot_y(q[3]), [0, 0, 0])
        R_bucket = T[:3, :3].copy()
        tl = GEOM["tip_local"]
        tip = T @ np.array([tl[0], tl[1], tl[2], 1.0])
        return tip[:3], R_bucket

    def tip_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Current bucket-tip position + orientation from MuJoCo FK (truth)."""
        pos = self.data.site_xpos[self.tip_site].copy()
        mat = self.data.site_xmat[self.tip_site].reshape(3, 3).copy()
        return pos, mat

    def tip_velocity(self) -> np.ndarray:
        jacp = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, None, self.tip_site)
        return jacp @ self.data.qvel

    def tip_jacobian(self) -> np.ndarray:
        """3x4 translational Jacobian of the tip wrt the 4 canonical joints."""
        jacp = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, None, self.tip_site)
        cols = [self.jnt_dof[n] for n in JOINT_NAMES]
        return jacp[:, cols].copy()

    @property
    def qfrc_bias(self) -> np.ndarray:
        """Gravity + Coriolis bias forces for the 4 canonical joints."""
        return np.array([self.data.qfrc_bias[self.jnt_dof[n]] for n in JOINT_NAMES])


def _homog(R: np.ndarray, p) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(p, dtype=float)
    return T
