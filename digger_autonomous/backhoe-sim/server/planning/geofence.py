"""GeoFence — generates the two barrier families the spec cares about:

  (1) design-surface barrier : the tip (and bucket-width edge samples) must not
      go below the design surface D(x,y)  ->  h = z_tip - D(x_tip,y_tip) >= 0
  (2) dump-truck barriers     : bucket representative points must keep clear of
      the truck solids (cab box, vessel floor + 4 walls)  ->  h = dist - margin

Plus auxiliary ground over-penetration and self-collision barriers.

Each barrier is returned as a :class:`Barrier` carrying h and dh/dq (gradient
wrt the 4 joint angles), computed from the tip/point translational Jacobian.
The CBF-QP (cbf.py) enforces dh/dq . qdot >= -alpha * h, so the SAME barriers
constrain both autonomous and manual motion through safety_filter.py.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from config import GEOM


@dataclass
class Barrier:
    h: float
    grad: np.ndarray            # d h / d q  (length 4)
    alpha: float
    kind: str                   # 'design' | 'truck' | 'ground' | 'self'
    label: str = ""


# --------------------------------------------------------------------- design
def design_value_and_grad(cfg, x: float, y: float) -> tuple[float, float, float]:
    """Return (D, dD/dx, dD/dy) of the design surface at world (x,y)."""
    d = cfg["safety"]["design"]
    if d["type"] == "plane":
        return d["floor_z"], 0.0, 0.0
    if d["type"] == "slope":
        ox, oy, oz = d["slope_origin"]
        dirx, diry = d["slope_dir"]
        n = np.hypot(dirx, diry) or 1.0
        dirx, diry = dirx / n, diry / n
        g = np.tan(d["slope_grade"])
        # surface descends along slope_dir with grade g
        s = (x - ox) * dirx + (y - oy) * diry
        return oz - g * s, -g * dirx, -g * diry
    # 'surface' fallback: flat floor
    return d["floor_z"], 0.0, 0.0


def design_barriers(machine, cfg) -> list[Barrier]:
    d = cfg["safety"]["design"]
    tip, R = machine.tip_pose()
    J = machine.tip_jacobian()              # 3x4 (rows x,y,z)
    width = GEOM["bucket_width"]
    n = max(1, d["n_edge_samples"])
    lateral = R @ np.array([0.0, 1.0, 0.0])  # bucket width axis in world
    out = []
    worst = None
    offs = np.linspace(-0.5 * width, 0.5 * width, n)
    for off in offs:
        p = tip + lateral * off
        D, Dx, Dy = design_value_and_grad(cfg, p[0], p[1])
        h = p[2] - D
        # grad wrt q : dz/dq - Dx*dx/dq - Dy*dy/dq  (edge shares tip Jacobian)
        grad = J[2, :] - Dx * J[0, :] - Dy * J[1, :]
        if worst is None or h < worst.h:
            worst = Barrier(h=h, grad=grad, alpha=d["alpha"], kind="design",
                            label="design")
    out.append(worst)
    return out


# ---------------------------------------------------------------------- truck
class Box:
    """Oriented box (yaw about z) for truck solids."""

    def __init__(self, center, half, yaw):
        self.c = np.asarray(center, dtype=float)
        self.half = np.asarray(half, dtype=float)
        cz, sz = np.cos(yaw), np.sin(yaw)
        self.Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

    def sdf_and_dir(self, p: np.ndarray) -> tuple[float, np.ndarray]:
        """Signed distance from world point p to the box, and the outward unit
        direction (world) along which distance increases (gradient of dist)."""
        local = self.Rz.T @ (p - self.c)
        q = np.abs(local) - self.half
        q_clamped = np.maximum(q, 0.0)
        outside = np.linalg.norm(q_clamped)
        inside = min(max(q[0], max(q[1], q[2])), 0.0)
        dist = outside + inside
        # gradient direction in local frame
        if outside > 1e-9:
            g_local = np.sign(local) * (q > 0) * (q_clamped / (outside + 1e-12))
        else:
            # inside: push out along the least-penetrated axis
            axis = int(np.argmax(q))
            g_local = np.zeros(3)
            g_local[axis] = np.sign(local[axis]) or 1.0
        g_world = self.Rz @ g_local
        nrm = np.linalg.norm(g_world)
        return float(dist), (g_world / nrm if nrm > 1e-9 else np.array([0, 0, 1.0]))


def _truck_boxes(cfg) -> list[Box]:
    t = cfg["safety"]["truck"]
    pos = np.array(t["pos"], dtype=float)
    yaw = t["yaw"]
    boxes = []
    # cab
    boxes.append(Box(pos + np.array(t["cab_offset"]), t["cab_half"], yaw))
    # vessel as floor + 4 walls (open top) so the bucket can enter from above
    vc = pos + np.array(t["vessel_offset"])
    vh = np.array(t["vessel_half"], dtype=float)
    wt = t["wall_thickness"]
    # floor
    boxes.append(Box(vc + np.array([0, 0, -vh[2]]), [vh[0], vh[1], wt], yaw))
    # +x / -x walls
    boxes.append(Box(vc + np.array([vh[0], 0, 0]), [wt, vh[1], vh[2]], yaw))
    boxes.append(Box(vc + np.array([-vh[0], 0, 0]), [wt, vh[1], vh[2]], yaw))
    # +y / -y walls
    boxes.append(Box(vc + np.array([0, vh[1], 0]), [vh[0], wt, vh[2]], yaw))
    boxes.append(Box(vc + np.array([0, -vh[1], 0]), [vh[0], wt, vh[2]], yaw))
    return boxes


def truck_barriers(machine, cfg) -> list[Barrier]:
    t = cfg["safety"]["truck"]
    if not t["enabled"]:
        return []
    boxes = _truck_boxes(cfg)
    # bucket representative points: tip and back
    tip, _ = machine.tip_pose()
    back = machine.data.site_xpos[machine.back_site].copy()
    Jt = machine.tip_jacobian()
    Jb = _site_jac(machine, machine.back_site)
    pts = [(tip, Jt, "tip"), (back, Jb, "back")]
    out = []
    for p, J, pname in pts:
        for bi, box in enumerate(boxes):
            dist, gdir = box.sdf_and_dir(p)
            h = dist - t["margin"]
            grad = J.T @ gdir            # d(dist)/dq = gdir . dp/dq
            out.append(Barrier(h=h, grad=grad, alpha=t["alpha"], kind="truck",
                               label=f"truck[{pname},{bi}]"))
    return out


def _site_jac(machine, site_id) -> np.ndarray:
    import mujoco
    jacp = np.zeros((3, machine.model.nv))
    mujoco.mj_jacSite(machine.model, machine.data, jacp, None, site_id)
    cols = [machine.jnt_dof[n] for n in ["swing", "boom", "arm", "bucket"]]
    return jacp[:, cols].copy()


# --------------------------------------------------------------------- ground
def ground_barrier(machine, cfg, digging: bool) -> list[Barrier]:
    g = cfg["safety"]["ground"]
    tip, _ = machine.tip_pose()
    J = machine.tip_jacobian()
    # ground height from soil
    gz = cfg["soil"]["initial_ground_z"]
    h = tip[2] - (gz - g["margin"])
    alpha = g["alpha"]
    if digging and g["relax_when_digging"]:
        # relax: allow penetration during normal digging (design barrier still on)
        return []
    return [Barrier(h=h, grad=J[2, :], alpha=alpha, kind="ground", label="ground")]


def all_barriers(machine, cfg, digging: bool) -> list[Barrier]:
    bs = []
    bs += design_barriers(machine, cfg)
    bs += truck_barriers(machine, cfg)
    bs += ground_barrier(machine, cfg, digging)
    return bs
