"""Simplified soil model: a grid height-field + semi-empirical digging force +
mass balance.  MuJoCo has no soil, so cutting resistance is computed here and
injected as an external force at the bucket tip; terrain deformation and bucket
payload are bookkept on the height-field.

Fidelity goal (per spec): qualitatively correct dig-force trend, payload
balance and terrain change — NOT continuum-accurate.

Digging resistance uses a reduced Fundamental Earthmoving Equation (Reece):

    F = (gamma*d^2*N_gamma + c*d*N_c + gamma*d*v^2/g*N_a) * w

with d = penetration depth, v = tip speed, w = bucket width.  The force opposes
the tip's horizontal travel and has an upward normal component.
"""
from __future__ import annotations

import numpy as np

from config import GEOM

GRAV = 9.81


class Soil:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        s = cfg["soil"]
        self.res = s["res"]
        self.nx = int(round(s["size_x"] / self.res))
        self.ny = int(round(s["size_y"] / self.res))
        self.origin = np.array(s["origin"], dtype=float)   # world (x,y) of cell (0,0)
        self.H = np.full((self.nx, self.ny), s["initial_ground_z"], dtype=float)
        self.H0 = self.H.copy()                              # original ground
        self.payload = 0.0                                   # m^3 of soil in bucket
        self.density_flag = np.zeros((self.nx, self.ny), dtype=np.uint8)  # compaction
        self.last_force = np.zeros(3)
        self.last_depth = 0.0

    # ------------------------------------------------------------- grid helpers
    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        i = int(round((x - self.origin[0]) / self.res))
        j = int(round((y - self.origin[1]) / self.res))
        return (int(np.clip(i, 0, self.nx - 1)), int(np.clip(j, 0, self.ny - 1)))

    def cell_to_world(self, i: int, j: int) -> tuple[float, float]:
        return (self.origin[0] + i * self.res, self.origin[1] + j * self.res)

    def height_at(self, x: float, y: float) -> float:
        i, j = self.world_to_cell(x, y)
        return float(self.H[i, j])

    # ----------------------------------------------------------- digging force
    def dig_force(self, tip_pos: np.ndarray, tip_vel: np.ndarray,
                  cut_dir: np.ndarray) -> np.ndarray:
        """Cutting resistance (N, world frame) on the bucket tip.

        ``cut_dir`` is the bucket cutting (forward) direction unit vector.
        Returns zero when the tip is above the local ground surface.
        """
        s = self.cfg["soil"]
        ground_z = self.height_at(tip_pos[0], tip_pos[1])
        depth = ground_z - tip_pos[2]
        self.last_depth = max(0.0, depth)
        if depth <= 0.0 or self.payload >= s["bucket_capacity"] * s["swell"]:
            self.last_force = np.zeros(3)
            return np.zeros(3)

        w = GEOM["bucket_width"]
        d = depth
        v = float(np.linalg.norm(tip_vel[:2]))    # horizontal cutting speed
        gamma, c = s["gamma"], s["cohesion"]
        F_mag = (gamma * d * d * s["N_gamma"]
                 + c * d * s["N_c"]
                 + gamma * d * v * v / GRAV * s["N_a"]) * w

        # direction: oppose horizontal travel + upward reaction
        horiz = tip_vel[:2]
        hn = np.linalg.norm(horiz)
        if hn > 1e-4:
            opp = -np.array([horiz[0], horiz[1], 0.0]) / hn
        else:
            opp = -np.array([cut_dir[0], cut_dir[1], 0.0])
            on = np.linalg.norm(opp)
            opp = opp / on if on > 1e-6 else np.zeros(3)
        force = F_mag * (0.85 * opp + 0.15 * np.array([0.0, 0.0, 1.0]))
        self.last_force = force
        return force

    # --------------------------------------------------------- mass bookkeeping
    def excavate(self, tip_pos: np.ndarray, tip_vel: np.ndarray, dt: float) -> float:
        """Remove soil the bucket sweeps through this tick; add it to payload.

        Returns the excavated volume (m^3).  Terrain is lowered locally over the
        bucket-width footprint around the tip.
        """
        s = self.cfg["soil"]
        if self.last_depth <= 0.0:
            return 0.0
        cap = s["bucket_capacity"] * s["swell"]
        if self.payload >= cap:
            return 0.0
        w = GEOM["bucket_width"]
        v = float(np.linalg.norm(tip_vel[:2]))
        # swept volume ~ depth * width * distance travelled this tick
        dV = self.last_depth * w * v * dt
        dV = min(dV, cap - self.payload)
        if dV <= 0:
            return 0.0
        self.payload += dV
        self._lower_terrain(tip_pos, w, dV)
        return dV

    def _lower_terrain(self, tip_pos: np.ndarray, w: float, dV: float) -> None:
        i0, j0 = self.world_to_cell(tip_pos[0], tip_pos[1])
        r = max(1, int(round(0.5 * w / self.res)))
        cells = []
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                i, j = i0 + di, j0 + dj
                if 0 <= i < self.nx and 0 <= j < self.ny:
                    cells.append((i, j))
        if not cells:
            return
        drop = dV / (len(cells) * self.res * self.res)
        for i, j in cells:
            self.H[i, j] -= drop

    def dump(self, pos: np.ndarray, fraction: float = 1.0) -> float:
        """Release a fraction of the payload onto the terrain at ``pos`` and
        spread it (crude angle-of-repose smoothing).  Returns released volume."""
        if self.payload <= 0:
            return 0.0
        released = self.payload * float(np.clip(fraction, 0.0, 1.0))
        self.payload -= released
        i0, j0 = self.world_to_cell(pos[0], pos[1])
        r = 3
        cells = [(i0 + di, j0 + dj) for di in range(-r, r + 1) for dj in range(-r, r + 1)
                 if 0 <= i0 + di < self.nx and 0 <= j0 + dj < self.ny]
        if cells:
            rise = released / (len(cells) * self.res * self.res)
            for i, j in cells:
                self.H[i, j] += rise
        return released

    def compact(self, pos: np.ndarray, amount: float = 0.02) -> None:
        i0, j0 = self.world_to_cell(pos[0], pos[1])
        for di in range(-2, 3):
            for dj in range(-2, 3):
                i, j = i0 + di, j0 + dj
                if 0 <= i < self.nx and 0 <= j < self.ny:
                    self.H[i, j] -= amount
                    self.density_flag[i, j] = 1

    # ------------------------------------------------------------------- export
    def height_diff(self) -> np.ndarray:
        """H - H0 (terrain change) for visualisation / band-saving."""
        return self.H - self.H0

    def reset(self) -> None:
        self.H[:] = self.cfg["soil"]["initial_ground_z"]
        self.H0 = self.H.copy()
        self.payload = 0.0
        self.density_flag[:] = 0
        self.last_force = np.zeros(3)
        self.last_depth = 0.0
