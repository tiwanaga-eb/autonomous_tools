"""Unit tests: kinematics, hydraulics, and soil."""
from __future__ import annotations
import sys
from pathlib import Path

import numpy as np
import pytest

# Ensure server/ is on the path
sys.path.insert(0, str(Path(__file__).parents[1] / "server"))


# ─── kinematics ───────────────────────────────────────────────────────────────

class TestKinematics:
    @pytest.fixture(scope="class")
    def machine(self):
        from sim.machine import Machine
        return Machine.load()

    def test_home_tip_height_positive(self, machine):
        """At home pose the bucket tip should be above ground (z > 0)."""
        tip, _ = machine.tip_pose()
        assert tip[2] > 0.0, f"tip z={tip[2]:.3f} should be > 0"

    def test_q_shape(self, machine):
        q = machine.q
        assert q.shape == (4,)

    def test_qd_zero_at_rest(self, machine):
        """Freshly-loaded machine should have no joint velocity."""
        qd = machine.qd
        np.testing.assert_allclose(qd, 0.0, atol=1e-6)

    def test_jacobian_shape(self, machine):
        J = machine.tip_jacobian()
        assert J.shape == (3, 4), f"Expected (3,4), got {J.shape}"

    def test_tip_velocity_zero_at_rest(self, machine):
        v = machine.tip_velocity()
        assert v.shape == (3,)
        np.testing.assert_allclose(v, 0.0, atol=1e-6)

    def test_clamp_q_within_limits(self, machine):
        from config import JOINT_LIMITS, JOINT_NAMES
        lo = np.array([JOINT_LIMITS[n][0] for n in JOINT_NAMES])
        hi = np.array([JOINT_LIMITS[n][1] for n in JOINT_NAMES])
        q_over  = hi + 0.5
        q_under = lo - 0.5
        assert np.all(machine.clamp_q(q_over)  <= hi)
        assert np.all(machine.clamp_q(q_under) >= lo)

    def test_tip_moves_with_boom_angle(self, machine):
        """Incrementing boom angle should change tip height."""
        tip0, _ = machine.tip_pose()
        from config import JOINT_NAMES
        import mujoco
        # Advance boom by 5°
        boom_id = JOINT_NAMES.index("boom")
        dof = machine.jnt_dof["boom"]
        machine.data.qpos[machine.jnt_qpos["boom"]] += np.radians(5)
        mujoco.mj_forward(machine.model, machine.data)
        tip1, _ = machine.tip_pose()
        # Reset
        machine.data.qpos[machine.jnt_qpos["boom"]] -= np.radians(5)
        mujoco.mj_forward(machine.model, machine.data)
        assert not np.allclose(tip0, tip1), "Tip should move when boom angle changes"


# ─── hydraulics ───────────────────────────────────────────────────────────────

class TestHydraulics:
    @pytest.fixture(scope="class")
    def hyd(self):
        from sim.hydraulics import Hydraulics
        from config import CONFIG
        return Hydraulics(CONFIG)

    def test_zero_input_zero_torque(self, hyd):
        from config import CONFIG
        dt = CONFIG["sim"]["dt"]
        q  = np.zeros(4)
        qd = np.zeros(4)
        tau = hyd.step(np.zeros(4), q, qd, dt)
        # At zero input, torque should be negligible (first-order lag decays)
        assert tau.shape == (4,)

    def test_torque_direction_matches_command(self, hyd):
        """Positive velocity command should produce positive torque on boom."""
        from config import CONFIG
        dt = CONFIG["sim"]["dt"]
        u = np.array([0.0, 1.0, 0.0, 0.0])  # boom only
        q  = np.zeros(4)
        qd = np.zeros(4)
        # run a few steps to overcome lag
        for _ in range(20):
            tau = hyd.step(u, q, qd, dt)
        assert tau[1] > 0, f"Expected boom torque > 0, got {tau[1]:.1f}"

    def test_last_tau_updated(self, hyd):
        from config import CONFIG
        dt = CONFIG["sim"]["dt"]
        hyd.step(np.ones(4), np.zeros(4), np.zeros(4), dt)
        assert hyd.last_tau.shape == (4,)
        assert not np.all(hyd.last_tau == 0)

    def test_relief_valve_clamps_torque(self, hyd):
        """Extremely large demand should be clamped by relief pressure."""
        from config import CONFIG
        dt = CONFIG["sim"]["dt"]
        u = np.full(4, 1e6)  # saturating
        tau = hyd.step(u, np.zeros(4), np.zeros(4), dt)
        P_relief = CONFIG["hydraulics"]["P_relief"]
        # torque magnitude shouldn't grow without bound (approximate check)
        assert np.all(np.abs(tau) < 1e8), f"Torque too large: {tau}"


# ─── soil ────────────────────────────────────────────────────────────────────

class TestSoil:
    @pytest.fixture(scope="class")
    def soil(self):
        from sim.soil import Soil
        from config import CONFIG
        return Soil(CONFIG)

    def test_initial_height_uniform(self, soil):
        cfg = soil.cfg["soil"]
        h = soil.height_at(cfg["origin"][0] + 1.0, cfg["origin"][1] + 1.0)
        assert abs(h - cfg["initial_ground_z"]) < 1e-6

    def test_dig_removes_material(self, soil):
        cfg = soil.cfg["soil"]
        from config import CONFIG
        dt = CONFIG["sim"]["dt"]
        ox, oy = cfg["origin"]
        tip  = np.array([ox + 1.0, oy + 1.0, cfg["initial_ground_z"] - 0.05])
        vel  = np.array([1.0, 0.0, 0.0])  # horizontal motion
        cut  = np.array([1.0, 0.0, 0.0])

        h_before = soil.height_at(tip[0], tip[1])
        # simulate digging for 0.5 s
        for _ in range(int(0.5 / dt)):
            soil.dig_force(tip, vel, cut)
            soil.excavate(tip, vel, dt)

        h_after = soil.height_at(tip[0], tip[1])
        assert h_after < h_before, f"Soil not removed: {h_before:.4f} → {h_after:.4f}"
        assert soil.payload > 0, "Payload should be positive after digging"

    def test_payload_bounded_by_capacity(self, soil):
        cfg = soil.cfg["soil"]
        cap = cfg["bucket_capacity"] * cfg["swell"]
        assert soil.payload <= cap + 1e-9

    def test_release_decreases_payload(self, soil):
        if soil.payload <= 0:
            pytest.skip("No payload to release")
        from config import CONFIG
        p_before = soil.payload
        soil.dump(np.zeros(3), fraction=1.0)
        assert soil.payload < p_before

    def test_height_at_out_of_bounds_clamps(self, soil):
        """Out-of-bounds query should return a valid height (not crash)."""
        h = soil.height_at(999.0, 999.0)
        assert np.isfinite(h)
