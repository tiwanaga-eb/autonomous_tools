import math

import numpy as np

from planning_core.fleet import junction_pose


def test_junction_pose_midpoint_heading():
    """東向き直線の中点: 位置=中央、heading=0度（接線）。"""
    pts = np.column_stack([np.linspace(0, 100, 50), np.zeros(50)])
    x, y, h = junction_pose(pts, 0.5)
    assert abs(x - 50.0) < 1.0 and abs(y) < 1e-6
    assert abs(((h + 180) % 360) - 180) < 1e-6  # 0度


def test_junction_pose_tangent_on_turn():
    """90度コーナー: 終端側(s_frac=0.9)では北向き(+90度)に近い接線。"""
    a = np.column_stack([np.linspace(0, 50, 30), np.zeros(30)])
    b = np.column_stack([np.full(30, 50.0), np.linspace(0, 50, 30)])
    pts = np.vstack([a, b])
    _, _, h = junction_pose(pts, 0.9)
    assert abs(h - 90.0) < 5.0  # 北向き


def test_junction_pose_endpoints():
    pts = np.column_stack([np.linspace(0, 10, 11), np.zeros(11)])
    x0, y0, _ = junction_pose(pts, 0.0)
    x1, y1, _ = junction_pose(pts, 1.0)
    assert abs(x0) < 1e-6 and abs(x1 - 10.0) < 1e-6
