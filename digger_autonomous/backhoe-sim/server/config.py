"""Single source of truth for all tunable parameters.

All units are SI: metres (m), radians (rad), Newtons (N), Pascals (Pa),
kilograms (kg), seconds (s).  Nothing physical is hard-coded elsewhere; every
module reads from :data:`CONFIG` (or a copy mutated live by the GUI).

The dict is intentionally plain (JSON/YAML serialisable) so the web GUI can
push ``set_param`` deltas by dotted path (e.g. ``hydraulics.Q_pump``).
"""
from __future__ import annotations

import copy
import math
from typing import Any

import yaml

# ---------------------------------------------------------------------------
# Kinematic chain (sagittal plane, x forward, z up).  These numbers are mirrored
# exactly by sim/model.xml; tests/test_kinematics.py asserts they stay in sync.
# Offsets are expressed in the *parent* body frame.
# ---------------------------------------------------------------------------
# Komatsu PC200-11 (標準機 / 2.9 m 標準アーム). 寸法はスペックシート外形図・作業範囲図に整合。
# リンク長は実機公称（標準ブーム 5.7 m / 標準アーム 2.9 m / バケット先端半径 ~1.5 m）。
GEOM = {
    # lower travel body (tracks) — box half-extents and centre height
    # クローラ全長 4070 mm -> 半長 2.035, 全幅 2805 mm -> 半幅 1.40
    "base_half": [2.035, 1.40, 0.50],
    "base_z": 0.50,            # centre height of lower body (sits on ground)
    # swing (upper structure) pivot, relative to base origin (旋回ベアリング ~1.0 m)
    "swing_pivot": [0.0, 0.0, 1.0],
    # 上部旋回体: 後端旋回半径 2830 mm に合わせ x オフセットを model.xml 側で設定
    "cab_half": [1.6, 1.40, 0.95],
    # boom foot pin, relative to swing origin -> world z ~= 2.45 (PC200-11)
    "boom_pivot": [0.55, 0.0, 1.45],
    "boom_len": 5.70,          # 標準ブーム長
    # arm pin, relative to boom origin (along boom local +x)
    "arm_pivot": [5.70, 0.0, 0.0],
    "arm_len": 2.90,           # 標準アーム長 (2.9 m アーム)
    # bucket pin, relative to arm origin
    "bucket_pivot": [2.90, 0.0, 0.0],
    "bucket_len": 1.50,        # バケットピン-先端半径
    "bucket_width": 1.05,      # 標準バケット幅(サイドカッタ含む) 1045 mm
    # bucket-tip site position in the bucket body frame (matches model.xml site)
    "tip_local": [1.50, 0.0, -0.35],
}

# Joint order is the single canonical order used by every vector in the system.
JOINT_NAMES = ["swing", "boom", "arm", "bucket"]

# Joint limits (rad).  Sign conventions: see docs in machine.py.
# 符号系(検証済): 各リンクの水平からの仰角 = -(関節角の総和)。よって
#   boom q<0 = ブーム上げ(elbow up) / q>0 = 下げ,  arm q>0 = スティックを下げて掘る向き
#   (バックホウ). これで安静姿勢が「ブーム上げ+スティック下げ+バケット機械側巻込み」になる。
# PC200-11: スペックの作業範囲（最大掘削高さ 10.0 m / 掘削深さ 6.62 m / 掘削半径 9.875 m /
# 床面半径 9.70 m）を再現するよう、実機リンク長を固定して可動域を数値同定（cylinder stroke
# は資料に無いため作業範囲から逆算）。
JOINT_LIMITS = {
    "swing": [-math.pi, math.pi],
    "boom": [math.radians(-40.0), math.radians(54.0)],
    "arm": [math.radians(-20.0), math.radians(160.0)],
    "bucket": [math.radians(-25.0), math.radians(175.0)],
}

# A comfortable "ready to dig" home pose (rad), used on reset (PC200-11).
# ブーム上げ・スティック下げ・バケット前方地面付近 -> 先端 ~(7.5, 0.9) m の掘削準備姿勢。
HOME_POSE = {
    "swing": 0.0,
    "boom": math.radians(-22.0),
    "arm": math.radians(70.0),
    "bucket": math.radians(40.0),
}

CONFIG: dict[str, Any] = {
    # -------------------------------------------------------------- simulation
    "sim": {
        "dt": 0.002,            # 500 Hz physics
        "viz_hz": 60.0,         # state stream rate
        "realtime": True,       # throttle sim to wall clock
        "speed": 1.0,           # playback multiplier
        "gravity": -9.81,
    },
    # -------------------------------------------------------------- hydraulics
    "hydraulics": {
        "level": "L1",          # L0 | L1 | L2
        "Q_pump": 200.0 / 60000.0,   # 200 L/min -> m^3/s（注: ポンプ吐出量は資料に記載なし＝据置）
        "P_relief": 34.8e6,          # 作業時 最大セット圧力 34.8 MPa (PC200-11, 走行時は37.3)
        "beta": 1.5e9,               # bulk modulus (Pa)
        "deadband": 0.08,
        "expo": 1.5,
        "tau_response": 0.12,        # 1st-order lag time constant (L1)
        # per-cylinder areas (bore / rod side), m^2
        "A_a": math.pi * (0.110 / 2) ** 2,
        "A_b": math.pi * (0.110 / 2) ** 2 - math.pi * (0.070 / 2) ** 2,
        "visc_friction": 4.0e3,      # N per (m/s)
        "coulomb_friction": 2.0e3,   # N
        # nominal moment-arm length per axis (m) for force->torque (L0/L1)
        "moment_arm": {"swing": 0.9, "boom": 1.4, "arm": 1.2, "bucket": 0.7},
        # velocity-servo stiffness (N*m per rad/s) — tuned below the explicit
        # stability bound k_v*dt/I_eff for each (light) joint at 500 Hz
        # valve-closed lock: position stiffness (servo_kv) + damping (lock_kd)
        "servo_kv": {"swing": 3.0e5, "boom": 2.5e5, "arm": 1.2e5, "bucket": 5.0e4},
        "lock_kd": {"swing": 6.0e4, "boom": 5.0e4, "arm": 2.4e4, "bucket": 1.0e4},
        # meter-out velocity-cap braking (N*m per rad/s of overspeed). Models the
        # return-line back-pressure that stops a load outrunning the metered flow.
        # Kept below the 500 Hz explicit-stability bound k*dt/I_eff < 2 per axis.
        "k_meter": {"swing": 2.0e5, "boom": 1.5e5, "arm": 8.0e4, "bucket": 2.5e4},
        # peak torque used to scale the static L0 map (N*m), per axis
        # bucket/arm は PC200-11 の最大掘削力(JIS A 8403-5: バケット138 kN, アーム101 kN)に整合。
        # bucket: 138 kN × 先端半径 1.5 m ≈ 2.07e5 N*m。
        "tau_max": {"swing": 1.3e5, "boom": 4.5e5, "arm": 2.6e5, "bucket": 2.07e5},
        # cylinder dead volume + stroke (L2 pressure dynamics), m^3 / m
        "V_dead": 1.0e-3,
        "stroke": 1.2,
    },
    # -------------------------------------------------------------- joint control
    "control": {
        # gains live in normalised torque-demand space: tau = kp*tau_scale*e + ...
        "tau_scale": 3.0e4,
        "kp": {"swing": 9.0, "boom": 14.0, "arm": 12.0, "bucket": 8.0},
        "kd": {"swing": 1.2, "boom": 2.0, "arm": 1.6, "bucket": 1.0},
        "ki": {"swing": 0.0, "boom": 0.5, "arm": 0.5, "bucket": 0.2},
        "i_clamp": 0.5,
        "gravity_comp": True,       # use qfrc_bias feed-forward
        # admittance (force control) for digging
        "admittance": {
            "target_dig_force": 9.0e4,   # N target cutting force (PC200-11級; 最大破断力138kNの約65%)
            "gain": 2.0e-6,              # m/s per N of error
            "max_retract": 0.4,         # m/s cap on depth give
        },
    },
    # -------------------------------------------------------------- soil
    "soil": {
        "size_x": 20.0, "size_y": 20.0, "res": 0.10,   # height-field grid
        "origin": [-4.0, -10.0],     # world (x,y) of grid cell (0,0)
        "gamma": 18000.0,            # unit weight N/m^3
        "cohesion": 5.0e3,           # Pa
        "phi": math.radians(30.0),   # internal friction angle
        # FEE coefficients (shape/friction dependent, simplified)
        "N_gamma": 8.0, "N_c": 12.0, "N_a": 1.2,
        "bucket_capacity": 0.80,     # m^3 (PC200-11 標準バケット容量 JIS A 8403-4)
        "swell": 1.25,               # bulking factor when loaded
        "repose_angle": math.radians(34.0),
        "initial_ground_z": 0.0,
    },
    # -------------------------------------------------------------- safety
    "safety": {
        "enabled": True,
        "design": {
            "type": "plane",         # plane | slope | surface
            "floor_z": -1.5,         # horizontal design floor (m)
            "slope_origin": [2.0, 0.0, 0.0],
            "slope_dir": [1.0, 0.0],     # heading of descending slope (x,y)
            "slope_grade": math.radians(30.0),
            "alpha": 3.0,            # class-K gain for design barrier
            "n_edge_samples": 3,     # bucket-width edge points
        },
        "truck": {
            "enabled": True,
            "pos": [0.0, 6.0, 0.0],      # world position of vessel centre-ish
            "yaw": 0.0,
            "cab_half": [1.2, 1.0, 1.3],
            "cab_offset": [-2.6, 0.0, 1.3],   # relative to truck pos
            "vessel_half": [2.0, 1.3, 0.9],
            "vessel_offset": [0.6, 0.0, 1.6],
            "wall_thickness": 0.15,
            "margin": 0.4,           # safety clearance (m)
            "alpha": 2.5,
        },
        "ground": {                 # generic over-penetration barrier
            "alpha": 4.0, "margin": 0.05,
            "relax_when_digging": True,
        },
        "selfcollision": {"alpha": 5.0, "margin": 0.2},
        "qp": {"solver": "osqp", "u_min": -1.0, "u_max": 1.0},
    },
    # -------------------------------------------------------------- maneuvers
    "maneuvers": {
        "dig": {
            "penetration_angle": math.radians(45.0),
            "target_depth": 0.5,
            "drag_distance": 2.0,
            "curl_rate": 0.6,
            "approach_x": 5.5, "approach_y": 0.0,
        },
        "swing": {"target_yaw": math.pi / 2, "speed": 0.6, "boom_lift": 0.3},
        "dump": {"open_rate": 1.2, "hold_time": 1.0},
        "trench": {"dx": 0.6, "depth": 1.0, "width": 1.0, "tol": 0.08},
        "grade": {"target_z": -0.2, "forward_speed": 0.4, "overlap": 0.3},
        "slope": {"grade": math.radians(30.0), "speed": 0.3},
        "compaction": {"target_force": 8.0e4, "hold_time": 0.6, "grid": [3, 3]},
    },
    # -------------------------------------------------------------- teleop
    "teleop": {
        "mode": "AUTO",                  # MANUAL | MANUAL_ASSIST | AUTO | AUTO_OVERRIDE
        "pattern": "ISO",                # ISO | SAE
        "deadzone": 0.12,
        "expo": 1.4,
        "max_rate": {"swing": 0.7, "boom": 0.6, "arm": 0.8, "bucket": 1.0},  # rad/s
        "override_blend": 0.8,           # beta for AUTO_OVERRIDE
        "comm_timeout": 0.2,             # s before rate-zero safe stop
    },
}


def deep_copy() -> dict[str, Any]:
    """Return a mutable deep copy of the default config."""
    return copy.deepcopy(CONFIG)


def get_by_path(cfg: dict, path: str) -> Any:
    node = cfg
    for key in path.split("."):
        node = node[key]
    return node


def set_by_path(cfg: dict, path: str, value: Any) -> None:
    """Set ``cfg[a][b][c] = value`` from dotted ``path = "a.b.c"``."""
    keys = path.split(".")
    node = cfg
    for key in keys[:-1]:
        node = node[key]
    node[keys[-1]] = value


def load_yaml(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as fh:
        override = yaml.safe_load(fh) or {}
    cfg = deep_copy()
    _merge(cfg, override)
    return cfg


def _merge(base: dict, override: dict) -> None:
    for key, val in override.items():
        if isinstance(val, dict) and isinstance(base.get(key), dict):
            _merge(base[key], val)
        else:
            base[key] = val
