"""WebSocket message encode/decode for the backhoe simulator."""
from __future__ import annotations

import json

import mujoco
import numpy as np

from config import JOINT_NAMES

_VIZ_BODIES = ["base", "swing", "boom", "arm", "bucket"]


def encode_state(engine) -> str:
    """Encode the current engine state as a JSON string (60 Hz viz stream)."""
    machine = engine.machine
    status = engine.last_status

    q = machine.q.tolist()
    qd = machine.qd.tolist()
    tip, _ = machine.tip_pose()

    # Body world-frame poses for Three.js visualisation
    model = machine.model
    data = machine.data
    xpos: dict = {}
    xquat: dict = {}
    for name in _VIZ_BODIES:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid >= 0:
            xpos[name] = [round(v, 4) for v in data.xpos[bid].tolist()]
            xquat[name] = [round(v, 6) for v in data.xquat[bid].tolist()]  # (w,x,y,z)

    return json.dumps({
        "type": "state",
        "t": round(engine.t, 4),
        "q": [round(v, 5) for v in q],
        "qd": [round(v, 5) for v in qd],
        "tip": [round(v, 4) for v in tip.tolist()],
        "mode": engine.cfg["teleop"]["mode"],
        "fsm_phase": engine.fsm.phase_name,
        "payload": round(float(engine.soil.payload), 4),
        "tau": [round(float(v), 1) for v in engine.hyd.last_tau.tolist()],
        "safety": {
            "geofence": bool(status.get("geofence", False)),
            "cbf": bool(status.get("cbf", False)),
            "active": bool(status.get("active", False)),
            "min_h": round(float(status.get("min_h", 999.0)), 4),
        },
        "xpos": xpos,
        "xquat": xquat,
        "paused": engine.paused,
    })


def encode_terrain(engine) -> str:
    """Encode the terrain height-field as a JSON string."""
    soil = engine.soil
    cfg = engine.cfg["soil"]
    return json.dumps({
        "type": "terrain",
        "origin": cfg["origin"],
        "res": cfg["res"],
        "size_x": cfg["size_x"],
        "size_y": cfg["size_y"],
        "grid": soil.H.tolist(),
    })


def decode_command(raw: str) -> dict:
    """Parse an incoming command JSON string. Returns {} on error."""
    try:
        data = json.loads(raw)
        return data if isinstance(data, dict) else {}
    except (json.JSONDecodeError, ValueError):
        return {}
