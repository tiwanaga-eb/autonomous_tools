"""Backhoe simulator WebSocket server.

Architecture
------------
* Sim thread  : runs `engine.step()` at 500 Hz (2 ms / tick) in a background
                thread with a `threading.Lock` guard.
* asyncio loop: handles WebSocket connections (websockets ≥ 14 async API) and
                receives state broadcasts from the sim thread via
                `loop.call_soon_threadsafe`.

Protocol (see bridge/ws.py)
---------------------------
Server → client (60 Hz):  {"type": "state", ...}
Server → client (on change): {"type": "terrain", ...}
Client → server:          {"cmd": "...", ...}
"""
from __future__ import annotations

import asyncio
import os
import sys
import threading
import time

# Allow `from config import ...` and `from sim.simulation import ...`
sys.path.insert(0, os.path.dirname(__file__))

from websockets.asyncio.server import serve as ws_serve  # websockets ≥ 14

from bridge.ws import decode_command, encode_state, encode_terrain
from config import set_by_path
from sim.simulation import SimEngine

HOST = "0.0.0.0"
PORT = int(os.environ.get("BACKHOE_WS_PORT", "8765"))

# ── shared globals ────────────────────────────────────────────────────────────
_engine: SimEngine | None = None
_lock = threading.Lock()
_clients: set = set()
_loop: asyncio.AbstractEventLoop | None = None
_running = True
_terrain_dirty = True  # send on first connect and after excavation


# ── sim thread (500 Hz) ───────────────────────────────────────────────────────
def _sim_thread() -> None:
    global _terrain_dirty

    cfg = _engine.cfg
    dt: float = cfg["sim"]["dt"]
    viz_period = max(1, round(1.0 / (cfg["sim"]["viz_hz"] * dt)))  # ≈ 8 steps
    step = 0

    while _running:
        t0 = time.perf_counter()

        with _lock:
            prev_payload = _engine.soil.payload
            _engine.step()
            step += 1
            if _engine.soil.payload != prev_payload:
                _terrain_dirty = True
            if step % viz_period == 0 and _clients and _loop is not None:
                state_msg = encode_state(_engine)
                terrain_msg: str | None = None
                if _terrain_dirty:
                    terrain_msg = encode_terrain(_engine)
                    _terrain_dirty = False

        if step % viz_period == 0 and _clients and _loop is not None:
            _loop.call_soon_threadsafe(_schedule_broadcast, state_msg)
            if terrain_msg is not None:
                _loop.call_soon_threadsafe(_schedule_broadcast, terrain_msg)

        if cfg["sim"].get("realtime", True):
            elapsed = time.perf_counter() - t0
            remaining = dt - elapsed
            if remaining > 1e-4:
                time.sleep(remaining)


def _schedule_broadcast(msg: str) -> None:
    """Called in the asyncio thread; fans out msg to all connected clients."""
    for ws in list(_clients):
        asyncio.ensure_future(ws.send(msg))


# ── WebSocket handler ─────────────────────────────────────────────────────────
async def _handler(websocket) -> None:
    _clients.add(websocket)
    try:
        # greet new client with full terrain + initial state
        with _lock:
            greet_terrain = encode_terrain(_engine)
            greet_state = encode_state(_engine)
        await websocket.send(greet_terrain)
        await websocket.send(greet_state)

        async for raw in websocket:
            cmd = decode_command(raw)
            if cmd:
                _apply_command(cmd)
    except Exception:
        pass
    finally:
        _clients.discard(websocket)


def _apply_command(cmd: dict) -> None:
    verb = cmd.get("cmd", "")
    with _lock:
        if verb == "joystick":
            _engine.teleop.update_input(
                cmd.get("axes", []), cmd.get("buttons", []), _engine.t
            )
        elif verb == "mode":
            _engine.set_mode(str(cmd["mode"]))
        elif verb == "maneuver":
            _engine.select_maneuver(str(cmd["name"]), bool(cmd.get("cycle", False)))
        elif verb == "set_param":
            set_by_path(_engine.cfg, str(cmd["path"]), cmd["value"])
        elif verb == "set_joint":
            _engine.set_joint_target(str(cmd["name"]), float(cmd["value"]))
        elif verb == "reset":
            _engine.reset()
        elif verb == "pause":
            _engine.paused = True
        elif verb == "resume":
            _engine.paused = False


# ── entry point ───────────────────────────────────────────────────────────────
async def _amain() -> None:
    global _engine, _loop
    _loop = asyncio.get_event_loop()

    print("Initialising simulation engine …")
    _engine = SimEngine()
    print("Engine ready.")

    sim_t = threading.Thread(target=_sim_thread, daemon=True, name="sim")
    sim_t.start()

    print(f"WebSocket server: ws://{HOST}:{PORT}")
    async with ws_serve(_handler, HOST, PORT):
        await asyncio.get_event_loop().create_future()  # run until interrupted


def main() -> None:
    try:
        asyncio.run(_amain())
    except KeyboardInterrupt:
        global _running
        _running = False
        print("\nServer stopped.")


if __name__ == "__main__":
    main()
