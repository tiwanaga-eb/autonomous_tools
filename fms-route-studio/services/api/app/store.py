"""Local file-based layer registry (Phase 1).

ローカル単独運用。Phase 1 は JSON レジストリ + ファイル。SQLite 化は Phase 6（設計書 §15）。
"""
from __future__ import annotations

import json
import os
import shutil
import threading
import uuid
from pathlib import Path

from .settings import LAYERS_DIR, REGISTRY_PATH

# read-modify-write をシリアライズ（FastAPI の threadpool で複数リクエストが同時に
# レジストリを書き換えても last-writer-wins で失われないように）。
_LOCK = threading.RLock()


def _load() -> dict:
    if REGISTRY_PATH.exists():
        return json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))
    return {"layers": {}}


def _save(reg: dict) -> None:
    LAYERS_DIR.mkdir(parents=True, exist_ok=True)
    # アトミック書き込み: tmp へ書いてから os.replace（書き込み中断/競合でレジストリが空になるのを防ぐ）。
    text = json.dumps(reg, indent=2, ensure_ascii=False)
    tmp = REGISTRY_PATH.with_suffix(".json.tmp")
    tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, REGISTRY_PATH)


def layer_dir(layer_id: str) -> Path:
    return LAYERS_DIR / layer_id


def new_layer_id(kind: str) -> str:
    return f"{kind}_{uuid.uuid4().hex[:8]}"


def add_layer(meta: dict) -> dict:
    with _LOCK:
        reg = _load()
        reg["layers"][meta["id"]] = meta
        _save(reg)
    return meta


def list_layers() -> list[dict]:
    return list(_load()["layers"].values())


def get_layer(layer_id: str) -> dict | None:
    return _load()["layers"].get(layer_id)


def delete_layer(layer_id: str) -> bool:
    with _LOCK:
        reg = _load()
        if layer_id not in reg["layers"]:
            return False
        del reg["layers"][layer_id]
        _save(reg)
    d = layer_dir(layer_id)
    if d.exists():
        shutil.rmtree(d, ignore_errors=True)
    return True
