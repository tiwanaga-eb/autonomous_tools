"""プロジェクト保存/読込 API（設計書 §15 / Phase 6）。

FE のワーキング状態（waypoints / areas / 車両・パラメータ / 寄り付き姿勢 / レイヤ参照 など）を
名前付きで永続化する。状態は FE が組み立てる不透明な JSON blob としてサーバが保管・返却する。
ローカル単独運用のため JSON ファイル（projects.json）に保存（SQLite 化は将来）。
"""
from __future__ import annotations

import json
import os
import threading
import uuid
from datetime import datetime, timezone

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from ..settings import PROJECTS_PATH

router = APIRouter(prefix="/api/projects", tags=["projects"])

# read-modify-write をシリアライズ（並行保存での last-writer-wins 消失を防ぐ）。
_LOCK = threading.RLock()


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


class ProjectIn(BaseModel):
    name: str = Field(min_length=1, max_length=200)
    state: dict = Field(default_factory=dict)
    updated_at: str | None = None  # 互換のため受けるが、サーバ側で権威的に上書きする


def _load() -> dict:
    if PROJECTS_PATH.exists():
        return json.loads(PROJECTS_PATH.read_text(encoding="utf-8"))
    return {"projects": {}}


def _save(reg: dict) -> None:
    PROJECTS_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = PROJECTS_PATH.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(reg, indent=2, ensure_ascii=False), encoding="utf-8")
    os.replace(tmp, PROJECTS_PATH)  # アトミック書き込み


def _summary(p: dict) -> dict:
    return {"id": p["id"], "name": p["name"], "updated_at": p.get("updated_at")}


@router.get("")
def list_projects():
    reg = _load()
    items = sorted(reg["projects"].values(), key=lambda p: p.get("updated_at") or "", reverse=True)
    return [_summary(p) for p in items]


@router.post("")
def create_project(req: ProjectIn):
    with _LOCK:
        reg = _load()
        pid = f"proj_{uuid.uuid4().hex[:8]}"
        now = _now_iso()
        reg["projects"][pid] = {
            "id": pid, "name": req.name, "state": req.state, "created_at": now, "updated_at": now,
        }
        _save(reg)
        return _summary(reg["projects"][pid])


@router.get("/{project_id}")
def get_project(project_id: str):
    p = _load()["projects"].get(project_id)
    if not p:
        raise HTTPException(404, "project not found")
    return p


@router.put("/{project_id}")
def update_project(project_id: str, req: ProjectIn):
    with _LOCK:
        reg = _load()
        existing = reg["projects"].get(project_id)
        if not existing:
            raise HTTPException(404, "project not found")
        reg["projects"][project_id] = {
            "id": project_id, "name": req.name, "state": req.state,
            "created_at": existing.get("created_at") or _now_iso(), "updated_at": _now_iso(),
        }
        _save(reg)
        return _summary(reg["projects"][project_id])


@router.delete("/{project_id}")
def delete_project(project_id: str):
    with _LOCK:
        reg = _load()
        if project_id not in reg["projects"]:
            raise HTTPException(404, "project not found")
        del reg["projects"][project_id]
        _save(reg)
    return {"deleted": project_id}
