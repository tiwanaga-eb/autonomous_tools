from __future__ import annotations

import numpy as np
from fastapi import APIRouter
from pydantic import BaseModel

from planning_core.geometry import project

router = APIRouter(prefix="/api/geo", tags=["geo"])


class TransformRequest(BaseModel):
    points: list[tuple[float, float]]
    src_epsg: int
    dst_epsg: int


@router.post("/transform")
def transform(req: TransformRequest):
    arr = np.asarray(req.points, dtype=float).reshape(-1, 2)
    out = project(arr, req.src_epsg, req.dst_epsg)
    return {
        "points": [[float(a), float(b)] for a, b in out],
        "src_epsg": req.src_epsg,
        "dst_epsg": req.dst_epsg,
    }
