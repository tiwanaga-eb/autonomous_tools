from __future__ import annotations

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from .routers import agent, costmap, drivable, earthworks, fleet, geo, layers, planning, projects, simulate, tiles, vehicles
from .settings import JGD2011_ZONES, get_working_epsg, set_working_epsg

app = FastAPI(title="FMS Route Studio API", version="0.1.0")

# ローカル単独運用。FE(Vite)からのアクセスを想定して CORS は緩め。
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/health", tags=["meta"])
def health():
    return {
        "status": "ok",
        "service": "fms-route-studio-api",
        "default_epsg": get_working_epsg(),
        "zones": list(JGD2011_ZONES),
    }


class CrsRequest(BaseModel):
    epsg: int


@app.put("/api/crs", tags=["meta"])
def set_crs(req: CrsRequest):
    """作業ゾーン(投影座標系)を実行時に変更する（セッション内・メモリ保持）。
    以後のアップロード/コストマップ生成はこのEPSGへ正規化される。"""
    try:
        epsg = set_working_epsg(req.epsg)
    except ValueError as e:
        raise HTTPException(400, str(e))
    return {"default_epsg": epsg}


app.include_router(layers.router)
app.include_router(tiles.router)
app.include_router(costmap.router)
app.include_router(drivable.router)
app.include_router(geo.router)
app.include_router(planning.router)
app.include_router(vehicles.router)
app.include_router(simulate.router)
app.include_router(projects.router)
app.include_router(agent.router)
app.include_router(fleet.router)
app.include_router(earthworks.router)
