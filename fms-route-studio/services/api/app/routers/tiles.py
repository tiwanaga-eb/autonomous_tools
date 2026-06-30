from __future__ import annotations

from fastapi import APIRouter, HTTPException, Response
from rio_tiler.errors import TileOutsideBounds
from rio_tiler.io import Reader

from .. import store

router = APIRouter(prefix="/api", tags=["tiles"])

_PNG = "image/png"


def _cog_path(layer_id: str) -> str:
    meta = store.get_layer(layer_id)
    if not meta or "cog" not in meta:
        raise HTTPException(404, "raster layer not found")
    return meta["cog"]


@router.get("/tiles/{layer_id}/{z}/{x}/{y}.png")
def tile(layer_id: str, z: int, x: int, y: int):
    cog = _cog_path(layer_id)
    try:
        with Reader(cog) as r:
            img = r.tile(x, y, z)
    except TileOutsideBounds:
        raise HTTPException(404, "tile out of bounds")
    return Response(content=img.render(img_format="PNG"), media_type=_PNG)


@router.get("/layers/{layer_id}/preview.png")
def preview(layer_id: str, max_size: int = 512):
    cog = _cog_path(layer_id)
    with Reader(cog) as r:
        img = r.preview(max_size=max_size)
    return Response(content=img.render(img_format="PNG"), media_type=_PNG)
