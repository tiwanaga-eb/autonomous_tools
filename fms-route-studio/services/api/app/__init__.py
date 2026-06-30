"""FMS Route Studio API (FastAPI, API-only)。

PROJ 修正を最初に適用してから rasterio/rio_tiler 系を使う。
"""
from ._proj_fix import apply_proj_fix

apply_proj_fix()
