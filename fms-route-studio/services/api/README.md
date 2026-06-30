# services/api (FastAPI · API専用)

`planning_core` を import する薄いAPI層。**Phase 1 実装済み**（layers/tiles/geo/plan/analyze）。

## エンドポイント（Phase 1）

| Method | Path | 内容 |
|---|---|---|
| GET | `/health` | ヘルス + 既定EPSG |
| POST | `/api/layers/{kind}` | ortho/cost/dsm/las アップロード→COG化・登録（CRS無しはassign） |
| GET | `/api/layers` `/api/layers/{id}` | 一覧 / 詳細 |
| DELETE | `/api/layers/{id}` | 削除 |
| GET | `/api/tiles/{id}/{z}/{x}/{y}.png` | COGタイル（rio-tiler, WebMercator） |
| GET | `/api/layers/{id}/preview.png` | プレビューPNG |
| POST | `/api/geo/transform` | 任意EPSG間の点群変換 |
| POST | `/api/plan` | spline経路生成（曲率/dκ/ds付き Trajectory + 解析） |
| POST | `/api/analyze` | 既存ポリラインの曲率/最小半径解析 |

## 起動 / テスト

```bash
# planning_core を editable で入れた venv 前提
pip install -e packages/planning_core
pip install -e "services/api[dev]"

# 起動（ローカル）
cd services/api && uvicorn app.main:app --reload --port 8077

# テスト
pytest -q services/api/tests
```

> PROJ 競合環境（Anaconda 等）では `app/_proj_fix.py` が rasterio 同梱の PROJ データへ自動で切替える。
> 地図ライブラリ/再投影方針は Phase 2 で PoC 確定（設計書 §6.2・残課題9）。
