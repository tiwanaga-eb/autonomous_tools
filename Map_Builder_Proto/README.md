# FMS Map Building Tool

`map_tool.py` を中心にした、社内向けの地図編集・ルート生成ツールです。  
Ortho(オルソ画像)を基準グリッドとして、CostMapを重ね表示しながら Route と Area(Polygon) を作成し、JSONとして出力できます。

## 主な機能

- Ortho / Cost GeoTIFF のアップロード
- 地図表示（ズーム、パン、Cost表示ON/OFF、透過率変更）
- Route 編集
  - Start / Goal / Via 追加
  - Via の途中挿入（Insert Via）
  - ドラッグ編集（Edit Points）
  - 最小旋回半径制約付きルート生成
- Area 編集
  - Polygon 作成・確定・取消・全削除
  - エリア名: プリセット選択 + 自由入力
- 保存済みアイテム管理
  - Draft Route / Polygon の一覧表示
  - 選択ハイライト、個別削除、選択アイテムへフォーカス
- JSON 出力
  - `haulRoutes` と `polygons` を同時に保存

## ファイル構成

- `map_tool.py`
  - FastAPI サーバ
  - UI(HTML/CSS/JS)
  - ルート生成ロジック
- `costmap_ortho_out.py`
  - LAS から CostMap GeoTIFF を作成するユーティリティ

## セットアップ

```bash
cd /Users/tosuke_iwanaga/Documents/GitHub/thomas.21-eb/test
python -m venv .venv
source .venv/bin/activate
pip install fastapi uvicorn rasterio pillow numpy pyproj scipy laspy python-multipart
```

## 起動方法

### 1) Webアプリ起動

```bash
python map_tool.py
```

または

```bash
uvicorn map_tool:app --host 0.0.0.0 --port 8000 --reload
```

ブラウザで以下を開きます:

- `http://127.0.0.1:8000/`

### 2) CostMap作成（必要な場合）

```bash
python costmap_ortho_out.py
```

`costmap_ortho_out.py` 内の `las_path`, `out_tif`, EPSG などを環境に合わせて変更してください。

## 使い方（基本フロー）

1. `Upload Ortho` でオルソ画像を読み込む
2. 必要に応じて `Upload Cost` でコストマップを重ねる
3. Route Workspace で Start / Goal / Via を設定
4. `Generate Preview` でルート生成
5. 必要なら `Save Draft Route` で仮保存し、`New Route` で次のルート作成
6. Area Workspace で Polygon を作成し `Finish Polygon` で確定
7. `Save JSON` で `haulRoutes + polygons` を保存

## ルート制約モード

- `strict_via`
  - 通過点厳守
  - 最小旋回半径を満たせない場合はエラー
- `prefer_min_turn_radius`
  - 最小旋回半径の達成を優先（未達時は警告）

## JSON出力仕様（現行）

```json
{
  "haulRoutes": [
    {
      "name": "Route_1",
      "route": [
        { "lat": 35.0, "lng": 139.0 }
      ]
    }
  ],
  "polygons": [
    {
      "name": "ParkingZone",
      "points": [
        { "lat": 35.0, "lng": 139.0 }
      ],
      "points_xy": [
        { "x": 30465.1, "y": 119228.8 }
      ]
    }
  ]
}
```

- `points`: WGS84 (`EPSG:4326`)
- `points_xy`: canonical grid CRS（OrthoのEPSG）

## エリア名プリセット

- `ParkingZone`
- `LoadingZone`
- `DumpingZone`
- `AutonomousOperationZone`
- `NoGoZone`

自由入力がある場合は自由入力名が優先されます。

## 主なAPI

- `GET /status`
- `POST /upload_raster/{layer}` (`layer`: `ortho` or `cost`)
- `POST /clear_all`
- `GET /raster.png?layer=ortho|cost&v=...`
- `GET /xy_to_canonical`
- `GET /lonlat_to_canonical`
- `POST /canonical_to_lonlat_points`
- `POST /waypoints_from_map`
- `POST /json_to_canonical_with_offset`

## 備考

- 起動時に既存の `ortho.tif` / `costmap.tif` がある場合、自動同期で表示されます。
- 画像読み込み失敗時はフォールバック読み込みを実施します。
- 社内限定運用を前提にした実装です（認証は未実装）。
