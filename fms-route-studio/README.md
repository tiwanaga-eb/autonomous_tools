# FMS Route Studio (SmartConstruction AI)

地図・コストマップ・走行可能領域・経路生成（静的/動的）・解析・安全検証・AIアシスタントを統合した、
建機向けルート設計プラットフォーム。'FMS Map Building Tool'
## 設計思想（制約ベースを主・AIを補助）

FMS/管制側の標準パイプラインに沿う:

1. **目標姿勢** — 寄り付きターゲット（停止位置・方位・許容誤差）
2. **走行可能領域** — 地形/コストマップ → occupancy/cost → drivable（障害物膨張・進入禁止減算）
3. **経路候補** — Hybrid A*（運動学的・R_min保証・障害物考慮ヒューリスティック）/ grid A* / spline / dubins、寄り付きは切り返し0/1
4. **軌道最適化** — Elastic Band（中央寄せ・余裕確保）/ R_min保証 / クロソイド平滑 / **速度プロファイル v(s)**
5. **安全検証** — 車両包絡線・最小旋回半径・操舵・勾配・最小離隔を統合し、**配信可否＋不可理由**を返す
6. **配信** — 速度付き trajectory（CSV/GeoJSON エクスポート）

AIは「経路のブラックボックス生成」ではなく**補助**: 自然言語で工程をオーケストレーション
（コストマップ→領域→経路→寄り付き）し、パラメータ調整・状態確認を行う。最終安全判定はルール/幾何/物理で固定。

## モノレポ構成

```
fms-route-studio/
├── packages/planning_core/   # 共通プランニング基盤（純Python・本体ロジック / pip install -e 可）
│   └── planning_core/
│       ├── planners/   spline / dubins / grid_astar / hybrid_astar / elastic_band / curvature_limit
│       ├── analysis/   curvature / grade / velocity / safety / trajectory
│       ├── drivable/   costmap/ footprint.py / simulator(spotting) / scenarios.py
│       ├── fleet/      multi-vehicle: conflict / network / passing / sim（進行中）
│       ├── vehicle/    車両諸元 YAML（HD785/HD605/HM400/CD110R）
│       └── geometry/ io/ models/
├── services/api/             # FastAPI（API専用, :8077）。planning_core への薄いラッパ
│   └── app/routers/   layers / costmap / drivable / geo(経路生成・解析) / simulate / fleet / vehicles / projects / agent（+ /api 直下に plan/tiles）
├── apps/web/                 # React+TS+Vite+Zustand（:5173）OpenLayers native 6677 + three.js 3D
├── scripts/scenario_report.py
└── docs/                     # 要件定義書 / 設計書 / シナリオレポート（docs/README.md 参照）
```

## ドキュメント

| 文書 | 内容 |
|---|---|
| [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md) | 要件定義書（StRS/SLQDC・SysRS/F·P·I·S·R・トレーサビリティ） |
| [docs/DESIGN.md](docs/DESIGN.md) | 設計書（アーキテクチャ・アルゴリズム・API・データモデル・非機能） |
| [docs/SCENARIO_REPORT.md](docs/SCENARIO_REPORT.md) | 鉱山/土木シナリオの自動チューニングレポート |

要件定義は `unmanned-requirements` のメソドロジー（SLQDC / F·P·I·S·R / トレーサビリティ）に準拠し、ID は `FRS-` 接頭辞で採番、上位パッケージ AutonomousDumptruck（FMS 運航前工程 S-00）へトレースする。

## 主な機能

- **マップ**: LAS→コストマップ(生cost/DSM/RGB COG)、走行可能領域生成（閾値/形態学/平滑/穴埋め/最大領域/クリアランス＋人/AI編集）
- **経路**: 4アルゴリズム＋自動(A*)、最小旋回半径(R_min)保証、Elastic Band洗練、フットプリント包含、進入禁止(NoGoZone)回避
- **車両**: HD785/HD605/HM400/CD110R。寸法/運動学/速度/加速度/安全しきい値を画面で調整（可逆オーバーライド）
- **寄り付き**: 切り返し0/1・必須後進・道幅・コスト関数（距離/時間/後進/切返/コスト）・退出軌道・不可理由・再生
- **解析**: κ/dκ-ds/操舵/勾配/**標高**/速度プロファイルのチャート、安全検証（配信可否＋不可理由・寄り付き到達精度±0.5m/±5°）
- **高さ(Z)埋め込み**: 点群由来DSMから経路各点に標高を自動付与（生成時）＋保存済みルートへの後付け（`POST /api/elevation/sample`）。CSV `z_m` 列・GeoJSON 3D座標・3D表示に反映
- **3D**: LAS点群(RGB)/DSM地形メッシュ・道幅帯・経路/寄り付き
- **マルチ車両（進行中）**: 複数台のルート競合検出・待避/追い越し・フリート走行シミュレーション（`planning_core/fleet`, `/api/fleet`）
- **AIアシスタント**: マルチプロバイダ（Claude / Groq / OpenRouter / ローカルOllama）。tool-use で全工程を操作
- **永続化/IO**: プロジェクト保存（寄り付き詳細設定含む）・CSV/GeoJSONエクスポート（速度・標高z含む）

## 実行

前提: **Python 3.10+** / **Node 18+**。working CRS = **EPSG:6677**。
レイヤレジストリ等のデータは `~/.fms-route-studio/data` に保存（環境変数 `FRS_DATA_DIR` で変更可）。

### 1. セットアップ（初回のみ）

```bash
# Python: venv を作り、両パッケージを editable install（api は planning_core に依存）
python -m venv .venv && source .venv/bin/activate
pip install -e "packages/planning_core[dev]"
pip install -e "services/api[dev]"

# フロント
cd apps/web && npm install && cd -
```

### 2. 起動

```bash
# venv を有効化していれば PYTHONPATH 指定は不要（editable install 済みのため）
source .venv/bin/activate

# API（:8077）。AI を使うなら鍵を env で（下記「AI」参照）
uvicorn app.main:app --app-dir services/api --host 127.0.0.1 --port 8077

# 別ターミナルで — フロント（:5173, /api と /health を :8077 にプロキシ）
cd apps/web && npm run dev   # → http://localhost:5173
```

起動完了まで数秒〜十数秒。`curl -s localhost:8077/health` が通れば準備完了。
planning_core / app のコードを変更したら API を再起動（バックグラウンド起動時は `pkill -f "uvicorn app.main"` で停止してから再起動）。

> editable install を使わない場合は、各コマンドに `PYTHONPATH=packages/planning_core` を付けても動作する。

## テスト / ビルド

```bash
source .venv/bin/activate

# planning_core（純ロジック）
pytest packages/planning_core/tests -q
# api（planning_core と分けて実行：合同だと PROJ db が競合する）
pytest services/api/tests -q
# web（型チェック付きビルド＋ vitest）
cd apps/web && npm run build && npm run test && cd -
# 鉱山/土木シナリオのチューニングレポート生成 → docs/SCENARIO_REPORT.md
python scripts/scenario_report.py
```

## AI アシスタント（マルチプロバイダ）

サーバ起動シェルで env を1つ設定（鍵はチャットに貼らない）。自動検出順 = Anthropic → Groq → OpenRouter → `FRS_AI_BASE_URL`(Ollama等) → OpenAI。

```bash
export GROQ_API_KEY=gsk_...                 # 無料・推奨（llama-3.3-70b-versatile）
# or  export OPENROUTER_API_KEY=...  +  export FRS_AI_MODEL=...
# or  export FRS_AI_BASE_URL=http://localhost:11434/v1  export FRS_AI_MODEL=qwen2.5   # Ollama・鍵不要
# or  export ANTHROPIC_API_KEY=sk-...        # Claude（既定 claude-opus-4-8）
```

`GET /api/agent/status` の `available`/`provider`/`model` で確認。無料/小型モデルは多段ツール操作の信頼性が落ちる点に注意。
