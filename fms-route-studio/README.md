# FMS Route Studio (SmartConstruction AI)

地図・コストマップ・走行可能領域・経路生成（静的/動的）・解析・安全検証・AIアシスタントを統合した、
建機向けルート設計プラットフォーム。`Map_Builder_Proto`（FMS Map Building Tool）の全面刷新版。

設計の全体像: [docs/REARCHITECTURE.md](../Map_Builder_Protoのコピー/docs/REARCHITECTURE.md)。

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
├── packages/planning_core/   # 共通プランニング基盤（純Python・本体ロジック）
│   ├── planners/   spline / dubins / grid_astar / hybrid_astar / elastic_band / curvature_limit
│   ├── analysis/   curvature / grade / velocity / safety / trajectory
│   ├── drivable/ costmap/ footprint.py / simulator(spotting) / vehicle / scenarios.py
├── services/api/             # FastAPI（API専用, :8077）  layers/costmap/drivable/plan/analyze/simulate/projects/agent
├── apps/web/                 # React+TS+Vite+Zustand（:5173）OpenLayers native 6677 + three.js 3D
├── docs/SCENARIO_REPORT.md   # 鉱山/土木シナリオの自動チューニングレポート
└── scripts/scenario_report.py
```

## 主な機能

- **マップ**: LAS→コストマップ(生cost/DSM/RGB COG)、走行可能領域生成（閾値/形態学/平滑/穴埋め/最大領域/クリアランス＋人/AI編集）
- **経路**: 4アルゴリズム＋自動(A*)、最小旋回半径(R_min)保証、Elastic Band洗練、フットプリント包含、進入禁止(NoGoZone)回避
- **車両**: HD785/HD605/HM400/CD110R。寸法/運動学/速度/加速度/安全しきい値を画面で調整（可逆オーバーライド）
- **寄り付き**: 切り返し0/1・必須後進・道幅・コスト関数（距離/時間/後進/切返/コスト）・退出軌道・不可理由・再生
- **解析**: κ/dκ-ds/操舵/勾配/速度プロファイルのチャート、安全検証（配信可否＋不可理由）
- **3D**: LAS点群(RGB)/DSM地形メッシュ・道幅帯・経路/寄り付き
- **AIアシスタント**: マルチプロバイダ（Claude / Groq / OpenRouter / ローカルOllama）。tool-use で全工程を操作
- **永続化/IO**: プロジェクト保存・CSV/GeoJSONエクスポート（速度プロファイル含む）

## 実行

Python venv は隣の `Map_Builder_Protoのコピー/.venv`。working CRS = **EPSG:6677**。

```bash
VENV=/path/to/Map_Builder_Protoのコピー/.venv/bin/python   # 実環境のパスに合わせる

# API（:8077）。AI を使うなら鍵を env で（下記「AI」参照）
PYTHONPATH=packages/planning_core nohup $VENV -m uvicorn app.main:app \
  --app-dir services/api --host 127.0.0.1 --port 8077 &

# フロント（:5173, /api と /health を :8077 にプロキシ）
cd apps/web && npm install && npm run dev
```

## テスト / ビルド

```bash
# planning_core（純ロジック）
PYTHONPATH=packages/planning_core $VENV -m pytest packages/planning_core/tests -q
# api（planning_core と分けて実行：合同だと PROJ db 競合）
PYTHONPATH=packages/planning_core $VENV -m pytest services/api/tests -q
# web
cd apps/web && npm run build && npm run test
# 鉱山/土木シナリオのチューニングレポート生成
PYTHONPATH=packages/planning_core $VENV scripts/scenario_report.py   # → docs/SCENARIO_REPORT.md
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
