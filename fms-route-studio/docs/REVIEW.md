# 実装・構造レビューと改善記録 — FMS Route Studio

> **本書のステータス**: 2026-07-01 初版 / **2026-07-03 第2回改善を反映**。バックエンド（planning_core / services/api）とフロント（apps/web）の実装レビュー結果、および段階的改善の対応状況を記録する。
> **総評**: 全体として堅実な設計。planning_core は純 Python を保ち（I/O・FastAPI 非依存を確認）、CRS 中央管理・COG 原子的書込・レジストリのロック・「安全検証はルール固定／AI は補助」の設計分離は意図が明快。以下は「立て直し」ではなく「堅牢化」の指摘。

関連: [要件定義書](REQUIREMENTS.md) / [設計書](DESIGN.md) / [突き合わせ](GAP_ANALYSIS.md)

---

## 1. 対応状況サマリ

| ID | 区分 | 重大度 | 指摘 | 状態 |
|---|---|---|---|---|
| S1 | BE/安全 | **High** | 安全ゲートが情報不足時に `passed=True`（fail-open） | ✅ 対応済 |
| B1 | BE/正当性 | **High** | `min_turning_radius` がカスプ非除外で後進経路の測定半径が誤る | ✅ 対応済 |
| B2 | BE/正当性 | Med | `fit_spline_curvature_limited` の `best=None` アンパッククラッシュ | ✅ 対応済 |
| B5 | BE/観測性 | Med | DSM 失敗時の bare except（想定外バグを握り潰す） | ✅ 対応済（analyze/simulate 狭め） |
| FE① | FE/正当性 | **High** | Undo スナップショットが shallow copy（履歴汚染リスク） | ✅ 対応済 |
| FE④ | FE/構造 | **High** | API クライアント重複・abort 無し・r.ok 抜け | ✅ 対応済 |
| FE⑦ | FE/UX | Med | busy/error ハンドリングがパネルごとにバラバラ | ✅ 共通化ヘルパ導入・一部適用 |
| FE⑧ | FE/性能 | Med | three.js/OpenLayers の dispose 不完全（GPU/worker リーク） | ✅ 対応済 |
| A1/A2 | BE/構造 | High | 太いルーター（planning/simulate に業務ロジック） | ✅ **対応済 (07-03)** `planning_core.orchestrator` 抽出 |
| FE② | FE/構造 | High | god-store（80+ フィールド単一ストア） | ✅ **対応済 (07-03)** 5スライスへ分割（外部API不変） |
| FE③ | FE/構造 | High | MapView.tsx 1147行・巨大 effect（deps23） | ✅ **対応済 (07-03)** styles/geom分離＋fleet再生を専用ソース化 |
| P1/P4 | BE/性能 | High | `/plan` でラスタ2-4回読込・レジストリ6回パース | ✅ **対応済 (07-02)** リクエスト内キャッシュ＋meta一回解決 |
| P3/P5/P6 | BE/性能 | High/Med | RRT* O(n²)、fleet 自動待避非収束、conflict 二重ループ | ✅ **対応済 (07-03)** numpyベクトル化/改善時のみ採用＋巻き戻し/EDT表引き |
| B3 | BE/正当性 | Med | ラスタ co-registration を shape のみで判定（transform/CRS 未確認） | ✅ **対応済 (07-03)** `same_grid`（plan=422 / spotting=安全側デグレード） |
| D5 | BE/API | Med | 入力の点数上限無し | ✅ **対応済 (07-03)** waypoints/points/routes に max_length |
| D1/D3 | BE/API | Med | ページネーション無し・機械可読エラーコード無し | ⏳ 未（ローカル単一ユーザー前提では低優先） |
| FE⑤ | FE/型 | Med | FE/BE 型ドリフト（手書き型・`as` 多用） | ⏳ 未（OpenAPI 生成） |
| FE⑥ | FE/品質 | Med | ESLint 未設定（`exhaustive-deps` 無効化が多数） | ⏳ 未（要 npm install） |
| T1/T2/T3, FE⑩ | 両/テスト | High/Med | store/agent/verify_safety・純粋関数のテスト不足 | ⏳ 一部（safety/curvature/undo/orchestrator/projectState は追加済） |

### 第2回改善（2026-07-02〜03）で追加した機能・修正

| 項目 | 内容 |
|---|---|
| **高さ(Z)埋め込み**（ユーザー要望） | `TrajPoint.z` 追加。点群由来 DSM から `elevation_and_grade()` で z＋勾配を一括サンプル（grade=100·dz/ds と厳密整合）。/plan・/analyze・寄り付きで自動埋め込み、`POST /api/elevation/sample` で保存済みルートへ後付け。CSV `z_m` 列・GeoJSON 3D座標(RFC 7946)・3D表示は実測z優先・解析パネルに標高チャート・RoutePanel「高さ(Z)を埋め込む」ボタン |
| **プロジェクト保存の欠落修正**（High） | 寄り付き詳細設定（コスト重み/退出/手動切り返し点/切り返しゾーン/マージン）が保存されず再読込で消えていた → project v3 で保存/復元。fleetBays は保存経路に存在する id のみ復元＋一括置換（孤児/残留を排除）、保存経路削除時に紐づく待避所も削除 |
| **可視化/入力**（High/Med） | 封じ込めエリア（走行を収めるエリア）を選択時に青破線でハイライト（適用中が見える）。方位入力は [0,360) ラップ、コスト重みは負値クランプ |
| **寄り付き一発到達精度の合否**（GAP G-06 / ADT P-008） | `approach_error_deg`（方位誤差）を新設し、safety に「到達精度（位置±0.5m/方位±5°）」チェックを追加（既定しきい値変更可）。メトリクス表示に方位誤差併記 |
| **UXポリッシュ** | スケールバー＋カーソル座標（作業CRSメートル）を地図に常設。解析メトリクスのラベルを日本語へ統一 |

---

## 1.5 検証状況（第2回改善後）

- planning_core: **169 tests** / api: **80 tests** / web: **30 tests + tsc + build** — すべて緑
- 各改善は1件ずつコミット（`git log` 参照）。API レスポンス形式・FE の import 経路は後方互換

---

## 2. 対応済みの改善（詳細）

### S1: 安全ゲートを fail-closed 化（最重要）
- **問題**: `verify_safety` の `passed = all(c.ok for c in checks if c.applicable)`。車両未指定・走行可能領域なしのとき、適用チェックが実質 0 件でも `passed=True` を返し、AI も「安全: 合格」と報告していた（設計思想「安全判定はルール固定」に反する）。
- **対応**: `footprint_evaluated` 引数を追加し、走行可能領域 mask 未供給時は包絡線チェックを非適用に。**適用チェックが 1 件も無ければ `passed=False`＋不可理由**を返す fail-closed に変更。呼び出し側（plan/spotting）で `footprint_evaluated=dmask is not None` を渡す。
- **ファイル**: `analysis/safety.py`, `routers/planning.py`, `routers/simulate.py`。テスト `test_safety.py` に2件追加。

### B1: カスプ考慮の `min_turning_radius`
- **問題**: 切り返し点では3点外接円の曲率が見かけ上∞に化けるが、`min_turning_radius()` はカスプを除外せず max|κ| を取っていた。後進対応（hybrid/reeds_shepp/rrt*）で `measured_min_radius_m` が誤り、UI/AI と R_min 判定に伝播。
- **対応**: `cusp_mask` で反転点を除外（`build_trajectory.min_radius_m` と整合）。`test_curvature.py` に回帰テスト追加。
- **ファイル**: `analysis/curvature.py`。

### B2: spline の `best=None` ガード
- **対応**: 候補が 1 つも得られない異常時に、不可解な `TypeError` ではなく明示的 `ValueError` を投げる。`planners/spline.py`。

### B5: DSM 失敗時の例外を狭める
- **対応**: `analyze` / spotting の grade 計算の `except Exception` を `(RasterioIOError, ValueError, IndexError)` に狭め、想定内（IO/CRS/範囲外）のみ握って想定外バグは伝播。`routers/planning.py`, `routers/simulate.py`。

### FE①: Undo スナップショットの深いコピー
- **問題**: `snap()` が編集対象の参照をそのまま保持。将来 in-place 編集が入ると履歴が汚染される地雷。
- **対応**: `structuredClone` で深く複製。`commandBus.test.ts` に「スナップショット隔離」「in-place 変異が履歴を汚染しない」「max-50 キャップ＋redo クリア」の回帰テストを追加。
- **ファイル**: `store/useStore.ts`。

### FE④: API クライアント統合 + AbortController + detail 整形
- **対応**: `jget/jpost/jpatch/jput` を単一 `request()` に統合。`ApiError`（status/detail 保持）を導入し FastAPI `{detail}` を人間可読メッセージへ整形。`r.ok` 抜け（delete 系）を修正。`layerPoints`/`dsmGrid` に `AbortSignal` を通し、ThreeView のレイヤ切替/アンマウントで fetch を abort。
- **ファイル**: `api/client.ts`, `map/ThreeView.tsx`。

### FE⑦: busy/error 共通化
- **対応**: `ui/busy.ts` に `runBusy(label, fn, opts)`（二重起動ガード＋例外整形＋finally で busy 解除）と `errMessage(e)` を新設。busy 制御が無かった `startBranch` に適用、`generate` の catch を `errMessage` 化。他パネルも順次移行可能。
- **ファイル**: `ui/busy.ts`, `panels/RoutePanel.tsx`。

### FE⑧: three.js / OpenLayers の dispose 完全化
- **対応**: ThreeView アンマウント時に terrain/points/content/edit の4グループを `disposeGroup`。MapView はラスタ除去時・アンマウント時に GeoTIFF source（worker/decoder）と WebGLTileLayer を `dispose`。2D⇔3D 往復での GPU/worker リークを解消。
- **ファイル**: `map/ThreeView.tsx`, `map/MapView.tsx`。

---

## 3. 未対応（優先度つき・今後の候補）

### 正当性・安全（Med、フリート pre-production 前に）
- **Fleet H2/H3**: サンプリングのみの衝突判定・中点のみの同方向判定で、2台の交差を見逃す恐れ。フリートを試作から出す前に対応。
- **S2**: spotting の `advisory_kinds=("kappa_rate",)` を無条件付与 → 実測速度に応じてゲートすべき。

### API 衛生（Med・ローカル単一ユーザー前提では低優先）
- **D1**: list 系（layers/projects/vehicles）にページネーション・要約返却。
- **D3**: 機械可読エラーコード `{code,message,detail}`、override キーの allowlist。
- 重い計算エンドポイントの並行数制御 or ジョブ化（単一ユーザーローカル前提の明文化）。

### 型・品質・テスト
- **FE⑤**: OpenAPI から FE 型を自動生成し、手書き型と `as`/`as never` を撲滅。
- **FE⑥**: ESLint + react-hooks 導入（`exhaustive-deps` 無効化の点検）。※ 依存追加（npm install）が必要。
- **T2/T3**: agent の Anthropic 経路 + `MAX_GENERATE_CALLS`、`verify_safety` の steer/footprint 分岐のテスト拡充。
- **T4**: PROJ-db 分離実行契約を root conftest で強制（現状は runbook 頼み）。
- **C1/C2**: `FRS_*`/AI 系 env を単一 pydantic-settings へ集約。`_working_epsg` プロセスグローバルの単一ユーザー制約を明文化 or リクエストスコープ化。

### GAP 分析由来（機能・要件レベル。[GAP_ANALYSIS.md](GAP_ANALYSIS.md) 参照）
- **G-01/G-02 [高]**: エリア種別体系（無人/立入制限/積込/排土/駐機）＋完全性検知、エリア/区間別制限車速（要件化から）。
- **G-03/G-05**: データセット全体検証（到達不能/デッドロック）、承認ワークフロー/バージョン管理。
- ※ G-06（寄り付き到達精度の合否）は 2026-07-03 対応済。

---

## 4. 検証

対応済み改善はすべて既存テストスイートで回帰なしを確認:
- planning_core: `pytest packages/planning_core/tests`（安全・曲率の新規テスト含め全通過）
- api: `pytest services/api/tests`（全通過）
- web: `npx tsc --noEmit` クリーン、`npx vitest run`（commandBus の Undo 隔離/上限テスト含め全通過）
