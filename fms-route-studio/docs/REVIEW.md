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
- ~~**Fleet H2/H3**: サンプリングのみの衝突判定・中点のみの同方向判定で、2台の交差を見逃す恐れ~~ → ✅ **2026-07-05 対応済**（§7: SAT車体判定＋区間全体の方向判定）。
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

---

## 5. 第3回 大規模レビュー（2026-07-04）

対象: UX / 車両運動特性 / UI / コード / アーキテクチャ / テスト。
4視点（車両運動・BEアーキ・FE/UX・テスト）の独立レビューを統合し、疑わしい指摘はコードで裏取りした上で記載。

**対応状況（2026-07-04 実装完了・全11コミット）**: P1〜P5 を承認順に実施済み。
- ✅ K2 cusp実効マージン診断（min_cusp_margin_m＋UI警告） / K3 積載時decel（max_decel_loaded、fleetは保守側） / K4 skid近似のwarning/note
- ✅ U1 NumberField統一＋日本語化（Drivable×7/Fleet×4/Spotting×1。VehiclePanelは文字列state方式でバグ対象外と判明し維持） / U2 点群ロードbusy / U3 進捗ドット（寄り付き/排土/複数台）
- ✅ B1+B8 resolve_las_epsg一元化 / B3 stationary.py分離（spotting.py 1114→942行） / B5 points.binチャンク送出
- ✅ T-A agentActions/ioテスト / T-C hybrid決定性+狭所Uターン / T-D 負系（壊れLAS 422早期拒否ガード追加・未知vehicle 404・矛盾指定） / T-E NumberField純関数化+テスト
- ✅ P5 文書（DESIGN §2.7 運動学近似の既知の制約・§7 CRS単一ユーザー前提、HM400.yaml/velocity.py/models注記）
- 検証: planning_core 188 / api 86 / web 55 全通過。未対応で残るのは K1 v2（アーティキュレート厳密化）と B4（endpoint-marginコストのプロファイル）、B6/B7、T-F/T-G（Low）。

### 5.0 規模・実行時間スナップショット

| 領域 | 行数 | テスト数 | スイート実行時間の支配項 |
|---|---|---|---|
| planning_core | 7,013 | 182 | switchback_needs_reverse 8.2s / best_effort_minimizes_overhang 7.5s / rrt_star決定性 4.5s |
| services/api | 2,796 | 82 | spotting_containment_polygon 4.5s / containment_with_cost 2.0s |
| apps/web | 7,770 | 37 (vitest) | 全体 ~0.6s |

巨大ファイル: spotting.py 1114 / MapView.tsx 896 / agent.py 773 / ThreeView.tsx 567 / orchestrator.py 488。

### 5.1 車両運動特性（K系）

- **K1 [High/設計明文化]** HM400 はアーティキュレート式だが、全プランナ/軌道生成が剛体バイシクル近似
  （`bicycle_step`、δ=atan(L·κ) 単一ホイールベース）。オフトラッキング（後部ユニットの内輪差/外輪はみ出し）
  を持たないため、狭所K-turnの成立判定は矩形フットプリントの保守性に依存しており「証明」ではない
  （旋回包絡の誤差 ~5–15% 見込み）。→ v1 は「バイシクル近似」と HM400.yaml / DESIGN.md に明記し、
  v2 でタンデムバイシクル or スイープパス包絡ポリゴンを計画。
- **K2 [Med/安全・診断]** 据え切り禁止時、端点リードは `endpoint_margin_start/goal_m` で診断可能だが、
  **内部 cusp のマージンは `_adapt_margin` が狭所で 0 に縮退しても記録・警告されない**
  （spotting.py:154 で黙って挿入スキップ）。禁止フラグの約束が内部 cusp で静かに破れる。
  → 各 cusp の実効マージン（最小値）を結果に記録し、禁止時に 0 があれば UI 警告。
- **K3 [Med/安全]** fleet sim の予約距離 `v_max²/(2·decel)+gap`（sim.py:161）は単一 decel。
  積載時の実減速度は空車の半分以下になり得るため、mutex ゾーンが過小 → 進入後停止しきれない恐れ。
  → プロファイルに `max_decel_loaded` を追加するか、fleet sim は保守側（積載時）値を使用。
- **K4 [Med/ガード]** CD110R（スキッドステア、`can_turn_in_place: true`・暫定スペック）に対応する
  プランナプリミティブが無く、bicycle_step で計画される。safety.py は R_min/steer_rate を除外済みだが
  プランナ側は無防備。→ skid 車のプランナ投入時に明示エラー or 「将来対応」をドキュメント化。
- **K5 [Low/文書]** 下り坂速度上限（max_speed_downhill_*）は grades 供給時のみ有効。DSM 無しでは
  無制限になることを VehicleProfile に明記。
- **K6 [Low]** steer-rate 制約は bisection 化済みで正しいが、数値微分由来の dκ/ds はサンプリング
  ノイズで過制約になり得る。analytic curvature source（Dubins/RS 区分一定 κ）を優先する方針を明文化。
- 妥当性確認済み（指摘なし）: 横G上限 v=√(a_lat/κ)、δ=atan(L·κ)、cusp で v=0 強制、
  ギア別上限、停止距離式、cusp での片側差分ヘディング。

### 5.2 アーキテクチャ / バックエンド（B系）

- **B1 [High]** WGS84 ヒューリスティック（全点 |x|≤180 ∧ |y|≤90 → 4326 とみなす）が
  layers.py と costmap.py に重複実装。→ planning_core（io/las or crs）へ抽出し一元化。
- **B2 [Med]** `_working_epsg` がモジュールグローバル（プロセス全体で共有）。単一ユーザー・ローカル
  前提を README/DESIGN に明文化（済みなら参照）、将来マルチユーザー化するなら contextvars 化。
- **B3 [Med]** spotting.py 1114行。据え切り deny の候補再ランク＋直線化ループ（~90行, ev() 14回）、
  endpoint margin 群は `stationary_avoidance.py` 等へ抽出可能。テスト24本が資産としてあるので分割は安全。
- **B4 [Med]** `_insert_endpoint_margin` は端点あたり最大 ~21 回の Dubins 接続を試す。
  狭所で deny 時のみ実行されるが、ランキング上位 14 候補 × 両端で最悪 ~600 接続。
  プロファイル取ってから必要なら sQ ラダーの早期打ち切りを検討。
- **B5 [Med]** /points.bin は 16M 点で ~230MB を一括バイト列構築（メモリピーク）。
  → チャンク書き出し（StreamingResponse）に変更可能。
- **B6 [Low]** min_area_rect は凸包後 O(n²)（実質問題なし）、place_grid の辺距離計算が pure-Python
  O(n·m)（グリッド数千点で顕在化しうる）→ numpy 化。
- **B7 [Low]** agent.py に broad except が複数。復旧経路として意図的だが、ログ付与を推奨。
- **B8 [Low]** CRS 検出が upload 時（ヘッダ→meta 保存）と costmap 時（req.src_epsg→ヘッダ→度数
  ヒューリスティック）で非対称。B1 と同時に判定順序を共通関数へ。

### 5.3 FE / UX / UI（U系）

- **U1 [High/UX一貫性]** NumberField 未採用パネルが残存: DrivableAreaPanel に raw `type="number"` ×7
  （＋ラベル英語混在）、FleetPanel ×4、SpottingPanel ×1、VehiclePanel ×1（共通レンダラのため
  1箇所直せば全フィールドに効く）。パイル/ルート系で解決済みの「入力中クランプ・NaN」問題が
  これらのパネルには残っている。→ NumberField へ統一＋ラベル日本語化。
- **U2 [Med]** 大規模点群（800万〜1600万点）の /points.bin ロード〜GPU 転送中に busy 表示がない
  （数秒間無反応に見える）。→ 既存 busy オーバーレイに載せる＋点数セレクタに負荷ヒント。
- **U3 [Low]** Sidebar 進捗ドットが data/map/route のみ（Sidebar.tsx:48）。piles: !!pilePlan、
  spotting/fleet も追加すると工程の見通しが揃う。
- **U4 [Low]** キャプチャの empty-state（レイヤ無し3D等）ガード、AreaPanel/IOPanel の英語ラベル残り、
  projectState の `as never` キャスト（FE⑤ OpenAPI 型生成で根治）。
- **U5 [Low]** エリアを編集しても既存 pilePlan が残置され不整合（再計算までstale）。
  → エリア変更時に該当 plan をクリア or 「要再計算」バッジ。
- 誤指摘として棄却: 「ThreeView 点群のレイヤ切替時メモリリーク」→ buildPoints() 冒頭で
  disposeGroup(s.points) 実施済み（ThreeView.tsx:322）。

### 5.4 テスト（T系。数値は 5.0 参照）

- **T-A [High]** FE の agentActions.ts（エージェント action→store 反映、~90行）と io.ts
  （プロジェクト JSON 入出力・座標変換）が完全未テスト。ユーザー向け入出力なので回帰リスク大。
- **T-B [Med]** /api/agent/chat の疑似 E2E（プロバイダをモックし context+message→actions の型を検証）
  が無い。test_agent.py 17本はガード/パーサ中心。
- **T-C [Med]** hybrid_astar のテストが3本と希薄（grid_astar は13本）。同シード決定性・狭所 U-turn の
  成立/明示失敗テストを追加。
- **T-D [Med]** 負系不足: 不正 LAS（壊れたヘッダ）、サイズ超過、矛盾制約（req_switchback ∧ max_sb=0）、
  未知 vehicle_id。
- **T-E [Med]** NumberField 単体テスト（"-" 入力→blur で復元、範囲外→クランプ commit 等）。
- **T-F [Low]** tiles は正常系カバー済み（test_api.py:714）。OOB z/x/y → 404 の負系のみ追加余地。
  レビューエージェントの「tiles テストゼロ」「projects ロック不備」は誤指摘（RLock 実装済み）として棄却。
- **T-G [Low]** 緩い許容値の根拠コメント化（radius−0.8m、Z±1.0m、hybrid 1.3x 等）。

### 5.5 棄却した指摘（裏取りで否定）

| 指摘 | 棄却理由 |
|---|---|
| ThreeView 点群 dispose 漏れ | disposeGroup を再構築前に必ず呼ぶ実装を確認 |
| tiles ルータ テストゼロ | test_api.py:714 で PNG マジックまで検証済み |
| projects.json 並行破壊 | threading.RLock で全書き込み保護済み（並行テスト追加は Low） |
| 後退ギア上限が遷移で破られる | velocity.py が cusp で v=0 を強制、正しい（エージェント自身も撤回） |

### 5.6 実施順の提案（ユーザー承認待ち）

1. **P1 安全・小改修**: K2（cusp マージン診断＋UI警告）→ K3（fleet 制動距離の保守化）→ K4（skid ガード）
2. **P2 UX 一貫性**: U1（NumberField 統一＋日本語化）→ U2(点群ロード busy) → U3（進捗ドット）
3. **P3 コード健全性**: B1+B8（CRS 判定一元化）→ B3（spotting 分割）→ B5（points.bin ストリーミング）
4. **P4 テスト**: T-A（agentActions/io）→ T-C/T-D/T-E
5. **P5 文書・将来**: K1（アーティキュレート近似の明記と v2 計画）→ K5/K6、B2 明文化

---

## 6. 総括（プッシュ前 最終レビュー・2026-07-05）

本ブランチ（test_lane_auto_creation, terrain_segmentation 比 +43 コミット）の最終状態。

### 6.1 第3回レビュー以降の追加機能

- **寄り付き 切り返し候補の空間拡張**: 密格子の外側に粗シェル（横±2〜3ρ×距離7ρ×方位24°）
  ＋リング5.5ρ を追加（計~2,790姿勢）。旧候補域外ゾーンでの成立を判別テストで固定化。
- **LAS 座標系の後付け指定**: `PATCH /api/layers/{id}/epsg`。ヘッダ CRS 欠落の現場 LAS
  （las2ortho も override_srs 前提の運用）を、レイヤパネルから系指定で正しい位置へ。
  costmap/3D点群/オルソの全解決経路が resolve_las_epsg（メタ指定>ヘッダ>推定）で一貫。
- **LAS→オルソ自動生成（las2ortho 内蔵化）**: planning_core/ortho.py（RGB平均→Zグレー
  fallback、密度から解像度自動）＋取込時自動生成（既定ON）＋手動再生成ボタン。
- **計測ツール＋エリア面積常時表示**: 📏計測（距離2点〜/面積3点〜、Backspace/Esc）、
  エリアの地図ラベル（名前＋面積）・一覧・作図中プレビュー。
- 負系ガード: 壊れた LAS はアップロード時 422 で早期拒否。

### 6.2 品質ゲート（プッシュ時点）

| ゲート | 結果 |
|---|---|
| planning_core | 192 tests 全通過 |
| services/api | 88 tests 全通過 |
| apps/web (vitest) | 62 tests 全通過・tsc --noEmit クリーン |
| 機密情報スキャン | APIキー/パスワード/デバッグ残骸/巨大バイナリ（LAS/TIF）の混入なし |
| 既知の残 TODO | HD605.yaml の暫定スペック注記4件のみ（spec_status: estimated と整合） |

### 6.3 既知の制約・次の候補（実装済みの文書化は DESIGN §2.7 / §7）

- K1 v2: HM400 アーティキュレート厳密運動学（現状は保守的バイシクル近似）
- B4: 据え切り deny 時の endpoint-margin リトライコストのプロファイル
- Fleet H2/H3・S2・GAP G-01/02/03/05（§3 参照）
- merged.las のような現場ローカル座標 LAS は EPSG 指定では配置しきれない場合がある
  → 必要ならローカル→公共座標の平行移動（オフセット指定）機能を追加

---

## 7. 複数台シミュレーション 集中レビュー＋修正（2026-07-05）

対象: fleet/sim.py・conflict.py・passing.py・network.py・API・FE。コード精読＋実機再現（60°/20°交差・積載decel・駐機シナリオ）で裏取りし、同日に全件修正済み（5コミット）。

### 指摘と対応

| # | 指摘 | 対応 |
|---|---|---|
| H1 | 衝突判定が「中心間距離<半幅和」のみで**車長を無視**。60°交差の再現で10m級車体が重なっても collision=False/DEADLOCK と誤報告 | 向き付き矩形の分離軸(SAT)判定へ置換（外接円プレフィルタ付）。縦方向接触の単体テスト追加 |
| H2 | 同方向判定（Mutex不要条件）が共有区間の**中点1点 cos>0.3（±72°）**で、60°交差でも追従任せ→H1と複合し車体接触 | 区間全体7点の対応方向の**平均 cos>0.85（±30°）**へ厳格化。60°交差はMutex直列化・接触ゼロを回帰固定 |
| M1 | 浅い合流（20°）で相互を「前方車」と認識し両者停止＝**偽デッドロック** | 相互認識かつほぼ同方向（cos>0.5）のとき高優先が進み低優先が譲る。対向は方向ゲートで除外（単線ヘッドオンは従来どおり両者停止→待避所解消） |
| M2 | K3で積載decelを制動カーブまで使い、停止点の数百m手前から徐行になる副作用 | SimVehicle.reserve_decel を新設。予約距離のみ保守側=積載値、制動カーブは通常値。予約前倒し・所要不変をテスト固定 |
| M3 | 到達車が退場扱い（追従・衝突判定から除外）で、単線の奥に停まる運用では後続がすり抜け | 仕様を docstring に明文化＋ park_at_goal オプション（API/FEチェックボックス）で駐機扱いを選択可 |
| Low | record()のno-op if・未使用 _sample_le/_intervals_from_mask | 削除 |

### 問題なしと確認した点

conflict.py の競合検出（EDT前計算・連結成分分割・弧長逆引き）、Mutexの車体長膨張と解放、デッドロック検出（2秒停滞）、auto-passing の悪化巻き戻し、sequential モード、network.py。

### 残る既知の簡略化（文書化のみ・将来課題）

- 制動が v_stop クランプで**1ステップ無限減速**を許す（実車より楽観的な停止能力）
- 停止中でも先のゾーンを予約し続ける（優先度処理のためハードデッドロックにはならないが待ち時間が延び得る）
- デッドロック時の auto_bays 対象が「最接近ペア」でデッドロック当事者と一致しない場合がある

検証: planning_core 197 / api 88 / web 62 全通過。
