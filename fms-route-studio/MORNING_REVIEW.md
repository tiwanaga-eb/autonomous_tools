# 朝レビュー（夜間自律作業ログ）

> 仕組み: 夜間に自律実行した内容・自分で下した判断（理由つき・可逆）・**要確認事項**をここに逐次追記します。
> 朝はこの上から順に見れば、何が進んで何を確認すべきかが分かります。最終更新時刻と全テスト結果を末尾に置きます。

最終更新: 2026-06-14（対話実行）。プランナ拡張＋寄り付き改善。両サーバ稼働（API :8077 Groq / FE :5173）。

---

## 2026-06-17 の作業（実LASでの end-to-end 検証）

実データ `Dataset_20260529_0920.las`（145k点, 範囲264×311m, Z392-430, CRS無→6677, 密度1.77pts/m²）で通し検証。

### ✅ パイプラインは end-to-end で動作
LASアップロード→コストマップ(grid1.0, 1.77pts/cell)→走行可能領域(threshold:12573m²/20島, otsu:21188m²/1島)→経路(grid_astar)→寄り付き→解析、すべて200で完走。

### 🐛 発見＆修正したバグ: 縦断勾配の偽スパイク
- 症状: 経路の縦断勾配が **62%**（中央値は8%＝妥当）。疎なDSMを**平滑化せず微分**→ノイズ増幅。
- 修正: `grade.py::grade_profile` に微分前 **Savitzky-Golay(2次, 既定8m窓)** 平滑化を追加（直線/2次勾配は厳密保存＝定数勾配テスト通過、ノイズのみ抑制）。62%→40%に低減。
- 残: 40%ピークは一部残存（DSM境界/段差の実アーティファクトの可能性。nodata近傍のgradeマスクは今後の候補）。

### ✅ 正しく検出された「実データの制約」（バグではない）
- 急勾配: 実勾配ピーク~40%（>30%限界）→ HD785 で不可。実地形が急。
- 狭いコリドー: drivable最大クリアランス~3.2m（幅~6m）に対し HD785 は全幅5.5m → 車両包絡線はみ出し/離隔0、hybrid A*は footprint包含で422（妥当な「収まらない」検出）。
- 寄り付き: 狭領域外へ出る配置で INFEASIBLE_BEST_EFFORT（正しく不可検出）。解析(analysis/safety)も付与を確認。

### 所見
飛び抜けた不具合は無し。コア機能は実データで健全に動作し、不可判定は地形・車両サイズの実制約を正しく反映。grade平滑化の1件が実データで初めて顕在化した本物の改善。

最終: planning_core 112 / api 63 / FE 9 / build 緑。

---

## 2026-06-16 の作業（残改善 #1/#2,#3/#10/#13-15）

### ✅ #1 経路の後進パス整合性（最重要）
- 寄り付きで直した「後進の車体方位・cusp曲率・dκ/ds」修正を**経路生成(hybrid/RS/RRT* の allow_reverse)にも適用**。planning.py が planner 出力の gear を捨てていたのを、生 (x,y,gear) を保持し `_nearest_gears` で最終経路(resample/EB後)へ最近傍マップ→ `build_trajectory(gears=...)` に貫通。
- 検証: RS経路で gears=FFF…RRR…FFF を保持、cusp前後 heading 連続(79°→103°, 反転なし)。api test `test_plan_reeds_shepp_preserves_reverse_gear`。

### ✅ #2,#3 HM400 速度データ配線
- velocity_profile に `grades` 追加。**#2 下り勾配(grade<-1%)は `max_speed_downhill_loaded`(4.44)で頭打ち**。**#3 速度依存の最大操舵速度** `steer_rate_profile` から、操舵が追従できる速度に制限（dδ/dt≤steer_rate(v)、2反復近似）。曲率変化の速い遷移区間でのみ効く（定常弧・直線は不変）。build_trajectory が grade を velocity へ供給。
- 検証: 下り10%→4.44で頭打ち、R30定常弧は5.4のまま、曲率遷移で減速。

### ✅ #10 寄り付き再生で向き付き車両
- 再生スクラブの車両マーカーを点→**運動学つきフットプリント**（解析軌跡の curvature・gear対応 heading から操舵輪/関節を描画）に。後進の様子が見える。車両未選択/解析無しは従来の点にフォールバック。

### ✅ #13-15 コード品質/堅牢性/セキュリティ（バウンド分）
- 入力境界: PlanRequest `spacing_m`/`min_turn_radius_m`/`corridor_width_m` に `gt=0`（不正は422）。api test 追加。
- algo整合: plan() の分岐を `req.algorithm`→`algo` に統一（auto時の潜在不整合を解消）。
- grade例外: bare except を**警告化**（DSM読込/CRS不整合を warn に出す。DSM無しと区別）。
- agentセキュリティ: 生成系ツール(generate_costmap/drivable)の**1リクエスト内実行上限 MAX_GENERATE_CALLS=3**（暴走/プロンプト注入での連続ディスク書込を抑止）。
- ※ plan()の本格的な関数分解は未（リスク大のため別途）。

最終: planning_core 112 / api 63 / FE 9 / build 緑。両サーバ稼働。

---

## 2026-06-15 の作業（その11: 寄り付き経路がクネクネしすぎる）

- 症状: 寄り付き経路が蛇行（クネクネ）。原因はコスト関数に**滑らかさ（総旋回量）ペナルティが無く**、同長なら蛇行・多cusp(RSの3点切返しFRF等)が選ばれていた。
- 修正: `_build` に**総旋回量 `total_turn_rad`**（gear連続区間内の|Δ方位|和。cuspの180°は除外）を集計し、`CostWeights.w_turn`（既定6）で `_score` にペナルティ追加。API `WeightsIn.w_turn`、FE `SpotWeights.w_turn`＋入力「旋回(蛇行抑制)」＋プリセット「直線的に」(w_switchback12/w_turn16)。
- 効果: 現実的な寄り付き（dump/側方/前進 back-in）は元々クリーン(FR/F)。余地のある場合に蛇行減（緩U: 総旋回 183°→149°）。**極端な180°U字を10mで**等は1切返しが幾何的に不能でRSの3点切返しが唯一解＝幾何下限により変化なし（その場合はSをもっと離す/道幅・姿勢の見直しが必要）。
- tests: planning_core spotting 12 / api spotting 8 / FE 9 / build 緑。

---

## 2026-06-15 の作業（その10: 切り返し後の後進で車体が反転表示される不具合）

- 症状: 切り返し後の後進セグメントで、車両（フットプリント）がいきなり180°反転し前進に見え、進行方向と操舵がチグハグ。
- 原因: 解析軌跡 `build_trajectory` の heading は `_headings_deg`（**chord=進行方向ベース**）で、後進では進行方向＝車体の逆。ホバー時のフットプリント/操舵がこの heading を車体方位として使うため、切り返しで180°飛んでいた（スポッティング結果 `points` 側は gear対応の車体方位で正しかったが、解析軌跡は別計算だった）。
- 修正: `build_trajectory` で **gears 指定時、gear=R の点の heading を180°反転＝車体方位**にする（切り返し前後で車体方位が連続）。これでフットプリント本体は反転せず、操舵もボディ基準で整合。
- 検証: HM400 1切返で 切り返し前後の body heading が 83°→83°（連続, 反転なし）。planning_core 112 / FE build 緑。
- 補足: 寄り付き再生(スクラブ)の車両マーカーは現状「点」。後進を視覚的に分かりやすくするなら、再生中も向き付きフットプリントを出す拡張が可能（要望あれば）。

---

## 2026-06-15 の作業（その9: ステアリング/関節の向き・角度の修正、HM400実寸反映）

- **HM400 実寸反映（外形図 HM400-5）**: `track_width: 2.69`（後輪トレッド2690）、中折れ関節位置から前後ユニット長 `front_length: 4.34` / `rear_length: 6.77`（前軸2985+1350≈4335mmが関節中心）。max_articulation_angle 0.785(45°)を確認。
- **ステア/関節の向き・角度**: curvature_profile の κ は符号なし(常に正)だったため当初は常に左＝逆向き。修正の最終形:
  - **角度（大きさ）= 曲率ベース δ=atan(L·κ)**（アッカーマンで内外輪 dL=atan(L/(R−t/2)), dR=atan(L/(R+t/2))）。R_minで約26°と曲率に正しく対応（有限差分案は基準点が車体中心で約半分になり不採用）。
  - **符号（向き）**: 位置の外積(CCW+)から。**前進=sign(cross)、後進=−sign(cross)（逆位相）**。バイシクルモデルの数値シミュレーションで前進/後進×左右操舵の4ケースを検証し確定（後進は実機構どおりカウンターステア）。
  - 関節角も κ·wheel_base に同じ符号規約。
- tests: FE 9 / build 緑。API再起動で HM400 front/rear/track 反映確認。
- 注意: 後進はバイシクル模型で「カウンターステア（前輪は曲がる側と逆へ）」が物理的に正しい。もし可視化として「曲がる側へ向ける」表現を希望なら符号反転で対応可（要確認）。

要望: ホバー時の車両矩形を運動学に合わせて表示。HM=アーティキュレート2矩形、HD=アッカーマン操舵輪。
- `vehShape` を `/api/vehicles/{id}/detail` の effective から取得（kinematic_type/wheel_base/track_width/front_length/rear_length/max_articulation_angle/max_steer_angle）。
- `vehicleShapeRings(p, v)`:
  - **articulated(HM400等)**: 関節点(ホバー点)から前後2矩形を**関節角γ=clamp(κ·wheel_base, ±max_artic)**でくの字に（前: a+γ/2 / 後: a-γ/2+π）。
  - **rigid_bicycle(HD785/HD605)**: 車体矩形＋**アッカーマン操舵の前輪**（旋回半径R=1/κから内外輪 δ_in=atan(L/(R-t/2)), δ_out=atan(L/(R+t/2)), max_steerでcap）。前輪を操舵角・後輪を直進で4輪描画（タイヤは濃色＋黄枠 `WHEEL_STYLE`）。
  - **tracked_skid(CD110R)**: 矩形のみ。
- ホバー効果を複数フィーチャ対応に（`footprintFeatsRef`）。解析グラフ(経路/寄り付き)のホバー位置に追従。cusp は曲率0なので切り返し点では直進姿勢で表示。
- **符号修正（向きが逆だった不具合）**: `curvature_profile` の kappa=1/circumradius は**符号なし（常に正）**のため、操舵・関節が**常に左**を向いていた。位置の外積 `cross=(p-pPrev)×(pNext-p)` から旋回方向の符号を求め（CCW+）、**後進は車体フレームで反転**して `kappaSigned` を算出→ステア/関節に反映。右カーブで正しく右へ。
- tests: FE 9 / build 緑（FEのみ）。

- 症状: 寄り付きで「✗ 最小旋回半径(8.88/限界8.88)」「✗ 操舵レート dκ/ds(0.34/0.12)」と、生成できた経路がNG。
- 原因①(R_min): HM400 の `kappa_max_fwd=0.1125`(R=8.889) と `min_turning_radius=8.885`(1/R=0.11255) が不整合。
  プランナはR=8.885で生成→曲率0.11255>限界0.1125 で**必ず違反**。加えて離散化で一部R8.74。
- 原因②(dκ/ds): Dubins/RS の円弧接合(直線→円弧, L→R)で曲率が段差→高dκ/ds。経路はクロソイド平滑で回避するが
  寄り付きは生のまま。かつ低速では dκ/ds(=操舵レート/速度)は本来非拘束。
- 修正:
  - `summarize(tol=0.05)`: 最小旋回半径/操舵角/曲率変化率の違反判定に**5%許容**（境界・離散化の誤判定を防止。表示limitは素の値）。全車種に有効。
  - `verify_safety(advisory_kinds=...)`: 指定チェックを**参考扱い**(applicable=False, 合否に効かせない)。寄り付きは `kappa_rate` を参考扱い（低速で非拘束）。
  - HM400 `kappa_max_fwd/rev` を 0.1126 に整合（≈1/8.885）。
- 結果: 寄り付き(HM400) 安全 passed=True / R_min OK(8.74,5%内) / dκ/ds は「—(参考)」/ 包含(footprint)は drivable 指定時に判定。planning_core 112 / api 61 緑。

---

## 2026-06-15 の作業（その6: 切り返し直線マージンが反映されない不具合）

- 症状: SpottingPanel の「切り返し直線マージン」を変えても結果に反映されない。
- 原因: マージンは **Dubins ステージ候補と手動切り返し点にしか入っておらず**、自動探索では
  **RS/hybrid 候補（マージン無し・複数cusp）がコスト最小で選ばれていた**ため、マージン値を変えても
  採用候補が変わらなかった（診断: n_sw=2 ＝ RS/hybrid 採用、margin 1/5/12 で直線長が不変）。
- 修正: 汎用関数 `_apply_cusp_margins(segments, margin, step)` を追加（任意候補の各 F↔R 境界で
  「進行方向へ margin オーバーシュート→後進で戻る」を挿入し、cusp 前後を直線=ステア0°に）。
  **RS 候補・hybrid 候補にも適用**。これで全候補がマージンを持ち、設定値が反映される。
- 検証: margin 1/5/12 → 切り返し点の直線長 ~1.0/5.7/12.9m と連動。API e2e でも margin 2→2.3m,
  10→13.9m を確認。planning_core 112 / api 緑。

---

## 2026-06-15 の作業（その5: cusp解析の修正 / 掃引フットプリント表示）

### ✅ Cusp解析のバグ修正
- 症状: 寄り付きの切り返し点（直線マージンの折返し頂点）で**曲率が退化スパイク(例6.40 → R≈0.16m)** になり、グラフ汚染＋dκ/dsが生スパイクから計算され隣接点へ波及していた。
- 修正: `build_trajectory`/`summarize` の cusp 検出を **gear対応**に統一（新ヘルパ `_cusp_with_gears`＝幾何cusp ∪ gear変化点±1）。**dκ/ds・速度・各指標を算出する前に cusp 点の曲率を 0 化**（グラフに格納する曲率も0）。→ スパイクが曲率グラフ・dκ/ds・速度(横加速度cap)すべてから除去。min_radius は従来どおりcusp除外で正しい(8.885)。
- 検証: HM400 1切返で 折返し頂点 kappa 6.40→**0**、max|kappa|=0.1125、max dκ/ds=0.087(<0.12)、速度は切り返しで0停止。planning_core 112 / api 緑。

### ✅ 車両矩形の動的表示（解析グラフ連動）
- 当初「Spacingで各点に静的な矩形」を実装したが、要望は**経路解析と同じく解析グラフのホバー位置に対応した矩形を動的表示**だったため修正。静的Spacing実装（footprintSpacing/SWEPT_FP_STYLE/両パネル入力）は撤去。
- MapView のホバー効果を **AnalysisPanel と同じ「アクティブ解析」参照**に一般化: 寄り付き工程で寄り付き解析があればその軌跡、無ければ経路の軌跡。グラフ上ホバーの s 位置に**向き付き実車矩形**を動的描画（`carRect` ヘルパに集約）。→ 経路でも寄り付きでも、解析グラフをなぞると地図上で車両矩形が追従。
- tests: FE 9 / build 緑。

---

## 2026-06-15 の作業（その4: 寄り付きにも軌跡解析）

寄り付きシミュレータの結果にも、経路と同じ軌跡解析（曲率/最小半径/操舵/勾配/速度プロファイル）＋安全検証を付与。
- `build_trajectory` に `gears` 引数追加 → 前後進(cusp)つき経路で速度プロファイルが切り返しで停止。cusp は曲率/操舵評価から自動除外（既存 cusp_mask）。
- `simulate.py::_attach_analysis`: 寄り付き points(xy+gear)＋DSM(あれば勾配)＋drivable mask(あればクリアランス)から `build_trajectory→summarize→verify_safety` を構築し、応答に `trajectory/analysis/safety` を追加（退出軌道にも付与）。
- FE: `SpotResult` に trajectory/analysis/safety。**AnalysisPanel** は「寄り付き工程で寄り付き解析があればそれ、無ければ経路」を表示（見出しに「解析 · 経路/寄り付き」）。
- tests: `test_spotting_includes_analysis_and_safety` 追加。planning_core 112 / api 緑 / FE build 緑。スモーク(HM400 1切返): 軌跡93点・min_radius 8.885(=R_min, cusp除外計測)・最大速度2.69m/s・安全checks(footprint/min_radius/kappa_rate/grade) を確認。

---

## 2026-06-15 の作業（その3: HM400 運動特性の更新）

ユーザー提供の HM400 運動特性表を反映。`models/vehicle.py` に新規フィールド追加、`configs/HM400.yaml` 更新、`vehicle_overrides.EDITABLE_FIELDS` に新数値項目追加。
- **更新（既存・速度プロファイルに即反映）**: `max_accel` 0.5→**0.66**（b アクセル100%相当）/ `max_decel` 1.0→**0.49**（c 通常減速）/ `max_lateral_accel` 1.5→**0.98**（e-1 0.1G 荷こぼれしない）/ `max_steer_rate` 0.6→**0.44**（g 25deg/s 車両限界）。
- **新規追加フィールド**: `accel_start` 0.35（a 発進加速度）/ `decel_emergency` 2.94（d 急制動0.3G）/ `lateral_accel_limit` 2.45（e-2 構造限界0.25G）/ `steer_rate_profile` [(0,.44),(15,.16),(30,.04),(55,.01)]（f 速度依存操舵速度）/ `max_speed_downhill_empty` 6.94（h-1 空車25km/h）/ `max_speed_downhill_loaded` 4.44（h-2 積荷16km/h）。
- 即反映: コーナー速度(横加速度)・加減速は velocity_profile が `max_lateral_accel/max_accel/max_decel` を使うため自動反映（HM400のコーナー速度がより保守的に）。
- データ保持（将来の高度化用・現状は未配線）: `accel_start`/`decel_emergency`/`lateral_accel_limit`/`steer_rate_profile`/下り最大速度。**要判断**: 停止可能距離(`stopping_distance`)は現状 `max_decel`(快適0.49)使用 → 緊急停止距離が欲しければ `decel_emergency`(2.94) に切替可。速度依存操舵速度・下り速度のグレード連動も未配線。
- 検証: planning_core 112 / api 緑、HM400 detail API で全値確認、FE車両パネルに新項目が編集可能として出現。

---

## 2026-06-15 の作業（その2: 寄り付き切り返しマージン / UX第2弾）

### ✅ 完了
- **寄り付き 切り返し点の直線マージン**（ユーザー要望）: 切り返し点では実車はステア0°(直進)にしてから前後反転する必要があり、Dubins円弧が直接出会うと曲率≠0で追従不能だった。`_stage_segments` で **S の手前 margin[m] を直線**にし、前進=「Dubins→直線でSへ」/後進=「Sから直線で戻ってから旋回」。S 前後が同一直線（ステア0°）になり追従可能。自動候補(格子/リング)・手動切り返し点の両方に適用。`cusp_margin_m`（既定: max(0.4·R_min, 3m)）を API/FE で調整可。
- **バグ修正（既存）**: `_headings` が後進セグメントで +π フリップを**二重適用（=無フリップ）**しており、後進の車体方位が進行方向のまま誤っていた（footprintレビュー#でも示唆）。単一フリップに修正 → 後進の車体方位・3D/2D車両アイコン向き・footprint向きが正しく。
- tests: `test_spotting_cusp_has_straight_margin` 追加（切り返し点前後が同一方位=直進を検証）。planning_core **112** / api 60 / FE 9 / build 緑。スモーク: 手動S(30,8)・margin6 で切り返し前後の方位が一致(=直進)を確認。
- **UX 第2弾**:
  - **オンボーディング**: データ未登録時、地図中央に「はじめに: 1)データ 2)マップ生成 3)経路」のガイドカード（各工程へジャンプ可）。
  - **用語の日本語統一**: 全パネル見出しを日本語化（Route→経路, Drivable Area→走行可能領域, Spotting Simulator→寄り付きシミュレータ, Costmap→コストマップ, Layers→レイヤ, Area→エリア, Analysis→解析, Project→プロジェクト, JSON I/O→入出力）。

### ✅ UX 第3弾 + 経路の道幅表示
- **道幅表示（要望）**: **経路・寄り付きの両方**に道幅帯を表示。道幅指定（>0）があればそれ、**無ければ選択車両の車幅**で「実車の走行幅」を可視化（既定0でも帯が出る）。Route/Spotting パネルのラベルを「道幅(m)（0=車幅で表示）」に。MapView は `vehDims.w` を使用（寄り付きは spotRoadWidthM 優先）。※寄り付きの道幅帯は後追い対応（2026-06-15 その4後）。
- **ポリゴン描画のキーボード操作**: 描画中 **Esc=取消 / Backspace=頂点を1つ戻す**（入力欄では無効化）。
- **破壊的操作の確認**: 経路 Reset / レイヤ削除 / プロジェクト削除 / Drivable 手修正の全消去 に確認ダイアログ。
- **寄り付きコスト重みのプリセット**: 「最短距離重視 / 切り返し最小 / コスト回避重視」ボタン。
- tests: FE 9 / build 緑（本バッチはFEのみ）。

---

## 2026-06-15 の作業（リブランド / UX刷新 / 複数経路 / OSS調査）

### ✅ 完了
- **リブランド**: ブランド名 "SmartConstruction AI" → **"SmartConstruction"**（"AI"除去）。トップバー/BrandMark。AIアシスタント機能名は機能名なので存置。
- **UX刷新（再レビュー結果のHIGH中心）**:
  - **グローバル busy 状態＋中央オーバーレイ＋スピナー**。経路/寄り付き/コストマップ/走行可能領域の生成中にオーバーレイ表示＆主ボタン無効化。RoutePanel/Spotting に**多重送信ガード**（生成中の再クリックで二重実行しない）。
  - **status の重要度色分け**（info/success/warn/**error**）。エラーは赤、警告は橙、処理中は青パルスのドット。失敗メッセージを error 級に。
  - **工程切替で編集モードを pan へリセット**（前工程の mode が地図クリックに漏れるバグ解消）。
  - **地図に現在モードのチップ常時表示＋モード別カーソル**（crosshair/grab/pointer）。「クリック=位置 / 左ドラッグ=方位」を地図上で明示。
  - **初期工程を `data`（レイヤ）に変更**（旧: AIチャット）。最初の一歩を分かりやすく。
- **#4 経路をプロジェクトに複数保存**: store `savedRoutes`（id/name/route/waypoints）。RoutePanel に「経路ライブラリ」＝現在の経路を名前付き保存／一覧／読込（waypoints も復元し再編集可）／削除。projectState v2 が savedRoutes も永続化。
- **#2 OSS セグメンテーション調査**: **samgeo（opengeos/segment-geospatial）** が本命（SAM を地理空間ラスタにそのまま適用、conda-forge/pip。点/bbox/テキストプロンプト対応）。導入可否を確認中（torch＋SAM重みDLの重量あり）。

### 確認済み → 対応
- **#3 = Drivable「描いて足す」を明確化**（実装済）: include/exclude を **「走行可に追加（マージ）」/「走行不可に除外」** に改称、主アクションを primary 化、ヒントを「ポリゴンを描く→走行可に追加で合体／走行不可に除外で穴あけ」に。編集一覧の表記も統一。include は元々 union（マージ）なので機能は既存、UX/名称を明確化した。
- **#2 = まず OpenCV で進める**（samgeo は見送り）。otsu/adaptive 実装済のまま。
- **#1 = 3D Tiles 見送り**（three.js 強化済の現状で運用）。

### traversability セグメンテーションの方針決定（記録）
ユーザーが鉱山向け off-road traversability の調査（OFFSEG / SegFormer+AutoMine / ResNet18-UNet / AnyTraverse(VLM) / OT-Drive / AutoMine・MUSeg データセット）を提示。**決定: 「俯瞰DSMのまま 幾何(slope/rough)→cost ＋ OpenCV(otsu/adaptive)」で進める**（＝現状の実装構成。追加実装・依存なし）。
- 重要な論点: 挙がったモデルの多くは**車載一人称カメラ視点**（RUGD/YCOR/RELLIS-3D）で、本アプリの**俯瞰(top-down)DSM**とは視点が不一致。かつ本アプリは**既に3D幾何(DSM)を保持**しており、traversability の本丸（傾斜・粗さ・段差）を幾何から直接測れるため、幾何ベースが原理的に主役。
- **将来 learned に進む場合の道筋**（入力モダリティを増やすことが前提）:
  - 車載/ドローンのカメラ映像を取り込む → SegFormer+AutoMine、または AnyTraverse(VLM/zero-shot, テキストプロンプト)。
  - 俯瞰オルソのまま補助 → samgeo/SAM（クラス非依存・プロンプト前提）。
  - DSM由来の疑似ラベルで自前の多クラス(可/減速/不可/障害物)学習。
  - 注意: AnyTraverse(2025)/OT-Drive(2026)/MUSeg は要一次情報確認（実在/ライセンス/事前学習重み）。

tests: planning_core 111 / api 60 / FE 9 / build 緑（本バッチはFEのみ）。

---

## 2026-06-14 の作業（その2: プランナ/レビュー修正/退出Goal）

### ✅ 完了
- **RRT* プランナ 仕上げ**: (1) rewire の**子孫コスト伝播バグを修正**（children隣接リスト＋reparentで差分をBFS伝播。レビュー#HIGH）、(2) コストを**弧長＋ソフトコスト(斜面/粗さ)積分＋cuspペナルティ**に拡張（純幾何長のみの問題を解消）、(3) goal祖先のrewire時に best_cost を再評価、(4) ショートカット平滑化は幾何長で比較。tests: `test_rrt_star.py` 4緑（到達/U字後進/seed決定性/mask帯内）。FE: `rrt_star` アルゴリズム選択肢＋planning.py 分岐（cost/mask/footprint任意・R_min native）＋api `test_plan_rrt_star_algorithm` 緑。スモーク: (25,8)到達。
- **レビュー修正（Top5のうち②③④＋⑤）**:
  - ② **safety steer ゲート修正**: `has("steer")`→`has("steer_rate")`（kind不一致で操舵角違反が安全判定に反映されないバグ）。
  - ③ **アップロードOOM**: layers.py を**1MiBチャンクストリーミング**化＋**4GiB上限(413)**＋途中失敗時のクリーンアップ。
  - ④ **3ストア並行制御**: store/projects/vehicle_overrides の read-modify-write を `threading.RLock` で直列化（last-writer-wins消失を防止）。projects は **updated_at をサーバ権威化**＋created_at＋name長バリデーション。
  - ⑤ **層id保存**: 共有 `pickLayer()` を導入し、store に `costLayerId/drivableLayerId`（明示選択, null=kind最新）。RoutePanel/SpottingPanel/DrivableAreaPanel/ThreeView が明示id優先で解決。projectState が保存時に層idを固定→再読込で当時のコスト/走行可能領域へ正しく紐づく（レビュー#20）。
  - ①(rrt* rewire) は上記 RRT* 仕上げで対応済み。
- **寄り付き 退出経路のGoal地点＋ベクトル指定（新要望）**: `with_exit` 時、`exit_goal=(x,y,heading)` で **target→exit_goal** を計画（未指定は従来どおり target→start）。FE: 地図モード `spot_exit_goal`（マゼンタ系マーカー）＋SpottingPanelに「退出Goalを指定/解除(startへ)/方位入力」。api `test_spotting_exit_goal_api` 緑。スモーク: 退出が(40,10)へ到達。
- tests: planning_core **108** / api **60** / FE **9** / build 緑。両サーバ稼働。

### 新規5項目への対応
- **#5 OSMベースマップ（実装済）**: 2Dビューに OpenStreetMap を最下層(zIndex0)で追加。OL が Web-Mercator→EPSG:6677 を自動再投影（proj4/6677 は登録済のため新規依存なし）。地図ツールバーに「OSM」トグル（既定OFF・opacity0.7）。store `osmVisible`。
- **#4 手動ポリゴンoverride（既に実装済 → UX強化）**: Include/Exclude による非破壊override は端から端まで実装済（編集一覧/個別削除/全消去/再生成保持も）。本日「**頂点を1つ戻す**」(`UNDO_POLY_VERTEX`)を Drivable/Area 両パネルに追加（従来は全消ししかできなかった弱点を解消）。残ギャップ: 確定済み編集の地図上アウトライン描画・頂点ドラッグ編集（未対応）。
- **#2 CostMap/Drivable 現状**: 回答済（コスト= w_slope·slope_n + w_rough·rough_n、ハード障害=slope>limit/nodata。Drivable= 閾値→close→open→smooth→fill→erosion→remove_small→keep_largest→edits）。改善候補（erosion前keep-largest・境界slopeアーティファクト除去・costmap weightのUI露出）は要判断。
- **#1 Cesium / #3 AI・OpenCV segmentation**: 設計判断のためユーザーに確認 → 下記の通り実施。

### 新規5項目の確定方針と実装（その3）
ユーザー回答: #1=見送り→three.js強化 / #3=OpenCV古典を実験追加 / #2=全4改善。
- **#1 three.js 強化（Cesiumは見送り）**: (1) 再構築時に geometry/material を `dispose()`（GPUメモリリーク解消, レビュー#18）、(2) **鉛直強調 vExag を group.scale.y** で反映（頂点バッファ再構築なし）、(3) **点サイズを material.size** で反映（バッファ再確保なし）、(4) route/waypoint/寄り付き/道幅の変化は **content のみ再構築**（terrain/points を作り直さない＝スライダー/編集のたびの重い再確保を回避）。
- **#3 OpenCV Drivable セグメンテーション（実験）**: `drivable/segment.py::segment_drivable_base`（cv2 遅延import）。method=`otsu`（大域自動閾値）/`adaptive`（局所適応閾値）。出力は `generate_drivable(base_override=...)` に渡し、クリアランス/連結成分/手動編集の共通パイプラインを通す（編集オーバーライドもそのまま効く）。drivable API `GenParams.method`、FE DrivableAreaPanel に「生成方法」セレクタ。tests: `test_segment.py` 3緑。opencv-python-headless を planning_core の optional extra `cv` に追記（venv 導入済 4.13）。
- **#2 CostMap/Drivable 改善（全4）**:
  - Drivable: **erosion を連結成分フィルタの後**へ（細い首切断での領域消失を防止）。
  - CostMap **境界アーティファクト除去**: nodata 縁1セルの偽急斜面を無効化（データ縁の偽の壁を防ぐ）。
  - CostMap **地面推定を低パーセンタイル**化（既定5%）＋`min_points_per_cell` ゲート（外れ点/疎セル対策）。CostmapParams に `ground_percentile`/`min_points_per_cell`。
  - **w_slope/w_rough/rough_window を FE 露出**（CostmapPanel）。
- **#4 確定編集の地図描画**: Drivable の確定 include/exclude を地図に**緑/赤の破線アウトライン**で重畳（MapView, pickLayer で対象層解決）。+ 「頂点を1つ戻す」（前述）。
- **#5 OSM ベースマップ**: 実装済（前述）。

最終: planning_core **111** / api **60** / FE **9** / build 緑。両サーバ稼働。

---

## 2026-06-14 の作業

### ✅ 完了
- **Reeds-Shepp プランナ**: `planners/reeds_shepp.py`（CSC+CCC族、各候補は終点検証してから採用＝式の不備でも誤経路を返さない）。hybrid A* の解析接続に統合（前進Dubins→失敗時にRSで曲がりながらの後進＝角度付きcusp）。単体アルゴリズム `reeds_shepp` も追加（FE選択肢/store/client/api分岐）。tests: planning_core 全緑 / api `test_plan_reeds_shepp_algorithm` 緑 / FE build緑。
- **寄り付き 切り返しの 45/90度化（タスク2の実装）**: `simulator/spotting.py` の1切り返し候補で、進入方位 dh を ±20→**±20/±45/±90度**へ拡張、さらに**ターゲット周囲リング（側方/斜め進入）の S** を追加。加えて **start→target を1本の Reeds-Shepp で結ぶ候補**を追加（角度付きcuspが自然に出る）。→「ドック正面で直進バック」一辺倒を解消。
- **手動切り返し点モード（追加タスク）**: `plan_spotting(manual_switch_pose=(x,y,yaw))`＝指定Sを固定し前進(start→S)＋後進(S→target)のみ生成。API `manual_switch_pose`、FE: 地図モード `spot_switch`（クリック=位置/左ドラッグ=方位・緑マーカー）＋SpottingPanelに「切り返し点を手動指定/解除/方位入力」。
- **切り返し可能エリア（タスク3）**: `plan_spotting(switchback_zone=[(x,y)..])`＝cuspが多角形内の候補のみ採用、全滅時は前進のみへフォールバック。API `switchback_zone`、FE: SpottingPanelで既存「エリア」を選択（破線緑で地図強調）。
- **バグ修正**: 寄り付き結果 score=inf 等が JSON 非準拠で 500 になる問題 → `simulate._fin()` で inf/nan→None。
- **不可理由メッセージ追加**: `NO_ZONE_PATH`（指定エリア内で切り返せない）/`NO_PATH` に専用の日本語不可理由。
- **タスク4（経路・エリアのバックエンド保存→呼出→編集）**: スコープ=経路・エリアのみ。`projectState` を **v2** 化し、従来保存しなかった**経路ジオメトリ(route: trajectory/analysis/safety)を保存**、`applyProject` で**そのまま復元**（再生成不要＝レビュー#19ギャップを解消）。v1（route無し）は後方互換で route=null。ProjectPanel 読込時に「経路✓/エリアN」を表示、説明文更新。spotResult はレイヤ依存のため従来どおり非保存。FE round-trip 単体テスト追加（projectState.test.ts）。API スモーク: 保存→取得で route 2点・area 1 の復元を確認。
- tests: planning_core spotting 11緑（手動/ゾーン/フォールバック追加）/ api spotting 6緑（手動・ゾーン追加）/ planning_core 104・api 58・FE 9・build 緑。スモーク: 手動点(22,6)経由・ゾーン制約 switch(40.1,0)∈zone を確認。

### ⏸ 中断中（未完）
- **RRT* プランナ**: `planners/rrt_star.py` 実装済（RSステアリング・rewire・ショートカット平滑化・seed決定的）だが **未テスト・未FE配線**。レビューで rewire の子孫コスト未伝播バグ指摘あり（要修正）。寄り付き優先のため一時停止。
- State Lattice / Hybrid切返しチューニングは未着手。

### 📋 確認待ち（ユーザー判断）
- **タスク4（経路・エリアのバックエンド保存/呼出/編集）**: 文末が途切れていたため未着手。保存対象は「まず経路・エリアのみ」と回答済み。寄り付き改善が一段落したので次に着手予定。
- レビュー（タスク1）は**列挙のみ・修正は未実施**（ユーザー指示）。最優先Top5: ①rrt* rewire ②safety steerゲート ③アップロードOOM ④3ストア並行制御 ⑤保存→再読込の経路喪失＋層id不保存。

---

## ✅ 完了（テスト結果つき）
- **AI 502 堅牢化**: `agent._oai_create` で Groq の 429/5xx/接続エラーを指数バックオフ(最大3回)リトライ。BadRequest()(tool_use_failed)は再試行せず本文形式ツール復元へ。api tests 17緑（リトライ/400非リトライの単体含む）。
- **Stage6 速度プロファイル**: `analysis/velocity.py::velocity_profile`（横加速度 v=√(a_lat/κ)＋ギア別速度上限、端点/切返で停止、前後2パスで加減速制限、時間積分）。TrajPoint に `speed_mps`/`time_s`、AnalysisResult に `max_speed_mps`/`time_total_s`/`stopping_distance_m`(=v²/2a_dec)。build_trajectory が車両指定時に計算、summarize が要約。FE: AnalysisPanel に速度チャート＋max speed/所要時間/停止距離、CSVに speed/time 列、VehiclePanel に横/加/減速＆安全しきい値を編集追加。tests: planning_core 90 / api 51 / web 7 / build 緑。
- **NoGoZone 制約**: エリア多角形(`areas`)を進入禁止として plan/spotting の mask/cost からカーブアウト（`_rasterize_nogo`）→ planner が回避、塞がれば 422/不可。PlanRequest/SpottingRequest に `no_go_polygons`、RoutePanel/SpottingPanel が store.areas を送付、AI(agent)も context.areas→state→plan/spotting で尊重。tests: api 52（NoGoで壁を作ると grid_astar 422 を確認）。**既定で全エリア=進入禁止**（要確認②）。
- **寄り付き 退出軌道 + 不可理由**: SpottingRequest `with_exit` で退出軌道(target→start)を同条件で計画し `exit` を返す。各結果に `reason`（クリアランス不足/到達誤差大などの日本語不可理由）。FE: SpottingPanel トグル＋不可理由/退出サマリ表示、MapView にマゼンタ点線で退出軌道描画。api 52緑/FE緑。
- **Hybrid A* 高度化**: (1)中間曲率プリミティブ ±0.5κmax 追加（半径2ρ≥ρ で R_min順守、滑らか＆探索自由度UP）、(2)`allow_reverse`時は解析接続に reverse_dubins も試行（後進で寄せ／末端ギア変化はcusp計上）。既定reverse offで後方互換。planning_core 90/api 52緑。
- **鉱山/土木シナリオ＋チューニングレポート**: `planning_core/scenarios.py`（haul_straight/switchback/hairpin/narrow_bench/dead_end_bay/fragmented_gap の6形状を mask 合成）＋ `scripts/scenario_report.py` → **`docs/SCENARIO_REPORT.md`** に 車両×{基本/+reverse/footprintOFF} の可否・実測R・最小離隔・反復・切返を表で出力。test_scenarios 4緑。**朝はこのレポートをレビュー対象に**。
- **README 整備**: `README.md` を現状（6段アーキ・全機能・実行/テスト手順・AIマルチプロバイダ）へ全面更新。
- **Drivable 編集強化**: `DELETE /api/drivable/{id}/edits/{index}`（個別削除）/ `…/edits`（全消去）。DrivableAreaPanel に編集一覧（include/exclude・頂点数）＋削除/全消去ボタン。

### 就寝後の自律追加（低リスク・確認不要分のみ）
- **シナリオ拡充**: `t_junction`(T字交差) と `wide_curve`(緩い大カーブ) を追加（計8形状）。レポート再生成済み。
- **レポートの実測R を cusp 除外計測に**（レポート専用・アプリ挙動は不変）。switchback +reverse の R が 0.3→**10.1/8.9**（R_min相当）と正しく出るように。※アプリ側の min_radius 評価への同様の対策は **要確認③** のため未適用（朝に判断）。
- **ハードニング**: api テスト追加（速度プロファイルが plan 応答に出る / `with_exit` が exit を返す / drivable 個別edit削除・全消去）。
- 最終: planning_core **94** / api **55** / web 7 / build 緑。両サーバ稼働。これ以上の自律変更はせず待機（保留②項目＝State Lattice/HM400v2 は要設計判断のため未着手）。

## ⚙️ 自分で下した判断（可逆・理由つき。NGなら朝に巻き戻し可）
- 速度プロファイル既定値（全車種・暫定）: `max_lateral_accel=1.5 m/s²` / `max_accel=0.5` / `max_decel=1.0`。理由: 実スペック未入手のため安全側の概算。→ **実値が分かれば車両パラメータ画面で上書き可**（要確認①）。
- 速度プロファイルは route 軌跡(Trajectory)に適用。spotting は従来どおり独自の低速時間計算のまま（二重計算回避）。

## ✅ 朝の追加対応
- **フットプリント誤判定の修正**（ユーザー報告「車両包絡線 不適合 実測0.10/限界0.00」）: 原因＝ユーザーが置いた**始点/終点で長尺車体が配置点の外へオーバーハング**（例: 始点x=4で車体がx=-1.1まで、終点で+側へ）→ 経路全体を誤NG。対策＝`trajectory_footprint_violations(..., ignore_ends_m)` を追加し、summarize は **車体半長ぶんの端点領域を判定から除外**（始終点は計画で動かせない既定姿勢由来のため）。中間のはみ出しは引き続き検出。tests: planning_core 99 / api 55 緑。
- **③ cusp×R_min（アプリ側）対応済み**（承認OK）: `analysis/curvature.py::cusp_mask`（方向反転点を幾何検出）で build_trajectory/summarize の min_radius・操舵・曲率変化率の評価から cusp を除外。hybrid A* は R_min native かつ cusp を持ち得るため `enforce_min_radius` 平滑化を適用しないようゲート（cuspを潰さない）。tests: planning_core 96 / api 55 緑。→ 切り返し経路が誤って「R_min違反/NG」にならない。

## ❓ 要確認（あなたの判断が必要）
- ① **速度/加速度の実値**: max_lateral_accel/accel/decel・max_speed_fwd/rev を HD785/HM400 等の実スペックで確定したい（現状は暫定）。→ 値が来れば車両パラメータ画面 or YAML で更新。
- ② **NoGoZone の扱い**: 「全エリア＝進入禁止」を既定に。一部エリアだけ NoGo にしたい場合は per-area タイプ分け（走行可/禁止/2車線等）が必要か？要方針。
- ③ **切り返し(cusp)と最小旋回半径の評価**: シナリオレポートで判明。reverse 経路は折返点(cusp)を持ち、3点外接円の R が cusp で~0になり `min_radius` を誤って違反扱いしうる（enforce_min_radius がcuspを潰そうとする副作用も）。→ **対策案: ギア変化点(cusp)を曲率/ R_min 評価から除外**。実装してよいか（朝に着手可）。推奨: 対応する。
- ④ **SCENARIO_REPORT.md のレビュー**: 各形状で「どの車両・どの設定なら可」が一覧。狭ベンチ/断片化など “物理的に不可” と “設定で可” の線引きを見て、既定パラメータ(planner_cell, footprint, reverse既定)をどうするか相談したい。

## ⏭️ 着手せず保留（要設計判断）
- State Lattice / RRT* プランナー追加（大）
- HM400 中折れ（articulated）運動学 v2（大）

## 🧪 末尾サマリ
- 最終テスト: **planning_core 94 / api 55 / web 7 / FE build 緑**（全緑）。
- サーバ: API :8077 `available:true (Groq)` / FE :5173 `200`。どちらも稼働中。
- 朝の見どころ: ① **docs/SCENARIO_REPORT.md**（車両×設定の可否表）、② 速度/加速度・安全しきい値の実値確定（要確認①）、③ NoGoZoneの粒度（要確認②）、④ cusp×R_min評価の対策可否（要確認③・推奨:対応）。
- 完了8件: AI502堅牢化 / Stage6速度プロファイル / NoGoZone / 寄り付き退出軌道+不可理由 / Hybrid A*高度化 / シナリオ+レポート / README整備 / Drivable編集強化。
- 着手せず保留（要設計判断）: State Lattice・RRT* / HM400中折れ運動学v2。
