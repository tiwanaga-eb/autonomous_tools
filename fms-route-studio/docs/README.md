# FMS Route Studio — ドキュメント

| 文書 | 内容 | 主視点 |
|---|---|---|
| [REQUIREMENTS.md](REQUIREMENTS.md) | **要件定義書** — 背景・目的・スコープ・ステークホルダー要求（StRS/SLQDC）・システム要求（SysRS/F·P·I·S·R）・トレーサビリティ・オープン論点 | SE / PM |
| [DESIGN.md](DESIGN.md) | **設計書** — アーキテクチャ・コンポーネント設計（planning_core / API / web）・データモデル・API仕様・アルゴリズム・座標系・永続化・AI・非機能 | SE |
| [GAP_ANALYSIS.md](GAP_ANALYSIS.md) | **突き合わせ** — 上位 unmanned-requirements（ADT の FMS/S-00）と FRS のカバレッジ／ギャップ分析 | SE/PM |
| [REVIEW.md](REVIEW.md) | **実装・構造レビュー** — FE/BE のレビュー指摘と段階的改善の対応状況 | SE |
| [SCENARIO_REPORT.md](SCENARIO_REPORT.md) | 鉱山/土木シナリオの自動チューニングレポート（`scripts/scenario_report.py` 生成） | SE |

## 方法論

要件定義は `unmanned-requirements/MethodGuide.md` のメソドロジー（SLQDC / F·P·I·S·R / トレーサビリティ / SE・PM 視点分離）に準拠する。ID は製品ローカルに `FRS-` 接頭辞を付与し、上位パッケージ AutonomousDumptruck（FMS 運航前工程 S-00）へトレースする。

## 文書間の流れ

```
[上位] AutonomousDumptruck ConOps/SysRS（FMS S-00 運航前データセット準備）
          ↓ FRS が実装・検証ツール化
REQUIREMENTS.md（StRS → SysRS、上位 SysRS-F-001〜010 等へトレース）
          ↓ どう実装するか
DESIGN.md（アーキテクチャ・アルゴリズム・API・データモデル）
          ↓ 検証
SCENARIO_REPORT.md / tests
```
