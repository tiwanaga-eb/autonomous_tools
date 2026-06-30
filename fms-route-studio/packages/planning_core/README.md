# planning_core

FMS Route Studio の共通プランニング基盤（純Python・副作用最小）。
API / CLI / AIエージェントが共通利用する単一ソース。

## モジュール

| パッケージ | 役割 | 状態(Phase 0) |
|---|---|---|
| `models/` | Pydantic ドメインモデル（geo/route/vehicle/costmap/analysis） | ✅ |
| `geometry/` | 座標変換（canonical/working/WGS84 一元化） | ✅ projection |
| `costmap/` | LAS→**生cost(float32)＋DSM** 生成、表示RGBは分離 | ✅ from_las / colorize |
| `planners/` | 経路生成（Phase 0 は spline） | ✅ spline |
| `analysis/` | κ・dκ/ds・最小旋回半径 | ✅ curvature |
| `vehicle/` | 4機種プロファイル（HD785/HD605/HM400/CD110R, kinematic_type） | ✅ |
| `drivable/` `simulator/` `smoothing/` `io/` | 後続フェーズ | ⏳ |

## テスト

```bash
# 依存が入った既存venvを使う場合（リポジトリの Map_Builder_Proto系 .venv 等）
PYTHONPATH=. python -m pytest        # pytest があれば
PYTHONPATH=. python tests/run_tests.py  # pytest が無くても動く簡易ランナー
```

設計は `docs/REARCHITECTURE.md`（§5 planning_core, §8 モデル, §21 レビュー反映）参照。
