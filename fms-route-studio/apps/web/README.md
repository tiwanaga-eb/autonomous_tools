# apps/web (React + TS + Vite フロントエンド)

**Phase 2 以降**で実装。

- 状態管理: Zustand / Command Bus（spotting_planner_ui の流儀を参照。※あちらは純SVGビューアで地図ライブラリの参照ではない）
- 地図描画: **Phase 2 でPoC選定**（投影座標系6677/一部4979とWeb Mercatorの衝突を回避。再投影配信 or 投影ネイティブ。設計書 §6.2・§7.1・残課題9）
- 解析チャート: uPlot

> Phase 0 ではプレースホルダ。
