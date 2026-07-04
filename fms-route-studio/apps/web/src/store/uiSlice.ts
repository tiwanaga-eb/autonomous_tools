// UI 状態（工程・編集モード・ステータス・表示トグル・2D/3D ビュー）。
import type { StateCreator } from "zustand";

import type { AppState } from "./useStore";
import type { EditMode, FeatureId, StatusLevel } from "./types";

export interface UiSlice {
  activeFeature: FeatureId; // サイドバーで選択中の工程
  mode: EditMode;
  status: string;
  statusLevel: StatusLevel; // 状態メッセージの重要度（色分け表示用）
  busy: boolean;            // 長時間処理中（オーバーレイ/ボタン無効化）
  busyLabel: string;        // 処理中ラベル
  costOpacity: number; // コスト オーバーレイの不透明度(0..1)
  costVisible: boolean; // コスト オーバーレイの表示
  drivableOpacity: number; // 走行可能領域 オーバーレイの不透明度(0..1)
  drivableVisible: boolean; // 走行可能領域 オーバーレイの表示
  osmVisible: boolean; // OpenStreetMap ベースマップ（地理的文脈）の表示
  showWaypoints: boolean; // 経路Waypoint(点)の表示
  hoverPointIndex: number | null; // Analysisグラフでホバー中の軌跡点index（地図と同期）
  view3d: boolean; // 中央ビューを 2D地図 / 3D に切替
  view3dMode: "points" | "mesh"; // 3Dの表示: LAS点群 / DSM地形メッシュ
  pointSize: number; // 点群の点サイズ[px]
  pointBudget: number; // 3D点群の表示点数上限（バイナリ転送で大点数可）
  vExag: number;   // 3Dの鉛直強調倍率

  setActiveFeature: (f: FeatureId) => void;
  setMode: (mode: EditMode) => void;
  setStatus: (status: string, level?: StatusLevel) => void;
  setBusy: (active: boolean, label?: string) => void;
  setCostOpacity: (v: number) => void;
  setCostVisible: (v: boolean) => void;
  setDrivableOpacity: (v: number) => void;
  setDrivableVisible: (v: boolean) => void;
  setOsmVisible: (v: boolean) => void;
  setShowWaypoints: (v: boolean) => void;
  setHoverPoint: (i: number | null) => void;
  setView3d: (v: boolean) => void;
  setView3dMode: (v: "points" | "mesh") => void;
  setPointSize: (v: number) => void;
  setPointBudget: (v: number) => void;
  setVExag: (v: number) => void;
}

export const createUiSlice: StateCreator<AppState, [], [], UiSlice> = (set) => ({
  activeFeature: "data",
  mode: "start",
  status: "ready",
  statusLevel: "info",
  busy: false,
  busyLabel: "",
  costOpacity: 0.6,
  costVisible: true,
  drivableOpacity: 0.5,
  drivableVisible: true,
  osmVisible: false,
  showWaypoints: true,
  hoverPointIndex: null,
  view3d: false,
  view3dMode: "points",
  pointSize: 2.0,
  pointBudget: 1_000_000,
  vExag: 1.0,

  // 工程を切り替えたら編集モードを中立(pan)へ戻す（前工程のモードが地図クリックに漏れるのを防ぐ）
  setActiveFeature: (activeFeature) =>
    set((s) => (s.activeFeature === activeFeature ? { activeFeature } : { activeFeature, mode: "pan" })),
  setMode: (mode) => set({ mode }),
  setStatus: (status, level = "info") => set({ status, statusLevel: level }),
  setBusy: (busy, busyLabel = "") => set({ busy, busyLabel }),
  setCostOpacity: (costOpacity) => set({ costOpacity }),
  setCostVisible: (costVisible) => set({ costVisible }),
  setDrivableOpacity: (drivableOpacity) => set({ drivableOpacity }),
  setDrivableVisible: (drivableVisible) => set({ drivableVisible }),
  setOsmVisible: (osmVisible) => set({ osmVisible }),
  setShowWaypoints: (showWaypoints) => set({ showWaypoints }),
  setHoverPoint: (hoverPointIndex) => set({ hoverPointIndex }),
  setView3d: (view3d) => set({ view3d }),
  setView3dMode: (view3dMode) => set({ view3dMode }),
  setPointSize: (pointSize) => set({ pointSize }),
  setPointBudget: (pointBudget) => set({ pointBudget }),
  setVExag: (vExag) => set({ vExag }),
});
