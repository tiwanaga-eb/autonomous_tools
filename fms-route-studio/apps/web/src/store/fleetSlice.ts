// 複数台（Fleet）: 保存経路ライブラリ・競合判定・簡易シミュレーション・待避所。
import type { StateCreator } from "zustand";

import type { BayCfg, FleetConflict, FleetSimResult } from "@/types/api";

import type { AppState } from "./useStore";
import type { SavedRoute } from "./types";

export interface FleetSlice {
  savedRoutes: SavedRoute[]; // プロジェクトに保存した経路ライブラリ（複数台制御の経路集合）
  showSavedRoutes: boolean;  // 保存経路ライブラリを地図に重ねて表示（複数経路の可視化）
  fleetConflicts: FleetConflict[] | null; // 直近の経路競合判定結果（savedRoutes の index 参照）
  fleetSim: FleetSimResult | null; // 直近の簡易シミュレーション結果
  fleetSimT: number; // 再生中の現在時刻[s]
  fleetBays: Record<string, BayCfg>; // 待避所（すれ違い点）。savedRoute.id → 設定

  setSavedRoutes: (routes: SavedRoute[]) => void;
  setShowSavedRoutes: (v: boolean) => void;
  setFleetConflicts: (c: FleetConflict[] | null) => void;
  setFleetSim: (s: FleetSimResult | null) => void;
  setFleetSimT: (t: number) => void;
  setFleetBay: (routeId: string, cfg: BayCfg | null) => void;
  setFleetBays: (bays: Record<string, BayCfg>) => void; // 一括置換（プロジェクト読込で残留を掃除）
}

export const createFleetSlice: StateCreator<AppState, [], [], FleetSlice> = (set) => ({
  savedRoutes: [],
  showSavedRoutes: true,
  fleetConflicts: null,
  fleetSim: null,
  fleetSimT: 0,
  fleetBays: {},

  setSavedRoutes: (savedRoutes) => set({ savedRoutes }),
  setShowSavedRoutes: (showSavedRoutes) => set({ showSavedRoutes }),
  setFleetConflicts: (fleetConflicts) => set({ fleetConflicts }),
  setFleetSim: (fleetSim) => set({ fleetSim }),
  setFleetSimT: (fleetSimT) => set({ fleetSimT }),
  setFleetBay: (routeId, cfg) =>
    set((s) => {
      const next = { ...s.fleetBays };
      if (cfg) next[routeId] = cfg;
      else delete next[routeId];
      return { fleetBays: next };
    }),
  setFleetBays: (fleetBays) => set({ fleetBays }),
});
