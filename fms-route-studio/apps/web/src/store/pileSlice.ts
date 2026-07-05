// 排土（パイル）配置計画の状態。
import type { StateCreator } from "zustand";

import type { PilePlanResult } from "@/types/api";

import type { AppState } from "./useStore";

export interface PileSlice {
  pileAreaId: string | null;                 // 対象エリア（areas の id）
  pileSizeMode: "volume" | "height";         // パイル大きさの指定方法
  pileVolumeM3: number;                      // パイル体積[m³]（ダンプ1杯 目安 HM400≈24）
  pileHeightM: number;                       // パイル高さ[m]
  pileReposeDeg: number;                     // 安息角[°]
  pilePlaceMode: "spacing" | "spread";       // 間隔指定 / 撒き出しから計算
  pileDx: number;                            // 配置間隔 横[m]
  pileDy: number;                            // 配置間隔 縦[m]（0=横と同じ）
  pileStagger: boolean;                      // 千鳥配置
  pileStaggerInv: boolean;                   // 千鳥のオフセット行を逆に（斜め方向を反転）
  pileSpreadT: number;                       // 撒き出し厚 t[m]
  pileSpreadDx: number;                      // 撒き出しモードの横間隔[m]（基準。0=自動√(V/t)。縦=(V/t)/横）
  pileEdgeMarginM: number;                   // 縁マージン[m]（-1=自動: パイル基部半径）
  pilePlan: PilePlanResult | null;           // 直近の配置結果（地図に描画）

  setPileAreaId: (id: string | null) => void;
  setPileSizeMode: (m: "volume" | "height") => void;
  setPileVolumeM3: (v: number) => void;
  setPileHeightM: (v: number) => void;
  setPileReposeDeg: (v: number) => void;
  setPilePlaceMode: (m: "spacing" | "spread") => void;
  setPileDx: (v: number) => void;
  setPileDy: (v: number) => void;
  setPileStagger: (v: boolean) => void;
  setPileStaggerInv: (v: boolean) => void;
  setPileSpreadT: (v: number) => void;
  setPileSpreadDx: (v: number) => void;
  setPileEdgeMarginM: (v: number) => void;
  setPilePlan: (p: PilePlanResult | null) => void;
}

export const createPileSlice: StateCreator<AppState, [], [], PileSlice> = (set) => ({
  pileAreaId: null,
  pileSizeMode: "volume",
  pileVolumeM3: 24,
  pileHeightM: 1.5,
  pileReposeDeg: 37,
  pilePlaceMode: "spacing",
  pileDx: 8,
  pileDy: 0,
  pileStagger: false,
  pileStaggerInv: false,
  pileSpreadT: 0.5,
  pileSpreadDx: 0,
  pileEdgeMarginM: -1,
  pilePlan: null,

  setPileAreaId: (pileAreaId) => set({ pileAreaId }),
  setPileSizeMode: (pileSizeMode) => set({ pileSizeMode }),
  setPileVolumeM3: (pileVolumeM3) => set({ pileVolumeM3 }),
  setPileHeightM: (pileHeightM) => set({ pileHeightM }),
  setPileReposeDeg: (pileReposeDeg) => set({ pileReposeDeg }),
  setPilePlaceMode: (pilePlaceMode) => set({ pilePlaceMode }),
  setPileDx: (pileDx) => set({ pileDx }),
  setPileDy: (pileDy) => set({ pileDy }),
  setPileStagger: (pileStagger) => set({ pileStagger }),
  setPileStaggerInv: (pileStaggerInv) => set({ pileStaggerInv }),
  setPileSpreadT: (pileSpreadT) => set({ pileSpreadT }),
  setPileSpreadDx: (pileSpreadDx) => set({ pileSpreadDx }),
  setPileEdgeMarginM: (pileEdgeMarginM) => set({ pileEdgeMarginM }),
  setPilePlan: (pilePlan) => set({ pilePlan }),
});
