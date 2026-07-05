// 寄り付きシミュレータ（§13）の状態。
import type { StateCreator } from "zustand";

import type { SpotPose, SpotResult, SpottingMethod, SpotWeights } from "@/types/api";

import type { AppState } from "./useStore";

export interface SpotSlice {
  spotStart: SpotPose | null;
  spotTarget: SpotPose | null;
  spotSwitchPose: SpotPose | null;   // 手動切り返し点（指定時は前進→S→後進のみ）
  spotSwitchZoneId: string | null;   // 切り返し可能エリア（areas の id を流用）
  spotContainAreaId: string | null;  // 走行を収めるエリア（経路＋車体をこの中に収める。areas の id）
  spotMaxSwitch: 0 | 1;
  spotMethod: SpottingMethod; // 寄り付き候補生成アルゴリズム（auto/dubins/reeds_shepp/hybrid_astar）
  spotSmooth: boolean; // 経路平滑化（曲率不連続=速度低下/蛇行を抑制）
  spotStationaryMode: "auto" | "allow" | "deny"; // 据え切り(出発/到着の端点その場操舵): 自動(車種既定)/許可/禁止
  spotMinSpeedKmh: number; // 最低速度[km/h]（出発/到着/切返付近以外で下限。0=無効）
  spotRoadWidthM: number; // 寄り付きの道幅[m]（0=車両footprintを使用）
  spotCuspMargin: number; // 切り返し点の直線マージン[m]（0=自動）
  spotWithExit: boolean;  // 退出軌道も生成
  spotExitGoal: SpotPose | null; // 退出の行先Goal姿勢（null=start へ戻る）
  spotWeights: SpotWeights;
  spotResult: SpotResult | null;
  spotIndex: number; // 再生/スクラブの現在サンプルindex

  setSpotStart: (p: SpotPose | null) => void;
  setSpotTarget: (p: SpotPose | null) => void;
  setSpotSwitchPose: (p: SpotPose | null) => void;
  setSpotSwitchZoneId: (id: string | null) => void;
  setSpotContainAreaId: (id: string | null) => void;
  setSpotMaxSwitch: (v: 0 | 1) => void;
  setSpotMethod: (v: SpottingMethod) => void;
  setSpotSmooth: (v: boolean) => void;
  setSpotStationaryMode: (v: "auto" | "allow" | "deny") => void;
  setSpotMinSpeedKmh: (v: number) => void;
  setSpotRoadWidthM: (v: number) => void;
  setSpotCuspMargin: (v: number) => void;
  setSpotWithExit: (v: boolean) => void;
  setSpotExitGoal: (p: SpotPose | null) => void;
  setSpotWeights: (w: SpotWeights) => void;
  setSpotResult: (r: SpotResult | null) => void;
  setSpotIndex: (i: number) => void;
}

export const createSpotSlice: StateCreator<AppState, [], [], SpotSlice> = (set) => ({
  spotStart: null,
  spotTarget: null,
  spotSwitchPose: null,
  spotSwitchZoneId: null,
  spotContainAreaId: null,
  spotMaxSwitch: 1,
  spotMethod: "auto",
  spotSmooth: true,
  spotStationaryMode: "auto",
  spotMinSpeedKmh: 0,
  spotRoadWidthM: 0,
  spotCuspMargin: 0,
  spotWithExit: false,
  spotExitGoal: null,
  spotWeights: { w_distance: 1, w_time: 0, w_reverse: 1, w_switchback: 8, w_costmap: 2, w_turn: 6 },
  spotResult: null,
  spotIndex: 0,

  setSpotStart: (spotStart) => set({ spotStart }),
  setSpotTarget: (spotTarget) => set({ spotTarget }),
  setSpotSwitchPose: (spotSwitchPose) => set({ spotSwitchPose }),
  setSpotSwitchZoneId: (spotSwitchZoneId) => set({ spotSwitchZoneId }),
  setSpotContainAreaId: (spotContainAreaId) => set({ spotContainAreaId }),
  setSpotMaxSwitch: (spotMaxSwitch) => set({ spotMaxSwitch }),
  setSpotMethod: (spotMethod) => set({ spotMethod }),
  setSpotSmooth: (spotSmooth) => set({ spotSmooth }),
  setSpotStationaryMode: (spotStationaryMode) => set({ spotStationaryMode }),
  setSpotMinSpeedKmh: (spotMinSpeedKmh) => set({ spotMinSpeedKmh }),
  setSpotRoadWidthM: (spotRoadWidthM) => set({ spotRoadWidthM }),
  setSpotCuspMargin: (spotCuspMargin) => set({ spotCuspMargin }),
  setSpotWithExit: (spotWithExit) => set({ spotWithExit }),
  setSpotExitGoal: (spotExitGoal) => set({ spotExitGoal }),
  setSpotWeights: (spotWeights) => set({ spotWeights }),
  setSpotResult: (spotResult) => set({ spotResult, spotIndex: 0 }),
  setSpotIndex: (spotIndex) => set({ spotIndex }),
});
