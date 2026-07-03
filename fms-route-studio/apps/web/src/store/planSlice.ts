// 経路生成の設定（レイヤ選択・車両・アルゴリズム・生成オプション）。
import type { StateCreator } from "zustand";

import type { Layer } from "@/types/api";

import type { AppState } from "./useStore";
import type { Algorithm, PlanMode } from "./types";

export interface PlanSlice {
  layers: Layer[];
  routeSpacing: number; // 経路リサンプル間隔[m]
  roadWidthM: number; // 道幅[m]（0=帯を描かない）
  enforceFootprint: boolean; // hybrid A* で実車体フットプリントを厳密な走行可能制約に
  enforceMinRadius: boolean; // R_min を持つ車種で最小旋回半径を保証（違反コーナーを局所平滑化）
  allowReverse: boolean; // hybrid A* で後進を許可（切り返しで狭所到達性UP）
  refineElasticBand: boolean; // 生成後に Elastic Band で洗練（中央寄せ・余裕確保）
  vehicleId: string | null; // 選択中の車両プロファイル
  costLayerId: string | null; // 明示選択中のコスト層（null=kind最新）。プロジェクトで固定/復元
  drivableLayerId: string | null; // 明示選択中の走行可能領域層（null=kind最新）
  vehiclesRev: number; // 車両パラメータ更新の世代（編集後に consumer を再取得させる）
  planMode: PlanMode; // auto / waypoint_guided
  algorithm: Algorithm; // spline / dubins / grid_astar

  setLayers: (layers: Layer[]) => void;
  setRouteSpacing: (v: number) => void;
  setRoadWidthM: (v: number) => void;
  setEnforceFootprint: (v: boolean) => void;
  setEnforceMinRadius: (v: boolean) => void;
  setAllowReverse: (v: boolean) => void;
  setRefineElasticBand: (v: boolean) => void;
  setVehicleId: (v: string | null) => void;
  setCostLayerId: (v: string | null) => void;
  setDrivableLayerId: (v: string | null) => void;
  bumpVehiclesRev: () => void;
  setPlanMode: (v: PlanMode) => void;
  setAlgorithm: (v: Algorithm) => void;
}

export const createPlanSlice: StateCreator<AppState, [], [], PlanSlice> = (set) => ({
  layers: [],
  routeSpacing: 2.0,
  roadWidthM: 0,
  enforceFootprint: true,
  enforceMinRadius: true,
  allowReverse: false,
  refineElasticBand: false,
  vehicleId: "HD785",
  costLayerId: null,
  drivableLayerId: null,
  vehiclesRev: 0,
  planMode: "waypoint_guided",
  algorithm: "spline",

  setLayers: (layers) => set({ layers }),
  setRouteSpacing: (routeSpacing) => set({ routeSpacing }),
  setRoadWidthM: (roadWidthM) => set({ roadWidthM }),
  setEnforceFootprint: (enforceFootprint) => set({ enforceFootprint }),
  setEnforceMinRadius: (enforceMinRadius) => set({ enforceMinRadius }),
  setAllowReverse: (allowReverse) => set({ allowReverse }),
  setRefineElasticBand: (refineElasticBand) => set({ refineElasticBand }),
  setVehicleId: (vehicleId) => set({ vehicleId }),
  setCostLayerId: (costLayerId) => set({ costLayerId }),
  setDrivableLayerId: (drivableLayerId) => set({ drivableLayerId }),
  bumpVehiclesRev: () => set((s) => ({ vehiclesRev: s.vehiclesRev + 1 })),
  setPlanMode: (planMode) => set({ planMode }),
  setAlgorithm: (algorithm) => set({ algorithm }),
});
