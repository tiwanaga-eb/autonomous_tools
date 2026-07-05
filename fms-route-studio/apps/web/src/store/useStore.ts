// アプリ状態ストア（zustand・スライス構成）。
// 単一巨大ストアをドメイン別スライス（ui / edit+履歴 / plan / spot / fleet）へ分割し、
// 同一の useStore として合成する。外部 API（useStore と型の import 経路）は従来と不変。
import { create } from "zustand";

import { createEditSlice } from "./editSlice";
import type { EditSlice } from "./editSlice";
import { createFleetSlice } from "./fleetSlice";
import type { FleetSlice } from "./fleetSlice";
import { createPileSlice } from "./pileSlice";
import type { PileSlice } from "./pileSlice";
import { createPlanSlice } from "./planSlice";
import type { PlanSlice } from "./planSlice";
import { createSpotSlice } from "./spotSlice";
import type { SpotSlice } from "./spotSlice";
import { createUiSlice } from "./uiSlice";
import type { UiSlice } from "./uiSlice";

// 型は従来どおり "@/store/useStore" から import できるよう再エクスポート。
export type {
  Algorithm,
  AreaPoly,
  EditMode,
  FeatureId,
  ImportedRoute,
  PlanMode,
  RouteResult,
  SavedRoute,
  StatusLevel,
  Waypoint,
} from "./types";

export type AppState = UiSlice & EditSlice & PlanSlice & SpotSlice & FleetSlice & PileSlice;

export const useStore = create<AppState>()((...a) => ({
  ...createUiSlice(...a),
  ...createEditSlice(...a),
  ...createPlanSlice(...a),
  ...createSpotSlice(...a),
  ...createFleetSlice(...a),
  ...createPileSlice(...a),
}));
