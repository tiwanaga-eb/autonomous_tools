// 編集対象（waypoints/route/areas/作図中ポリゴン/インポート経路）＋ Undo/Redo 履歴。
import type { StateCreator } from "zustand";

import type { XY } from "@/types/api";

import type { AppState } from "./useStore";
import type { AreaPoly, ImportedRoute, RouteResult, Snapshot, Waypoint } from "./types";

const MAX_HISTORY = 50;

// 履歴へ積む際は編集対象を「深く」複製する。これにより、コマンドが万一 in-place で
// 要素オブジェクト（Waypoint.xy 等）を書き換えても、過去スナップショットが汚染されない。
const clone = <T>(v: T): T =>
  typeof structuredClone === "function" ? structuredClone(v) : (JSON.parse(JSON.stringify(v)) as T);

function snap(s: Snapshot): Snapshot {
  return {
    waypoints: clone(s.waypoints),
    route: clone(s.route),
    areas: clone(s.areas),
    activePolygon: clone(s.activePolygon),
    importedRoutes: clone(s.importedRoutes),
    selectedId: s.selectedId,
  };
}

export interface EditSlice extends Snapshot {
  undoStack: Snapshot[];
  redoStack: Snapshot[];

  setWaypoints: (waypoints: Waypoint[]) => void;
  setRoute: (route: RouteResult | null) => void;
  setAreas: (areas: AreaPoly[]) => void;
  setActivePolygon: (pts: XY[]) => void;
  setImportedRoutes: (routes: ImportedRoute[]) => void;
  select: (id: string | null) => void;

  commit: () => void; // 変更前スナップショットを履歴に積む
  undo: () => void;
  redo: () => void;
  resetRoute: () => void;
}

export const createEditSlice: StateCreator<AppState, [], [], EditSlice> = (set) => ({
  waypoints: [],
  route: null,
  areas: [],
  activePolygon: [],
  importedRoutes: [],
  selectedId: null,
  undoStack: [],
  redoStack: [],

  setWaypoints: (waypoints) => set({ waypoints }),
  setRoute: (route) => set({ route }),
  setAreas: (areas) => set({ areas }),
  setActivePolygon: (activePolygon) => set({ activePolygon }),
  setImportedRoutes: (importedRoutes) => set({ importedRoutes }),
  select: (selectedId) => set({ selectedId }),

  commit: () =>
    set((s) => ({ undoStack: [...s.undoStack.slice(-(MAX_HISTORY - 1)), snap(s)], redoStack: [] })),

  undo: () =>
    set((s) => {
      if (s.undoStack.length === 0) return {};
      const prev = s.undoStack[s.undoStack.length - 1];
      return { ...prev, undoStack: s.undoStack.slice(0, -1), redoStack: [...s.redoStack, snap(s)] };
    }),

  redo: () =>
    set((s) => {
      if (s.redoStack.length === 0) return {};
      const next = s.redoStack[s.redoStack.length - 1];
      return { ...next, redoStack: s.redoStack.slice(0, -1), undoStack: [...s.undoStack, snap(s)] };
    }),

  resetRoute: () => set({ waypoints: [], route: null, activePolygon: [], selectedId: null }),
});
