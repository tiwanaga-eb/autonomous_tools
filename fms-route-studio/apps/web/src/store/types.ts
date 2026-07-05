// store の共有型（スライス間で参照）。外部からは従来どおり "@/store/useStore" 経由で import できる。
import type { AnalysisResult, SafetyReport, Trajectory, XY } from "@/types/api";

export type EditMode =
  | "start"
  | "goal"
  | "via"
  | "insert_via"
  | "edit"
  | "pan"
  | "polygon"
  | "measure"
  | "spot_start"
  | "spot_target"
  | "spot_switch"
  | "spot_exit_goal";

export interface Waypoint {
  id: string;
  role: "start" | "via" | "goal";
  xy: XY;
  heading_deg?: number | null;
}

export interface AreaPoly {
  id: string;
  name: string;
  points: XY[];
}

export interface ImportedRoute {
  id: string;
  name: string;
  pts: XY[];
}

export interface RouteResult {
  trajectory: Trajectory;
  analysis: AnalysisResult;
  safety?: SafetyReport;
}

// プロジェクトに複数保存できる名前付き経路
export interface SavedRoute {
  id: string;
  name: string;
  route: RouteResult;
  waypoints: Waypoint[]; // 再編集できるよう生成時の waypoints も保持
  vehicleId?: string | null; // 生成時の車両（複数台競合判定で車幅に使う）
}

export type StatusLevel = "info" | "success" | "warn" | "error";

// Undo/Redo 用スナップショット（編集対象の状態のみ）
export interface Snapshot {
  waypoints: Waypoint[];
  route: RouteResult | null;
  areas: AreaPoly[];
  activePolygon: XY[];
  importedRoutes: ImportedRoute[];
  selectedId: string | null;
}

export type PlanMode = "auto" | "waypoint_guided";
export type Algorithm = "spline" | "dubins" | "grid_astar" | "hybrid_astar" | "reeds_shepp" | "rrt_star";

// サイドバーの工程（選択中の機能だけを作業パネルに出す）
export type FeatureId =
  | "ai" | "data" | "map" | "route" | "vehicle" | "spotting" | "areas" | "piles" | "fleet" | "project";
