import { create } from "zustand";

import type { AnalysisResult, BayCfg, FleetConflict, FleetSimResult, Layer, SafetyReport, SpotPose, SpotResult, SpottingMethod, SpotWeights, Trajectory, XY } from "@/types/api";

export type EditMode =
  | "start"
  | "goal"
  | "via"
  | "insert_via"
  | "edit"
  | "pan"
  | "polygon"
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
interface Snapshot {
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
export type FeatureId = "ai" | "data" | "map" | "route" | "vehicle" | "spotting" | "areas" | "fleet" | "project";

export interface AppState extends Snapshot {
  layers: Layer[];
  activeFeature: FeatureId; // サイドバーで選択中の工程
  mode: EditMode;
  status: string;
  statusLevel: StatusLevel; // 状態メッセージの重要度（色分け表示用）
  busy: boolean;            // 長時間処理中（オーバーレイ/ボタン無効化）
  busyLabel: string;        // 処理中ラベル
  savedRoutes: SavedRoute[]; // プロジェクトに保存した経路ライブラリ（複数台制御の経路集合）
  showSavedRoutes: boolean;  // 保存経路ライブラリを地図に重ねて表示（複数経路の可視化）
  fleetConflicts: FleetConflict[] | null; // 直近の経路競合判定結果（savedRoutes の index 参照）
  fleetSim: FleetSimResult | null; // 直近の簡易シミュレーション結果
  fleetSimT: number; // 再生中の現在時刻[s]
  fleetBays: Record<string, BayCfg>; // 待避所（すれ違い点）。savedRoute.id → 設定
  costOpacity: number; // コスト オーバーレイの不透明度(0..1)
  costVisible: boolean; // コスト オーバーレイの表示
  drivableOpacity: number; // 走行可能領域 オーバーレイの不透明度(0..1)
  drivableVisible: boolean; // 走行可能領域 オーバーレイの表示
  osmVisible: boolean; // OpenStreetMap ベースマップ（地理的文脈）の表示
  routeSpacing: number; // 経路リサンプル間隔[m]
  roadWidthM: number; // 道幅[m]（0=帯を描かない）
  enforceFootprint: boolean; // hybrid A* で実車体フットプリントを厳密な走行可能制約に
  enforceMinRadius: boolean; // R_min を持つ車種で最小旋回半径を保証（違反コーナーを局所平滑化）
  allowReverse: boolean; // hybrid A* で後進を許可（切り返しで狭所到達性UP）
  refineElasticBand: boolean; // 生成後に Elastic Band で洗練（中央寄せ・余裕確保）
  showWaypoints: boolean; // 経路Waypoint(点)の表示
  hoverPointIndex: number | null; // Analysisグラフでホバー中の軌跡点index（地図と同期）
  view3d: boolean; // 中央ビューを 2D地図 / 3D に切替
  view3dMode: "points" | "mesh"; // 3Dの表示: LAS点群 / DSM地形メッシュ
  pointSize: number; // 点群の点サイズ[px]
  vExag: number;   // 3Dの鉛直強調倍率
  // ---- 寄り付きシミュレータ（§13）----
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
  vehicleId: string | null; // 選択中の車両プロファイル
  costLayerId: string | null; // 明示選択中のコスト層（null=kind最新）。プロジェクトで固定/復元
  drivableLayerId: string | null; // 明示選択中の走行可能領域層（null=kind最新）
  vehiclesRev: number; // 車両パラメータ更新の世代（編集後に consumer を再取得させる）
  planMode: PlanMode; // auto / waypoint_guided
  algorithm: Algorithm; // spline / dubins / grid_astar
  undoStack: Snapshot[];
  redoStack: Snapshot[];

  setLayers: (layers: Layer[]) => void;
  setActiveFeature: (f: FeatureId) => void;
  setMode: (mode: EditMode) => void;
  setCostOpacity: (v: number) => void;
  setCostVisible: (v: boolean) => void;
  setDrivableOpacity: (v: number) => void;
  setDrivableVisible: (v: boolean) => void;
  setOsmVisible: (v: boolean) => void;
  setRouteSpacing: (v: number) => void;
  setRoadWidthM: (v: number) => void;
  setEnforceFootprint: (v: boolean) => void;
  setEnforceMinRadius: (v: boolean) => void;
  setAllowReverse: (v: boolean) => void;
  setRefineElasticBand: (v: boolean) => void;
  setShowWaypoints: (v: boolean) => void;
  setHoverPoint: (i: number | null) => void;
  setView3d: (v: boolean) => void;
  setView3dMode: (v: "points" | "mesh") => void;
  setPointSize: (v: number) => void;
  setVExag: (v: number) => void;
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
  setVehicleId: (v: string | null) => void;
  setCostLayerId: (v: string | null) => void;
  setDrivableLayerId: (v: string | null) => void;
  bumpVehiclesRev: () => void;
  setPlanMode: (v: PlanMode) => void;
  setAlgorithm: (v: Algorithm) => void;
  setWaypoints: (waypoints: Waypoint[]) => void;
  setRoute: (route: RouteResult | null) => void;
  setAreas: (areas: AreaPoly[]) => void;
  setActivePolygon: (pts: XY[]) => void;
  setImportedRoutes: (routes: ImportedRoute[]) => void;
  select: (id: string | null) => void;
  setStatus: (status: string, level?: StatusLevel) => void;
  setBusy: (active: boolean, label?: string) => void;
  setSavedRoutes: (routes: SavedRoute[]) => void;
  setShowSavedRoutes: (v: boolean) => void;
  setFleetConflicts: (c: FleetConflict[] | null) => void;
  setFleetSim: (s: FleetSimResult | null) => void;
  setFleetSimT: (t: number) => void;
  setFleetBay: (routeId: string, cfg: BayCfg | null) => void;
  setFleetBays: (bays: Record<string, BayCfg>) => void; // 一括置換（プロジェクト読込で残留を掃除）

  commit: () => void; // 変更前スナップショットを履歴に積む
  undo: () => void;
  redo: () => void;
  resetRoute: () => void;
}

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

export const useStore = create<AppState>((set) => ({
  layers: [],
  activeFeature: "data",
  mode: "start",
  status: "ready",
  statusLevel: "info",
  busy: false,
  busyLabel: "",
  savedRoutes: [],
  showSavedRoutes: true,
  fleetConflicts: null,
  fleetSim: null,
  fleetSimT: 0,
  fleetBays: {},
  costOpacity: 0.6,
  costVisible: true,
  drivableOpacity: 0.5,
  drivableVisible: true,
  osmVisible: false,
  routeSpacing: 2.0,
  roadWidthM: 0,
  enforceFootprint: true,
  enforceMinRadius: true,
  allowReverse: false,
  refineElasticBand: false,
  showWaypoints: true,
  hoverPointIndex: null,
  view3d: false,
  view3dMode: "points",
  pointSize: 2.0,
  vExag: 1.0,
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
  vehicleId: "HD785",
  costLayerId: null,
  drivableLayerId: null,
  vehiclesRev: 0,
  planMode: "waypoint_guided",
  algorithm: "spline",
  waypoints: [],
  route: null,
  areas: [],
  activePolygon: [],
  importedRoutes: [],
  selectedId: null,
  undoStack: [],
  redoStack: [],

  setLayers: (layers) => set({ layers }),
  // 工程を切り替えたら編集モードを中立(pan)へ戻す（前工程のモードが地図クリックに漏れるのを防ぐ）
  setActiveFeature: (activeFeature) =>
    set((s) => (s.activeFeature === activeFeature ? { activeFeature } : { activeFeature, mode: "pan" })),
  setMode: (mode) => set({ mode }),
  setCostOpacity: (costOpacity) => set({ costOpacity }),
  setCostVisible: (costVisible) => set({ costVisible }),
  setDrivableOpacity: (drivableOpacity) => set({ drivableOpacity }),
  setDrivableVisible: (drivableVisible) => set({ drivableVisible }),
  setOsmVisible: (osmVisible) => set({ osmVisible }),
  setRouteSpacing: (routeSpacing) => set({ routeSpacing }),
  setRoadWidthM: (roadWidthM) => set({ roadWidthM }),
  setEnforceFootprint: (enforceFootprint) => set({ enforceFootprint }),
  setEnforceMinRadius: (enforceMinRadius) => set({ enforceMinRadius }),
  setAllowReverse: (allowReverse) => set({ allowReverse }),
  setRefineElasticBand: (refineElasticBand) => set({ refineElasticBand }),
  setShowWaypoints: (showWaypoints) => set({ showWaypoints }),
  setHoverPoint: (hoverPointIndex) => set({ hoverPointIndex }),
  setView3d: (view3d) => set({ view3d }),
  setView3dMode: (view3dMode) => set({ view3dMode }),
  setPointSize: (pointSize) => set({ pointSize }),
  setVExag: (vExag) => set({ vExag }),
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
  setVehicleId: (vehicleId) => set({ vehicleId }),
  setCostLayerId: (costLayerId) => set({ costLayerId }),
  setDrivableLayerId: (drivableLayerId) => set({ drivableLayerId }),
  bumpVehiclesRev: () => set((s) => ({ vehiclesRev: s.vehiclesRev + 1 })),
  setPlanMode: (planMode) => set({ planMode }),
  setAlgorithm: (algorithm) => set({ algorithm }),
  setWaypoints: (waypoints) => set({ waypoints }),
  setRoute: (route) => set({ route }),
  setAreas: (areas) => set({ areas }),
  setActivePolygon: (activePolygon) => set({ activePolygon }),
  setImportedRoutes: (importedRoutes) => set({ importedRoutes }),
  select: (selectedId) => set({ selectedId }),
  setStatus: (status, level = "info") => set({ status, statusLevel: level }),
  setBusy: (busy, busyLabel = "") => set({ busy, busyLabel }),
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
}));
