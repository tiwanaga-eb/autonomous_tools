// バックエンド(planning_core モデル)に対応する型。Phase 4 で OpenAPI 自動生成へ置換予定。

export interface XY {
  x: number;
  y: number;
}

export interface Vehicle {
  id: string;
  name: string;
  kinematic_type: "rigid_bicycle" | "articulated" | "tracked_skid";
  min_turning_radius: number | null;
  overall_width: number;
  overall_length: number;
  road_width: number | null;
  spec_status: "measured" | "estimated";
  overridden?: boolean;
}

export interface Layer {
  id: string;
  kind: string;
  version: number;
  filename?: string;
  cog?: string;
  epsg?: number;
  crs_source?: "detected" | "assigned";
  width?: number;
  height?: number;
  bands?: number;
  geographic_bounds?: [number, number, number, number]; // [west, south, east, north]
  // costmap 生成時のメタ
  density_pts_m2?: number;
  pts_per_cell?: number;
  recommended_grid_m?: number;
  warning?: string | null;
  // drivable レイヤのメタ
  cost_layer_id?: string;
  edits?: { op: "include" | "exclude"; polygon: [number, number][] }[];
  stats?: {
    area_m2: number;
    island_count: number;
    hole_count?: number;
    largest_island_m2: number;
    width_median_m?: number;
    width_max_m?: number;
  };
}

export interface TrajPoint {
  s: number;
  x: number;
  y: number;
  z?: number | null; // 標高[m]（点群由来DSMからサンプル。DSM無し/範囲外は null）
  heading_deg: number;
  curvature: number;
  curvature_rate: number;
  grade_pct: number | null;
  steer_deg: number | null;
  gear: "F" | "R" | null;
  speed_mps?: number | null;
  time_s?: number | null;
}

export interface Trajectory {
  points: TrajPoint[];
  length_m: number;
  min_radius_m: number | null;
  curvature_source: "analytic" | "numeric";
}

export interface Violation {
  kind: string;
  s_start: number;
  s_end: number;
  measured: number;
  limit: number;
}

export interface AnalysisResult {
  min_radius_m: number | null;
  max_curvature: number;
  max_curvature_rate: number;
  max_steer_rate_required: number | null;
  max_grade_pct: number | null;
  feasible: boolean;
  not_applicable: string[];
  violations: Violation[];
  max_speed_mps?: number | null;
  time_total_s?: number | null;
  stopping_distance_m?: number | null;
}

export interface SpotPose {
  x: number;
  y: number;
  heading_deg: number;
}

export interface SpotPoint {
  x: number;
  y: number;
  heading_deg: number;
  gear: "F" | "R";
  s: number;
  t: number;
}

// 寄り付き候補生成アルゴリズム。auto=全手法をコスト比較 / 個別手法。
export type SpottingMethod = "auto" | "dubins" | "reeds_shepp" | "hybrid_astar";

// 複数台制御(Phase B): 経路の重なり/競合
export interface FleetConflictInterval {
  s_start: number;
  s_end: number;
}
export interface FleetConflict {
  a: number; // routes 配列のインデックス
  b: number;
  kind: string; // "crossing"(交差) / "shared"(区間共有)
  overlap_area_m2: number;
  a_intervals: FleetConflictInterval[];
  b_intervals: FleetConflictInterval[];
  bbox: { minx: number; miny: number; maxx: number; maxy: number };
}

// すれ違い点（待避所 / passing bay）の設定（Phase C）。経路ごとに最大1つ。
export interface BayCfg {
  s_frac: number;   // 経路上の中心位置（弧長割合 0..1）
  offset_m: number; // 横退避量[m]
  side: 1 | -1;     // +1=左 / -1=右
  ramp_m: number;   // 出入りの遷移長[m]
  hold_m: number;   // 退避平坦長[m]
}

// 複数台 簡易シミュレーション(Phase D)
export interface FleetSimFrame {
  t: number;
  s: number;
  x: number;
  y: number;
  heading_deg: number;
  v: number;
  state: "run" | "wait" | "done";
}
export interface FleetSimResult {
  status: string;
  deadlock: boolean;
  deadlock_time_s: number | null;
  collision: boolean;
  min_separation_m: number | null;
  min_sep_time_s: number | null;
  makespan_s: number;
  traces: FleetSimFrame[][]; // 車両ごとの時系列
  events: { t: number; vehicle: number; type: string; zone: unknown }[];
  auto_bays?: { vehicle: number; s_center: number; offset: number; side: number }[];
  total_wait_s?: number;
  wait_time_s?: number[];
  travel_time_s?: number[];
  names: string[];
}

export interface SpotMetrics {
  length_total_m: number;
  length_fwd_m: number;
  length_rev_m: number;
  time_total_s: number;
  n_switchbacks: number;
  min_clearance_m: number | null;
  approach_error_m: number;
  approach_error_deg?: number | null; // 目標方位との到達誤差[°]（P-008 合否は safety 側）
  cost_integral: number;
  score: number;
  footprint_inside?: boolean | null;
  footprint_max_overhang_frac?: number | null;
}

export interface SpotWeights {
  w_distance: number;
  w_time: number;
  w_reverse: number;
  w_switchback: number;
  w_costmap: number;
  w_turn: number;
}

export interface SpotResult {
  points: SpotPoint[];
  switch_points: XY[];
  metrics: SpotMetrics;
  feasible: boolean;
  status: string;
  rho_m: number;
  method?: SpottingMethod;
  reason?: string | null;
  exit?: SpotResult | null;
  // 据え切り診断: 解決後の可否と、端点に入れた直線リードイン/アウト長[m]（0=据え切り）
  allow_stationary?: boolean;
  endpoint_margin_start_m?: number;
  endpoint_margin_goal_m?: number;
  // 経路と同じ軌跡解析＋安全検証（寄り付きにも付与）
  trajectory?: Trajectory;
  analysis?: AnalysisResult;
  safety?: SafetyReport;
}

export interface SafetyCheck {
  name: string;
  label: string;
  ok: boolean;
  applicable: boolean;
  measured: number | null;
  limit: number | null;
  detail: string;
}

export interface SafetyReport {
  passed: boolean;
  checks: SafetyCheck[];
  reasons: string[];
}

export interface PlanResponse {
  trajectory: Trajectory;
  analysis: AnalysisResult;
  safety?: SafetyReport;
  used_smoothing_s: number;
  measured_min_radius_m: number;
  min_clearance_m?: number | null;
  warning: string | null;
  refined_elastic_band?: boolean;
}
