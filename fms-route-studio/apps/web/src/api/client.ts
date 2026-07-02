import type {
  AnalysisResult,
  Layer,
  PlanResponse,
  SpotResult,
  Trajectory,
  Vehicle,
  XY,
} from "@/types/api";

// API エラー。FastAPI の {detail: ...} を人間可読メッセージへ整形し、status/detail も保持する。
export class ApiError extends Error {
  readonly status: number;
  readonly detail: unknown;
  constructor(status: number, message: string, detail: unknown) {
    super(message);
    this.name = "ApiError";
    this.status = status;
    this.detail = detail;
  }
}

async function toApiError(r: Response): Promise<ApiError> {
  const raw = await r.text().catch(() => "");
  let detail: unknown = raw;
  let msg = raw;
  try {
    const j = JSON.parse(raw);
    if (j && typeof j === "object" && "detail" in j) {
      detail = (j as { detail: unknown }).detail;
      if (typeof detail === "string") msg = detail;
      else if (Array.isArray(detail))
        msg = detail.map((d) => (d && typeof d === "object" && "msg" in d ? String((d as { msg: unknown }).msg) : JSON.stringify(d))).join("; ");
      else msg = JSON.stringify(detail);
    }
  } catch {
    /* not JSON — raw text のまま */
  }
  return new ApiError(r.status, msg || `HTTP ${r.status}`, detail);
}

interface ReqOpts {
  body?: unknown;
  signal?: AbortSignal;
}

// 全 JSON リクエストの単一入口。r.ok 検査・detail 整形・空ボディ(204)・abort を一元化。
async function request<T>(method: string, url: string, opts: ReqOpts = {}): Promise<T> {
  const init: RequestInit = { method, signal: opts.signal };
  if (opts.body !== undefined) {
    init.headers = { "Content-Type": "application/json" };
    init.body = JSON.stringify(opts.body);
  }
  const r = await fetch(url, init);
  if (!r.ok) throw await toApiError(r);
  if (r.status === 204) return undefined as T;
  const text = await r.text();
  return (text ? JSON.parse(text) : undefined) as T;
}

const jget = <T>(url: string, signal?: AbortSignal): Promise<T> => request<T>("GET", url, { signal });
const jpost = <T>(url: string, body: unknown, signal?: AbortSignal): Promise<T> =>
  request<T>("POST", url, { body, signal });
const jpatch = <T>(url: string, body: unknown): Promise<T> => request<T>("PATCH", url, { body });
const jput = <T>(url: string, body: unknown): Promise<T> => request<T>("PUT", url, { body });
const jdelete = (url: string): Promise<void> => request<void>("DELETE", url);

export interface DrivableParams {
  threshold: number;
  close_m: number;
  open_m: number;
  min_area_m2: number;
  clearance_m: number;
  smooth_m: number;       // 境界平滑（メディアン）半径[m]
  max_hole_m2: number;    // この面積以下の閉じ穴を埋める[m²]（0=埋めない）
  keep_largest: boolean;  // 最大の連結領域のみ残す
  method: "threshold" | "otsu" | "adaptive";  // 生成方法（otsu/adaptive=OpenCV自動セグメント）
}

export type PlanMode = "auto" | "waypoint_guided";
export type Algorithm = "spline" | "dubins" | "grid_astar" | "hybrid_astar" | "reeds_shepp" | "rrt_star";

// AI アシスタント（§14）。サーバの tool-use が返す「操作」を FE が適用する。
export interface AgentAction {
  type: string;
  [k: string]: unknown;
}
export interface AgentStep {
  tool: string;
  input: Record<string, unknown>;
  ok: boolean;
}
export interface AgentChatResponse {
  reply: string;
  actions: AgentAction[];
  steps: AgentStep[];
}
export interface AgentChatMessage {
  role: "user" | "assistant";
  content: string;
}

export interface PlanBody {
  waypoints: (XY & { heading_deg?: number | null })[];
  mode?: PlanMode;
  algorithm?: Algorithm;
  min_turn_radius_m?: number | null;
  spacing_m?: number;
  corridor_width_m?: number | null;
  enforce_footprint?: boolean;
  enforce_min_radius?: boolean;
  allow_reverse?: boolean;
  refine_elastic_band?: boolean;
  vehicle_id?: string | null;
  costmap_layer_id?: string | null;
  drivable_layer_id?: string | null;
  no_go_polygons?: [number, number][][];
}

export const api = {
  health: () => jget<{ status: string; default_epsg: number; zones?: number[] }>("/health"),

  // 作業ゾーン(投影座標系)を実行時に変更（セッション内・backendメモリ保持）。以後のアップロードはこのEPSGへ。
  setWorkingCrs: (epsg: number) => jput<{ default_epsg: number }>("/api/crs", { epsg }),

  listLayers: () => jget<{ layers: Layer[] }>("/api/layers").then((d) => d.layers),

  async uploadLayer(kind: string, file: File): Promise<Layer> {
    const fd = new FormData();
    fd.append("file", file);
    const r = await fetch(`/api/layers/${kind}`, { method: "POST", body: fd });
    if (!r.ok) throw await toApiError(r);
    return (await r.json()) as Layer;
  },

  deleteLayer: (id: string) => jdelete(`/api/layers/${id}`),

  tileUrlTemplate: (id: string) => `/api/tiles/${id}/{z}/{x}/{y}.png`,
  previewUrl: (id: string) => `/api/layers/${id}/preview.png`,
  cogUrl: (id: string, version = 0) => `/api/layers/${id}/cog.tif${version ? `?v=${version}` : ""}`,

  generateDrivable: (cost_layer_id: string, params: DrivableParams) =>
    jpost<Layer>("/api/drivable", { cost_layer_id, params }),
  regenerateDrivable: (id: string, params: DrivableParams) =>
    jpost<Layer>(`/api/drivable/${id}/regenerate`, params),
  editDrivable: (id: string, op: "include" | "exclude", polygon: [number, number][]) =>
    jpatch<Layer>(`/api/drivable/${id}`, { op, polygon }),
  deleteDrivableEdit: (id: string, index: number) =>
    request<Layer>("DELETE", `/api/drivable/${id}/edits/${index}`),
  clearDrivableEdits: (id: string) => request<Layer>("DELETE", `/api/drivable/${id}/edits`),

  transform: (points: [number, number][], srcEpsg: number, dstEpsg: number) =>
    jpost<{ points: [number, number][] }>("/api/geo/transform", {
      points,
      src_epsg: srcEpsg,
      dst_epsg: dstEpsg,
    }),

  generateCostmap: (body: {
    las_layer_id: string;
    params?: Record<string, number>;
    src_epsg?: number;
    target_epsg?: number;
  }) => jpost<Layer>("/api/costmap", body),

  plan: (body: PlanBody) => jpost<PlanResponse>("/api/plan", body),

  analyze: (body: { points: XY[]; vehicle_id?: string | null; costmap_layer_id?: string | null }) =>
    jpost<{ trajectory: Trajectory; analysis: AnalysisResult }>("/api/analyze", body),

  // 任意の点列（保存済みルート等）に点群由来 DSM の標高 z[m] を後付けサンプリング。
  elevationSample: (points: [number, number][], costmapLayerId?: string | null, smoothM?: number) =>
    jpost<{ z: (number | null)[]; layer_id: string; n: number; n_missing: number }>("/api/elevation/sample", {
      points: points.map(([x, y]) => ({ x, y })),
      costmap_layer_id: costmapLayerId ?? null,
      ...(smoothM != null ? { smooth_m: smoothM } : {}),
    }),

  listVehicles: () => jget<Vehicle[]>("/api/vehicles"),

  vehicleDetail: (id: string) =>
    jget<{
      effective: Record<string, unknown>;
      default: Record<string, unknown>;
      override: Record<string, number>;
      editable_fields: string[];
    }>(`/api/vehicles/${id}/detail`),
  saveVehicleOverride: (id: string, fields: Record<string, number>) =>
    jput<{ effective: Record<string, unknown>; override: Record<string, number> }>(`/api/vehicles/${id}`, { fields }),
  resetVehicleOverride: (id: string) => jdelete(`/api/vehicles/${id}/override`),

  layerPoints: (lasLayerId: string, maxPoints = 200000, signal?: AbortSignal) =>
    jget<{
      n: number; x: number[]; y: number[]; z: number[];
      has_rgb: boolean; rgb: number[][] | null; zmin: number; zmax: number;
    }>(`/api/layers/${lasLayerId}/points?max_points=${maxPoints}`, signal),

  dsmGrid: (costLayerId: string, maxSize = 160, signal?: AbortSignal) =>
    jget<{
      nx: number; ny: number; x0: number; y0: number; dx: number; dy: number;
      z: (number | null)[][]; zmin: number; zmax: number;
    }>(`/api/costmap/${costLayerId}/dsm_grid?max_size=${maxSize}`, signal),

  listProjects: () => jget<{ id: string; name: string; updated_at: string | null }[]>("/api/projects"),
  getProject: (id: string) => jget<{ id: string; name: string; state: Record<string, unknown> }>(`/api/projects/${id}`),
  createProject: (name: string, state: Record<string, unknown>) =>
    jpost<{ id: string; name: string }>("/api/projects", { name, state, updated_at: new Date().toISOString() }),
  updateProject: (id: string, name: string, state: Record<string, unknown>) =>
    jput<{ id: string; name: string }>(`/api/projects/${id}`, { name, state, updated_at: new Date().toISOString() }),
  deleteProject: (id: string) => jdelete(`/api/projects/${id}`),

  simulateSpotting: (body: {
    start: { x: number; y: number; heading_deg: number };
    target: { x: number; y: number; heading_deg: number };
    max_switchbacks?: number | null;
    require_switchback?: boolean;
    min_turn_radius_m?: number | null;
    road_width_m?: number | null;
    vehicle_id?: string | null;
    drivable_layer_id?: string | null;
    costmap_layer_id?: string | null;
    no_go_polygons?: [number, number][][];
    with_exit?: boolean;
    exit_goal?: { x: number; y: number; heading_deg: number } | null;
    manual_switch_pose?: { x: number; y: number; heading_deg: number } | null;
    switchback_zone?: [number, number][] | null;
    containment_polygon?: [number, number][] | null;
    cusp_margin_m?: number | null;
    method?: import("@/types/api").SpottingMethod;
    smooth_path?: boolean;
    allow_stationary_steer?: boolean | null;
    min_speed_kmh?: number;
    weights?: import("@/types/api").SpotWeights;
  }) => jpost<SpotResult>("/api/simulate/spotting", body),

  agentStatus: () =>
    jget<{ available: boolean; provider: string | null; model: string | null; detail?: string | null }>(
      "/api/agent/status",
    ),
  agentChat: (messages: AgentChatMessage[], context: Record<string, unknown>) =>
    jpost<AgentChatResponse>("/api/agent/chat", { messages, context }),

  // 複数台制御(Phase B): 経路の重なり/競合判定
  fleetConflicts: (body: {
    routes: { name?: string; points: [number, number][]; vehicle_id?: string | null; half_width_m?: number | null }[];
    cell_m?: number;
    clearance_m?: number;
  }) =>
    jpost<{ conflicts: import("@/types/api").FleetConflict[]; n_routes: number; n_conflicts: number }>(
      "/api/fleet/conflicts",
      body,
    ),

  fleetSimulate: (body: {
    routes: { name?: string; points: [number, number][]; vehicle_id?: string | null; half_width_m?: number | null; priority?: number; start_time_s?: number; v_max_mps?: number | null }[];
    dt_s?: number;
    gap_m?: number;
    clearance_m?: number;
    max_time_s?: number;
    auto_passing?: boolean;
    dispatch?: "simultaneous" | "sequential";
    loops?: number;
  }) => jpost<import("@/types/api").FleetSimResult>("/api/fleet/simulate", body),

  // 分岐起点姿勢（親経路上の点の接線姿勢）
  fleetJunction: (points: [number, number][], s_frac: number) =>
    jpost<{ x: number; y: number; heading_deg: number }>("/api/fleet/junction", { points, s_frac }),
};
