// 生成物（経路・寄り付き）のエクスポート（設計書 Phase 6）。
// CSV は working CRS(6677) のメートル座標。GeoJSON は標準に合わせ WGS84(lng,lat) に変換して出力。

import { api } from "@/api/client";
import { WORKING_EPSG } from "@/map/proj";
import { useStore } from "@/store/useStore";
import type { PilePlanResult, XY } from "@/types/api";

const WGS84 = 4326;

function download(filename: string, text: string, mime: string): void {
  const blob = new Blob([text], { type: mime });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
}

/** data URL（スクリーンキャプチャ等）をファイル保存する。 */
export function downloadDataUrl(filename: string, dataUrl: string): void {
  const a = document.createElement("a");
  a.href = dataUrl;
  a.download = filename;
  a.click();
}

/** キャプチャ等のファイル名用タイムスタンプ（ローカル時刻 YYYYMMDD-HHMMSS）。 */
export function timestampName(prefix: string, ext: string): string {
  const d = new Date();
  const p = (v: number) => String(v).padStart(2, "0");
  return `${prefix}_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}-${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}.${ext}`;
}

async function toLngLat(pts: XY[]): Promise<[number, number][]> {
  if (pts.length === 0) return [];
  const out = await api.transform(pts.map((p) => [p.x, p.y] as [number, number]), WORKING_EPSG, WGS84);
  return out.points as [number, number][]; // [lng, lat]
}

export function csv(rows: (string | number | null)[][]): string {
  return rows
    .map((r) => r.map((v) => (v === null || v === undefined ? "" : String(v))).join(","))
    .join("\n");
}

/** [lng,lat] 座標列に標高 z[m] を第3要素として付与（RFC 7946）。z が1点も無ければ 2D のまま。 */
export function coordsWithZ(coords: [number, number][], zs: (number | null | undefined)[]): number[][] {
  if (!zs.some((z) => z != null)) return coords;
  return coords.map((c, i) => (zs[i] != null ? [c[0], c[1], Number((zs[i] as number).toFixed(3))] : c));
}

// ---- 経路（解析軌跡） ----
export function exportRouteCsv(): boolean {
  const route = useStore.getState().route;
  if (!route || route.trajectory.points.length < 2) return false;
  const header = ["s_m", "x", "y", "z_m", "heading_deg", "curvature_1pm", "dkappa_ds", "grade_pct", "steer_deg", "gear", "speed_mps", "time_s"];
  const rows = route.trajectory.points.map((p) => [
    p.s.toFixed(3), p.x.toFixed(3), p.y.toFixed(3),
    p.z != null ? p.z.toFixed(3) : null,
    p.heading_deg.toFixed(2),
    p.curvature.toFixed(6), p.curvature_rate.toFixed(6),
    p.grade_pct ?? null, p.steer_deg ?? null, p.gear ?? "F",
    p.speed_mps != null ? p.speed_mps.toFixed(3) : null, p.time_s != null ? p.time_s.toFixed(2) : null,
  ]);
  download("route.csv", csv([header, ...rows]), "text/csv");
  return true;
}

export async function exportRouteGeoJson(): Promise<boolean> {
  const route = useStore.getState().route;
  if (!route || route.trajectory.points.length < 2) return false;
  const pts = route.trajectory.points;
  const coords = coordsWithZ(await toLngLat(pts.map((p) => ({ x: p.x, y: p.y }))), pts.map((p) => p.z));
  const fc = {
    type: "FeatureCollection",
    features: [
      {
        type: "Feature",
        properties: {
          kind: "route",
          length_m: route.trajectory.length_m,
          min_radius_m: route.trajectory.min_radius_m,
          feasible: route.analysis.feasible,
          source_crs: `EPSG:${WORKING_EPSG}`,
        },
        geometry: { type: "LineString", coordinates: coords },
      },
    ],
  };
  download("route.geojson", JSON.stringify(fc, null, 2), "application/geo+json");
  return true;
}

// ---- 保存ルートの一括エクスポート（プロジェクトの全ルート。元の名前を引き継ぐ） ----
export function sanitizeFilename(name: string): string {
  return (name || "route").replace(/[\\/:*?"<>|]/g, "_").trim() || "route";
}

function routeFeature(
  name: string,
  vehicleId: string | null | undefined,
  traj: { length_m?: number; min_radius_m?: number | null },
  coords: number[][],
  feasible: boolean | null,
) {
  return {
    type: "Feature",
    properties: {
      kind: "route",
      name, // 元のルート名を引き継ぐ
      vehicle_id: vehicleId ?? null,
      length_m: traj.length_m ?? null,
      min_radius_m: traj.min_radius_m ?? null,
      feasible,
      source_crs: `EPSG:${WORKING_EPSG}`,
    },
    geometry: { type: "LineString", coordinates: coords },
  };
}

/** プロジェクトの全保存ルートを **1つの GeoJSON** にまとめて保存（各 Feature に元のルート名）。返値=出力数。 */
export async function exportAllRoutesGeoJson(): Promise<number> {
  const saved = useStore.getState().savedRoutes;
  const usable = saved.filter((r) => (r.route?.trajectory?.points?.length ?? 0) >= 2);
  if (usable.length === 0) return 0;
  const features: unknown[] = [];
  for (const sr of usable) {
    const t = sr.route.trajectory;
    const coords = coordsWithZ(await toLngLat(t.points.map((p) => ({ x: p.x, y: p.y }))), t.points.map((p) => p.z));
    features.push(routeFeature(sr.name, sr.vehicleId, t, coords, sr.route.analysis?.feasible ?? null));
  }
  download("routes.geojson", JSON.stringify({ type: "FeatureCollection", features }, null, 2), "application/geo+json");
  return usable.length;
}

/** プロジェクトの全保存ルートを **ルートごとの個別ファイル**（ファイル名=元のルート名）で保存。返値=出力数。 */
export async function exportAllRoutesSeparate(): Promise<number> {
  const saved = useStore.getState().savedRoutes;
  const usable = saved.filter((r) => (r.route?.trajectory?.points?.length ?? 0) >= 2);
  for (const sr of usable) {
    const t = sr.route.trajectory;
    const coords = coordsWithZ(await toLngLat(t.points.map((p) => ({ x: p.x, y: p.y }))), t.points.map((p) => p.z));
    const fc = { type: "FeatureCollection", name: sr.name, features: [routeFeature(sr.name, sr.vehicleId, t, coords, sr.route.analysis?.feasible ?? null)] };
    download(`${sanitizeFilename(sr.name)}.geojson`, JSON.stringify(fc, null, 2), "application/geo+json");
    await new Promise((res) => setTimeout(res, 150)); // 連続ダウンロードの取りこぼし回避
  }
  return usable.length;
}

// ---- 排土（パイル配置） ----
/** パイル配置の CSV 行（純粋関数・テスト用に分離）。座標は作業CRS[m]。 */
export function pilesCsvRows(plan: PilePlanResult): (string | number | null)[][] {
  const header = ["no", "x", "y", "height_m", "radius_m", "volume_m3", "repose_deg", "dx_m", "dy_m"];
  const rows = plan.centers.map(([x, y], i) => [
    i + 1, x.toFixed(3), y.toFixed(3),
    plan.pile.height_m, plan.pile.radius_m, plan.pile.volume_m3, plan.pile.repose_deg,
    plan.spacing.dx_m, plan.spacing.dy_m,
  ]);
  return [header, ...rows];
}

export function exportPilesCsv(): boolean {
  const plan = useStore.getState().pilePlan;
  if (!plan || plan.centers.length === 0) return false;
  download("piles.csv", csv(pilesCsvRows(plan)), "text/csv");
  return true;
}

export async function exportPilesGeoJson(): Promise<boolean> {
  const plan = useStore.getState().pilePlan;
  if (!plan || plan.centers.length === 0) return false;
  const coords = await toLngLat(plan.centers.map(([x, y]) => ({ x, y })));
  const features = coords.map((c, i) => ({
    type: "Feature",
    properties: {
      kind: "pile",
      no: i + 1,
      x: plan.centers[i][0],           // 作業CRS座標も残す（現場座標で扱う下流向け）
      y: plan.centers[i][1],
      height_m: plan.pile.height_m,
      radius_m: plan.pile.radius_m,
      volume_m3: plan.pile.volume_m3,
      repose_deg: plan.pile.repose_deg,
      source_crs: `EPSG:${WORKING_EPSG}`,
    },
    geometry: { type: "Point", coordinates: c },
  }));
  const fc = {
    type: "FeatureCollection",
    // 計画メタ（foreign member。QGIS等は無視して読める）
    pile_plan: {
      count: plan.count,
      spacing: plan.spacing,
      grid_angle_deg: plan.grid_angle_deg,
      edge_margin_m: plan.edge_margin_m,
      area_m2: plan.area_m2,
      total_volume_m3: plan.total_volume_m3,
      n_theory: plan.n_theory,
      suggested_spacing_m: plan.suggested_spacing_m,
    },
    features,
  };
  download("piles.geojson", JSON.stringify(fc, null, 2), "application/geo+json");
  return true;
}

// ---- エリア（多角形） ----
export async function exportAreasGeoJson(): Promise<boolean> {
  const areas = useStore.getState().areas.filter((a) => a.points.length >= 3);
  if (areas.length === 0) return false;
  const features: unknown[] = [];
  for (const a of areas) {
    const ring = await toLngLat(a.points);
    ring.push(ring[0]);
    features.push({
      type: "Feature",
      properties: { kind: "area", name: a.name, source_crs: `EPSG:${WORKING_EPSG}` },
      geometry: { type: "Polygon", coordinates: [ring] },
    });
  }
  download("areas.geojson", JSON.stringify({ type: "FeatureCollection", features }, null, 2), "application/geo+json");
  return true;
}

// ---- 寄り付き（spotting） ----
// 寄り付きの標高 z は解析軌跡（trajectory: 同一点列から構築＝index 対応）から引く。
function spotZ(sr: { points: unknown[]; trajectory?: { points: { z?: number | null }[] } }): (number | null)[] {
  const tp = sr.trajectory?.points;
  if (!tp || tp.length !== sr.points.length) return sr.points.map(() => null);
  return tp.map((p) => p.z ?? null);
}

export function exportSpottingCsv(): boolean {
  const sr = useStore.getState().spotResult;
  if (!sr || sr.points.length < 2) return false;
  const zs = spotZ(sr);
  const header = ["s_m", "t_s", "x", "y", "z_m", "heading_deg", "gear"];
  const rows = sr.points.map((p, i) => [
    p.s.toFixed(3), p.t.toFixed(2), p.x.toFixed(3), p.y.toFixed(3),
    zs[i] != null ? (zs[i] as number).toFixed(3) : null,
    p.heading_deg.toFixed(2), p.gear,
  ]);
  download("spotting.csv", csv([header, ...rows]), "text/csv");
  return true;
}

export async function exportSpottingGeoJson(): Promise<boolean> {
  const sr = useStore.getState().spotResult;
  if (!sr || sr.points.length < 2) return false;
  const coords = coordsWithZ(await toLngLat(sr.points.map((p) => ({ x: p.x, y: p.y }))), spotZ(sr));
  const switchCoords = await toLngLat(sr.switch_points);
  const features: unknown[] = [
    {
      type: "Feature",
      properties: {
        kind: "spotting",
        length_total_m: sr.metrics.length_total_m,
        length_rev_m: sr.metrics.length_rev_m,
        n_switchbacks: sr.metrics.n_switchbacks,
        time_total_s: sr.metrics.time_total_s,
        source_crs: `EPSG:${WORKING_EPSG}`,
      },
      geometry: { type: "LineString", coordinates: coords },
    },
    ...switchCoords.map((c) => ({
      type: "Feature",
      properties: { kind: "switchback" },
      geometry: { type: "Point", coordinates: c },
    })),
  ];
  download("spotting.geojson", JSON.stringify({ type: "FeatureCollection", features }, null, 2), "application/geo+json");
  return true;
}
