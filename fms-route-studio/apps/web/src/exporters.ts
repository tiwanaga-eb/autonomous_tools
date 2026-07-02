// 生成物（経路・寄り付き）のエクスポート（設計書 Phase 6）。
// CSV は working CRS(6677) のメートル座標。GeoJSON は標準に合わせ WGS84(lng,lat) に変換して出力。

import { api } from "@/api/client";
import { WORKING_EPSG } from "@/map/proj";
import { useStore } from "@/store/useStore";
import type { XY } from "@/types/api";

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
