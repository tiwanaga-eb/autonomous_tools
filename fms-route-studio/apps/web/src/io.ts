// JSON 入出力（旧 map_tool の haulRoutes/polygons 形式と後方互換）。
// 内部は working CRS(6677)。出力は WGS84(lat/lng) + points_xy(6677) を両方持つ。

import { api } from "@/api/client";
import { dispatch } from "@/commandBus";
import { WORKING_EPSG } from "@/map/proj";
import { useStore } from "@/store/useStore";
import type { XY } from "@/types/api";

const WGS84 = 4326;

interface LL {
  lat: number;
  lng: number;
}
interface ProjectJson {
  haulRoutes?: { name: string; route: LL[] }[];
  polygons?: { name: string; points?: LL[]; points_xy?: { x: number; y: number }[] }[];
}

async function xyToLatLng(pts: XY[]): Promise<LL[]> {
  if (pts.length === 0) return [];
  const out = await api.transform(
    pts.map((p) => [p.x, p.y] as [number, number]),
    WORKING_EPSG,
    WGS84,
  );
  return out.points.map(([lng, lat]) => ({ lat, lng }));
}

async function latLngToXy(ll: LL[]): Promise<XY[]> {
  if (ll.length === 0) return [];
  const out = await api.transform(
    ll.map((p) => [p.lng, p.lat] as [number, number]),
    WGS84,
    WORKING_EPSG,
  );
  return out.points.map(([x, y]) => ({ x, y }));
}

export async function exportProjectJson(): Promise<void> {
  const s = useStore.getState();
  const haulRoutes: ProjectJson["haulRoutes"] = [];
  if (s.route && s.route.trajectory.points.length > 1) {
    const xy = s.route.trajectory.points.map((p) => ({ x: p.x, y: p.y }));
    haulRoutes.push({ name: "Route_1", route: await xyToLatLng(xy) });
  }
  for (const r of s.importedRoutes) {
    haulRoutes.push({ name: r.name, route: await xyToLatLng(r.pts) });
  }

  const polygons: ProjectJson["polygons"] = [];
  for (const a of s.areas) {
    polygons.push({
      name: a.name,
      points: await xyToLatLng(a.points),
      points_xy: a.points.map((p) => ({ x: p.x, y: p.y })),
    });
  }

  const payload: ProjectJson = { haulRoutes, polygons };
  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = "fms_route_studio.json";
  a.click();
  URL.revokeObjectURL(url);
}

export async function importProjectJson(file: File): Promise<{ areas: number; routes: number }> {
  const data = JSON.parse(await file.text()) as ProjectJson;

  const areas: { name: string; points: XY[] }[] = [];
  for (const p of data.polygons ?? []) {
    let pts: XY[] = [];
    if (p.points_xy && p.points_xy.length) {
      pts = p.points_xy.map((q) => ({ x: q.x, y: q.y }));
    } else if (p.points && p.points.length) {
      pts = await latLngToXy(p.points);
    }
    if (pts.length >= 3) areas.push({ name: p.name ?? "Area", points: pts });
  }

  const routes: { name: string; pts: XY[] }[] = [];
  for (const r of data.haulRoutes ?? []) {
    const pts = await latLngToXy(r.route ?? []);
    if (pts.length >= 2) routes.push({ name: r.name ?? "Route", pts });
  }

  if (areas.length) dispatch({ type: "LOAD_AREAS", areas });
  dispatch({ type: "SET_IMPORTED", routes });
  return { areas: areas.length, routes: routes.length };
}
