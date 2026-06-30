// Command Bus — UI と（将来の）AI が共有する単一操作面（設計書 §7.3 / §14）。
// 全状態変更はここを通す。履歴(Undo/Redo)もここで commit する。

import { useStore } from "@/store/useStore";
import type { AreaPoly, EditMode, ImportedRoute, RouteResult, Waypoint } from "@/store/useStore";
import type { XY } from "@/types/api";

export type Command =
  | { type: "SET_MODE"; mode: EditMode }
  | { type: "ADD_WAYPOINT"; role: "start" | "goal" | "via"; xy: XY; heading_deg?: number | null }
  | { type: "SET_WAYPOINTS"; waypoints: { x: number; y: number; role: "start" | "via" | "goal"; heading_deg?: number | null }[] }
  | { type: "SET_WAYPOINT_HEADING"; id: string; heading_deg: number | null }
  | { type: "INSERT_VIA"; xy: XY }
  | { type: "MOVE_WAYPOINT"; id: string; xy: XY }
  | { type: "DELETE_WAYPOINT"; id: string }
  | { type: "SET_ROUTE"; route: RouteResult | null }
  | { type: "ADD_POLY_VERTEX"; xy: XY }
  | { type: "UNDO_POLY_VERTEX" }
  | { type: "FINISH_POLYGON"; name: string }
  | { type: "CANCEL_POLYGON" }
  | { type: "DELETE_AREA"; id: string }
  | { type: "SET_AREA_POINTS"; id: string; points: XY[] }
  | { type: "LOAD_AREAS"; areas: { name: string; points: XY[] }[] }
  | { type: "SET_IMPORTED"; routes: { name: string; pts: XY[] }[] }
  | { type: "SELECT"; id: string | null }
  | { type: "RESET_ROUTE" }
  | { type: "UNDO" }
  | { type: "REDO" };

let _seq = 0;
const nid = (prefix: string) => `${prefix}_${(++_seq).toString(36)}_${Math.floor(performance.now())}`;

function roleOrder(role: Waypoint["role"]): number {
  return role === "start" ? 0 : role === "goal" ? 2 : 1;
}

function dist2ToSegment(p: XY, a: XY, b: XY): number {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const den = abx * abx + aby * aby;
  if (den < 1e-12) return (p.x - a.x) ** 2 + (p.y - a.y) ** 2;
  let t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / den;
  t = Math.max(0, Math.min(1, t));
  const qx = a.x + t * abx;
  const qy = a.y + t * aby;
  return (p.x - qx) ** 2 + (p.y - qy) ** 2;
}

export function dispatch(cmd: Command): void {
  const s = useStore.getState();

  // 履歴に積まない（モード/選択/Undo/Redo）以外は commit
  const noHistory =
    cmd.type === "SET_MODE" ||
    cmd.type === "SELECT" ||
    cmd.type === "UNDO" ||
    cmd.type === "REDO";
  if (!noHistory) s.commit();

  switch (cmd.type) {
    case "SET_MODE":
      s.setMode(cmd.mode);
      break;

    case "ADD_WAYPOINT": {
      let wp = [...s.waypoints];
      if (cmd.role === "start") wp = wp.filter((w) => w.role !== "start");
      if (cmd.role === "goal") wp = wp.filter((w) => w.role !== "goal");
      wp.push({ id: nid("wp"), role: cmd.role, xy: cmd.xy, heading_deg: cmd.heading_deg ?? null });
      wp = wp
        .map((w, i) => ({ w, i }))
        .sort((a, b) => roleOrder(a.w.role) - roleOrder(b.w.role) || a.i - b.i)
        .map((x) => x.w);
      s.setWaypoints(wp);
      s.setRoute(null);
      break;
    }

    case "SET_WAYPOINTS": {
      // AI が一式置換するときに使う。role 順（start→via→goal）に並べ替えて id を付与。
      const wp: Waypoint[] = cmd.waypoints
        .map((w, i) => ({ w, i }))
        .sort((a, b) => roleOrder(a.w.role) - roleOrder(b.w.role) || a.i - b.i)
        .map(({ w }) => ({
          id: nid("wp"),
          role: w.role,
          xy: { x: w.x, y: w.y },
          heading_deg: w.heading_deg ?? null,
        }));
      s.setWaypoints(wp);
      s.setRoute(null);
      break;
    }

    case "INSERT_VIA": {
      const wp = [...s.waypoints];
      if (wp.length < 2) {
        // start/goal 未設定なら単純に via 追加
        wp.push({ id: nid("wp"), role: "via", xy: cmd.xy });
        s.setWaypoints(wp);
        s.setRoute(null);
        break;
      }
      let bestSeg = 0;
      let bestD2 = Number.POSITIVE_INFINITY;
      for (let i = 0; i < wp.length - 1; i++) {
        const d2 = dist2ToSegment(cmd.xy, wp[i].xy, wp[i + 1].xy);
        if (d2 < bestD2) {
          bestD2 = d2;
          bestSeg = i;
        }
      }
      wp.splice(bestSeg + 1, 0, { id: nid("wp"), role: "via", xy: cmd.xy });
      s.setWaypoints(wp);
      s.setRoute(null);
      break;
    }

    case "MOVE_WAYPOINT":
      s.setWaypoints(s.waypoints.map((w) => (w.id === cmd.id ? { ...w, xy: cmd.xy } : w)));
      s.setRoute(null);
      break;

    case "SET_WAYPOINT_HEADING":
      s.setWaypoints(
        s.waypoints.map((w) => (w.id === cmd.id ? { ...w, heading_deg: cmd.heading_deg } : w)),
      );
      s.setRoute(null);
      break;

    case "DELETE_WAYPOINT":
      s.setWaypoints(s.waypoints.filter((w) => w.id !== cmd.id));
      s.setRoute(null);
      break;

    case "SET_ROUTE":
      s.setRoute(cmd.route);
      break;

    case "ADD_POLY_VERTEX":
      s.setActivePolygon([...s.activePolygon, cmd.xy]);
      break;

    case "UNDO_POLY_VERTEX":
      s.setActivePolygon(s.activePolygon.slice(0, -1));
      break;

    case "FINISH_POLYGON": {
      if (s.activePolygon.length < 3) break;
      const area: AreaPoly = { id: nid("area"), name: cmd.name, points: s.activePolygon };
      s.setAreas([...s.areas, area]);
      s.setActivePolygon([]);
      break;
    }

    case "CANCEL_POLYGON":
      s.setActivePolygon([]);
      break;

    case "DELETE_AREA":
      s.setAreas(s.areas.filter((a) => a.id !== cmd.id));
      break;

    case "SET_AREA_POINTS":
      if (cmd.points.length < 3) break; // 3頂点未満は不正なポリゴンなので無視
      s.setAreas(s.areas.map((a) => (a.id === cmd.id ? { ...a, points: cmd.points } : a)));
      break;

    case "LOAD_AREAS": {
      const added: AreaPoly[] = cmd.areas.map((a) => ({ id: nid("area"), name: a.name, points: a.points }));
      s.setAreas([...s.areas, ...added]);
      break;
    }

    case "SET_IMPORTED": {
      const imported: ImportedRoute[] = cmd.routes.map((r) => ({ id: nid("imp"), name: r.name, pts: r.pts }));
      s.setImportedRoutes(imported);
      break;
    }

    case "SELECT":
      s.select(cmd.id);
      break;

    case "RESET_ROUTE":
      s.resetRoute();
      break;

    case "UNDO":
      s.undo();
      break;

    case "REDO":
      s.redo();
      break;
  }
}
