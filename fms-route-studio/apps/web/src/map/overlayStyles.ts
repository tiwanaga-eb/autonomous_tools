// 地図オーバーレイのスタイル定義（MapView から分離）。
// feature の "kind" 属性 → Style の対応は overlayStyleFor() に集約する。
import type Feature from "ol/Feature";
import type { FeatureLike } from "ol/Feature";
import { Circle as CircleStyle, Fill, Stroke, Style, Text } from "ol/style";

export const ROUTE_STYLE = new Style({ stroke: new Stroke({ color: "#f59e0b", width: 3 }) });
export const WPLINE_STYLE = new Style({ stroke: new Stroke({ color: "#ffffffaa", width: 2 }) });
export const IMPORTED_STYLE = new Style({ stroke: new Stroke({ color: "#b91c1c", width: 3 }) });
// 複数台(Fleet): 経路ごとの色（FleetPanel の ROUTE_COLORS と一致）。競合は赤で強調。
export const FLEET_COLORS = ["#2563eb", "#16a34a", "#d97706", "#9333ea", "#0891b2", "#db2777", "#65a30d", "#dc2626"];
const _savedRouteStyleCache = new Map<string, Style>();
export function savedRouteStyle(color: string): Style {
  let st = _savedRouteStyleCache.get(color);
  if (!st) {
    st = new Style({ stroke: new Stroke({ color, width: 2.5 }) });
    _savedRouteStyleCache.set(color, st);
  }
  return st;
}
export const CONFLICT_BOX_STYLE = new Style({
  stroke: new Stroke({ color: "#ef4444", width: 1.5, lineDash: [5, 4] }),
  fill: new Fill({ color: "#ef444422" }),
});
export const CONFLICT_SEG_STYLE = new Style({ stroke: new Stroke({ color: "#ef4444", width: 6 }) });
// Fleet sim: 各車マーカー（経路色の●、停止中=赤縁、完了=細縁）。
const _fleetVehStyleCache = new Map<string, Style>();
export function fleetVehStyle(color: string, waiting: boolean): Style {
  const key = `${color}|${waiting}`;
  let st = _fleetVehStyleCache.get(key);
  if (!st) {
    st = new Style({
      image: new CircleStyle({
        radius: 7,
        fill: new Fill({ color }),
        stroke: new Stroke({ color: waiting ? "#ef4444" : "#ffffff", width: waiting ? 3 : 1.5 }),
      }),
    });
    _fleetVehStyleCache.set(key, st);
  }
  return st;
}
export const FLEET_ARROW_STYLE = new Style({ stroke: new Stroke({ color: "#111827", width: 1.5 }) });
// すれ違い点（待避所）: 退避先を示すマーカー（シアンの四角＋本線からの接続線）。
export const FLEET_BAY_STYLE = new Style({
  image: new CircleStyle({ radius: 6, fill: new Fill({ color: "#06b6d4cc" }), stroke: new Stroke({ color: "#ffffff", width: 1.5 }) }),
});
export const FLEET_BAY_LINK_STYLE = new Style({ stroke: new Stroke({ color: "#06b6d4", width: 1.5, lineDash: [4, 3] }) });
// 自動配置された待避所（auto_passing）: 手動(シアン)と区別してアンバー。
export const FLEET_AUTOBAY_STYLE = new Style({
  image: new CircleStyle({ radius: 6, fill: new Fill({ color: "#f59e0bcc" }), stroke: new Stroke({ color: "#ffffff", width: 1.5 }) }),
});
export const FLEET_AUTOBAY_LINK_STYLE = new Style({ stroke: new Stroke({ color: "#f59e0b", width: 1.5, lineDash: [4, 3] }) });

export const AREA_STYLE = new Style({
  fill: new Fill({ color: "#22c55e33" }),
  stroke: new Stroke({ color: "#22c55e", width: 2 }),
});
// エリアの名前＋面積ラベル（feature.get("label") を重心に表示）
const _areaLabelCache = new Map<string, Style>();
export function areaStyle(label: string | undefined): Style | Style[] {
  if (!label) return AREA_STYLE;
  let st = _areaLabelCache.get(label);
  if (!st) {
    st = new Style({
      text: new Text({
        text: label,
        font: "12px system-ui, sans-serif",
        fill: new Fill({ color: "#14532d" }),
        stroke: new Stroke({ color: "#ffffffcc", width: 3 }),
        overflow: true,
      }),
    });
    _areaLabelCache.set(label, st);
    if (_areaLabelCache.size > 512) _areaLabelCache.clear(); // 名称変更で無限に増えないように
  }
  return [AREA_STYLE, st];
}
// 計測ツール（距離・面積）: シアンの実線＋閉じ線は点線
export const MEASURE_LINE_STYLE = new Style({ stroke: new Stroke({ color: "#0891b2", width: 2.5 }) });
export const MEASURE_CLOSE_STYLE = new Style({ stroke: new Stroke({ color: "#0891b2aa", width: 1.5, lineDash: [6, 5] }) });
export const MEASURE_VERTEX_STYLE = new Style({
  image: new CircleStyle({ radius: 4, fill: new Fill({ color: "#0891b2" }), stroke: new Stroke({ color: "#ffffff", width: 1.5 }) }),
});
export const ACTIVE_POLY_STYLE = new Style({
  stroke: new Stroke({ color: "#fbbf24", width: 2, lineDash: [6, 4] }),
  fill: new Fill({ color: "#fbbf2422" }),
});
export const POLY_VERTEX_STYLE = new Style({
  image: new CircleStyle({ radius: 4, fill: new Fill({ color: "#fbbf24" }) }),
});

export const ROADBAND_STYLE = new Style({
  fill: new Fill({ color: "#f59e0b66" }),
  stroke: new Stroke({ color: "#f59e0bcc", width: 1.5 }),
});
export const RWPT_STYLE = new Style({
  image: new CircleStyle({
    radius: 2.5,
    fill: new Fill({ color: "#fde68a" }),
    stroke: new Stroke({ color: "#00000066", width: 1 }),
  }),
});
export const HILITE_STYLE = new Style({
  image: new CircleStyle({
    radius: 7,
    fill: new Fill({ color: "#ffffff00" }),
    stroke: new Stroke({ color: "#ffffff", width: 2 }),
  }),
});
// 車体フットプリント（ホバー点に実寸の向き付き矩形）。包含が一目で分かるよう半透明白。
export const FOOTPRINT_STYLE = new Style({
  fill: new Fill({ color: "#ffffff1f" }),
  stroke: new Stroke({ color: "#ffffffcc", width: 1.5 }),
});
// 操舵輪（タイヤ）。アッカーマン操舵の角度を可視化。塗りつぶしの濃色。
export const WHEEL_STYLE = new Style({
  fill: new Fill({ color: "#111827cc" }),
  stroke: new Stroke({ color: "#fbbf24", width: 1.5 }),
});

// 寄り付きシミュレータ用
export const SPOT_FWD_STYLE = new Style({ stroke: new Stroke({ color: "#22d3ee", width: 3 }) });
export const SPOT_REV_STYLE = new Style({ stroke: new Stroke({ color: "#22d3ee", width: 3, lineDash: [5, 5] }) });
export const SPOT_EXIT_STYLE = new Style({ stroke: new Stroke({ color: "#e879f9", width: 2.5, lineDash: [3, 4] }) });
export const SPOT_SWITCH_STYLE = new Style({
  image: new CircleStyle({ radius: 5, fill: new Fill({ color: "#fde047" }), stroke: new Stroke({ color: "#000", width: 1 }) }),
});
export const SPOT_VEHICLE_STYLE = new Style({
  image: new CircleStyle({ radius: 7, fill: new Fill({ color: "#ffffffcc" }), stroke: new Stroke({ color: "#0891b2", width: 2 }) }),
});
export const SWITCHZONE_STYLE = new Style({
  stroke: new Stroke({ color: "#16a34a", width: 2, lineDash: [8, 4] }),
  fill: new Fill({ color: "#16a34a22" }),
});
// 走行を収めるエリア（containment: 経路＋車体をこの中に収める）。選択中を青破線で明示。
export const CONTAINZONE_STYLE = new Style({
  stroke: new Stroke({ color: "#2563eb", width: 2.5, lineDash: [10, 5] }),
  fill: new Fill({ color: "#2563eb18" }),
});
// 排土（パイル）: 基部円（実寸・安息角の円錐の裾）＋中心点。土砂色で区別。
export const PILE_BASE_STYLE = new Style({
  fill: new Fill({ color: "#b4530926" }),
  stroke: new Stroke({ color: "#92400e", width: 1.5 }),
});
export const PILE_CENTER_STYLE = new Style({
  image: new CircleStyle({ radius: 3, fill: new Fill({ color: "#92400e" }), stroke: new Stroke({ color: "#ffffff", width: 1 }) }),
});
// 確定済み Drivable 編集のアウトライン（include=緑 / exclude=赤）。どこを手修正したか可視化。
export const EDIT_INCLUDE_STYLE = new Style({
  stroke: new Stroke({ color: "#22c55e", width: 2, lineDash: [4, 3] }),
  fill: new Fill({ color: "#22c55e1f" }),
});
export const EDIT_EXCLUDE_STYLE = new Style({
  stroke: new Stroke({ color: "#ef4444", width: 2, lineDash: [4, 3] }),
  fill: new Fill({ color: "#ef44441f" }),
});
export function spotColor(role: string): string {
  if (role === "spot_start") return "#a855f7";
  if (role === "spot_switch") return "#16a34a"; // 手動切り返し点（緑）
  if (role === "spot_exit_goal") return "#e879f9"; // 退出Goal（マゼンタ系）
  return "#f97316";
}
export function spotDotStyle(role: string): Style {
  return new Style({
    image: new CircleStyle({ radius: 6, fill: new Fill({ color: spotColor(role) }), stroke: new Stroke({ color: "#000a", width: 1 }) }),
  });
}
export function spotArrowStyle(role: string): Style {
  return new Style({ stroke: new Stroke({ color: spotColor(role), width: 2.5 }) });
}

export function roleColor(role: string): string {
  return role === "start" ? "#22c55e" : role === "goal" ? "#ef4444" : "#0ea5e9";
}

export function waypointStyle(role: string): Style {
  return new Style({
    image: new CircleStyle({
      radius: 6,
      fill: new Fill({ color: roleColor(role) }),
      stroke: new Stroke({ color: "#00000088", width: 1 }),
    }),
  });
}

export function headingStyle(role: string): Style {
  return new Style({ stroke: new Stroke({ color: roleColor(role), width: 2.5 }) });
}

export const HEADING_PREVIEW_STYLE = new Style({
  stroke: new Stroke({ color: "#fbbf24", width: 2, lineDash: [4, 3] }),
});

/** feature.get("kind") → Style の一元ディスパッチ（overlay/fleet-sim 両レイヤで共用）。 */
export function overlayStyleFor(feature: FeatureLike): Style | Style[] | undefined {
  const f = feature as Feature;
  const kind = f.get("kind");
  if (kind === "waypoint") return waypointStyle(f.get("role"));
  if (kind === "heading") return headingStyle(f.get("role"));
  if (kind === "headingpreview") return HEADING_PREVIEW_STYLE;
  if (kind === "route") return ROUTE_STYLE;
  if (kind === "roadband") return ROADBAND_STYLE;
  if (kind === "rwpt") return RWPT_STYLE;
  if (kind === "hilite") return HILITE_STYLE;
  if (kind === "footprint") return FOOTPRINT_STYLE;
  if (kind === "wheel") return WHEEL_STYLE;
  if (kind === "spotdot") return spotDotStyle(f.get("role"));
  if (kind === "spotarrow") return spotArrowStyle(f.get("role"));
  if (kind === "spotfwd") return SPOT_FWD_STYLE;
  if (kind === "spotrev") return SPOT_REV_STYLE;
  if (kind === "spotexit") return SPOT_EXIT_STYLE;
  if (kind === "spotswitch") return SPOT_SWITCH_STYLE;
  if (kind === "switchzone") return SWITCHZONE_STYLE;
  if (kind === "containzone") return CONTAINZONE_STYLE;
  if (kind === "editinclude") return EDIT_INCLUDE_STYLE;
  if (kind === "editexclude") return EDIT_EXCLUDE_STYLE;
  if (kind === "spotvehicle") return SPOT_VEHICLE_STYLE;
  if (kind === "wpline") return WPLINE_STYLE;
  if (kind === "imported") return IMPORTED_STYLE;
  if (kind === "savedroute") return savedRouteStyle(f.get("color") || "#2563eb");
  if (kind === "conflictbox") return CONFLICT_BOX_STYLE;
  if (kind === "conflictseg") return CONFLICT_SEG_STYLE;
  if (kind === "fleetveh") return fleetVehStyle(f.get("color") || "#2563eb", !!f.get("waiting"));
  if (kind === "fleetarrow") return FLEET_ARROW_STYLE;
  if (kind === "fleetbay") return FLEET_BAY_STYLE;
  if (kind === "fleetbaylink") return FLEET_BAY_LINK_STYLE;
  if (kind === "fleetautobay") return FLEET_AUTOBAY_STYLE;
  if (kind === "fleetautobaylink") return FLEET_AUTOBAY_LINK_STYLE;
  if (kind === "pilebase") return PILE_BASE_STYLE;
  if (kind === "pilecenter") return PILE_CENTER_STYLE;
  if (kind === "area") return areaStyle(f.get("label"));
  if (kind === "activepoly") return ACTIVE_POLY_STYLE;
  if (kind === "polyvertex") return POLY_VERTEX_STYLE;
  if (kind === "measureline") return MEASURE_LINE_STYLE;
  if (kind === "measureclose") return MEASURE_CLOSE_STYLE;
  if (kind === "measurevertex") return MEASURE_VERTEX_STYLE;
  return undefined;
}
