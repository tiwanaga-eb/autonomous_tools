// 地図ビュー（OpenLayers / ネイティブ EPSG:6677）。
// COG を ol/source/GeoTIFF でブラウザが直接読み、Web Mercator 再投影を挟まない
// → クリック座標は 6677 で厳密（3cm精度要件）。計画も backend で 6677 のまま。

import { useEffect, useRef, useState } from "react";

import Feature from "ol/Feature";
import OLMap from "ol/Map";
import View from "ol/View";
import MousePosition from "ol/control/MousePosition";
import ScaleLine from "ol/control/ScaleLine";
import { defaults as defaultControls } from "ol/control/defaults";
import type { Extent } from "ol/extent";
import { Circle as CircleGeom, LineString, Point, Polygon } from "ol/geom";
import DragPan from "ol/interaction/DragPan";
import Modify from "ol/interaction/Modify";
import WebGLTileLayer from "ol/layer/WebGLTile";
import TileLayer from "ol/layer/Tile";
import VectorLayer from "ol/layer/Vector";
import GeoTIFF from "ol/source/GeoTIFF";
import OSM from "ol/source/OSM";
import VectorSource from "ol/source/Vector";
import "ol/ol.css";

import { api } from "@/api/client";
import { dispatch } from "@/commandBus";
import "@/map/proj";
import { WORKING_CRS, WORKING_EPSG } from "@/map/proj";
import { arrowGeom, bayPointAt, offsetEdges, vehicleShapeRings } from "@/map/overlayGeom";
import type { VehShape } from "@/map/overlayGeom";
import { FLEET_COLORS, overlayStyleFor } from "@/map/overlayStyles";
import { downloadDataUrl, timestampName } from "@/exporters";
import { pickLayer } from "@/layerSelect";
import { useStore } from "@/store/useStore";


// 現在の編集モードのラベル（地図上チップ表示用）
const MODE_LABELS: Record<string, string> = {
  start: "始点を配置", goal: "終点を配置", via: "経由点を配置", insert_via: "経由点を挿入",
  edit: "点を編集（ドラッグ）", pan: "移動（パン）", polygon: "ポリゴン描画",
  spot_start: "寄り付き 開始姿勢", spot_target: "寄り付き 目標姿勢",
  spot_switch: "切り返し点", spot_exit_goal: "退出Goal",
};
const POSE_MODE_SET = new Set([
  "start", "goal", "spot_start", "spot_target", "spot_switch", "spot_exit_goal",
]);

export function MapView() {
  const containerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<OLMap | null>(null);
  const overlaySrcRef = useRef<VectorSource>(new VectorSource());
  const fleetSimSrcRef = useRef<VectorSource>(new VectorSource()); // fleet 再生（毎フレーム更新）専用
  const modifyRef = useRef<Modify | null>(null);
  const dragPanRef = useRef<DragPan | null>(null);
  const previewRef = useRef<Feature | null>(null);
  const highlightRef = useRef<Feature | null>(null);
  const footprintFeatsRef = useRef<Feature[]>([]);
  const spotVehFeatsRef = useRef<Feature[]>([]);
  const rasterLayersRef = useRef<globalThis.Map<string, { layer: WebGLTileLayer; version: number }>>(
    new globalThis.Map(),
  );
  const osmLayerRef = useRef<TileLayer<OSM> | null>(null);
  const viewSetRef = useRef(false);
  const dataExtentRef = useRef<Extent | null>(null);

  const layers = useStore((s) => s.layers);
  const waypoints = useStore((s) => s.waypoints);
  const route = useStore((s) => s.route);
  const areas = useStore((s) => s.areas);
  const activePolygon = useStore((s) => s.activePolygon);
  const importedRoutes = useStore((s) => s.importedRoutes);
  const mode = useStore((s) => s.mode);
  const costOpacity = useStore((s) => s.costOpacity);
  const costVisible = useStore((s) => s.costVisible);
  const drivableOpacity = useStore((s) => s.drivableOpacity);
  const drivableVisible = useStore((s) => s.drivableVisible);
  const osmVisible = useStore((s) => s.osmVisible);
  const showWaypoints = useStore((s) => s.showWaypoints);
  const roadWidthM = useStore((s) => s.roadWidthM);
  const hoverPointIndex = useStore((s) => s.hoverPointIndex);
  const vehicleId = useStore((s) => s.vehicleId);
  const vehiclesRev = useStore((s) => s.vehiclesRev);
  const spotStart = useStore((s) => s.spotStart);
  const spotTarget = useStore((s) => s.spotTarget);
  const spotSwitchPose = useStore((s) => s.spotSwitchPose);
  const spotSwitchZoneId = useStore((s) => s.spotSwitchZoneId);
  const spotContainAreaId = useStore((s) => s.spotContainAreaId);
  const spotExitGoal = useStore((s) => s.spotExitGoal);
  const spotRoadWidthM = useStore((s) => s.spotRoadWidthM);
  const drivableLayerId = useStore((s) => s.drivableLayerId);
  const spotResult = useStore((s) => s.spotResult);
  const spotIndex = useStore((s) => s.spotIndex);
  const activeFeature = useStore((s) => s.activeFeature);
  const savedRoutes = useStore((s) => s.savedRoutes);
  const showSavedRoutes = useStore((s) => s.showSavedRoutes);
  const fleetConflicts = useStore((s) => s.fleetConflicts);
  const fleetSim = useStore((s) => s.fleetSim);
  const fleetSimT = useStore((s) => s.fleetSimT);
  const fleetBays = useStore((s) => s.fleetBays);
  const pilePlan = useStore((s) => s.pilePlan);

  // 選択車両の寸法＋運動学（ホバー点の車両形状描画用: アーティキュレート2矩形 / アッカーマン操舵輪）。
  const [vehDims, setVehDims] = useState<VehShape | null>(null);
  useEffect(() => {
    if (!vehicleId) {
      setVehDims(null);
      return;
    }
    api
      .vehicleDetail(vehicleId)
      .then((d) => {
        const e = d.effective as Record<string, number | string | null | undefined>;
        const num = (k: string) => (typeof e[k] === "number" ? (e[k] as number) : null);
        const L = num("overall_length") ?? 0;
        // footprint_polygon の x 範囲から前後端を取得（基準点が車体中心でない車両に対応）。
        const fp = (d.effective as Record<string, unknown>).footprint_polygon as number[][] | null | undefined;
        let frontExt: number | null = null;
        let rearExt: number | null = null;
        if (Array.isArray(fp) && fp.length) {
          const xs = fp.map((p) => p[0]);
          frontExt = Math.max(...xs);
          rearExt = Math.min(...xs);
        }
        setVehDims({
          l: L,
          w: num("overall_width") ?? 0,
          kind: String(e.kinematic_type ?? "rigid_bicycle"),
          wheelBase: num("wheel_base"),
          trackWidth: num("track_width"),
          frontLen: num("front_length"),
          rearLen: num("rear_length"),
          maxArtic: num("max_articulation_angle"),
          maxSteer: num("max_steer_angle"),
          frontExt,
          rearExt,
        });
      })
      .catch(() => setVehDims(null));
  }, [vehicleId, vehiclesRev]);

  // ---- init map once ----
  useEffect(() => {
    if (!containerRef.current || mapRef.current) return;

    const overlayLayer = new VectorLayer({
      source: overlaySrcRef.current,
      style: overlayStyleFor,
    });
    overlayLayer.setZIndex(1000);

    // Fleet 再生専用レイヤ: fleetSimT は再生中に毎フレーム変わるため、
    // 全オーバーレイの再構築を避けて車両マーカーだけを別ソースで更新する。
    const fleetSimLayer = new VectorLayer({
      source: fleetSimSrcRef.current,
      style: overlayStyleFor,
    });
    fleetSimLayer.setZIndex(1001);

    // OSM ベースマップ（地理的文脈）。Web-Mercator タイルを 6677 ビューへ OL が自動再投影。
    // 最下層(zIndex 0)。既定は非表示で、データ層が無いときの位置把握用。
    const osmLayer = new TileLayer({ source: new OSM(), visible: useStore.getState().osmVisible, opacity: 0.7 });
    osmLayer.setZIndex(0);
    osmLayerRef.current = osmLayer;

    const map = new OLMap({
      target: containerRef.current,
      layers: [osmLayer, overlayLayer, fleetSimLayer],
      view: new View({ projection: WORKING_CRS, center: [0, 0], zoom: 2 }),
      // 既定のズーム(+/-)コントロールは左上の 2D/3D 切替と重なるため非表示
      // （ホイール/トラックパッドでズーム可。属性表示は残す）。
      // スケールバー＋カーソル座標（作業CRSメートル）を常時表示＝現場座標の読み取り・距離感の基準。
      controls: defaultControls({ zoom: false }).extend([
        new ScaleLine({ units: "metric", minWidth: 80 }),
        new MousePosition({
          coordinateFormat: (c) => (c ? `X ${c[0].toFixed(1)} / Y ${c[1].toFixed(1)} m` : ""),
          placeholder: "",
        }),
      ]),
    });
    mapRef.current = map;

    const viewport = map.getViewport();

    // Start/Goal 以外（via/insert_via/polygon）は従来どおりクリックで配置。
    const onClick = (e: MouseEvent) => {
      const st = useStore.getState();
      const [x, y] = map.getEventCoordinate(e);
      const xy = { x, y };
      if (st.mode === "via") {
        dispatch({ type: "ADD_WAYPOINT", role: "via", xy });
      } else if (st.mode === "insert_via") {
        dispatch({ type: "INSERT_VIA", xy });
      } else if (st.mode === "polygon") {
        dispatch({ type: "ADD_POLY_VERTEX", xy });
      }
    };
    viewport.addEventListener("click", onClick);

    // Start/Goal/Spotting姿勢: 「クリック＝位置」「左ドラッグ＝方位ベクトル」。
    // パンは 右/中ボタンドラッグ・ホイール・Pan モード・ズームボタンで（左ドラッグは方位に使う）。
    let downCoord: number[] | null = null;
    let downClient: { x: number; y: number } | null = null;
    const clearPreview = () => {
      if (previewRef.current) {
        overlaySrcRef.current.removeFeature(previewRef.current);
        previewRef.current = null;
      }
    };
    const POSE_MODES = ["start", "goal", "spot_start", "spot_target", "spot_switch", "spot_exit_goal"];
    const onPointerDown = (e: PointerEvent) => {
      const st = useStore.getState();
      if (!POSE_MODES.includes(st.mode)) return;
      if (e.button !== 0) return; // 左ボタンのみ方位指定（中/右はパン）
      downCoord = map.getEventCoordinate(e);
      downClient = { x: e.clientX, y: e.clientY };
    };
    const onPointerMove = (e: PointerEvent) => {
      if (!downCoord) return; // 左ドラッグ中だけ方位プレビュー
      const cur = map.getEventCoordinate(e);
      const geom = new LineString([downCoord, cur]);
      if (!previewRef.current) {
        previewRef.current = new Feature(geom);
        previewRef.current.set("kind", "headingpreview");
        overlaySrcRef.current.addFeature(previewRef.current);
      } else {
        previewRef.current.setGeometry(geom);
      }
    };
    const onPointerUp = (e: PointerEvent) => {
      if (!downCoord || !downClient) return;
      const st = useStore.getState();
      const up = map.getEventCoordinate(e);
      const pixelDist = Math.hypot(e.clientX - downClient.x, e.clientY - downClient.y);
      const isDragHeading = pixelDist > 8;
      const heading = isDragHeading
        ? (Math.atan2(up[1] - downCoord[1], up[0] - downCoord[0]) * 180) / Math.PI
        : null;
      const xy = { x: downCoord[0], y: downCoord[1] };
      if (st.mode === "start" || st.mode === "goal") {
        dispatch({ type: "ADD_WAYPOINT", role: st.mode, xy, heading_deg: heading });
      } else if (st.mode === "spot_start") {
        useStore.getState().setSpotStart({ ...xy, heading_deg: heading ?? (useStore.getState().spotStart?.heading_deg ?? 0) });
      } else if (st.mode === "spot_target") {
        useStore.getState().setSpotTarget({ ...xy, heading_deg: heading ?? (useStore.getState().spotTarget?.heading_deg ?? 0) });
      } else if (st.mode === "spot_switch") {
        useStore.getState().setSpotSwitchPose({ ...xy, heading_deg: heading ?? (useStore.getState().spotSwitchPose?.heading_deg ?? 0) });
      } else if (st.mode === "spot_exit_goal") {
        useStore.getState().setSpotExitGoal({ ...xy, heading_deg: heading ?? (useStore.getState().spotExitGoal?.heading_deg ?? 0) });
      }
      clearPreview();
      downCoord = null;
      downClient = null;
    };
    const onPointerLeave = () => {
      clearPreview();
      downCoord = null;
      downClient = null;
    };
    viewport.addEventListener("pointerdown", onPointerDown);
    viewport.addEventListener("pointermove", onPointerMove);
    viewport.addEventListener("pointerup", onPointerUp);
    viewport.addEventListener("pointerleave", onPointerLeave);
    // 右ドラッグでパンする際のコンテキストメニューを抑止
    const onCtxMenu = (e: Event) => e.preventDefault();
    viewport.addEventListener("contextmenu", onCtxMenu);

    // 既定の DragPan を置換: 姿勢モード中は「左ドラッグ＝方位」なのでパンしない。
    // 右/中ボタンドラッグ、または姿勢モード以外では従来どおりパンする。
    const defPan = map.getInteractions().getArray().find((i) => i instanceof DragPan) as DragPan | undefined;
    if (defPan) map.removeInteraction(defPan);
    const pan = new DragPan({
      condition: (mbe) => {
        const oe = mbe.originalEvent as PointerEvent;
        const m = useStore.getState().mode;
        const isPose = ["start", "goal", "spot_start", "spot_target", "spot_switch", "spot_exit_goal"].includes(m);
        const leftHeld = oe.buttons != null ? (oe.buttons & 1) === 1 : oe.button === 0;
        return !(isPose && leftHeld); // 姿勢モードの左ドラッグ以外はパン許可
      },
    });
    map.addInteraction(pan);
    dragPanRef.current = pan;

    const modify = new Modify({ source: overlaySrcRef.current });
    modify.on("modifyend", (e) => {
      // 実際に編集された feature のみ反映（全件dispatchは履歴を汚すため）。
      e.features.forEach((f) => {
        if (f.get("kind") === "waypoint") {
          const [x, y] = (f.getGeometry() as Point).getCoordinates();
          dispatch({ type: "MOVE_WAYPOINT", id: String(f.getId()), xy: { x, y } });
        } else if (f.get("kind") === "area") {
          // ポリゴン外環の座標（末尾の閉じ点を除く）を元エリアへ反映。
          // Modify は線分クリックで頂点追加・Alt+クリックで削除も可能。
          const ring = (f.getGeometry() as Polygon).getCoordinates()[0];
          const pts = ring.slice(0, -1).map(([x, y]) => ({ x, y }));
          dispatch({ type: "SET_AREA_POINTS", id: String(f.getId()), points: pts });
        }
      });
    });
    modify.setActive(useStore.getState().mode === "edit");
    map.addInteraction(modify);
    modifyRef.current = modify;

    return () => {
      viewport.removeEventListener("click", onClick);
      viewport.removeEventListener("pointerdown", onPointerDown);
      viewport.removeEventListener("pointermove", onPointerMove);
      viewport.removeEventListener("pointerup", onPointerUp);
      viewport.removeEventListener("pointerleave", onPointerLeave);
      viewport.removeEventListener("contextmenu", onCtxMenu);
      // COG レイヤの source(worker/decoder) と layer を破棄してからマップを解放（リーク防止）。
      for (const [, ent] of rasterLayersRef.current) {
        (ent.layer.getSource() as GeoTIFF | null)?.dispose();
        ent.layer.dispose();
      }
      map.setTarget(undefined);
      mapRef.current = null;
      viewSetRef.current = false;
      rasterLayersRef.current.clear();
    };
  }, []);

  // ---- raster layers (COG via ol/source/GeoTIFF, native 6677) ----
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;
    const existing = rasterLayersRef.current;
    // ラスタ(COG)はアップロード時のゾーンに焼き込まれている。作業ゾーンと異なるものを表示すると
    // ビュー投影と不一致になり位置がずれるため、現作業ゾーンと一致するラスタのみ表示する。
    // (epsg 不明の旧データは後方互換で許可)
    const wanted = layers.filter(
      (l) =>
        (l.kind === "ortho" || l.kind === "cost" || l.kind === "drivable") &&
        (l.epsg == null || l.epsg === WORKING_EPSG),
    );
    const wantedIds = new Set(wanted.map((l) => l.id));

    // 不要 or version変更 のレイヤを除去（GeoTIFF source と WebGLTileLayer も dispose し GPU/worker リークを防ぐ）
    for (const [id, ent] of existing) {
      const l = layers.find((x) => x.id === id);
      if (!wantedIds.has(id) || (l && (l.version ?? 1) !== ent.version)) {
        map.removeLayer(ent.layer);
        (ent.layer.getSource() as GeoTIFF | null)?.dispose();
        ent.layer.dispose();
        existing.delete(id);
      }
    }

    const fitFromSource = (source: GeoTIFF) => {
      source
        .getView()
        .then((v) => {
          if (v.extent) dataExtentRef.current = v.extent as Extent;
          map.setView(new View(v));
        })
        .catch(() => undefined);
    };

    wanted.forEach((l) => {
      if (existing.has(l.id)) return;
      const isRgba = l.kind === "cost" || l.kind === "drivable"; // RGBA(範囲外透明)
      const version = l.version ?? 1;
      const source = new GeoTIFF({
        sources: [{ url: api.cogUrl(l.id, version) }],
        convertToRGB: isRgba ? false : "auto",
      });
      const st = useStore.getState();
      const op = l.kind === "drivable" ? st.drivableOpacity : l.kind === "cost" ? st.costOpacity : 1;
      const vis = l.kind === "drivable" ? st.drivableVisible : l.kind === "cost" ? st.costVisible : true;
      const lyr = new WebGLTileLayer({ source, opacity: op, visible: vis });
      lyr.setZIndex(l.kind === "drivable" ? 20 : l.kind === "cost" ? 10 : 1);
      map.addLayer(lyr);
      existing.set(l.id, { layer: lyr, version });
      if (!viewSetRef.current && l.kind === "ortho") {
        viewSetRef.current = true;
        fitFromSource(source);
      }
    });

    if (!viewSetRef.current && wanted.length > 0) {
      const ent = existing.get(wanted[0].id);
      const src = ent?.layer.getSource() as GeoTIFF | null | undefined;
      if (src) {
        viewSetRef.current = true;
        fitFromSource(src);
      }
    }
  }, [layers]);

  // ---- overlay features ----
  useEffect(() => {
    const src = overlaySrcRef.current;
    src.clear();

    areas.forEach((a) => {
      if (a.points.length >= 3) {
        const ring = a.points.map((p) => [p.x, p.y]);
        ring.push(ring[0]);
        const f = new Feature(new Polygon([ring]));
        f.set("kind", "area");
        f.setId(a.id); // edit モードの Modify で頂点移動を元エリアへ反映するため
        src.addFeature(f);
      }
    });

    importedRoutes.forEach((r) => {
      if (r.pts.length > 1) {
        const f = new Feature(new LineString(r.pts.map((p) => [p.x, p.y])));
        f.set("kind", "imported");
        src.addFeature(f);
      }
    });

    // 複数台(Fleet): 保存経路ライブラリを経路ごとの色で重ねて表示（複数経路の可視化）。
    const showFleet = showSavedRoutes || activeFeature === "fleet";
    if (showFleet) {
      savedRoutes.forEach((r, i) => {
        const pts = r.route?.trajectory?.points ?? [];
        if (pts.length > 1) {
          const f = new Feature(new LineString(pts.map((p) => [p.x, p.y])));
          f.set("kind", "savedroute");
          f.set("color", FLEET_COLORS[i % FLEET_COLORS.length]);
          src.addFeature(f);
        }
        // すれ違い点（待避所）: 経路の s_frac 位置に、横退避先のマーカーと本線への接続線。
        const bay = fleetBays[r.id];
        if (bay && pts.length > 2) {
          const xy = pts.map((p) => [p.x, p.y] as [number, number]);
          const seg = xy.slice(1).map((p, k) => Math.hypot(p[0] - xy[k][0], p[1] - xy[k][1]));
          const total = seg.reduce((a, b) => a + b, 0);
          let target = bay.s_frac * total;
          let idx = 0;
          for (let k = 0; k < seg.length; k++) { if (target <= seg[k]) { idx = k; break; } target -= seg[k]; idx = k + 1; }
          const j = Math.min(idx, xy.length - 2);
          const p0 = xy[j], p1 = xy[j + 1];
          const dx = p1[0] - p0[0], dy = p1[1] - p0[1];
          const L = Math.hypot(dx, dy) || 1;
          const nx = (-dy / L) * bay.side, ny = (dx / L) * bay.side; // 左法線×side
          const bx = p0[0] + bay.offset_m * nx, by = p0[1] + bay.offset_m * ny;
          const link = new Feature(new LineString([[p0[0], p0[1]], [bx, by]]));
          link.set("kind", "fleetbaylink");
          src.addFeature(link);
          const m = new Feature(new Point([bx, by]));
          m.set("kind", "fleetbay");
          src.addFeature(m);
        }
      });
    }

    // 競合（経路の重なり）: 赤帯=競合弧長区間、赤枠=重なり範囲(bbox)。
    if (fleetConflicts && showFleet) {
      const segPoints = (routeIdx: number, s0: number, s1: number) => {
        const pts = savedRoutes[routeIdx]?.route?.trajectory?.points ?? [];
        const sub = pts.filter((p) => p.s >= s0 - 1e-6 && p.s <= s1 + 1e-6).map((p) => [p.x, p.y]);
        return sub.length >= 2 ? sub : null;
      };
      fleetConflicts.forEach((c) => {
        for (const [idx, ivs] of [[c.a, c.a_intervals], [c.b, c.b_intervals]] as const) {
          ivs.forEach((iv) => {
            const sub = segPoints(idx, iv.s_start, iv.s_end);
            if (sub) {
              const f = new Feature(new LineString(sub));
              f.set("kind", "conflictseg");
              src.addFeature(f);
            }
          });
        }
        const { minx, miny, maxx, maxy } = c.bbox;
        const ring = [[minx, miny], [maxx, miny], [maxx, maxy], [minx, maxy], [minx, miny]];
        const bf = new Feature(new Polygon([ring]));
        bf.set("kind", "conflictbox");
        src.addFeature(bf);
      });
    }

    // 道幅帯（中心線±幅/2）。道幅指定があればそれ、無ければ選択車両の車幅で「実車の走行幅」を可視化。
    // route線より先に追加して下に敷く。
    const bandWidth = roadWidthM > 0 ? roadWidthM : vehDims?.w ?? 0;
    if (route && bandWidth > 0 && route.trajectory.points.length > 1) {
      const center = route.trajectory.points.map((p) => [p.x, p.y]);
      const { left, right } = offsetEdges(center, bandWidth / 2);
      const ring = left.concat([...right].reverse());
      ring.push(ring[0]);
      const f = new Feature(new Polygon([ring]));
      f.set("kind", "roadband");
      src.addFeature(f);
    }

    if (route && route.trajectory.points.length > 1) {
      const f = new Feature(new LineString(route.trajectory.points.map((p) => [p.x, p.y])));
      f.set("kind", "route");
      src.addFeature(f);
    }

    // 経路Waypoint(点)
    if (route && showWaypoints) {
      route.trajectory.points.forEach((p) => {
        const f = new Feature(new Point([p.x, p.y]));
        f.set("kind", "rwpt");
        src.addFeature(f);
      });
    }

    if (waypoints.length > 1) {
      const f = new Feature(new LineString(waypoints.map((w) => [w.xy.x, w.xy.y])));
      f.set("kind", "wpline");
      src.addFeature(f);
    }
    waypoints.forEach((w) => {
      const f = new Feature(new Point([w.xy.x, w.xy.y]));
      f.setId(w.id);
      f.set("kind", "waypoint");
      f.set("role", w.role);
      src.addFeature(f);
      // 方位ベクトル（指定されていれば矢印で描画）
      if (w.heading_deg != null) {
        const arr = new Feature(arrowGeom(w.xy.x, w.xy.y, w.heading_deg));
        arr.set("kind", "heading");
        arr.set("role", w.role);
        src.addFeature(arr);
      }
    });

    // 作図中ポリゴン
    if (activePolygon.length >= 2) {
      const f = new Feature(new LineString(activePolygon.map((p) => [p.x, p.y])));
      f.set("kind", "activepoly");
      src.addFeature(f);
    }
    activePolygon.forEach((p) => {
      const f = new Feature(new Point([p.x, p.y]));
      f.set("kind", "polyvertex");
      src.addFeature(f);
    });

    // 寄り付き: start/target 姿勢（点＋方位矢印）
    const spotPose = (pose: { x: number; y: number; heading_deg: number } | null, role: string) => {
      if (!pose) return;
      const dot = new Feature(new Point([pose.x, pose.y]));
      dot.set("kind", "spotdot");
      dot.set("role", role);
      src.addFeature(dot);
      const arr = new Feature(arrowGeom(pose.x, pose.y, pose.heading_deg));
      arr.set("kind", "spotarrow");
      arr.set("role", role);
      src.addFeature(arr);
    };
    spotPose(spotStart, "spot_start");
    spotPose(spotTarget, "spot_target");
    spotPose(spotSwitchPose, "spot_switch");
    spotPose(spotExitGoal, "spot_exit_goal");

    // 確定済み Drivable 編集（include/exclude）のアウトラインを地図に重畳。
    {
      const dl = pickLayer(layers, "drivable", drivableLayerId);
      const edits = (dl?.edits ?? []) as { op: string; polygon: [number, number][] }[];
      edits.forEach((ed) => {
        if (ed.polygon && ed.polygon.length >= 3) {
          const ring = ed.polygon.map((pt) => [pt[0], pt[1]]);
          ring.push(ring[0]);
          const f = new Feature(new Polygon([ring]));
          f.set("kind", ed.op === "exclude" ? "editexclude" : "editinclude");
          src.addFeature(f);
        }
      });
    }

    // 排土（パイル）配置: 基部円（実寸）＋中心点
    if (pilePlan && pilePlan.centers.length) {
      const r = pilePlan.pile.radius_m;
      pilePlan.centers.forEach(([px, py]) => {
        if (r > 0) {
          const base = new Feature(new CircleGeom([px, py], r));
          base.set("kind", "pilebase");
          src.addFeature(base);
        }
        const dot = new Feature(new Point([px, py]));
        dot.set("kind", "pilecenter");
        src.addFeature(dot);
      });
    }

    // 切り返し可能エリア（選択中の area を強調）
    if (spotSwitchZoneId) {
      const z = areas.find((a) => a.id === spotSwitchZoneId);
      if (z && z.points.length >= 3) {
        const ring = z.points.map((p) => [p.x, p.y]);
        ring.push(ring[0]);
        const f = new Feature(new Polygon([ring]));
        f.set("kind", "switchzone");
        src.addFeature(f);
      }
    }

    // 走行を収めるエリア（containment: 選択中の area を青破線で強調＝適用中であることを可視化）
    if (spotContainAreaId) {
      const z = areas.find((a) => a.id === spotContainAreaId);
      if (z && z.points.length >= 3) {
        const ring = z.points.map((p) => [p.x, p.y]);
        ring.push(ring[0]);
        const f = new Feature(new Polygon([ring]));
        f.set("kind", "containzone");
        src.addFeature(f);
      }
    }

    // 寄り付き経路: gear ごとに連続区間を分けて前進(実線)/後進(破線)で描画
    if (spotResult && spotResult.points.length > 1) {
      const pts = spotResult.points;
      // 道幅帯（寄り付き道幅>0ならそれ、無ければ車幅）。経路線より先に敷く。
      const spotBand = spotRoadWidthM > 0 ? spotRoadWidthM : vehDims?.w ?? 0;
      if (spotBand > 0) {
        const center = pts.map((p) => [p.x, p.y]);
        const { left, right } = offsetEdges(center, spotBand / 2);
        const ring = left.concat([...right].reverse());
        ring.push(ring[0]);
        const f = new Feature(new Polygon([ring]));
        f.set("kind", "roadband");
        src.addFeature(f);
      }
      let runStart = 0;
      for (let i = 1; i <= pts.length; i++) {
        if (i === pts.length || pts[i].gear !== pts[runStart].gear) {
          const seg = pts.slice(runStart, i);
          if (seg.length > 1) {
            const f = new Feature(new LineString(seg.map((p) => [p.x, p.y])));
            f.set("kind", pts[runStart].gear === "R" ? "spotrev" : "spotfwd");
            src.addFeature(f);
          }
          runStart = i;
        }
      }
      spotResult.switch_points.forEach((sp) => {
        const f = new Feature(new Point([sp.x, sp.y]));
        f.set("kind", "spotswitch");
        src.addFeature(f);
      });
      // 退出軌道（target→start）。マゼンタ点線で重畳。
      if (spotResult.exit && spotResult.exit.points.length > 1) {
        const f = new Feature(new LineString(spotResult.exit.points.map((p) => [p.x, p.y])));
        f.set("kind", "spotexit");
        src.addFeature(f);
      }
    }
  }, [waypoints, route, areas, activePolygon, importedRoutes, roadWidthM, vehDims, showWaypoints, spotStart, spotTarget, spotSwitchPose, spotSwitchZoneId, spotContainAreaId, spotExitGoal, spotRoadWidthM, spotResult, layers, drivableLayerId, savedRoutes, showSavedRoutes, fleetConflicts, activeFeature, fleetBays, pilePlan]);

  // ---- スクリーンキャプチャ: 全レイヤの canvas を1枚に合成して PNG 保存（OL公式パターン） ----
  useEffect(() => {
    const onCapture = () => {
      const map = mapRef.current;
      const container = containerRef.current;
      if (!map || !container) return;
      map.once("rendercomplete", () => {
        const size = map.getSize();
        if (!size) return;
        // OL 公式の map-export パターン: 各レイヤ canvas を style.transform で合成（CSSピクセル基準）
        const out = document.createElement("canvas");
        out.width = size[0];
        out.height = size[1];
        const ctx = out.getContext("2d");
        if (!ctx) return;
        ctx.fillStyle = "#ffffff";
        ctx.fillRect(0, 0, out.width, out.height);
        container.querySelectorAll<HTMLCanvasElement>(".ol-layer canvas, canvas.ol-layer").forEach((cv) => {
          if (cv.width === 0) return;
          const parent = cv.parentNode as HTMLElement | null;
          const opacity = parent?.style.opacity || cv.style.opacity;
          ctx.globalAlpha = opacity === "" ? 1 : Number(opacity);
          const tf = cv.style.transform;
          let m = tf ? tf.match(/^matrix\(([^(]*)\)$/)?.[1].split(",").map(Number) : undefined;
          if (!m || m.length !== 6) {
            const sx = (parseFloat(cv.style.width) || out.width) / cv.width;
            const sy = (parseFloat(cv.style.height) || out.height) / cv.height;
            m = [sx, 0, 0, sy, 0, 0];
          }
          ctx.setTransform(m[0], m[1], m[2], m[3], m[4], m[5]);
          const bg = parent?.style.backgroundColor;
          if (bg) {
            ctx.fillStyle = bg;
            ctx.fillRect(0, 0, cv.width, cv.height);
          }
          ctx.drawImage(cv, 0, 0);
        });
        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.globalAlpha = 1;
        downloadDataUrl(timestampName("map", "png"), out.toDataURL("image/png"));
        useStore.getState().setStatus("地図をキャプチャしました（PNG保存）", "success");
      });
      map.renderSync();
    };
    window.addEventListener("frs:capture", onCapture);
    return () => window.removeEventListener("frs:capture", onCapture);
  }, []);

  // ---- Fleet 再生（現在時刻 fleetSimT の各車位置）: 毎フレーム変わるため専用ソースだけを更新 ----
  useEffect(() => {
    const src = fleetSimSrcRef.current;
    src.clear();
    const showFleet = showSavedRoutes || activeFeature === "fleet";
    if (!fleetSim || !showFleet || !fleetSim.traces.length) return;
    fleetSim.traces.forEach((tr, i) => {
      if (!tr.length) return;
      const dt = tr[1]?.t ? tr[1].t - tr[0].t : 0.2;
      const idx = Math.max(0, Math.min(tr.length - 1, Math.round(fleetSimT / dt)));
      const fr = tr[idx];
      const color = FLEET_COLORS[i % FLEET_COLORS.length];
      const dot = new Feature(new Point([fr.x, fr.y]));
      dot.set("kind", "fleetveh");
      dot.set("color", color);
      dot.set("waiting", fr.state === "wait");
      src.addFeature(dot);
      const arr = new Feature(arrowGeom(fr.x, fr.y, fr.heading_deg));
      arr.set("kind", "fleetarrow");
      src.addFeature(arr);
    });
    // 自動配置された待避所（auto_passing で挿入）をアンバーで表示。
    (fleetSim.auto_bays ?? []).forEach((ab) => {
      const pts = savedRoutes[ab.vehicle]?.route?.trajectory?.points ?? [];
      if (pts.length < 2) return;
      const bp = bayPointAt(pts.map((p) => [p.x, p.y]), ab.s_center, ab.offset, ab.side);
      if (!bp) return;
      const link = new Feature(new LineString([bp.foot, bp.pos]));
      link.set("kind", "fleetautobaylink");
      src.addFeature(link);
      const m = new Feature(new Point(bp.pos));
      m.set("kind", "fleetautobay");
      src.addFeature(m);
    });
  }, [fleetSim, fleetSimT, savedRoutes, showSavedRoutes, activeFeature]);

  // ---- Analysisグラフのホバー点を地図上にハイライト＋車両矩形を動的表示（共有index）----
  // AnalysisPanel と同じ「アクティブ解析」を参照: 寄り付き工程で寄り付き解析があればその軌跡、
  // 無ければ経路の軌跡。グラフ上をホバーした s 位置に、向き付きの実車矩形を描く。
  useEffect(() => {
    const src = overlaySrcRef.current;
    if (highlightRef.current && src.hasFeature(highlightRef.current)) src.removeFeature(highlightRef.current);
    highlightRef.current = null;
    for (const f of footprintFeatsRef.current) {
      if (src.hasFeature(f)) src.removeFeature(f);
    }
    footprintFeatsRef.current = [];
    const pts =
      activeFeature === "spotting" && spotResult?.trajectory
        ? spotResult.trajectory.points
        : route?.trajectory.points;
    if (pts && hoverPointIndex != null && hoverPointIndex >= 0 && hoverPointIndex < pts.length) {
      const p = pts[hoverPointIndex];
      const f = new Feature(new Point([p.x, p.y]));
      f.set("kind", "hilite");
      src.addFeature(f);
      highlightRef.current = f;
      // 運動学を模擬した車両形状（各タイヤ/フレームの実経路接線から角度を算出 → 曲率に応じた舵角・
      // 前後進も自動で正しい）。articulated=2矩形 / rigid_bicycle=車体＋操舵輪 / tracked=矩形。
      if (vehDims) {
        const pPrev = pts[Math.max(0, hoverPointIndex - 1)];
        const pNext = pts[Math.min(pts.length - 1, hoverPointIndex + 1)];
        for (const { ring, kind } of vehicleShapeRings(p, pPrev, pNext, vehDims)) {
          const fp = new Feature(new Polygon([ring]));
          fp.set("kind", kind);
          src.addFeature(fp);
          footprintFeatsRef.current.push(fp);
        }
      }
    }
  }, [hoverPointIndex, route, spotResult, activeFeature, vehDims]);

  // ---- 寄り付き再生/スクラブ: 現在indexに向き付き車両（運動学つきフットプリント）を表示 ----
  useEffect(() => {
    const src = overlaySrcRef.current;
    for (const f of spotVehFeatsRef.current) if (src.hasFeature(f)) src.removeFeature(f);
    spotVehFeatsRef.current = [];
    const traj = spotResult?.trajectory?.points;
    const pts = spotResult?.points;
    if (!pts || pts.length === 0) return;
    const i = Math.min(Math.max(spotIndex, 0), pts.length - 1);
    if (traj && vehDims && i < traj.length) {
      // 解析軌跡（curvature・gear対応の車体方位つき）から、操舵輪/関節つきの車両形状を描画
      const p = traj[i];
      const pPrev = traj[Math.max(0, i - 1)];
      const pNext = traj[Math.min(traj.length - 1, i + 1)];
      for (const { ring, kind } of vehicleShapeRings(p, pPrev, pNext, vehDims)) {
        const f = new Feature(new Polygon([ring]));
        f.set("kind", kind);
        src.addFeature(f);
        spotVehFeatsRef.current.push(f);
      }
    } else {
      // 解析がない/車両未選択時は従来の点マーカー
      const f = new Feature(new Point([pts[i].x, pts[i].y]));
      f.set("kind", "spotvehicle");
      src.addFeature(f);
      spotVehFeatsRef.current.push(f);
    }
  }, [spotResult, spotIndex, vehDims]);

  // ---- mode → 頂点ドラッグは edit 時のみ（パンは全モードで常時有効）----
  useEffect(() => {
    modifyRef.current?.setActive(mode === "edit");
  }, [mode]);

  // ---- cost/drivable overlay opacity & visibility（独立トグル/スライダー）----
  useEffect(() => {
    for (const [id, ent] of rasterLayersRef.current) {
      const l = layers.find((x) => x.id === id);
      if (l?.kind === "cost") {
        ent.layer.setOpacity(costOpacity);
        ent.layer.setVisible(costVisible);
      } else if (l?.kind === "drivable") {
        ent.layer.setOpacity(drivableOpacity);
        ent.layer.setVisible(drivableVisible);
      }
    }
  }, [costOpacity, costVisible, drivableOpacity, drivableVisible, layers]);

  // ---- OSM ベースマップの表示トグル ----
  useEffect(() => {
    osmLayerRef.current?.setVisible(osmVisible);
  }, [osmVisible]);

  // ---- モード別カーソル（地図クリックの効果を視覚的に示す）----
  useEffect(() => {
    const map = mapRef.current; if (!map) return;
    const vp = map.getViewport();
    vp.style.cursor =
      mode === "pan" ? "grab"
      : mode === "edit" ? "pointer"
      : POSE_MODE_SET.has(mode) || mode === "polygon" || mode === "insert_via" || mode === "via" ? "crosshair"
      : "default";
  }, [mode]);

  const fit = () => {
    const map = mapRef.current;
    if (!map) return;
    const featExt: Extent | null = overlaySrcRef.current.getExtent();
    const featValid =
      !!featExt &&
      featExt.every((v) => Number.isFinite(v)) &&
      featExt[2] > featExt[0] &&
      featExt[3] > featExt[1];
    const ext: Extent | null = featValid ? featExt : dataExtentRef.current;
    if (ext && ext.every((v) => Number.isFinite(v)) && ext[2] > ext[0]) {
      map.getView().fit(ext, { padding: [40, 40, 40, 40], maxZoom: 23, duration: 200 });
    }
  };

  const zoomBy = (delta: number) => {
    const view = mapRef.current?.getView();
    if (!view) return;
    const z = view.getZoom();
    if (z != null) view.animate({ zoom: z + delta, duration: 150 });
  };

  const setActiveFeature = useStore((s) => s.setActiveFeature);
  const hasData = layers.some((l) => l.kind === "las" || l.kind === "cost" || l.kind === "ortho");

  return (
    <div className="map-wrap">
      <div ref={containerRef} className="ol-map" />
      {!hasData && (
        <div className="map-empty">
          <div className="me-card">
            <b>はじめに</b>
            <ol>
              <li><button className="link" onClick={() => setActiveFeature("data")}>データ</button> で LAS / オルソを追加</li>
              <li><button className="link" onClick={() => setActiveFeature("map")}>マップ生成</button> でコストマップ＆走行可能領域</li>
              <li><button className="link" onClick={() => setActiveFeature("route")}>経路</button> で waypoint を置いて生成</li>
            </ol>
            <span className="me-hint">右上「OSM」で地図の地理的文脈を表示できます</span>
          </div>
        </div>
      )}
      <div className="map-modechip">
        {MODE_LABELS[mode] ?? mode}
        {POSE_MODE_SET.has(mode) && <span className="mc-hint"> ・ クリック=位置 / 左ドラッグ=方位 / 右ドラッグ=移動</span>}
        {mode === "polygon" && <span className="mc-hint"> ・ クリックで頂点追加</span>}
      </div>
      <div className="map-toolbar">
        <button onClick={() => zoomBy(1)} title="ズームイン">
          ＋
        </button>
        <button onClick={() => zoomBy(-1)} title="ズームアウト">
          －
        </button>
        <button onClick={fit} title="データ全体にフィット">
          Fit
        </button>
        <button
          data-active={osmVisible}
          onClick={() => useStore.getState().setOsmVisible(!osmVisible)}
          title="OpenStreetMap ベースマップの表示/非表示"
        >
          OSM
        </button>
      </div>
    </div>
  );
}
