// 3Dビュー（three.js / 非地理シーン）。メートル座標(6677)を x=東西, z=南北, y=標高 にマップ。
// 表示は LAS点群(RGB) / DSM地形メッシュ を切替。経路・寄り付き・ウェイポイントを重畳。OrbitControls。

import { useEffect, useRef } from "react";

import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

import { api } from "@/api/client";
import type { PointCloud } from "@/api/client";
import { dispatch } from "@/commandBus";
import { pickLayer } from "@/layerSelect";
import { useStore } from "@/store/useStore";
import type { XY } from "@/types/api";

interface Grid {
  nx: number; ny: number; x0: number; y0: number; dx: number; dy: number;
  z: (number | null)[][]; zmin: number; zmax: number;
}

function elevColor(t: number): [number, number, number] {
  const stops: [number, number[]][] = [
    [0.0, [40, 90, 120]], [0.25, [60, 140, 90]], [0.5, [180, 190, 90]],
    [0.75, [150, 110, 70]], [1.0, [240, 240, 240]],
  ];
  for (let i = 1; i < stops.length; i++) {
    if (t <= stops[i][0]) {
      const [a, ca] = stops[i - 1];
      const [b, cb] = stops[i];
      const f = (t - a) / (b - a || 1);
      return [0, 1, 2].map((k) => (ca[k] + (cb[k] - ca[k]) * f) / 255) as [number, number, number];
    }
  }
  return [1, 1, 1];
}

export function ThreeView() {
  const containerRef = useRef<HTMLDivElement>(null);
  const st = useRef<{
    renderer: THREE.WebGLRenderer; scene: THREE.Scene; camera: THREE.PerspectiveCamera;
    controls: OrbitControls; content: THREE.Group; terrain: THREE.Group; points: THREE.Group;
    edit: THREE.Group; // 作成中ポリゴン＋編集ハンドル
    raycaster: THREE.Raycaster; ndc: THREE.Vector2;
    drag: { kind: "waypoint" | "areavtx"; id: string; index: number; obj: THREE.Object3D } | null;
    origin: { ox: number; oy: number; oz: number }; grid: Grid | null; pts: PointCloud | null; raf: number;
  } | null>(null);

  const layers = useStore((s) => s.layers);
  const route = useStore((s) => s.route);
  const areas = useStore((s) => s.areas);
  const activePolygon = useStore((s) => s.activePolygon);
  const mode = useStore((s) => s.mode);
  const waypoints = useStore((s) => s.waypoints);
  const spotResult = useStore((s) => s.spotResult);
  const spotStart = useStore((s) => s.spotStart);
  const spotTarget = useStore((s) => s.spotTarget);
  const pilePlan = useStore((s) => s.pilePlan);
  const vExag = useStore((s) => s.vExag);
  const roadWidthM = useStore((s) => s.roadWidthM);
  const mode3d = useStore((s) => s.view3dMode);
  const pointSize = useStore((s) => s.pointSize);
  const pointBudget = useStore((s) => s.pointBudget);
  const setStatus = useStore((s) => s.setStatus);

  const costLayerId = useStore((s) => s.costLayerId);
  const costLayer = pickLayer(layers, "cost", costLayerId);
  const lasLayer = [...layers].reverse().find((l) => l.kind === "las");

  // ---- init once ----
  useEffect(() => {
    const el = containerRef.current;
    if (!el || st.current) return;
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(el.clientWidth, el.clientHeight);
    renderer.setClearColor(0x0a1626);
    el.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(55, el.clientWidth / el.clientHeight, 0.5, 50000);
    camera.position.set(120, 140, 160);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    scene.add(new THREE.AmbientLight(0xffffff, 0.7));
    const sun = new THREE.DirectionalLight(0xffffff, 0.8);
    sun.position.set(150, 300, 120);
    scene.add(sun);
    scene.add(new THREE.GridHelper(400, 40, 0x335577, 0x223344));

    const terrain = new THREE.Group();
    const points = new THREE.Group();
    const content = new THREE.Group();
    const edit = new THREE.Group();
    scene.add(terrain, points, content, edit);

    const s = {
      renderer, scene, camera, controls, content, terrain, points, edit,
      raycaster: new THREE.Raycaster(), ndc: new THREE.Vector2(),
      drag: null as null | { kind: "waypoint" | "areavtx"; id: string; index: number; obj: THREE.Object3D },
      origin: { ox: 0, oy: 0, oz: 0 }, grid: null as Grid | null, pts: null as PointCloud | null, raf: 0,
    };
    st.current = s;
    const loop = () => { s.raf = requestAnimationFrame(loop); controls.update(); renderer.render(scene, camera); };
    loop();

    // ---- 2D同様の編集操作（クリックで作成 / editでドラッグ） ----
    const dom = renderer.domElement;
    let downNdcX = 0, downNdcY = 0, moved = false;
    const onDown = (e: PointerEvent) => {
      if (e.button !== 0) return; // 左ボタンのみ
      const m = useStore.getState().mode;
      downNdcX = e.clientX; downNdcY = e.clientY; moved = false;
      if (m === "edit") {
        const h = pickHandle(e);
        if (h) {
          const ud = h.userData as { kind: "waypoint" | "areavtx"; id: string; index: number };
          s.drag = { ...ud, obj: h };
          s.controls.enabled = false; // ドラッグ中はカメラ操作を止める
        }
      }
    };
    const onMove = (e: PointerEvent) => {
      if (Math.hypot(e.clientX - downNdcX, e.clientY - downNdcY) > 4) moved = true;
      const d = s.drag;
      if (d) {
        // store は変更せず（履歴破壊防止）、掴んだハンドルだけライブで動かす。確定は up。
        const w = screenToWorld(e); if (!w) return;
        const lift = d.kind === "waypoint" ? 1.0 : 0.8;
        const [lx, ly, lz] = toLocal(w.x, w.y, sampleElev(w.x, w.y) + lift);
        d.obj.position.set(lx, ly, lz);
      }
    };
    const onUp = (e: PointerEvent) => {
      const m = useStore.getState().mode;
      const d = s.drag;
      if (d) {
        s.drag = null; s.controls.enabled = true;
        const w = screenToWorld(e);
        if (w) {
          if (d.kind === "waypoint") dispatch({ type: "MOVE_WAYPOINT", id: d.id, xy: w });
          else {
            const a = useStore.getState().areas.find((p) => p.id === d.id);
            if (a) dispatch({ type: "SET_AREA_POINTS", id: d.id, points: a.points.map((p, i) => (i === d.index ? w : { ...p })) });
          }
        } else {
          rebuildEdit(); // 投影に失敗したらハンドルを元位置へ戻す
        }
        return;
      }
      if (moved) return; // ドラッグ（カメラ回転）はクリック扱いしない
      const w = screenToWorld(e); if (!w) return;
      if (m === "start" || m === "goal") dispatch({ type: "ADD_WAYPOINT", role: m, xy: w });
      else if (m === "via" || m === "insert_via") dispatch({ type: "INSERT_VIA", xy: w });
      else if (m === "polygon") dispatch({ type: "ADD_POLY_VERTEX", xy: w });
    };
    dom.addEventListener("pointerdown", onDown);
    dom.addEventListener("pointermove", onMove);
    dom.addEventListener("pointerup", onUp);
    const onResize = () => {
      if (!containerRef.current) return;
      const w = containerRef.current.clientWidth, h = containerRef.current.clientHeight;
      renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix();
    };
    window.addEventListener("resize", onResize);
    const ro = new ResizeObserver(onResize); ro.observe(el);
    return () => {
      cancelAnimationFrame(s.raf); window.removeEventListener("resize", onResize); ro.disconnect();
      dom.removeEventListener("pointerdown", onDown);
      dom.removeEventListener("pointermove", onMove);
      dom.removeEventListener("pointerup", onUp);
      // 各グループの geometry/material を最終破棄（アンマウント時の GPU メモリリーク防止）。
      disposeGroup(s.terrain); disposeGroup(s.points); disposeGroup(s.content); disposeGroup(s.edit);
      controls.dispose(); renderer.dispose();
      if (renderer.domElement.parentElement === el) el.removeChild(renderer.domElement);
      st.current = null;
    };
  }, []);

  // 標高は素のメートルで配置し、鉛直強調(vExag)は group.scale.y で表現する（スライダーで
  // 頂点バッファを作り直さずに反映＝軽量・GPUリーク回避）。
  function toLocal(x: number, y: number, elev: number): [number, number, number] {
    const o = st.current!.origin;
    return [x - o.ox, elev - o.oz, -(y - o.oy)];
  }

  // 再構築時に旧 geometry/material を破棄（dispose し忘れによる GPU メモリリークを防ぐ）。
  function disposeGroup(g: THREE.Group) {
    g.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (mesh.geometry) mesh.geometry.dispose();
      const mat = (mesh as unknown as { material?: THREE.Material | THREE.Material[] }).material;
      if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
      else if (mat) mat.dispose();
    });
    g.clear();
  }

  function applyVExag() {
    const s = st.current; if (!s) return;
    s.terrain.scale.y = vExag; s.points.scale.y = vExag; s.content.scale.y = vExag; s.edit.scale.y = vExag;
  }

  // 画面座標 → ワールドXY（作業CRS[m]）。地形メッシュに当て、外れたら平均標高平面へ投影。
  function screenToWorld(e: PointerEvent): XY | null {
    const s = st.current; if (!s) return null;
    const rect = s.renderer.domElement.getBoundingClientRect();
    s.ndc.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    s.ndc.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    s.raycaster.setFromCamera(s.ndc, s.camera);
    let hit: THREE.Vector3 | null = null;
    if (s.terrain.visible && s.terrain.children.length) {
      const ix = s.raycaster.intersectObjects(s.terrain.children, true);
      if (ix.length) hit = ix[0].point;
    }
    if (!hit) {
      // 地形なし/外した時は y=0 平面（=平均標高oz）へ投影。XYのみ使うので可。
      const p = new THREE.Vector3();
      if (!s.raycaster.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), p)) return null;
      hit = p;
    }
    // local(scene) → world: 群はyのみscaleなのでx,zは不変。x=hit.x+ox, y=oy-hit.z
    return { x: hit.x + s.origin.ox, y: s.origin.oy - hit.z };
  }

  // 編集ハンドル（球）をレイキャストで拾う。userData{kind,id,index} を返す。
  function pickHandle(e: PointerEvent): THREE.Object3D | null {
    const s = st.current; if (!s) return null;
    const rect = s.renderer.domElement.getBoundingClientRect();
    s.ndc.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    s.ndc.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    s.raycaster.setFromCamera(s.ndc, s.camera);
    const ix = s.raycaster.intersectObjects(s.edit.children, true);
    for (const h of ix) {
      let o: THREE.Object3D | null = h.object;
      while (o && !(o.userData && o.userData.kind)) o = o.parent;
      if (o && o.userData.kind) return o;
    }
    return null;
  }

  // 作成中ポリゴン(activePolygon)＋編集ハンドル(editモード)を再構築。
  function rebuildEdit() {
    const s = st.current; if (!s) return;
    disposeGroup(s.edit);
    const stt = useStore.getState();
    const ap = stt.activePolygon;
    if (ap.length) {
      ap.forEach((p) => {
        const m = new THREE.Mesh(new THREE.SphereGeometry(1.0, 10, 8), new THREE.MeshBasicMaterial({ color: 0xfacc15 }));
        const [lx, ly, lz] = toLocal(p.x, p.y, sampleElev(p.x, p.y) + 0.6); m.position.set(lx, ly, lz); s.edit.add(m);
      });
      const ring = ap.length >= 3 ? [...ap, ap[0]] : ap;
      const v: number[] = [];
      for (const p of ring) { const [lx, ly, lz] = toLocal(p.x, p.y, sampleElev(p.x, p.y) + 0.6); v.push(lx, ly, lz); }
      const geo = new THREE.BufferGeometry(); geo.setAttribute("position", new THREE.Float32BufferAttribute(v, 3));
      s.edit.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0xfacc15 })));
    }
    if (stt.mode === "edit") {
      stt.waypoints.forEach((wp) => {
        const h = new THREE.Mesh(new THREE.SphereGeometry(1.6, 12, 8), new THREE.MeshBasicMaterial({ color: 0xffffff }));
        const [lx, ly, lz] = toLocal(wp.xy.x, wp.xy.y, sampleElev(wp.xy.x, wp.xy.y) + 1.0); h.position.set(lx, ly, lz);
        h.userData = { kind: "waypoint", id: wp.id, index: 0 }; s.edit.add(h);
      });
      stt.areas.forEach((a) => a.points.forEach((p, i) => {
        const h = new THREE.Mesh(new THREE.SphereGeometry(1.3, 12, 8), new THREE.MeshBasicMaterial({ color: 0x22c55e }));
        const [lx, ly, lz] = toLocal(p.x, p.y, sampleElev(p.x, p.y) + 0.8); h.position.set(lx, ly, lz);
        h.userData = { kind: "areavtx", id: a.id, index: i }; s.edit.add(h);
      }));
    }
    s.edit.scale.y = vExag;
  }

  function applyPointSize() {
    const s = st.current; if (!s) return;
    s.points.traverse((o) => {
      const mat = (o as THREE.Points).material as THREE.PointsMaterial | undefined;
      if (mat && "size" in mat) { mat.size = pointSize; mat.needsUpdate = true; }
    });
  }

  function sampleElev(x: number, y: number): number {
    const s = st.current; const g = s?.grid;
    if (!s) return 0;
    if (!g) return s.origin.oz;
    const fc = (x - g.x0) / g.dx, fr = (y - g.y0) / g.dy;
    const c0 = Math.floor(fc), r0 = Math.floor(fr);
    if (c0 < 0 || r0 < 0 || c0 + 1 >= g.nx || r0 + 1 >= g.ny) return s.origin.oz;
    const z00 = g.z[r0][c0], z01 = g.z[r0][c0 + 1], z10 = g.z[r0 + 1][c0], z11 = g.z[r0 + 1][c0 + 1];
    const vals = [z00, z01, z10, z11].filter((v): v is number => v != null);
    if (vals.length < 4) return vals.length ? vals.reduce((a, b) => a + b, 0) / vals.length : s.origin.oz;
    const fx = fc - c0, fy = fr - r0;
    return z00! * (1 - fx) * (1 - fy) + z01! * fx * (1 - fy) + z10! * (1 - fx) * fy + z11! * fx * fy;
  }

  function buildTerrain() {
    const s = st.current; if (!s) return;
    disposeGroup(s.terrain);
    const g = s.grid; if (!g) return;
    const pos: number[] = [], col: number[] = [];
    const span = g.zmax - g.zmin || 1;
    for (let r = 0; r < g.ny; r++) for (let c = 0; c < g.nx; c++) {
      const x = g.x0 + g.dx * c, y = g.y0 + g.dy * r;
      const zv = g.z[r][c]; const elev = zv == null ? g.zmin : zv;
      const [lx, ly, lz] = toLocal(x, y, elev); pos.push(lx, ly, lz);
      const [cr, cg, cb] = elevColor((elev - g.zmin) / span); col.push(cr, cg, cb);
    }
    const idx: number[] = [];
    for (let r = 0; r < g.ny - 1; r++) for (let c = 0; c < g.nx - 1; c++) {
      const a = r * g.nx + c, b = a + 1, d = a + g.nx, e = d + 1;
      idx.push(a, d, b, b, d, e);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
    geo.setAttribute("color", new THREE.Float32BufferAttribute(col, 3));
    geo.setIndex(idx); geo.computeVertexNormals();
    s.terrain.add(new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ vertexColors: true, side: THREE.DoubleSide })));
  }

  function buildPoints() {
    const s = st.current; if (!s) return;
    disposeGroup(s.points);
    const p = s.pts; if (!p || p.n < 1) return;
    // バイナリ点群は点群中心(origin)相対の f32。シーン原点との差分だけ足して一括変換
    // （100万点級でも JS ループ1本＋色は uint8 正規化属性で GPU 直渡し）。
    const o = s.origin;
    const dx = p.origin[0] - o.ox;
    const dy = p.origin[1] - o.oy;
    const dz = p.origin[2] - o.oz;
    const n = p.n;
    const pos = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      pos[i * 3] = p.x[i] + dx;
      pos[i * 3 + 1] = p.z[i] + dz;
      pos[i * 3 + 2] = -(p.y[i] + dy);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
    if (p.rgb) {
      geo.setAttribute("color", new THREE.BufferAttribute(p.rgb, 3, true)); // uint8 → 0..1 正規化
    } else {
      const col = new Float32Array(n * 3);
      const span = p.zmax - p.zmin || 1;
      for (let i = 0; i < n; i++) {
        const [cr, cg, cb] = elevColor((p.z[i] + p.origin[2] - p.zmin) / span);
        col[i * 3] = cr; col[i * 3 + 1] = cg; col[i * 3 + 2] = cb;
      }
      geo.setAttribute("color", new THREE.BufferAttribute(col, 3));
    }
    s.points.add(new THREE.Points(geo, new THREE.PointsMaterial({ size: pointSize, vertexColors: true, sizeAttenuation: false })));
  }

  function lineFromXY(pts: (XY & { z?: number | null })[], color: number, lift: number) {
    if (pts.length < 2) return null;
    const v: number[] = [];
    for (const p of pts) {
      // 埋め込み済み標高 z（点群DSM実測）があれば優先。無ければ粗い表示グリッドから補間。
      const base = p.z ?? sampleElev(p.x, p.y);
      const [lx, ly, lz] = toLocal(p.x, p.y, base + lift);
      v.push(lx, ly, lz);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(v, 3));
    return new THREE.Line(geo, new THREE.LineBasicMaterial({ color }));
  }
  function marker(x: number, y: number, color: number, r = 1.6) {
    const m = new THREE.Mesh(new THREE.SphereGeometry(r, 12, 8), new THREE.MeshStandardMaterial({ color }));
    const [lx, ly, lz] = toLocal(x, y, sampleElev(x, y) + r); m.position.set(lx, ly, lz); return m;
  }
  // 道幅帯（中心線±halfW を地形に沿って貼る半透明リボン）
  function roadBand(pts: XY[], halfW: number) {
    const n = pts.length;
    if (n < 2 || halfW <= 0) return null;
    const lift = 0.4;
    const vert = (x: number, y: number) => toLocal(x, y, sampleElev(x, y) + lift);
    const pos: number[] = [];
    const edges = (i: number): [[number, number], [number, number]] => {
      const a = pts[Math.max(0, i - 1)], b = pts[Math.min(n - 1, i + 1)];
      let tx = b.x - a.x, ty = b.y - a.y;
      const L = Math.hypot(tx, ty) || 1; tx /= L; ty /= L;
      const nx = -ty, ny = tx;
      return [[pts[i].x + nx * halfW, pts[i].y + ny * halfW], [pts[i].x - nx * halfW, pts[i].y - ny * halfW]];
    };
    for (let i = 0; i < n - 1; i++) {
      const [l0, r0] = edges(i);
      const [l1, r1] = edges(i + 1);
      const L0 = vert(l0[0], l0[1]), R0 = vert(r0[0], r0[1]), L1 = vert(l1[0], l1[1]), R1 = vert(r1[0], r1[1]);
      pos.push(...L0, ...R0, ...L1, ...R0, ...R1, ...L1);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
    return new THREE.Mesh(
      geo,
      new THREE.MeshBasicMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.4, side: THREE.DoubleSide, depthWrite: false }),
    );
  }

  // エリア（ポリゴン）を地形に沿わせて表示（2Dと同色の緑: 塗り半透明＋枠線）。
  function areaGroup(pts: XY[], color = 0x22c55e) {
    const g = new THREE.Group();
    if (pts.length < 3) return g;
    // 枠線（閉ループ・少し浮かせて地形に埋もれないように）
    const ring = [...pts, pts[0]];
    const lv: number[] = [];
    for (const p of ring) { const [lx, ly, lz] = toLocal(p.x, p.y, sampleElev(p.x, p.y) + 0.6); lv.push(lx, ly, lz); }
    const lgeo = new THREE.BufferGeometry();
    lgeo.setAttribute("position", new THREE.Float32BufferAttribute(lv, 3));
    g.add(new THREE.Line(lgeo, new THREE.LineBasicMaterial({ color })));
    // 塗り（多角形を三角形分割し、各頂点を地形標高に沿わせる）
    const contour = pts.map((p) => new THREE.Vector2(p.x, p.y));
    const tris = THREE.ShapeUtils.triangulateShape(contour, []);
    const fv: number[] = [];
    for (const t of tris) for (const idx of t) {
      const p = pts[idx]; const [lx, ly, lz] = toLocal(p.x, p.y, sampleElev(p.x, p.y) + 0.4); fv.push(lx, ly, lz);
    }
    if (fv.length) {
      const fgeo = new THREE.BufferGeometry();
      fgeo.setAttribute("position", new THREE.Float32BufferAttribute(fv, 3));
      g.add(new THREE.Mesh(fgeo, new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.2, side: THREE.DoubleSide, depthWrite: false })));
    }
    return g;
  }

  function rebuildContent() {
    const s = st.current; if (!s) return;
    disposeGroup(s.content);
    areas.forEach((a) => s.content.add(areaGroup(a.points)));

    // 排土（パイル）計画: 実寸の円錐（安息角の錐体）を地形に沿わせて配置。
    // 全パイル同一寸法なので InstancedMesh 1つで描画（数百個でも軽量）。
    if (pilePlan && pilePlan.centers.length) {
      const r = pilePlan.pile.radius_m;
      const h = pilePlan.pile.height_m;
      if (r > 0 && h > 0) {
        const geo = new THREE.ConeGeometry(r, h, 28);
        const mat = new THREE.MeshStandardMaterial({ color: 0xb45309, transparent: true, opacity: 0.92, roughness: 0.95 });
        const inst = new THREE.InstancedMesh(geo, mat, pilePlan.centers.length);
        const m4 = new THREE.Matrix4();
        pilePlan.centers.forEach(([px, py], i) => {
          // ConeGeometry は高さ中央が原点（頂点+Y）→ 接地させるため 地面標高 + h/2 に置く
          const [lx, ly, lz] = toLocal(px, py, sampleElev(px, py) + h / 2);
          m4.makeTranslation(lx, ly, lz);
          inst.setMatrixAt(i, m4);
        });
        inst.instanceMatrix.needsUpdate = true;
        s.content.add(inst);
      }
    }
    if (route && route.trajectory.points.length > 1) {
      const center = route.trajectory.points.map((p) => ({ x: p.x, y: p.y, z: p.z }));
      if (roadWidthM > 0) {
        const band = roadBand(center, roadWidthM / 2);
        if (band) s.content.add(band);
      }
      const l = lineFromXY(center, 0xf59e0b, 0.8);
      if (l) s.content.add(l);
    }
    waypoints.forEach((w) => s.content.add(marker(w.xy.x, w.xy.y, w.role === "start" ? 0x22c55e : w.role === "goal" ? 0xef4444 : 0x0ea5e9)));
    if (spotResult && spotResult.points.length > 1) {
      let runStart = 0; const pts = spotResult.points;
      // 寄り付きの標高は解析軌跡（同一点列から構築＝index 対応）の z を使う
      const tz = spotResult.trajectory?.points?.length === pts.length ? spotResult.trajectory.points : null;
      for (let i = 1; i <= pts.length; i++) {
        if (i === pts.length || pts[i].gear !== pts[runStart].gear) {
          const seg = pts.slice(runStart, i).map((p, k) => ({ x: p.x, y: p.y, z: tz?.[runStart + k]?.z }));
          const l = lineFromXY(seg, pts[runStart].gear === "R" ? 0xf97316 : 0x22d3ee, 1.0);
          if (l) s.content.add(l); runStart = i;
        }
      }
    }
    if (spotStart) s.content.add(marker(spotStart.x, spotStart.y, 0xa855f7));
    if (spotTarget) s.content.add(marker(spotTarget.x, spotTarget.y, 0xf97316));
  }

  function applyMode() {
    const s = st.current; if (!s) return;
    s.terrain.visible = mode3d === "mesh";
    s.points.visible = mode3d === "points";
  }

  function fitCamera() {
    const s = st.current; if (!s) return;
    let half = 150;
    if (s.grid) half = Math.max(Math.abs(s.grid.dx * s.grid.nx), Math.abs(s.grid.dy * s.grid.ny)) / 2;
    else if (s.pts && s.pts.n) {
      // typed array は spread 不可（引数上限）なのでループで bbox を取る
      const p = s.pts;
      let minx = Infinity, maxx = -Infinity, miny = Infinity, maxy = -Infinity;
      for (let i = 0; i < p.n; i++) {
        const X = p.x[i], Y = p.y[i];
        if (X < minx) minx = X;
        if (X > maxx) maxx = X;
        if (Y < miny) miny = Y;
        if (Y > maxy) maxy = Y;
      }
      half = Math.max(maxx - minx, maxy - miny) / 2 || 150;
    }
    s.camera.position.set(half * 0.9, half * 1.1, half * 1.3);
    s.controls.target.set(0, 0, 0); s.controls.update();
  }

  // ---- データ読み込み（grid + points）→ 原点・構築 ----
  useEffect(() => {
    const s = st.current; if (!s) return;
    let cancelled = false;
    const ac = new AbortController();
    (async () => {
      const [grid, pts] = await Promise.all([
        costLayer ? api.dsmGrid(costLayer.id, 240, ac.signal).catch(() => null) : Promise.resolve(null),
        lasLayer ? api.layerPointsBin(lasLayer.id, pointBudget, ac.signal).catch(() => null) : Promise.resolve(null),
      ]);
      if (cancelled || !st.current) return;
      s.grid = grid; s.pts = pts;
      if (grid) {
        s.origin = { ox: grid.x0 + (grid.dx * (grid.nx - 1)) / 2, oy: grid.y0 + (grid.dy * (grid.ny - 1)) / 2, oz: (grid.zmin + grid.zmax) / 2 };
      } else if (pts && pts.n) {
        s.origin = { ox: pts.origin[0], oy: pts.origin[1], oz: pts.origin[2] };
      } else {
        const p = route?.trajectory.points ?? [];
        s.origin = p.length ? { ox: p[0].x, oy: p[0].y, oz: 0 } : { ox: 0, oy: 0, oz: 0 };
      }
      buildTerrain(); buildPoints(); rebuildContent(); rebuildEdit(); applyVExag(); applyPointSize(); applyMode(); fitCamera();
      if (mode3d === "points" && !pts) setStatus("3D点群: LASレイヤがありません（地形メッシュに切替可）");
      else if (pts && pts.n) setStatus(`3D点群: ${pts.n.toLocaleString()} 点を表示（上限 ${pointBudget.toLocaleString()}）`, "info");
    })();
    return () => { cancelled = true; ac.abort(); };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [costLayer?.id, lasLayer?.id, pointBudget]);

  // route/spotting/wp/道幅 変化は **content のみ**再構築（terrain/points は作り直さない＝重い再確保を回避）
  useEffect(() => {
    if (!st.current) return;
    rebuildContent();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [route, areas, waypoints, spotResult, spotStart, spotTarget, roadWidthM, pilePlan]);

  // 作成中ポリゴン・編集ハンドルは edit グループのみ再構築
  useEffect(() => {
    if (!st.current) return;
    rebuildEdit();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [activePolygon, mode, areas, waypoints]);

  // 鉛直強調は group.scale.y で反映（頂点再構築なし）
  useEffect(() => { applyVExag(); /* eslint-disable-next-line react-hooks/exhaustive-deps */ }, [vExag]);

  // 点サイズは material.size で反映（バッファ再確保なし）
  useEffect(() => { applyPointSize(); /* eslint-disable-next-line react-hooks/exhaustive-deps */ }, [pointSize]);

  // 表示モード切替（再構築不要、可視性のみ）
  useEffect(() => { applyMode(); /* eslint-disable-next-line react-hooks/exhaustive-deps */ }, [mode3d]);

  return <div ref={containerRef} className="ol-map" />;
}
