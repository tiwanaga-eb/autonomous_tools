// 地図オーバーレイの純粋ジオメトリ・ヘルパ（MapView から分離）。
// OL Feature を作らず座標リング/ジオメトリだけを返す＝単体テスト可能。
import { MultiLineString } from "ol/geom";

// 経路点列の弧長 s_abs[m] 位置に、横オフセット offset を side 方向へ取った点と本線上の足を返す。
export function bayPointAt(xy: number[][], sAbs: number, offset: number, side: number): { foot: number[]; pos: number[] } | null {
  if (xy.length < 2) return null;
  const seg = xy.slice(1).map((p, k) => Math.hypot(p[0] - xy[k][0], p[1] - xy[k][1]));
  let target = sAbs;
  let idx = 0;
  for (let k = 0; k < seg.length; k++) { if (target <= seg[k]) { idx = k; break; } target -= seg[k]; idx = k + 1; }
  const j = Math.min(idx, xy.length - 2);
  const p0 = xy[j], p1 = xy[j + 1];
  const dx = p1[0] - p0[0], dy = p1[1] - p0[1];
  const L = Math.hypot(dx, dy) || 1;
  const nx = (-dy / L) * side, ny = (dx / L) * side;
  return { foot: [p0[0], p0[1]], pos: [p0[0] + offset * nx, p0[1] + offset * ny] };
}

// (x,y) に heading[deg] で向けた車体矩形(全長l×全幅w)の四隅リングを返す（中心基準）。
export function carRect(x: number, y: number, headingDeg: number, l: number, w: number): number[][] {
  const a = (headingDeg * Math.PI) / 180;
  const ca = Math.cos(a);
  const sa = Math.sin(a);
  const hl = l / 2;
  const hw = w / 2;
  const ring: number[][] = ([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]] as [number, number][]).map(
    ([bx, by]) => [x + bx * ca - by * sa, y + bx * sa + by * ca],
  );
  ring.push(ring[0]);
  return ring;
}

// 基準点(x,y)から前 frontExt / 後 rearExt（符号付き, 後は負）に伸ばす非対称矩形。
// 基準点が車体中心でない車両（HM400=後輪軸中心）の車体を正しい位置に描く。
export function bodyRect(x: number, y: number, angleRad: number, frontExt: number, rearExt: number, w: number): number[][] {
  const ca = Math.cos(angleRad);
  const sa = Math.sin(angleRad);
  const hw = w / 2;
  const ring: number[][] = ([[frontExt, hw], [frontExt, -hw], [rearExt, -hw], [rearExt, hw]] as [number, number][]).map(
    ([bx, by]) => [x + bx * ca - by * sa, y + bx * sa + by * ca],
  );
  ring.push(ring[0]);
  return ring;
}

// ヒンジ点(hx,hy)から angle[rad] 方向へ length 伸ばす矩形（幅 width を横に中心化）。
export function segRectRing(hx: number, hy: number, angle: number, length: number, width: number): number[][] {
  const ca = Math.cos(angle);
  const sa = Math.sin(angle);
  const px = -sa;
  const py = ca;
  const hw = width / 2;
  const ex = hx + length * ca;
  const ey = hy + length * sa;
  const ring = [
    [hx + hw * px, hy + hw * py],
    [hx - hw * px, hy - hw * py],
    [ex - hw * px, ey - hw * py],
    [ex + hw * px, ey + hw * py],
  ];
  ring.push(ring[0]);
  return ring;
}

// 選択車両の運動学つき形状（ホバー点の描画用）。
export interface VehShape {
  l: number; w: number; kind: string;
  wheelBase: number | null; trackWidth: number | null;
  frontLen: number | null; rearLen: number | null;
  maxArtic: number | null; maxSteer: number | null;
  // 基準点(姿勢x,y,yaw)から見た車体の前後端 [m]（+前 / −後）。footprint_polygon の x 範囲。
  // 対称車両では ±l/2、HM400 は基準点=後輪軸中心なので非対称（前+8.32 / 後−2.785）。
  frontExt: number | null; rearExt: number | null;
}

export type Pose = { x: number; y: number; heading_deg: number; curvature?: number; gear?: string | null };

// 運動学を模擬した車両形状。舵角は曲率ベース δ=atan(L·κ)（アッカーマンで内外輪差）、関節角は κ·wheel_base。
// 旋回方向の符号は位置の外積（CCW+）から、後進は実機構どおり逆位相（バイシクルモデルで確認）。
// articulated: 前後2矩形をくの字に。rigid_bicycle: 車体＋アッカーマン操舵前輪。その他: 車体のみ。
export function vehicleShapeRings(
  p: Pose, pPrev: Pose, pNext: Pose, v: VehShape,
): { ring: number[][]; kind: string }[] {
  const out: { ring: number[][]; kind: string }[] = [];
  const a = (p.heading_deg * Math.PI) / 180;
  // 旋回方向（外積 CCW+）。前進=曲率方向、後進=逆位相。
  const cross = (p.x - pPrev.x) * (pNext.y - p.y) - (p.y - pPrev.y) * (pNext.x - p.x);
  let sgn = cross > 1e-9 ? 1 : cross < -1e-9 ? -1 : 0;
  if (p.gear === "R") sgn = -sgn;
  const kappa = Math.abs(p.curvature ?? 0); // 曲率の大きさ（符号は sgn）

  const frontExt = v.frontExt ?? v.l / 2;   // 基準点→前端（+）
  const rearExt = v.rearExt ?? -v.l / 2;    // 基準点→後端（−）

  if (v.kind === "articulated") {
    const fl = v.frontLen ?? v.l / 2;
    const rl = v.rearLen ?? v.l / 2;
    const base = v.wheelBase ?? v.l * 0.6;
    const cap = v.maxArtic ?? (45 * Math.PI) / 180;
    const g = Math.max(-cap, Math.min(cap, sgn * kappa * base)); // 関節角（曲率×WB, capで頭打ち）
    // 基準点=後輪軸中心。中折れ関節は前方 jointFwd = 前端 − 前ユニット長。
    // 後ユニットは基準フレーム（heading a）に固定、前ユニットのみ関節角 g で中折れ。
    const jointFwd = frontExt - fl;
    const jx = p.x + jointFwd * Math.cos(a);
    const jy = p.y + jointFwd * Math.sin(a);
    out.push({ ring: segRectRing(jx, jy, a + g, fl, v.w), kind: "footprint" });        // 前ユニット（中折れ）
    out.push({ ring: segRectRing(jx, jy, a + Math.PI, rl, v.w), kind: "footprint" });  // 後ユニット（基準フレーム）
  } else if (v.kind === "rigid_bicycle") {
    out.push({ ring: bodyRect(p.x, p.y, a, frontExt, rearExt, v.w), kind: "footprint" });
    const L = v.wheelBase ?? v.l * 0.6;
    const t = v.trackWidth ?? v.w * 0.85;
    const ca = Math.cos(a);
    const sa = Math.sin(a);
    // タイヤ実寸（上面視）: 進行方向長＝タイヤ直径、横＝断面幅。大型ダンプは直径2.5m級。
    const wlen = Math.max(v.l * 0.26, 1.6); // タイヤ直径
    const wwid = Math.max(v.w * 0.13, 0.5); // タイヤ断面幅（シングル1本）
    const cap = v.maxSteer ?? (45 * Math.PI) / 180;
    let dL = 0;
    let dR = 0;
    if (kappa > 1e-6 && sgn !== 0) {
      const R = 1 / (sgn * kappa); // 符号付き旋回半径。アッカーマン: 内外輪で角度が異なる
      dL = Math.atan(L / (R - t / 2));
      dR = Math.atan(L / (R + t / 2));
    }
    dL = Math.max(-cap, Math.min(cap, dL));
    dR = Math.max(-cap, Math.min(cap, dR));
    const wheel = (fx: number, fy: number, steer: number) => {
      const wx = p.x + fx * ca - fy * sa;
      const wy = p.y + fx * sa + fy * ca;
      out.push({ ring: carRect(wx, wy, ((a + steer) * 180) / Math.PI, wlen, wwid), kind: "wheel" });
    };
    // 後輪は左右ともダブルタイヤ（2本並列）。前輪はシングル＋操舵。
    const dualGap = wwid * 1.08; // ダブルタイヤ2本の中心間隔
    const dualWheel = (fx: number, fy: number) => {
      wheel(fx, fy + dualGap / 2, 0);
      wheel(fx, fy - dualGap / 2, 0);
    };
    wheel(L / 2, t / 2, dL);   // 前左（操舵・シングル）
    wheel(L / 2, -t / 2, dR);  // 前右（操舵・シングル）
    dualWheel(-L / 2, t / 2);  // 後左（ダブル）
    dualWheel(-L / 2, -t / 2); // 後右（ダブル）
  } else {
    out.push({ ring: bodyRect(p.x, p.y, a, frontExt, rearExt, v.w), kind: "footprint" });
  }
  return out;
}

// 位置(x,y)＋方位[deg, +East/CCW] の矢印を MultiLineString（軸＋2本の返し）で返す。
export function arrowGeom(x: number, y: number, headingDeg: number, L = 16, b = 5): MultiLineString {
  const a = (headingDeg * Math.PI) / 180;
  const tip: [number, number] = [x + L * Math.cos(a), y + L * Math.sin(a)];
  const bl: [number, number] = [tip[0] + b * Math.cos(a + Math.PI * 0.83), tip[1] + b * Math.sin(a + Math.PI * 0.83)];
  const br: [number, number] = [tip[0] + b * Math.cos(a - Math.PI * 0.83), tip[1] + b * Math.sin(a - Math.PI * 0.83)];
  return new MultiLineString([
    [[x, y], tip],
    [bl, tip],
    [br, tip],
  ]);
}

// 中心線を左右に halfW[m] オフセットした縁を返す（道幅帯の描画用）
export function offsetEdges(pts: number[][], halfW: number): { left: number[][]; right: number[][] } {
  const left: number[][] = [];
  const right: number[][] = [];
  const n = pts.length;
  for (let i = 0; i < n; i++) {
    const a = pts[Math.max(0, i - 1)];
    const b = pts[Math.min(n - 1, i + 1)];
    let tx = b[0] - a[0];
    let ty = b[1] - a[1];
    const L = Math.hypot(tx, ty) || 1;
    tx /= L;
    ty /= L;
    const nx = -ty;
    const ny = tx;
    left.push([pts[i][0] + nx * halfW, pts[i][1] + ny * halfW]);
    right.push([pts[i][0] - nx * halfW, pts[i][1] - ny * halfW]);
  }
  return { left, right };
}
