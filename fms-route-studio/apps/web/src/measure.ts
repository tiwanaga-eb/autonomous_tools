// 計測ユーティリティ（距離・面積）。
// 地図は作業CRS（JGD2011 平面直角座標系）ネイティブ＝平面メートルなので、
// ユークリッド距離と靴ひも公式がそのまま正しい（測地線計算は不要）。
import type { XY } from "@/types/api";

/** 折れ線の総延長[m]。 */
export function polylineLength(pts: XY[]): number {
  let s = 0;
  for (let i = 1; i < pts.length; i++) {
    s += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  }
  return s;
}

/** 多角形の面積[m²]（靴ひも公式・自動で閉じる・向き不問）。3頂点未満は 0。 */
export function polygonArea(pts: XY[]): number {
  if (pts.length < 3) return 0;
  let a = 0;
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i];
    const q = pts[(i + 1) % pts.length];
    a += p.x * q.y - q.x * p.y;
  }
  return Math.abs(a) / 2;
}

/** 距離の表示文字列: 1km 以上は km 併記。 */
export function fmtLength(m: number): string {
  if (m >= 1000) return `${m.toLocaleString("ja-JP", { maximumFractionDigits: 0 })} m (${(m / 1000).toFixed(2)} km)`;
  return `${m.toFixed(m >= 100 ? 0 : 1)} m`;
}

/** 面積の表示文字列: 1万m² 以上は ha 併記。 */
export function fmtArea(m2: number): string {
  const base = `${m2.toLocaleString("ja-JP", { maximumFractionDigits: 0 })} m²`;
  if (m2 >= 10_000) return `${base} (${(m2 / 10_000).toFixed(2)} ha)`;
  return m2 >= 100 ? base : `${m2.toFixed(1)} m²`;
}
