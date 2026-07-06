// 3D ビュー用の標高グリッド。
// 通常は DSM（コストマップ生成時の /dsm_grid）を使うが、コストマップ未生成の
// LAS だけの状態でも経路・エリア・パイルを点群表面に追従させるため、
// 読み込み済み点群からセル平均の粗グリッドを作るフォールバックを提供する。
import type { PointCloud } from "@/api/client";

export interface Grid {
  nx: number; ny: number; x0: number; y0: number; dx: number; dy: number;
  z: (number | null)[][]; zmin: number; zmax: number;
}

/** 点群→セル平均の標高グリッド（絶対座標）。点が無いセルは null。maxDim=長辺の分割数。 */
export function gridFromPoints(p: PointCloud, maxDim = 240): Grid | null {
  if (!p || p.n < 10) return null;
  let minx = Infinity, maxx = -Infinity, miny = Infinity, maxy = -Infinity;
  for (let i = 0; i < p.n; i++) {
    const X = p.x[i], Y = p.y[i];
    if (X < minx) minx = X;
    if (X > maxx) maxx = X;
    if (Y < miny) miny = Y;
    if (Y > maxy) maxy = Y;
  }
  const w = maxx - minx, h = maxy - miny;
  if (!(w > 0) || !(h > 0)) return null;
  const cell = Math.max(w, h) / maxDim;
  const nx = Math.max(2, Math.floor(w / cell) + 1);
  const ny = Math.max(2, Math.floor(h / cell) + 1);
  const sum = new Float64Array(nx * ny);
  const cnt = new Uint32Array(nx * ny);
  for (let i = 0; i < p.n; i++) {
    const c = Math.min(nx - 1, Math.floor((p.x[i] - minx) / cell));
    const r = Math.min(ny - 1, Math.floor((p.y[i] - miny) / cell));
    const k = r * nx + c;
    sum[k] += p.z[i];
    cnt[k] += 1;
  }
  const z: (number | null)[][] = [];
  let zmin = Infinity, zmax = -Infinity;
  for (let r = 0; r < ny; r++) {
    const row: (number | null)[] = [];
    for (let c = 0; c < nx; c++) {
      const k = r * nx + c;
      if (cnt[k] > 0) {
        const v = sum[k] / cnt[k] + p.origin[2]; // 相対 z → 絶対標高
        row.push(v);
        if (v < zmin) zmin = v;
        if (v > zmax) zmax = v;
      } else {
        row.push(null);
      }
    }
    z.push(row);
  }
  if (!Number.isFinite(zmin)) return null;
  // 12パス ≒ 半径12セル（240分割なら長辺の~5%）までの穴を埋める。それ以上の大穴
  // （サイト外周など）は null のまま＝過剰な外挿をしない。
  fillHoles(z, nx, ny, 12);
  return {
    nx, ny,
    x0: p.origin[0] + minx, y0: p.origin[1] + miny,
    dx: cell, dy: cell,
    z, zmin, zmax,
  };
}

/** 点の無いセル（null）を非nullの8近傍平均で埋める（最大 passes 回の膨張）。
    埋め残し（大きな空白域）は null のまま＝呼び出し側のフォールバックに任せる。
    穴を zmin で描くと地形メッシュがスパイク状に落ち込み、ピッキング面も歪むため。 */
function fillHoles(z: (number | null)[][], nx: number, ny: number, passes: number): void {
  for (let pass = 0; pass < passes; pass++) {
    const fills: [number, number, number][] = [];
    for (let r = 0; r < ny; r++) {
      for (let c = 0; c < nx; c++) {
        if (z[r][c] != null) continue;
        let sum = 0, n = 0;
        for (let dr = -1; dr <= 1; dr++) {
          for (let dc = -1; dc <= 1; dc++) {
            const v = z[r + dr]?.[c + dc];
            if (v != null) { sum += v; n++; }
          }
        }
        if (n >= 3) fills.push([r, c, sum / n]);
      }
    }
    if (!fills.length) break;
    for (const [r, c, v] of fills) z[r][c] = v;
  }
}
