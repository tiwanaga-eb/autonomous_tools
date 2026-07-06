// 点群→標高グリッド（3Dで経路/エリアを点群表面に追従させるフォールバック）の検証。
import { describe, expect, it } from "vitest";

import type { PointCloud } from "@/api/client";
import { gridFromPoints } from "@/map/elevGrid";

function makeCloud(nSide: number, zfn: (x: number, y: number) => number): PointCloud {
  // 0..100m の正方領域に nSide×nSide 点（origin 相対座標で格納）
  const n = nSide * nSide;
  const x = new Float32Array(n), y = new Float32Array(n), z = new Float32Array(n);
  let k = 0;
  for (let i = 0; i < nSide; i++) {
    for (let j = 0; j < nSide; j++) {
      x[k] = (100 * i) / (nSide - 1);
      y[k] = (100 * j) / (nSide - 1);
      z[k] = zfn(x[k], y[k]);
      k++;
    }
  }
  return { n, origin: [30000, 119000, 400], x, y, z, rgb: null, zmin: 0, zmax: 10 };
}

describe("gridFromPoints", () => {
  it("傾斜面の標高をセル平均で再現し、絶対座標のグリッドを返す", () => {
    const p = makeCloud(80, (x) => 0.1 * x); // 東西に10%勾配（相対z 0..10）
    const g = gridFromPoints(p, 50);
    expect(g).not.toBeNull();
    expect(g!.x0).toBeCloseTo(30000, 3);   // origin 相対 → 絶対座標
    expect(g!.y0).toBeCloseTo(119000, 3);
    // 絶対標高 = 相対 z + origin.z（西端 ≈400m、東端 ≈410m）
    expect(g!.zmin).toBeGreaterThanOrEqual(400);
    expect(g!.zmax).toBeLessThanOrEqual(410.01);
    const west = g!.z[0][0]!;
    const east = g!.z[0][g!.nx - 1]!;
    expect(east - west).toBeGreaterThan(8); // 勾配が保存されている
  });

  it("点の無いセルは近傍平均で埋まる（スパイク防止）", () => {
    // 中央に 20m 四方の穴をあけた点群
    const p0 = makeCloud(80, () => 5);
    const keep: number[] = [];
    for (let i = 0; i < p0.n; i++) {
      if (!(p0.x[i] > 40 && p0.x[i] < 60 && p0.y[i] > 40 && p0.y[i] < 60)) keep.push(i);
    }
    const p: PointCloud = {
      ...p0, n: keep.length,
      x: new Float32Array(keep.map((i) => p0.x[i])),
      y: new Float32Array(keep.map((i) => p0.y[i])),
      z: new Float32Array(keep.map((i) => p0.z[i])),
    };
    const g = gridFromPoints(p, 50)!;
    // 穴の中央セルも平坦面の値（405）で埋まっている
    const rc = Math.floor(((50 - 0) / 100) * (g.ny - 1));
    const cc = Math.floor(((50 - 0) / 100) * (g.nx - 1));
    expect(g.z[rc][cc]).not.toBeNull();
    expect(g.z[rc][cc]!).toBeCloseTo(405, 0);
  });

  it("点が少なすぎる/退化した入力は null", () => {
    const p = makeCloud(2, () => 0);
    p.n = 3; // <10 点
    expect(gridFromPoints(p)).toBeNull();
  });
});
