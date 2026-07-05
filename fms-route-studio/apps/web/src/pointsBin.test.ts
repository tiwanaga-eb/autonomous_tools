import { describe, expect, it } from "vitest";

import { parsePointsBin } from "@/api/client";

/** layers.py::las_points_bin と同じレイアウトのバッファを手組みする。 */
function buildBuf(opts: { n: number; rgb: boolean; origin: [number, number, number]; zmin: number; zmax: number;
                          pts: [number, number, number][] }): ArrayBuffer {
  const { n, rgb, origin, zmin, zmax, pts } = opts;
  const size = 52 + 12 * n + (rgb ? 3 * n : 0);
  const buf = new ArrayBuffer(size);
  const dv = new DataView(buf);
  [..."FRSP"].forEach((c, i) => dv.setUint8(i, c.charCodeAt(0)));
  dv.setUint8(4, 1);
  dv.setUint8(5, rgb ? 1 : 0);
  dv.setUint32(8, n, true);
  dv.setFloat64(12, origin[0], true);
  dv.setFloat64(20, origin[1], true);
  dv.setFloat64(28, origin[2], true);
  dv.setFloat64(36, zmin, true);
  dv.setFloat64(44, zmax, true);
  const x = new Float32Array(buf, 52, n);
  const y = new Float32Array(buf, 52 + 4 * n, n);
  const z = new Float32Array(buf, 52 + 8 * n, n);
  pts.forEach(([px, py, pz], i) => { x[i] = px; y[i] = py; z[i] = pz; });
  if (rgb) {
    const c = new Uint8Array(buf, 52 + 12 * n, 3 * n);
    for (let i = 0; i < 3 * n; i++) c[i] = i % 256;
  }
  return buf;
}

describe("parsePointsBin", () => {
  it("origin相対 f32 と rgb を正しく展開する", () => {
    const buf = buildBuf({
      n: 2, rgb: true, origin: [30000, 119000, 50], zmin: 48, zmax: 52,
      pts: [[-1.5, 2.25, -2], [3.5, -4.5, 2]],
    });
    const p = parsePointsBin(buf);
    expect(p.n).toBe(2);
    expect(p.origin).toEqual([30000, 119000, 50]);
    expect(p.x[0]).toBeCloseTo(-1.5);
    expect(p.y[1]).toBeCloseTo(-4.5);
    expect(p.z[0]).toBeCloseTo(-2);
    expect(p.zmin).toBe(48);
    expect(p.zmax).toBe(52);
    expect(p.rgb).not.toBeNull();
    expect(p.rgb![0]).toBe(0);
    expect(p.rgb![5]).toBe(5);
  });

  it("rgb 無しは null、マジック不一致は例外", () => {
    const buf = buildBuf({ n: 1, rgb: false, origin: [0, 0, 0], zmin: 0, zmax: 1, pts: [[0, 0, 0]] });
    expect(parsePointsBin(buf).rgb).toBeNull();
    const bad = buildBuf({ n: 1, rgb: false, origin: [0, 0, 0], zmin: 0, zmax: 1, pts: [[0, 0, 0]] });
    new DataView(bad).setUint8(0, 88); // magic 破壊
    expect(() => parsePointsBin(bad)).toThrow();
  });

  it("途中で切れたバッファは例外", () => {
    const buf = buildBuf({ n: 2, rgb: false, origin: [0, 0, 0], zmin: 0, zmax: 1, pts: [[0, 0, 0], [1, 1, 1]] });
    expect(() => parsePointsBin(buf.slice(0, 60))).toThrow();
  });
});
