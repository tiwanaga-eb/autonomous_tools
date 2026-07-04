import { describe, expect, it } from "vitest";

import { coordsWithZ, csv, pilesCsvRows, sanitizeFilename, timestampName } from "@/exporters";
import type { PilePlanResult } from "@/types/api";

describe("csv", () => {
  it("joins rows/cols and renders null/undefined as empty", () => {
    const out = csv([
      ["s_m", "x", "gear"],
      [1.5, 2, "F"],
      [3, null, "R"],
    ]);
    expect(out).toBe("s_m,x,gear\n1.5,2,F\n3,,R");
  });

  it("handles an empty table", () => {
    expect(csv([])).toBe("");
  });
});

describe("sanitizeFilename", () => {
  it("replaces filesystem-unsafe characters", () => {
    expect(sanitizeFilename('a/b\\c:d*e?f"g<h>i|j')).toBe("a_b_c_d_e_f_g_h_i_j");
  });

  it("falls back to 'route' for empty or whitespace-only names", () => {
    expect(sanitizeFilename("")).toBe("route");
    expect(sanitizeFilename("   ")).toBe("route");
  });

  it("keeps Japanese route names intact", () => {
    expect(sanitizeFilename("搬入路A")).toBe("搬入路A");
  });
});

describe("pilesCsvRows", () => {
  it("パイルごとに座標＋寸法＋間隔の行を作る", () => {
    const plan: PilePlanResult = {
      pile: { height_m: 2.35, radius_m: 3.12, volume_m3: 24, repose_deg: 37 },
      centers: [[30001.234, 119002.5], [30009.0, 119002.5]],
      count: 2,
      spacing: { dx_m: 8, dy_m: 6, stagger: true },
      grid_angle_deg: 0,
      edge_margin_m: 3.12,
      area_m2: 1800,
      total_volume_m3: 48,
      n_theory: null,
      suggested_spacing_m: null,
    };
    const rows = pilesCsvRows(plan);
    expect(rows[0]).toEqual(["no", "x", "y", "height_m", "radius_m", "volume_m3", "repose_deg", "dx_m", "dy_m"]);
    expect(rows).toHaveLength(3);
    expect(rows[1]).toEqual([1, "30001.234", "119002.500", 2.35, 3.12, 24, 37, 8, 6]);
    expect(rows[2][0]).toBe(2);
  });
});

describe("timestampName", () => {
  it("prefix_YYYYMMDD-HHMMSS.ext 形式", () => {
    expect(timestampName("map", "png")).toMatch(/^map_\d{8}-\d{6}\.png$/);
  });
});

describe("coordsWithZ", () => {
  const coords: [number, number][] = [
    [139.1, 35.1],
    [139.2, 35.2],
  ];

  it("appends z[m] as the third coordinate where present (RFC 7946)", () => {
    expect(coordsWithZ(coords, [120.1234, null])).toEqual([[139.1, 35.1, 120.123], [139.2, 35.2]]);
  });

  it("keeps 2D coordinates when no point has z", () => {
    expect(coordsWithZ(coords, [null, undefined])).toEqual(coords);
  });
});
