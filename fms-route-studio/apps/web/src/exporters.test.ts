import { describe, expect, it } from "vitest";

import { csv, sanitizeFilename } from "@/exporters";

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
