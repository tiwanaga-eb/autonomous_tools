import { describe, expect, it } from "vitest";

import { fmtArea, fmtLength, polygonArea, polylineLength } from "@/measure";

describe("polylineLength", () => {
  it("直角折れ線の総延長", () => {
    expect(polylineLength([{ x: 0, y: 0 }, { x: 30, y: 0 }, { x: 30, y: 40 }])).toBe(70);
  });
  it("1点以下は 0", () => {
    expect(polylineLength([])).toBe(0);
    expect(polylineLength([{ x: 5, y: 5 }])).toBe(0);
  });
});

describe("polygonArea", () => {
  it("矩形（時計回りでも反時計回りでも同じ）", () => {
    const ccw = [{ x: 0, y: 0 }, { x: 20, y: 0 }, { x: 20, y: 10 }, { x: 0, y: 10 }];
    expect(polygonArea(ccw)).toBe(200);
    expect(polygonArea([...ccw].reverse())).toBe(200);
  });
  it("三角形", () => {
    expect(polygonArea([{ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 0, y: 10 }])).toBe(50);
  });
  it("3頂点未満は 0", () => {
    expect(polygonArea([{ x: 0, y: 0 }, { x: 1, y: 1 }])).toBe(0);
  });
});

describe("表示フォーマット", () => {
  it("距離: 1km 以上は km 併記", () => {
    expect(fmtLength(52.34)).toBe("52.3 m");
    expect(fmtLength(1234)).toContain("km");
  });
  it("面積: 1万m²以上は ha 併記", () => {
    expect(fmtArea(200)).toContain("m²");
    expect(fmtArea(25_000)).toContain("ha");
  });
});
