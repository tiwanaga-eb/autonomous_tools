// プロジェクト JSON 入出力の検証（importProjectJson の座標系解決と後方互換）。
// api.transform はモック（WGS84↔working の線形ダミー変換）。
import { beforeEach, describe, expect, it, vi } from "vitest";

import { importProjectJson } from "@/io";
import { useStore } from "@/store/useStore";

// ダミー変換: WGS84→working は (lng*1000, lat*1000)、逆は /1000。
// 「変換が呼ばれた回数・方向」も検証できるよう vi.fn で包む。
const transform = vi.fn(async (pts: [number, number][], src: number, dst: number) => ({
  points: pts.map(([a, b]) => (dst === 4326 ? [a / 1000, b / 1000] : [a * 1000, b * 1000]) as [number, number]),
  src,
  dst,
}));

vi.mock("@/api/client", () => ({
  api: {
    transform: (pts: [number, number][], src: number, dst: number) => transform(pts, src, dst),
  },
}));

function asFile(obj: unknown): File {
  // importProjectJson は file.text() しか使わない（ブラウザ File を Node で偽装）
  return { text: async () => JSON.stringify(obj) } as unknown as File;
}

beforeEach(() => {
  transform.mockClear();
  useStore.setState({ areas: [], importedRoutes: [], undoStack: [], redoStack: [] });
});

describe("importProjectJson", () => {
  it("points_xy（working CRS）があれば変換せずそのまま使う", async () => {
    const n = await importProjectJson(
      asFile({
        polygons: [
          {
            name: "積込A",
            points: [{ lat: 35, lng: 139 }],
            points_xy: [
              { x: 0, y: 0 },
              { x: 10, y: 0 },
              { x: 10, y: 10 },
            ],
          },
        ],
      }),
    );
    expect(n).toEqual({ areas: 1, routes: 0 });
    expect(useStore.getState().areas[0].points).toHaveLength(3);
    expect(transform).not.toHaveBeenCalled(); // 変換 API 不要（オフラインでも読める）
  });

  it("旧形式（lat/lng のみ）は WGS84→working へ変換して読み込む", async () => {
    const n = await importProjectJson(
      asFile({
        haulRoutes: [
          {
            name: "搬入路",
            route: [
              { lat: 35.1, lng: 139.1 },
              { lat: 35.2, lng: 139.2 },
            ],
          },
        ],
      }),
    );
    expect(n).toEqual({ areas: 0, routes: 1 });
    const r = useStore.getState().importedRoutes[0];
    expect(r.name).toBe("搬入路");
    expect(r.pts[0]).toEqual({ x: 139.1 * 1000, y: 35.1 * 1000 });
    expect(transform).toHaveBeenCalledTimes(1);
  });

  it("頂点3未満のポリゴン・2点未満のルートは除外し、名前欠落は補完する", async () => {
    const n = await importProjectJson(
      asFile({
        polygons: [
          { points_xy: [{ x: 0, y: 0 }, { x: 1, y: 0 }] }, // 2頂点 → 除外
          { points_xy: [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 1, y: 1 }] }, // 名前なし → "Area"
        ],
        haulRoutes: [{ name: "short", route: [{ lat: 35, lng: 139 }] }], // 1点 → 除外
      }),
    );
    expect(n).toEqual({ areas: 1, routes: 0 });
    expect(useStore.getState().areas[0].name).toBe("Area");
  });

  it("壊れた JSON は例外を投げる（呼び出し側でユーザー通知）", async () => {
    const bad = { text: async () => "{ not json" } as unknown as File;
    await expect(importProjectJson(bad)).rejects.toThrow();
  });
});
