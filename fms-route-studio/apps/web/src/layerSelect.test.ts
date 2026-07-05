import { describe, expect, it } from "vitest";

import { pickLayer } from "@/layerSelect";
import type { Layer } from "@/types/api";

const L = (id: string, kind: string): Layer => ({ id, kind }) as Layer;

const layers: Layer[] = [
  L("cost_old", "cost"),
  L("ortho_1", "ortho"),
  L("cost_new", "cost"),
  L("drv_1", "drivable"),
];

describe("pickLayer", () => {
  it("returns the explicit layer when it exists and kind matches", () => {
    expect(pickLayer(layers, "cost", "cost_old")?.id).toBe("cost_old");
  });

  it("falls back to the newest (last) layer of the kind when explicit id is gone", () => {
    // プロジェクト保存後にレイヤが削除されたケース（レビュー#20 のフォールバック）
    expect(pickLayer(layers, "cost", "cost_deleted")?.id).toBe("cost_new");
  });

  it("ignores an explicit id whose kind does not match", () => {
    // id は存在するが kind 違い → 誤紐付けせず kind の最新へ
    expect(pickLayer(layers, "cost", "drv_1")?.id).toBe("cost_new");
  });

  it("falls back to newest when explicit id is null/undefined", () => {
    expect(pickLayer(layers, "cost", null)?.id).toBe("cost_new");
    expect(pickLayer(layers, "cost", undefined)?.id).toBe("cost_new");
  });

  it("returns undefined when no layer of the kind exists", () => {
    expect(pickLayer(layers, "las", null)).toBeUndefined();
  });

  it("does not mutate the input array (reverse copy)", () => {
    const before = layers.map((l) => l.id);
    pickLayer(layers, "cost", null);
    expect(layers.map((l) => l.id)).toEqual(before);
  });
});
