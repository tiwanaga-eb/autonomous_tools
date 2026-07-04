// AI アシスタント actions → store 反映の検証。
// api はモック（REFRESH_LAYERS のネットワーク呼び出しを遮断）。
import { beforeEach, describe, expect, it, vi } from "vitest";

import { applyAgentActions } from "@/agentActions";
import { useStore } from "@/store/useStore";
import type { SpotResult, Trajectory } from "@/types/api";

vi.mock("@/api/client", () => ({
  api: {
    listLayers: vi.fn(async () => [{ id: "L1", kind: "las", name: "test.las" }]),
  },
}));

beforeEach(() => {
  useStore.setState({
    vehicleId: null,
    waypoints: [],
    route: null,
    layers: [],
    spotStart: null,
    spotTarget: null,
    spotResult: null,
    undoStack: [],
    redoStack: [],
  });
});

describe("applyAgentActions", () => {
  it("SET_VEHICLE が vehicleId を更新する", async () => {
    await applyAgentActions([{ type: "SET_VEHICLE", vehicle_id: "HM400" }]);
    expect(useStore.getState().vehicleId).toBe("HM400");
  });

  it("SET_PLAN_OPTIONS は与えたキーだけを更新する（部分更新）", async () => {
    const before = useStore.getState().routeSpacing;
    await applyAgentActions([{ type: "SET_PLAN_OPTIONS", road_width_m: 12 }]);
    const s = useStore.getState();
    expect(s.roadWidthM).toBe(12);
    expect(s.routeSpacing).toBe(before); // 未指定キーは不変
  });

  it("SET_WAYPOINTS が commandBus 経由で waypoint を並べる", async () => {
    await applyAgentActions([
      {
        type: "SET_WAYPOINTS",
        waypoints: [
          { x: 0, y: 0, role: "start" },
          { x: 50, y: 10, role: "goal" },
        ],
      },
    ]);
    const wps = useStore.getState().waypoints;
    expect(wps.map((w) => w.role)).toEqual(["start", "goal"]);
    expect(wps[1].xy).toEqual({ x: 50, y: 10 });
  });

  it("SET_ROUTE が route を格納する", async () => {
    const trajectory: Trajectory = {
      points: [
        { x: 0, y: 0, s: 0 },
        { x: 10, y: 0, s: 10 },
      ],
    } as unknown as Trajectory;
    await applyAgentActions([{ type: "SET_ROUTE", trajectory, analysis: { length_m: 10 } }]);
    expect(useStore.getState().route?.trajectory.points).toHaveLength(2);
  });

  it("REFRESH_LAYERS が listLayers の結果を反映する", async () => {
    await applyAgentActions([{ type: "REFRESH_LAYERS" }]);
    expect(useStore.getState().layers.map((l) => l.id)).toEqual(["L1"]);
  });

  it("REFRESH_LAYERS は listLayers 失敗でも throw しない", async () => {
    const { api } = await import("@/api/client");
    (api.listLayers as ReturnType<typeof vi.fn>).mockRejectedValueOnce(new Error("offline"));
    await expect(applyAgentActions([{ type: "REFRESH_LAYERS" }])).resolves.toBeUndefined();
  });

  it("SET_SPOTTING は 'start' in a 判定（null 指定で明示クリア、未指定は保持）", async () => {
    useStore.setState({ spotStart: { x: 1, y: 2, heading_deg: 0 }, spotTarget: { x: 3, y: 4, heading_deg: 90 } });
    await applyAgentActions([{ type: "SET_SPOTTING", start: null }]);
    const s = useStore.getState();
    expect(s.spotStart).toBeNull(); // 明示 null → クリア
    expect(s.spotTarget).toEqual({ x: 3, y: 4, heading_deg: 90 }); // 未指定 → 保持
  });

  it("SET_SPOT_RESULT が結果を格納し、未知の action type は無視される", async () => {
    const res = { points: [], feasible: false, status: "NO_PATH" } as unknown as SpotResult;
    await applyAgentActions([
      { type: "SET_SPOT_RESULT", result: res },
      { type: "UNKNOWN_FUTURE_ACTION" },
    ]);
    expect(useStore.getState().spotResult?.status).toBe("NO_PATH");
  });
});
