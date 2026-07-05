import { beforeEach, describe, expect, it } from "vitest";

import { applyProject, serializeProject } from "@/projectState";
import { useStore } from "@/store/useStore";
import type { RouteResult } from "@/store/useStore";

beforeEach(() => {
  useStore.setState({
    waypoints: [],
    route: null,
    areas: [],
    activePolygon: [],
    importedRoutes: [],
    selectedId: null,
    undoStack: [],
    redoStack: [],
  });
});

const FAKE_ROUTE = {
  trajectory: { points: [{ x: 0, y: 0 }, { x: 5, y: 1 }, { x: 10, y: 0 }] },
  analysis: { length_m: 11.2 },
} as unknown as RouteResult;

describe("projectState — 経路・エリアの保存/復元ラウンドトリップ", () => {
  it("route と areas を保存し、読込で復元する（再生成不要）", () => {
    useStore.getState().setWaypoints([
      { id: "a", role: "start", xy: { x: 0, y: 0 } },
      { id: "b", role: "goal", xy: { x: 10, y: 0 } },
    ]);
    useStore.getState().setAreas([{ id: "z1", name: "エリア1", points: [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 1, y: 1 }] }]);
    useStore.getState().setRoute(FAKE_ROUTE);

    const blob = serializeProject();
    expect(blob.version).toBe(3);
    expect(blob.route).toBeTruthy();

    // 状態をクリアしてから復元
    useStore.setState({ waypoints: [], areas: [], route: null });
    applyProject(blob);

    const s = useStore.getState();
    expect(s.waypoints.map((w) => w.role)).toEqual(["start", "goal"]);
    expect(s.areas).toHaveLength(1);
    expect(s.areas[0].name).toBe("エリア1");
    expect(s.route?.trajectory.points).toHaveLength(3); // 経路ジオメトリが復元される
  });

  it("v1（route無し）プロジェクトは route=null で読み込める（後方互換）", () => {
    applyProject({ version: 1, waypoints: [], areas: [{ id: "z", name: "A", points: [] }] });
    const s = useStore.getState();
    expect(s.route).toBeNull();
    expect(s.areas).toHaveLength(1);
  });
});

describe("projectState v3 — 寄り付き詳細設定の保存/復元（データ損失回帰）", () => {
  it("weights/退出/手動切り返し/マージンがラウンドトリップする", () => {
    const st = useStore.getState();
    st.setSpotWeights({ w_distance: 2, w_time: 1, w_reverse: 3, w_switchback: 9, w_costmap: 4, w_turn: 5 });
    st.setSpotWithExit(true);
    st.setSpotExitGoal({ x: 12, y: 34, heading_deg: 90 });
    st.setSpotSwitchPose({ x: 5, y: 6, heading_deg: 45 });
    st.setSpotSwitchZoneId("zone-1");
    st.setSpotCuspMargin(2.5);

    const blob = serializeProject();
    expect(blob.version).toBe(3);

    // 既定値に戻してから復元
    useStore.setState({
      spotWeights: { w_distance: 1, w_time: 0, w_reverse: 1, w_switchback: 8, w_costmap: 2, w_turn: 6 },
      spotWithExit: false,
      spotExitGoal: null,
      spotSwitchPose: null,
      spotSwitchZoneId: null,
      spotCuspMargin: 0,
    });
    applyProject(blob);

    const s = useStore.getState();
    expect(s.spotWeights.w_switchback).toBe(9);
    expect(s.spotWithExit).toBe(true);
    expect(s.spotExitGoal).toEqual({ x: 12, y: 34, heading_deg: 90 });
    expect(s.spotSwitchPose).toEqual({ x: 5, y: 6, heading_deg: 45 });
    expect(s.spotSwitchZoneId).toBe("zone-1");
    expect(s.spotCuspMargin).toBe(2.5);
  });

  it("v2 以前（フィールド無し）は既定値へフォールバックする", () => {
    useStore.getState().setSpotWithExit(true);
    applyProject({ version: 2, waypoints: [], areas: [] });
    const s = useStore.getState();
    expect(s.spotWithExit).toBe(false);
    expect(s.spotWeights.w_distance).toBe(1);
  });

  it("fleetBays は保存経路に存在する routeId だけ復元し、残留 bay を置換する", () => {
    // 前プロジェクトの残留 bay
    useStore.getState().setFleetBay("stale", { s_frac: 0.5, offset_m: 4, side: 1, ramp_m: 10, hold_m: 10 });
    applyProject({
      version: 3,
      waypoints: [],
      areas: [],
      savedRoutes: [{ id: "r1", name: "A", route: FAKE_ROUTE, waypoints: [], vehicleId: null }],
      fleetBays: {
        r1: { s_frac: 0.3, offset_m: 5, side: -1, ramp_m: 12, hold_m: 8 },
        orphan: { s_frac: 0.9, offset_m: 3, side: 1, ramp_m: 10, hold_m: 10 },
      },
    });
    const s = useStore.getState();
    expect(Object.keys(s.fleetBays)).toEqual(["r1"]);
    expect(s.fleetBays.r1.offset_m).toBe(5);
  });
});
