import { beforeEach, describe, expect, it } from "vitest";

import { dispatch } from "@/commandBus";
import { useStore } from "@/store/useStore";

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

describe("commandBus — waypoints", () => {
  it("orders waypoints start … via … goal regardless of insertion order", () => {
    dispatch({ type: "ADD_WAYPOINT", role: "goal", xy: { x: 10, y: 0 } });
    dispatch({ type: "ADD_WAYPOINT", role: "start", xy: { x: 0, y: 0 } });
    dispatch({ type: "ADD_WAYPOINT", role: "via", xy: { x: 5, y: 1 } });
    expect(useStore.getState().waypoints.map((w) => w.role)).toEqual(["start", "via", "goal"]);
  });

  it("replaces an existing start", () => {
    dispatch({ type: "ADD_WAYPOINT", role: "start", xy: { x: 0, y: 0 } });
    dispatch({ type: "ADD_WAYPOINT", role: "start", xy: { x: 9, y: 9 } });
    const starts = useStore.getState().waypoints.filter((w) => w.role === "start");
    expect(starts).toHaveLength(1);
    expect(starts[0].xy.x).toBe(9);
  });

  it("INSERT_VIA inserts a via between start and goal", () => {
    dispatch({ type: "ADD_WAYPOINT", role: "start", xy: { x: 0, y: 0 } });
    dispatch({ type: "ADD_WAYPOINT", role: "goal", xy: { x: 10, y: 0 } });
    dispatch({ type: "INSERT_VIA", xy: { x: 5, y: 1 } });
    expect(useStore.getState().waypoints.map((w) => w.role)).toEqual(["start", "via", "goal"]);
  });

  it("INSERT_VIA picks the nearest segment, not just start-goal", () => {
    // start(0,0) — via(10,0) — goal(20,0)。(15,1) は via–goal 区間に最も近い
    dispatch({ type: "ADD_WAYPOINT", role: "start", xy: { x: 0, y: 0 } });
    dispatch({ type: "ADD_WAYPOINT", role: "via", xy: { x: 10, y: 0 } });
    dispatch({ type: "ADD_WAYPOINT", role: "goal", xy: { x: 20, y: 0 } });
    dispatch({ type: "INSERT_VIA", xy: { x: 15, y: 1 } });
    const xs = useStore.getState().waypoints.map((w) => w.xy.x);
    expect(xs).toEqual([0, 10, 15, 20]); // via–goal 間に挿入される
  });

  it("INSERT_VIA degenerates to append when fewer than 2 waypoints", () => {
    dispatch({ type: "ADD_WAYPOINT", role: "start", xy: { x: 0, y: 0 } });
    dispatch({ type: "INSERT_VIA", xy: { x: 5, y: 5 } });
    expect(useStore.getState().waypoints.map((w) => w.role)).toEqual(["start", "via"]);
  });

  it("MOVE_WAYPOINT clears the cached route", () => {
    dispatch({ type: "ADD_WAYPOINT", role: "start", xy: { x: 0, y: 0 } });
    const id = useStore.getState().waypoints[0].id;
    dispatch({ type: "MOVE_WAYPOINT", id, xy: { x: 3, y: 4 } });
    expect(useStore.getState().waypoints[0].xy).toEqual({ x: 3, y: 4 });
    expect(useStore.getState().route).toBeNull();
  });
});

describe("commandBus — polygons", () => {
  it("FINISH_POLYGON creates a named area and clears the active polygon", () => {
    dispatch({ type: "ADD_POLY_VERTEX", xy: { x: 0, y: 0 } });
    dispatch({ type: "ADD_POLY_VERTEX", xy: { x: 1, y: 0 } });
    dispatch({ type: "ADD_POLY_VERTEX", xy: { x: 1, y: 1 } });
    dispatch({ type: "FINISH_POLYGON", name: "DumpingZone" });
    expect(useStore.getState().activePolygon).toHaveLength(0);
    expect(useStore.getState().areas.at(-1)?.name).toBe("DumpingZone");
  });

  it("FINISH_POLYGON is a no-op with < 3 vertices", () => {
    dispatch({ type: "ADD_POLY_VERTEX", xy: { x: 0, y: 0 } });
    dispatch({ type: "FINISH_POLYGON", name: "X" });
    expect(useStore.getState().areas).toHaveLength(0);
  });
});

describe("commandBus — undo/redo", () => {
  it("undo reverts the last change; redo reapplies it", () => {
    dispatch({ type: "ADD_WAYPOINT", role: "start", xy: { x: 0, y: 0 } });
    dispatch({ type: "ADD_WAYPOINT", role: "goal", xy: { x: 1, y: 1 } });
    expect(useStore.getState().waypoints).toHaveLength(2);
    dispatch({ type: "UNDO" });
    expect(useStore.getState().waypoints).toHaveLength(1);
    dispatch({ type: "REDO" });
    expect(useStore.getState().waypoints).toHaveLength(2);
  });

  it("snapshots are deeply isolated from live state (no shared references)", () => {
    dispatch({ type: "ADD_WAYPOINT", role: "start", xy: { x: 0, y: 0 } });
    dispatch({ type: "ADD_WAYPOINT", role: "goal", xy: { x: 1, y: 1 } });
    // 2回目の commit が積んだスナップショットは「start のみ」の状態
    const snapWp = useStore.getState().undoStack.at(-1)!.waypoints;
    const liveStart = useStore.getState().waypoints.find((w) => w.role === "start")!;
    // 深いコピーなら、履歴の start とライブの start は別オブジェクト
    expect(snapWp[0]).not.toBe(liveStart);
  });

  it("in-place mutation of live state does not corrupt history", () => {
    dispatch({ type: "ADD_WAYPOINT", role: "start", xy: { x: 0, y: 0 } });
    dispatch({ type: "ADD_WAYPOINT", role: "goal", xy: { x: 1, y: 1 } });
    // 万一どこかが破壊的編集をした場合のシミュレーション
    useStore.getState().waypoints.find((w) => w.role === "start")!.xy.x = 999;
    dispatch({ type: "UNDO" }); // start のみの状態へ戻す
    const restored = useStore.getState().waypoints.find((w) => w.role === "start")!;
    expect(restored.xy.x).toBe(0); // 999 に汚染されていないこと
  });

  it("history is capped at 50 snapshots and redo is cleared on new edit", () => {
    for (let i = 0; i < 60; i++) {
      dispatch({ type: "ADD_WAYPOINT", role: "via", xy: { x: i, y: 0 } });
    }
    expect(useStore.getState().undoStack.length).toBeLessThanOrEqual(50);
    dispatch({ type: "UNDO" });
    expect(useStore.getState().redoStack.length).toBe(1);
    dispatch({ type: "ADD_WAYPOINT", role: "via", xy: { x: 99, y: 0 } });
    expect(useStore.getState().redoStack.length).toBe(0); // 新規編集で redo は破棄
  });
});
