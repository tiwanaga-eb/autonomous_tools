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
});
