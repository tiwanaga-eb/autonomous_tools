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
    expect(blob.version).toBe(2);
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
