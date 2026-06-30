// AI アシスタントが返す「操作(actions)」をストア/Command Bus へ適用する（設計書 §14）。
// UI と同じ操作面を通すことで、AI の結果が即座に地図/パネルへ反映される。

import { api } from "@/api/client";
import type { AgentAction } from "@/api/client";
import { dispatch } from "@/commandBus";
import { useStore } from "@/store/useStore";
import type { Algorithm, PlanMode } from "@/store/useStore";
import type { AnalysisResult, SafetyReport, SpotPose, SpotResult, SpottingMethod, SpotWeights, Trajectory } from "@/types/api";

interface WP {
  x: number;
  y: number;
  role: "start" | "via" | "goal";
  heading_deg?: number | null;
}

export async function applyAgentActions(actions: AgentAction[]): Promise<void> {
  const s = useStore.getState();
  for (const a of actions) {
    switch (a.type) {
      case "SET_VEHICLE":
        s.setVehicleId((a.vehicle_id as string) ?? null);
        break;

      case "SET_PLAN_OPTIONS":
        if (a.mode != null) s.setPlanMode(a.mode as PlanMode);
        if (a.algorithm != null) s.setAlgorithm(a.algorithm as Algorithm);
        if (a.spacing_m != null) s.setRouteSpacing(Number(a.spacing_m));
        if (a.road_width_m != null) s.setRoadWidthM(Number(a.road_width_m));
        if (a.enforce_footprint != null) s.setEnforceFootprint(Boolean(a.enforce_footprint));
        if (a.enforce_min_radius != null) s.setEnforceMinRadius(Boolean(a.enforce_min_radius));
        if (a.allow_reverse != null) s.setAllowReverse(Boolean(a.allow_reverse));
        if (a.refine_elastic_band != null) s.setRefineElasticBand(Boolean(a.refine_elastic_band));
        break;

      case "SET_WAYPOINTS":
        dispatch({ type: "SET_WAYPOINTS", waypoints: (a.waypoints as WP[]) ?? [] });
        break;

      case "SET_ROUTE":
        dispatch({
          type: "SET_ROUTE",
          route: {
            trajectory: a.trajectory as Trajectory,
            analysis: a.analysis as AnalysisResult,
            safety: (a.safety as SafetyReport) ?? undefined,
          },
        });
        break;

      case "REFRESH_LAYERS":
        try {
          s.setLayers(await api.listLayers());
        } catch {
          /* レイヤ再取得失敗は無視（次回 listLayers で回復） */
        }
        break;

      case "SET_SPOTTING":
        if ("start" in a) s.setSpotStart((a.start as SpotPose) ?? null);
        if ("target" in a) s.setSpotTarget((a.target as SpotPose) ?? null);
        if (a.max_switchbacks != null) s.setSpotMaxSwitch(a.max_switchbacks as 0 | 1);
        if (a.method != null) s.setSpotMethod(a.method as SpottingMethod);
        if (a.smooth_path != null) s.setSpotSmooth(Boolean(a.smooth_path));
        if (a.road_width_m != null) s.setSpotRoadWidthM(Number(a.road_width_m));
        if (a.weights != null) s.setSpotWeights(a.weights as SpotWeights);
        break;

      case "SET_SPOT_RESULT":
        s.setSpotResult((a.result as SpotResult) ?? null);
        break;
    }
  }
}
