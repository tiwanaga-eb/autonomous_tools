// プロジェクト永続化用: ワーキング状態の抽出/適用（設計書 §15 / Phase 6）。
// レイヤはサーバ側レジストリで別管理のため含めない（読込後もレイヤはそのまま）。
// v2: **経路ジオメトリ(route)も保存**し、再読込時にそのまま復元する（経路・エリアを保存→呼出→編集）。
//     spotResult はレイヤ依存のクリアランス評価を含むため従来どおり保存しない。
// v3: 寄り付きの詳細設定（手動切り返し点/切り返しゾーン/直線マージン/退出/コスト重み）も保存。
//     従来はこれらが復元されず、再読込のたびに既定値へ戻っていた（データ損失）。

import { pickLayer } from "@/layerSelect";
import { useStore } from "@/store/useStore";

const VERSION = 3;

// store の既定値と一致させる（applyProject で欠落時に使うフォールバック）
const DEFAULT_SPOT_WEIGHTS = { w_distance: 1, w_time: 0, w_reverse: 1, w_switchback: 8, w_costmap: 2, w_turn: 6 };

export function serializeProject(): Record<string, unknown> {
  const s = useStore.getState();
  return {
    version: VERSION,
    waypoints: s.waypoints,
    areas: s.areas,
    route: s.route, // アクティブ経路ジオメトリ(trajectory/analysis/safety)を保存（再読込で復元）
    savedRoutes: s.savedRoutes, // プロジェクトに保存した複数の名前付き経路ライブラリ（複数台の経路集合）
    showSavedRoutes: s.showSavedRoutes,
    fleetBays: s.fleetBays, // すれ違い点（待避所）設定: routeId → BayCfg
    // 経路が紐づくレイヤidを固定保存（再読込で当時のコスト/走行可能領域へ正しく紐づく）。
    costLayerId: pickLayer(s.layers, "cost", s.costLayerId)?.id ?? null,
    drivableLayerId: pickLayer(s.layers, "drivable", s.drivableLayerId)?.id ?? null,
    importedRoutes: s.importedRoutes,
    vehicleId: s.vehicleId,
    planMode: s.planMode,
    algorithm: s.algorithm,
    routeSpacing: s.routeSpacing,
    roadWidthM: s.roadWidthM,
    enforceFootprint: s.enforceFootprint,
    enforceMinRadius: s.enforceMinRadius,
    allowReverse: s.allowReverse,
    refineElasticBand: s.refineElasticBand,
    showWaypoints: s.showWaypoints,
    costOpacity: s.costOpacity,
    costVisible: s.costVisible,
    drivableOpacity: s.drivableOpacity,
    drivableVisible: s.drivableVisible,
    spotStart: s.spotStart,
    spotTarget: s.spotTarget,
    spotMaxSwitch: s.spotMaxSwitch,
    spotMethod: s.spotMethod,
    spotSmooth: s.spotSmooth,
    spotStationaryMode: s.spotStationaryMode,
    spotMinSpeedKmh: s.spotMinSpeedKmh,
    spotContainAreaId: s.spotContainAreaId,
    spotRoadWidthM: s.spotRoadWidthM,
    // v3: 寄り付き詳細設定
    spotSwitchPose: s.spotSwitchPose,
    spotSwitchZoneId: s.spotSwitchZoneId,
    spotCuspMargin: s.spotCuspMargin,
    spotWithExit: s.spotWithExit,
    spotExitGoal: s.spotExitGoal,
    spotWeights: s.spotWeights,
    // 排土（パイル）配置の設定と結果
    pileAreaId: s.pileAreaId,
    pileSizeMode: s.pileSizeMode,
    pileVolumeM3: s.pileVolumeM3,
    pileHeightM: s.pileHeightM,
    pileReposeDeg: s.pileReposeDeg,
    pilePlaceMode: s.pilePlaceMode,
    pileDx: s.pileDx,
    pileDy: s.pileDy,
    pileStagger: s.pileStagger,
    pileStaggerInv: s.pileStaggerInv,
    pileSpreadT: s.pileSpreadT,
    pileEdgeMarginM: s.pileEdgeMarginM,
    pilePlan: s.pilePlan,
  };
}

export function applyProject(state: Record<string, unknown>): void {
  const s = useStore.getState();
  const g = <T,>(k: string, fallback: T): T => (state[k] === undefined ? fallback : (state[k] as T));

  const ver = typeof state.version === "number" ? state.version : 0;
  if (ver > VERSION) {
    // 新しい版で保存されたプロジェクト: 未知フィールドは無視し、既知分のみ復元（best-effort）。
    console.warn(`project version ${ver} > supported ${VERSION}; loading known fields only`);
  }

  s.setWaypoints(g("waypoints", []) as never);
  s.setAreas(g("areas", []) as never);
  s.setImportedRoutes(g("importedRoutes", []) as never);
  // v2+: 保存された経路ジオメトリをそのまま復元。v1（route無し）は null（waypoints から再生成）。
  s.setRoute(g<unknown>("route", null) as never);
  s.setSpotResult(null);

  s.setSavedRoutes(g("savedRoutes", []) as never);
  s.setShowSavedRoutes(g("showSavedRoutes", true));
  s.setFleetConflicts(null);
  // 待避所は保存経路に存在する routeId のみ復元（孤児を持ち込まない）。前プロジェクトの残留も置換で消す。
  const bays = g<Record<string, import("@/types/api").BayCfg>>("fleetBays", {});
  const routeIds = new Set((g("savedRoutes", []) as { id: string }[]).map((r) => r.id));
  s.setFleetBays(Object.fromEntries(Object.entries(bays).filter(([rid]) => routeIds.has(rid))));
  s.setVehicleId(g("vehicleId", null));
  s.setCostLayerId(g("costLayerId", null));
  s.setDrivableLayerId(g("drivableLayerId", null));
  s.setPlanMode(g("planMode", "waypoint_guided") as never);
  s.setAlgorithm(g("algorithm", "spline") as never);
  s.setRouteSpacing(g("routeSpacing", 2.0));
  s.setRoadWidthM(g("roadWidthM", 0));
  s.setEnforceFootprint(g("enforceFootprint", true));
  s.setEnforceMinRadius(g("enforceMinRadius", true));
  s.setAllowReverse(g("allowReverse", false));
  s.setRefineElasticBand(g("refineElasticBand", false));
  s.setShowWaypoints(g("showWaypoints", true));
  s.setCostOpacity(g("costOpacity", 0.6));
  s.setCostVisible(g("costVisible", true));
  s.setDrivableOpacity(g("drivableOpacity", 0.5));
  s.setDrivableVisible(g("drivableVisible", true));
  s.setSpotStart(g("spotStart", null));
  s.setSpotTarget(g("spotTarget", null));
  s.setSpotMaxSwitch(g("spotMaxSwitch", 1) as 0 | 1);
  s.setSpotMethod(g("spotMethod", "auto") as import("@/types/api").SpottingMethod);
  s.setSpotSmooth(g("spotSmooth", true));
  s.setSpotStationaryMode(g("spotStationaryMode", "auto") as "auto" | "allow" | "deny");
  s.setSpotMinSpeedKmh(g("spotMinSpeedKmh", 0));
  s.setSpotContainAreaId(g("spotContainAreaId", null));
  s.setSpotRoadWidthM(g("spotRoadWidthM", 0));
  // v3: 寄り付き詳細設定（v2 以前のプロジェクトは既定値に戻る）
  s.setSpotSwitchPose(g("spotSwitchPose", null));
  s.setSpotSwitchZoneId(g("spotSwitchZoneId", null));
  s.setSpotCuspMargin(g("spotCuspMargin", 0));
  s.setSpotWithExit(g("spotWithExit", false));
  s.setSpotExitGoal(g("spotExitGoal", null));
  s.setSpotWeights(g("spotWeights", DEFAULT_SPOT_WEIGHTS));
  // 排土（パイル）配置
  s.setPileAreaId(g("pileAreaId", null));
  s.setPileSizeMode(g("pileSizeMode", "volume") as "volume" | "height");
  s.setPileVolumeM3(g("pileVolumeM3", 24));
  s.setPileHeightM(g("pileHeightM", 1.5));
  s.setPileReposeDeg(g("pileReposeDeg", 37));
  s.setPilePlaceMode(g("pilePlaceMode", "spacing") as "spacing" | "spread");
  s.setPileDx(g("pileDx", 8));
  s.setPileDy(g("pileDy", 0));
  s.setPileStagger(g("pileStagger", false));
  s.setPileStaggerInv(g("pileStaggerInv", false));
  s.setPileSpreadT(g("pileSpreadT", 0.5));
  s.setPileEdgeMarginM(g("pileEdgeMarginM", -1));
  s.setPilePlan(g("pilePlan", null));
}
