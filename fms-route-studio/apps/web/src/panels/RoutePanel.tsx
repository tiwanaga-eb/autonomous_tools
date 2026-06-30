import { useEffect, useState } from "react";

import { api } from "@/api/client";
import type { Algorithm, PlanMode } from "@/api/client";
import { dispatch } from "@/commandBus";
import { pickLayer } from "@/layerSelect";
import { useStore } from "@/store/useStore";
import type { EditMode } from "@/store/useStore";
import type { Vehicle } from "@/types/api";

const MODES: { mode: EditMode; label: string }[] = [
  { mode: "start", label: "Start" },
  { mode: "goal", label: "Goal" },
  { mode: "via", label: "Via" },
  { mode: "insert_via", label: "Insert Via" },
  { mode: "edit", label: "Edit" },
  { mode: "pan", label: "Pan" },
];

const ALGOS: { value: Algorithm; label: string }[] = [
  { value: "spline", label: "Spline（経由点ガイド）" },
  { value: "dubins", label: "Dubins（R_min保証・方位尊重）" },
  { value: "hybrid_astar", label: "Hybrid A*（運動学的・方位尊重）" },
  { value: "grid_astar", label: "Grid A*（コスト/領域に沿う）" },
  { value: "reeds_shepp", label: "Reeds-Shepp（前進＋後進・切返）" },
  { value: "rrt_star", label: "RRT*（サンプリング・狭所/複雑形状）" },
];
const AUTO_ALGOS: { value: Algorithm; label: string }[] = [
  { value: "hybrid_astar", label: "Hybrid A*（運動学的・推奨）" },
  { value: "grid_astar", label: "Grid A*（大域コリドー）" },
];

export function RoutePanel() {
  const mode = useStore((s) => s.mode);
  const layers = useStore((s) => s.layers);
  const areas = useStore((s) => s.areas);
  const waypoints = useStore((s) => s.waypoints);
  const setStatus = useStore((s) => s.setStatus);
  const canUndo = useStore((s) => s.undoStack.length > 0);
  const canRedo = useStore((s) => s.redoStack.length > 0);
  const routeSpacing = useStore((s) => s.routeSpacing);
  const setRouteSpacing = useStore((s) => s.setRouteSpacing);
  const roadWidthM = useStore((s) => s.roadWidthM);
  const setRoadWidthM = useStore((s) => s.setRoadWidthM);
  const enforceFootprint = useStore((s) => s.enforceFootprint);
  const setEnforceFootprint = useStore((s) => s.setEnforceFootprint);
  const enforceMinRadius = useStore((s) => s.enforceMinRadius);
  const setEnforceMinRadius = useStore((s) => s.setEnforceMinRadius);
  const allowReverse = useStore((s) => s.allowReverse);
  const setAllowReverse = useStore((s) => s.setAllowReverse);
  const refineElasticBand = useStore((s) => s.refineElasticBand);
  const setRefineElasticBand = useStore((s) => s.setRefineElasticBand);
  const showWaypoints = useStore((s) => s.showWaypoints);
  const setShowWaypoints = useStore((s) => s.setShowWaypoints);
  const vehicleId = useStore((s) => s.vehicleId);
  const setVehicleId = useStore((s) => s.setVehicleId);
  const planMode = useStore((s) => s.planMode);
  const setPlanMode = useStore((s) => s.setPlanMode);
  const algorithm = useStore((s) => s.algorithm);
  const setAlgorithm = useStore((s) => s.setAlgorithm);
  const busy = useStore((s) => s.busy);
  const route = useStore((s) => s.route);
  const savedRoutes = useStore((s) => s.savedRoutes);
  const setSavedRoutes = useStore((s) => s.setSavedRoutes);
  const [routeName, setRouteName] = useState("");

  function saveCurrentRoute() {
    const s = useStore.getState();
    if (!s.route) return;
    const name = routeName.trim() || `経路 ${savedRoutes.length + 1}`;
    const id = crypto.randomUUID();
    setSavedRoutes([...savedRoutes, { id, name, route: s.route, waypoints: s.waypoints, vehicleId: s.vehicleId }]);
    setRouteName("");
    setStatus(`経路を保存: ${name}（複数台＝Fleet の経路集合に追加。プロジェクト保存で永続化）`, "success");
  }
  function loadSavedRoute(id: string) {
    const sr = savedRoutes.find((r) => r.id === id);
    if (!sr) return;
    useStore.getState().setWaypoints(sr.waypoints);
    dispatch({ type: "SET_ROUTE", route: sr.route });
    setStatus(`経路を読込: ${sr.name}`, "info");
  }
  function deleteSavedRoute(id: string) {
    setSavedRoutes(savedRoutes.filter((r) => r.id !== id));
  }

  // 分岐(交差点): 保存経路上の点(s_frac)を起点姿勢(親の接線)にして、そこから枝経路を計画する。
  const [branchParent, setBranchParent] = useState("");
  const [branchFrac, setBranchFrac] = useState(0.5);
  async function startBranch() {
    const sr = savedRoutes.find((r) => r.id === branchParent) ?? savedRoutes[0];
    const pts = (sr?.route?.trajectory?.points ?? []).map((p) => [p.x, p.y] as [number, number]);
    if (!sr || pts.length < 2) {
      setStatus("分岐元の保存経路を選んでください（2点以上）", "warn");
      return;
    }
    try {
      const j = await api.fleetJunction(pts, branchFrac);
      useStore.getState().setWaypoints([
        { id: crypto.randomUUID(), role: "start", xy: { x: j.x, y: j.y }, heading_deg: j.heading_deg },
      ]);
      dispatch({ type: "SET_ROUTE", route: null });
      setStatus(
        `分岐起点を設定：${sr.name} の ${Math.round(branchFrac * 100)}% / 方位 ${j.heading_deg.toFixed(0)}°。` +
        "地図でゴールを追加→経路生成→「現在の経路を保存」でライブラリ（複数台）へ。",
        "success",
      );
    } catch (e) {
      setStatus(`分岐起点の取得に失敗: ${String(e)}`, "error");
    }
  }

  const [vehicles, setVehicles] = useState<Vehicle[]>([]);
  const vehiclesRev = useStore((s) => s.vehiclesRev);

  useEffect(() => {
    api.listVehicles().then(setVehicles).catch(() => undefined);
  }, [vehiclesRev]);

  const costLayerId = useStore((s) => s.costLayerId);
  const drivableLayerId = useStore((s) => s.drivableLayerId);
  const costLayer = pickLayer(layers, "cost", costLayerId);
  const drivableLayer = pickLayer(layers, "drivable", drivableLayerId);
  // auto モードは hybrid_astar / grid_astar のみ（spline/dubins が選ばれていれば hybrid に矯正）
  const effAlgo: Algorithm =
    planMode === "auto" ? (algorithm === "grid_astar" ? "grid_astar" : "hybrid_astar") : algorithm;
  const needsGrid = planMode === "auto" || effAlgo === "grid_astar" || effAlgo === "hybrid_astar";

  async function generate() {
    if (waypoints.length < 2) return;
    if (useStore.getState().busy) return; // 多重送信ガード
    if (needsGrid && !costLayer) {
      setStatus("auto/A* には先にコストマップが必要です", "warn");
      return;
    }
    const setBusy = useStore.getState().setBusy;
    setBusy(true, "経路を生成中…");
    setStatus("経路を生成中…");
    try {
      const res = await api.plan({
        waypoints: waypoints.map((w) => ({ x: w.xy.x, y: w.xy.y, heading_deg: w.heading_deg ?? null })),
        mode: planMode,
        algorithm: effAlgo,
        vehicle_id: vehicleId,
        spacing_m: routeSpacing,
        corridor_width_m: roadWidthM > 0 ? roadWidthM : null, // A* が領域内に確保する道幅
        enforce_footprint: enforceFootprint, // hybrid A* で実車体フットプリント包含を強制
        enforce_min_radius: enforceMinRadius, // R_min を持つ車種で最小旋回半径を保証
        allow_reverse: allowReverse, // hybrid A* で後進を許可
        refine_elastic_band: refineElasticBand, // 生成後 Elastic Band で洗練
        costmap_layer_id: costLayer?.id ?? null,
        drivable_layer_id: drivableLayer?.id ?? null,
        no_go_polygons: areas.length
          ? areas.map((a) => a.points.map((p) => [p.x, p.y] as [number, number]))
          : undefined,
      });
      dispatch({ type: "SET_ROUTE", route: { trajectory: res.trajectory, analysis: res.analysis, safety: res.safety } });
      const r = res.measured_min_radius_m;
      const rTxt = r == null ? "" : ` / R=${r.toFixed(1)}m`;
      const safeTxt = res.safety ? ` / 安全:${res.safety.passed ? "OK" : "NG"}` : "";
      setStatus(
        res.warning ? `経路生成完了（⚠ ${res.warning}）${safeTxt}` : `経路生成完了${rTxt}${safeTxt}`,
        res.warning || (res.safety && !res.safety.passed) ? "warn" : "success",
      );
    } catch (e) {
      setStatus(`経路生成に失敗: ${String(e)}`, "error");
    } finally {
      useStore.getState().setBusy(false);
    }
  }

  return (
    <section className="card">
      <h3>経路</h3>

      <div className="grid2">
        <label>
          車両
          <select value={vehicleId ?? ""} onChange={(e) => setVehicleId(e.target.value || null)}>
            <option value="">（指定なし）</option>
            {vehicles.map((v) => (
              <option key={v.id} value={v.id}>
                {v.id} {v.spec_status === "estimated" ? "≈" : ""}
              </option>
            ))}
          </select>
        </label>
        <label>
          生成モード
          <select value={planMode} onChange={(e) => setPlanMode(e.target.value as PlanMode)}>
            <option value="waypoint_guided">経由点ガイド</option>
            <option value="auto">自動生成（A*）</option>
          </select>
        </label>
      </div>
      <label>
        アルゴリズム
        <select value={effAlgo} onChange={(e) => setAlgorithm(e.target.value as Algorithm)}>
          {(planMode === "auto" ? AUTO_ALGOS : ALGOS).map((a) => (
            <option key={a.value} value={a.value}>
              {a.label}
            </option>
          ))}
        </select>
      </label>

      <div className="row wrap" style={{ marginTop: 6 }}>
        {MODES.map((m) => (
          <button key={m.mode} data-active={mode === m.mode} onClick={() => dispatch({ type: "SET_MODE", mode: m.mode })}>
            {m.label}
          </button>
        ))}
      </div>
      <div className="row" style={{ marginTop: 6 }}>
        <button className="primary" onClick={generate} disabled={waypoints.length < 2 || busy}>
          {busy ? "生成中…" : "Generate"}
        </button>
        <button
          onClick={() => {
            if (waypoints.length === 0 || window.confirm("waypoint と生成経路をすべて消去します。よろしいですか？")) {
              dispatch({ type: "RESET_ROUTE" });
            }
          }}
        >
          Reset
        </button>
        <button onClick={() => dispatch({ type: "UNDO" })} disabled={!canUndo} title="⌘/Ctrl+Z">
          Undo
        </button>
        <button onClick={() => dispatch({ type: "REDO" })} disabled={!canRedo} title="⌘/Ctrl+Shift+Z">
          Redo
        </button>
      </div>
      <div className="grid2">
        <label>
          spacing (m)
          <input type="number" step="0.5" min="0.1" value={routeSpacing} onChange={(e) => setRouteSpacing(+e.target.value)} />
        </label>
        <label title="0 のときは選択車両の車幅で道幅帯を表示。>0 で A* が領域内に確保する道幅にもなる">
          道幅 (m)（0=車幅で表示）
          <input type="number" step="0.5" min="0" value={roadWidthM} onChange={(e) => setRoadWidthM(+e.target.value)} />
        </label>
      </div>
      <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center" }}>
        <input type="checkbox" checked={showWaypoints} onChange={(e) => setShowWaypoints(e.target.checked)} />
        <span>show waypoints (点)</span>
      </label>
      {effAlgo === "hybrid_astar" && (
        <>
          <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center" }}>
            <input type="checkbox" checked={enforceFootprint} onChange={(e) => setEnforceFootprint(e.target.checked)} />
            <span>車体フットプリント包含を厳密化（Hybrid A*）</span>
          </label>
          <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center" }}>
            <input type="checkbox" checked={allowReverse} onChange={(e) => setAllowReverse(e.target.checked)} />
            <span>後進を許可（切り返しで狭所到達性UP）</span>
          </label>
        </>
      )}
      <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center" }}>
        <input type="checkbox" checked={enforceMinRadius} onChange={(e) => setEnforceMinRadius(e.target.checked)} />
        <span>最小旋回半径 R_min を保証（違反コーナーを開く）</span>
      </label>
      <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center" }}>
        <input
          type="checkbox"
          checked={refineElasticBand}
          onChange={(e) => setRefineElasticBand(e.target.checked)}
          disabled={!drivableLayer}
        />
        <span>Elastic Bandで洗練（中央寄せ・余裕確保）{!drivableLayer ? "（要走行可能領域）" : ""}</span>
      </label>
      <p className="hint">
        waypoints: {waypoints.length}
        {needsGrid && (costLayer ? "（A*: cost+領域に沿って探索）" : "（要コストマップ）")}
        。<b>クリックで配置 / 左ドラッグで方位（ベクトル）指定</b>。パンは右ドラッグ/Panモード、ズームはホイール/＋−。道幅&gt;0 で A* が道幅を確保。
      </p>

      {/* 経路ライブラリ: 生成した経路を名前付きで複数保存（プロジェクトに永続化）→ 呼び出して再編集 */}
      <div style={{ marginTop: 8, borderTop: "1px solid var(--line)", paddingTop: 8 }}>
        <p className="hint" style={{ margin: "0 0 4px" }}>経路ライブラリ（プロジェクトに複数保存）</p>
        <div className="row">
          <input
            type="text"
            placeholder="経路名"
            value={routeName}
            onChange={(e) => setRouteName(e.target.value)}
            style={{ flex: 1 }}
          />
          <button onClick={saveCurrentRoute} disabled={!route}>現在の経路を保存</button>
        </div>
        {savedRoutes.length > 0 && (
          <ul className="list" style={{ marginTop: 6 }}>
            {savedRoutes.map((sr) => (
              <li key={sr.id}>
                <span style={{ overflow: "hidden", textOverflow: "ellipsis" }}>
                  {sr.name} · {sr.waypoints.length}wp
                </span>
                <span className="row" style={{ gap: 4 }}>
                  <button onClick={() => loadSavedRoute(sr.id)}>読込</button>
                  <button className="del" onClick={() => deleteSavedRoute(sr.id)}>✕</button>
                </span>
              </li>
            ))}
          </ul>
        )}
        {savedRoutes.length > 0 && (
          <div style={{ marginTop: 8 }}>
            <p className="hint" style={{ margin: "0 0 4px" }}>分岐（交差点）: 既存経路の途中から枝を作る</p>
            <div className="row" style={{ gap: 4, flexWrap: "wrap" }}>
              <select value={branchParent || savedRoutes[0]?.id} onChange={(e) => setBranchParent(e.target.value)} style={{ flex: 1 }}>
                {savedRoutes.map((sr) => (<option key={sr.id} value={sr.id}>{sr.name}</option>))}
              </select>
              <label className="hint" style={{ display: "flex", flexDirection: "column" }}>
                位置 {Math.round(branchFrac * 100)}%
                <input type="range" min={0.05} max={0.95} step={0.05} value={branchFrac}
                       onChange={(e) => setBranchFrac(+e.target.value)} style={{ width: 90 }} />
              </label>
              <button onClick={startBranch}>分岐を開始</button>
            </div>
            <p className="hint" style={{ margin: "2px 0 0" }}>
              起点（親の接線方向）をセット→地図でゴール追加→経路生成→保存。なめらかに枝が出ます。
            </p>
          </div>
        )}
      </div>
    </section>
  );
}
