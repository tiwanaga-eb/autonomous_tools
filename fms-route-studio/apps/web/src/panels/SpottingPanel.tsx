import type { ChangeEvent } from "react";
import { useEffect, useRef, useState } from "react";

import { api } from "@/api/client";
import { dispatch } from "@/commandBus";
import { pickLayer } from "@/layerSelect";
import { useStore } from "@/store/useStore";
import type { SpottingMethod, Vehicle } from "@/types/api";

// 設計書 §13: エリア内の寄り付き（切り返し 0/1）。ターゲット姿勢へ低速マニューバを生成・再生。
export function SpottingPanel() {
  const mode = useStore((s) => s.mode);
  const layers = useStore((s) => s.layers);
  const areas = useStore((s) => s.areas);
  const vehicleId = useStore((s) => s.vehicleId);
  const setVehicleId = useStore((s) => s.setVehicleId);
  const setStatus = useStore((s) => s.setStatus);
  const spotStart = useStore((s) => s.spotStart);
  const setSpotStart = useStore((s) => s.setSpotStart);
  const spotTarget = useStore((s) => s.spotTarget);
  const setSpotTarget = useStore((s) => s.setSpotTarget);
  const spotSwitchPose = useStore((s) => s.spotSwitchPose);
  const setSpotSwitchPose = useStore((s) => s.setSpotSwitchPose);
  const spotSwitchZoneId = useStore((s) => s.spotSwitchZoneId);
  const setSpotSwitchZoneId = useStore((s) => s.setSpotSwitchZoneId);
  const spotContainAreaId = useStore((s) => s.spotContainAreaId);
  const setSpotContainAreaId = useStore((s) => s.setSpotContainAreaId);
  const spotMaxSwitch = useStore((s) => s.spotMaxSwitch);
  const setSpotMaxSwitch = useStore((s) => s.setSpotMaxSwitch);
  const spotMethod = useStore((s) => s.spotMethod);
  const setSpotMethod = useStore((s) => s.setSpotMethod);
  const spotSmooth = useStore((s) => s.spotSmooth);
  const setSpotSmooth = useStore((s) => s.setSpotSmooth);
  const spotStationaryMode = useStore((s) => s.spotStationaryMode);
  const setSpotStationaryMode = useStore((s) => s.setSpotStationaryMode);
  const spotMinSpeedKmh = useStore((s) => s.spotMinSpeedKmh);
  const setSpotMinSpeedKmh = useStore((s) => s.setSpotMinSpeedKmh);
  const spotRoadWidthM = useStore((s) => s.spotRoadWidthM);
  const setSpotRoadWidthM = useStore((s) => s.setSpotRoadWidthM);
  const spotCuspMargin = useStore((s) => s.spotCuspMargin);
  const setSpotCuspMargin = useStore((s) => s.setSpotCuspMargin);
  const spotWithExit = useStore((s) => s.spotWithExit);
  const setSpotWithExit = useStore((s) => s.setSpotWithExit);
  const spotExitGoal = useStore((s) => s.spotExitGoal);
  const setSpotExitGoal = useStore((s) => s.setSpotExitGoal);
  const spotWeights = useStore((s) => s.spotWeights);
  const setSpotWeights = useStore((s) => s.setSpotWeights);
  const spotResult = useStore((s) => s.spotResult);
  const setSpotResult = useStore((s) => s.setSpotResult);
  const spotIndex = useStore((s) => s.spotIndex);
  const setSpotIndex = useStore((s) => s.setSpotIndex);
  const busy = useStore((s) => s.busy);

  const [vehicles, setVehicles] = useState<Vehicle[]>([]);
  useEffect(() => {
    api.listVehicles().then(setVehicles).catch(() => undefined);
  }, []);
  const veh = vehicles.find((v) => v.id === vehicleId);

  const costLayerId = useStore((s) => s.costLayerId);
  const drivableLayerId = useStore((s) => s.drivableLayerId);
  const drivable = pickLayer(layers, "drivable", drivableLayerId);
  const costLayer = pickLayer(layers, "cost", costLayerId);
  const W = spotWeights;
  const setW = (k: keyof typeof W) => (e: ChangeEvent<HTMLInputElement>) =>
    setSpotWeights({ ...W, [k]: +e.target.value });
  const playRef = useRef<number | null>(null);

  const stopPlay = () => {
    if (playRef.current != null) {
      window.clearInterval(playRef.current);
      playRef.current = null;
    }
  };
  useEffect(() => stopPlay, []);

  async function simulate() {
    if (!spotStart || !spotTarget) {
      setStatus("Start と Target の姿勢を設定してください（クリック配置 / 左ドラッグで方位）", "warn");
      return;
    }
    if (useStore.getState().busy) return; // 多重送信ガード
    stopPlay();
    const setBusy = useStore.getState().setBusy;
    setBusy(true, "寄り付きをシミュレーション中…");
    setStatus("寄り付きをシミュレーション中…");
    try {
      const zone = spotSwitchZoneId ? areas.find((a) => a.id === spotSwitchZoneId) : null;
      const contain = spotContainAreaId ? areas.find((a) => a.id === spotContainAreaId) : null;
      // 役割を持つエリア（切り返しゾーン / 走行を収めるエリア）は進入禁止(no_go)から除外する。
      const roleIds = new Set([spotSwitchZoneId, spotContainAreaId].filter(Boolean) as string[]);
      const noGo = areas.filter((a) => !roleIds.has(a.id));
      const res = await api.simulateSpotting({
        start: spotStart,
        target: spotTarget,
        max_switchbacks: spotMaxSwitch,
        require_switchback: spotMaxSwitch === 1, // 「1」は切り返し必須（後進で寄り付き）
        method: spotMethod,
        smooth_path: spotSmooth,
        // 据え切り: 自動=車種既定(null) / 許可=true(端点に円弧) / 禁止=false(端点直線リードイン)
        allow_stationary_steer:
          spotStationaryMode === "allow" ? true : spotStationaryMode === "deny" ? false : null,
        // 最低速度[km/h]（出発/到着/切返付近以外で下限。0=無効）
        min_speed_kmh: spotMinSpeedKmh > 0 ? spotMinSpeedKmh : 0,
        road_width_m: spotRoadWidthM > 0 ? spotRoadWidthM : null, // >0 で要求クリアランス=道幅/2
        vehicle_id: vehicleId,
        drivable_layer_id: drivable?.id ?? null,
        costmap_layer_id: costLayer?.id ?? null,
        no_go_polygons: noGo.length
          ? noGo.map((a) => a.points.map((p) => [p.x, p.y] as [number, number]))
          : undefined,
        // 走行を収めるエリア（経路＋車体をこの多角形内に制約。外は走行不可化）
        containment_polygon:
          contain && contain.points.length >= 3
            ? contain.points.map((p) => [p.x, p.y] as [number, number])
            : null,
        with_exit: spotWithExit,
        // 退出の行先Goal（指定時 target→exit_goal。未指定は start へ戻る）
        exit_goal:
          spotWithExit && spotExitGoal
            ? { x: spotExitGoal.x, y: spotExitGoal.y, heading_deg: spotExitGoal.heading_deg }
            : null,
        // 手動切り返し点（指定時はこの S 経由の前進→後進のみ生成）
        manual_switch_pose:
          spotMaxSwitch === 1 && spotSwitchPose
            ? { x: spotSwitchPose.x, y: spotSwitchPose.y, heading_deg: spotSwitchPose.heading_deg }
            : null,
        // 切り返し可能エリア（cusp をこの多角形内に制約）
        switchback_zone:
          spotMaxSwitch === 1 && zone && zone.points.length >= 3
            ? zone.points.map((p) => [p.x, p.y] as [number, number])
            : null,
        // 切り返し点の直線マージン（0=自動）。切り返し点をステア0°で反転して追従可能に。
        cusp_margin_m: spotCuspMargin > 0 ? spotCuspMargin : null,
        weights: spotWeights,
      });
      setSpotResult(res);
      const m = res.metrics;
      // 据え切り診断: 端点に直線が入ったか（=据え切り回避できたか）を可視化
      const ss = res.allow_stationary
        ? "据え切り許可"
        : `据え切り回避(端点直線 ${res.endpoint_margin_start_m ?? 0}/${res.endpoint_margin_goal_m ?? 0}m)`;
      setStatus(
        res.feasible
          ? `寄り付き: ${m.length_total_m}m / ${m.time_total_s}s / 切返${m.n_switchbacks}回 / 誤差${m.approach_error_m}m ・ ${ss}`
          : `⚠ 実現困難(${res.status}): 誤差${m.approach_error_m}m ・ ${ss}`,
        res.feasible ? "success" : "warn",
      );
    } catch (e) {
      setStatus(`寄り付き失敗: ${String(e)}`, "error");
    } finally {
      useStore.getState().setBusy(false);
    }
  }

  function play() {
    if (!spotResult || spotResult.points.length < 2) return;
    stopPlay();
    const n = spotResult.points.length;
    if (spotIndex >= n - 1) setSpotIndex(0);
    playRef.current = window.setInterval(() => {
      const s = useStore.getState();
      const i = s.spotIndex + 1;
      if (i >= (s.spotResult?.points.length ?? 0)) {
        stopPlay();
      } else {
        s.setSpotIndex(i);
      }
    }, 40);
  }

  const n = spotResult?.points.length ?? 0;
  const cur = spotResult?.points[Math.min(spotIndex, Math.max(0, n - 1))];

  return (
    <section className="card">
      <h3>寄り付きシミュレータ</h3>
      <label>
        車両（パラメータ）
        <select value={vehicleId ?? ""} onChange={(e) => setVehicleId(e.target.value || null)}>
          <option value="">（指定なし）</option>
          {vehicles.map((v) => (
            <option key={v.id} value={v.id}>
              {v.id} {v.spec_status === "estimated" ? "≈" : ""}
            </option>
          ))}
        </select>
      </label>
      {veh && (
        <p className="hint" style={{ margin: "2px 0" }}>
          R_min {veh.min_turning_radius ?? "—"}m / 幅 {veh.overall_width}m / {veh.kinematic_type}
        </p>
      )}
      <div className="row wrap" style={{ marginTop: 6 }}>
        <button data-active={mode === "spot_start"} onClick={() => dispatch({ type: "SET_MODE", mode: "spot_start" })}>
          Set Start
        </button>
        <button data-active={mode === "spot_target"} onClick={() => dispatch({ type: "SET_MODE", mode: "spot_target" })}>
          Set Target
        </button>
      </div>
      {/* heading（方位）数値入力。地図は クリック=位置 / 左ドラッグ=方位 */}
      <div className="grid2">
        <label>
          Start方位 (°)
          <input
            type="number"
            step="5"
            value={spotStart ? Math.round(spotStart.heading_deg) : 0}
            disabled={!spotStart}
            onChange={(e) => spotStart && setSpotStart({ ...spotStart, heading_deg: +e.target.value })}
          />
        </label>
        <label>
          Target方位 (°)
          <input
            type="number"
            step="5"
            value={spotTarget ? Math.round(spotTarget.heading_deg) : 0}
            disabled={!spotTarget}
            onChange={(e) => spotTarget && setSpotTarget({ ...spotTarget, heading_deg: +e.target.value })}
          />
        </label>
      </div>
      <div className="grid2">
        <label title="候補生成アルゴリズム。auto=全手法をコスト比較して最良を採用。hybrid A* は走行可能領域かコストマップが必要">
          アルゴリズム
          <select value={spotMethod} onChange={(e) => setSpotMethod(e.target.value as SpottingMethod)}>
            <option value="auto">auto（全手法を比較・推奨）</option>
            <option value="reeds_shepp">Reeds-Shepp（可変半径近似）</option>
            <option value="hybrid_astar">Hybrid A*（コスト考慮・要マップ）</option>
            <option value="dubins">Dubins（前進＋ステージ切り返し）</option>
          </select>
        </label>
        <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center" }}
               title="曲率の不連続(dκ/ds スパイク)による速度低下と蛇行を抑える後処理。エリア内・最小旋回半径・始終点姿勢は保持。">
          <input type="checkbox" checked={spotSmooth} onChange={(e) => setSpotSmooth(e.target.checked)} />
          経路を平滑化（速度低下/蛇行を抑制）
        </label>
        <label title="出発/到着の端点での据え切り（停止中に操舵）。自動=車種既定（ホイール車は禁止＝端点に直線リードイン/アウト、履帯車CD110Rは許可）。許可=端点に円弧を許す。禁止=全車で端点を直線にし据え切り回避。">
          据え切り（端点その場操舵）
          <select value={spotStationaryMode} onChange={(e) => setSpotStationaryMode(e.target.value as "auto" | "allow" | "deny")}>
            <option value="auto">自動（車種既定）</option>
            <option value="deny">禁止（端点を直線に）</option>
            <option value="allow">許可（据え切りOK）</option>
          </select>
        </label>
        <label title="出発/到着/切り返し点の付近以外で保つ最低速度[km/h]。クロール（過減速）を防ぐ。0=無効。付近は停止のため自動で減速します。">
          最低速度 (km/h)
          <input type="number" min={0} max={20} step={1} value={spotMinSpeedKmh}
                 onChange={(e) => setSpotMinSpeedKmh(Math.max(0, +e.target.value))} />
        </label>
        <label>
          切り返し回数
          <select value={spotMaxSwitch} onChange={(e) => setSpotMaxSwitch(Number(e.target.value) as 0 | 1)}>
            <option value={0}>0（前進のみ）</option>
            <option value={1}>1（必須・後進で寄り付き）</option>
          </select>
        </label>
        <label title="0 のときは選択車両の車幅で道幅帯を表示。>0 で要求クリアランス=道幅/2 にもなる">
          道幅 (m)（0=車幅で表示）
          <input
            type="number"
            step="0.5"
            min="0"
            value={spotRoadWidthM}
            onChange={(e) => setSpotRoadWidthM(+e.target.value)}
          />
        </label>
        <label title="選択すると、経路＋車体（フットプリント）がこのエリア内に収まるよう制約します（外は走行不可）。エリアは「エリア」機能で作成。走行可能レイヤがあればAND（両方の内側）。">
          走行を収めるエリア（はみ出し禁止・任意）
          <select value={spotContainAreaId ?? ""} onChange={(e) => setSpotContainAreaId(e.target.value || null)}>
            <option value="">（指定なし）</option>
            {areas.map((a) => (
              <option key={a.id} value={a.id}>{a.name}</option>
            ))}
          </select>
        </label>
      </div>

      {/* 切り返し（1回）時のみ: 手動切り返し点 / 切り返し可能エリア */}
      {spotMaxSwitch === 1 && (
        <div style={{ marginTop: 8, padding: "6px 8px", border: "1px solid var(--line)", borderRadius: 6 }}>
          <p className="hint" style={{ margin: "0 0 4px" }}>切り返し（後進で寄り付き）の制御</p>
          <div className="row wrap">
            <button
              data-active={mode === "spot_switch"}
              onClick={() => dispatch({ type: "SET_MODE", mode: "spot_switch" })}
              title="地図で切り返し点を手動配置（クリック=位置 / 左ドラッグ=方位）"
            >
              切り返し点を手動指定
            </button>
            <button onClick={() => setSpotSwitchPose(null)} disabled={!spotSwitchPose}>解除</button>
          </div>
          {spotSwitchPose && (
            <label style={{ marginTop: 4 }}>
              切り返し点 方位 (°)
              <input
                type="number"
                step="5"
                value={Math.round(spotSwitchPose.heading_deg)}
                onChange={(e) => setSpotSwitchPose({ ...spotSwitchPose, heading_deg: +e.target.value })}
              />
            </label>
          )}
          <label style={{ marginTop: 4 }}>
            切り返し可能エリア（任意）
            <select value={spotSwitchZoneId ?? ""} onChange={(e) => setSpotSwitchZoneId(e.target.value || null)}>
              <option value="">（制約なし）</option>
              {areas.map((a) => (
                <option key={a.id} value={a.id}>{a.name}</option>
              ))}
            </select>
          </label>
          <label style={{ marginTop: 4 }}>
            切り返し直線マージン (m)（0=自動）
            <input
              type="number"
              step="0.5"
              min="0"
              value={spotCuspMargin}
              onChange={(e) => setSpotCuspMargin(+e.target.value)}
              title="切り返し点の前後に入れる直線距離。ステアを0°にしてから前後反転するため、経路を追従できる"
            />
          </label>
          <p className="hint" style={{ margin: "4px 0 0" }}>
            手動点を指定するとその点経由で固定。エリア選択時は切り返し点をその中に限定（エリアは「エリア」機能で作成）。
          </p>
        </div>
      )}

      <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center", marginTop: 6 }}>
        <input type="checkbox" checked={spotWithExit} onChange={(e) => setSpotWithExit(e.target.checked)} />
        <span>退出軌道も生成（目標→{spotExitGoal ? "退出Goal" : "start"}）</span>
      </label>
      {spotWithExit && (
        <div style={{ marginTop: 4, padding: "6px 8px", border: "1px solid var(--line)", borderRadius: 6 }}>
          <p className="hint" style={{ margin: "0 0 4px" }}>退出の行先（未指定なら start へ戻る）</p>
          <div className="row wrap">
            <button
              data-active={mode === "spot_exit_goal"}
              onClick={() => dispatch({ type: "SET_MODE", mode: "spot_exit_goal" })}
              title="地図で退出Goalを配置（クリック=位置 / 左ドラッグ=方位）"
            >
              退出Goalを指定
            </button>
            <button onClick={() => setSpotExitGoal(null)} disabled={!spotExitGoal}>解除（startへ）</button>
          </div>
          {spotExitGoal && (
            <label style={{ marginTop: 4 }}>
              退出Goal 方位 (°)
              <input
                type="number"
                step="5"
                value={Math.round(spotExitGoal.heading_deg)}
                onChange={(e) => setSpotExitGoal({ ...spotExitGoal, heading_deg: +e.target.value })}
              />
            </label>
          )}
        </div>
      )}

      <p className="hint" style={{ marginTop: 8, marginBottom: 2 }}>コスト関数の重み（最小化）</p>
      <div className="row wrap" style={{ marginBottom: 4 }}>
        <button onClick={() => setSpotWeights({ w_distance: 2, w_time: 0, w_reverse: 1, w_switchback: 8, w_costmap: 2, w_turn: 6 })}
          title="距離を重視した既定バランス">最短距離重視</button>
        <button onClick={() => setSpotWeights({ w_distance: 1, w_time: 0, w_reverse: 3, w_switchback: 20, w_costmap: 2, w_turn: 8 })}
          title="切り返し・後進を強く避ける">切り返し最小</button>
        <button onClick={() => setSpotWeights({ w_distance: 1, w_time: 0, w_reverse: 1, w_switchback: 6, w_costmap: 12, w_turn: 6 })}
          title="斜面/粗さ(コスト)の高い所を避ける">コスト回避重視</button>
        <button onClick={() => setSpotWeights({ w_distance: 1, w_time: 0, w_reverse: 1, w_switchback: 12, w_costmap: 2, w_turn: 16 })}
          title="蛇行を抑え直線的に（クネクネ低減）">直線的に</button>
      </div>
      <div className="grid2">
        <label>距離<input type="number" step="0.5" value={W.w_distance} onChange={setW("w_distance")} /></label>
        <label>時間<input type="number" step="0.5" value={W.w_time} onChange={setW("w_time")} /></label>
        <label>後進距離<input type="number" step="0.5" value={W.w_reverse} onChange={setW("w_reverse")} /></label>
        <label>切り返し<input type="number" step="1" value={W.w_switchback} onChange={setW("w_switchback")} /></label>
        <label>コストマップ<input type="number" step="0.5" value={W.w_costmap} onChange={setW("w_costmap")} /></label>
        <label>旋回(蛇行抑制)<input type="number" step="1" value={W.w_turn} onChange={setW("w_turn")} /></label>
      </div>
      <div className="row" style={{ marginTop: 6 }}>
        <button className="primary" onClick={simulate} disabled={!spotStart || !spotTarget || busy}>
          {busy ? "計算中…" : "Simulate"}
        </button>
        <button onClick={() => { setSpotResult(null); useStore.getState().setSpotStart(null); useStore.getState().setSpotTarget(null); setSpotSwitchPose(null); setSpotExitGoal(null); stopPlay(); }}>
          Clear
        </button>
      </div>
      <p className="hint">
        姿勢は<b>クリックで位置 / 左ドラッグで方位</b>（数値入力も可）。パンは右ドラッグ。Start={spotStart ? "✓" : "—"} / Target={spotTarget ? "✓" : "—"}
        {drivable ? "（クリアランス評価: ON）" : "（走行可能領域なし＝クリアランス未評価）"}
      </p>

      {spotResult && (
        <>
          <ul className="metrics" style={{ marginTop: 6 }}>
            <li><span>length</span><b>{spotResult.metrics.length_total_m} m</b></li>
            <li><span>前進/後進</span><b>{spotResult.metrics.length_fwd_m}/{spotResult.metrics.length_rev_m} m</b></li>
            <li><span>所要時間</span><b>{spotResult.metrics.time_total_s} s</b></li>
            <li><span>切り返し</span><b>{spotResult.metrics.n_switchbacks} 回</b></li>
            <li><span>最小クリアランス</span><b>{spotResult.metrics.min_clearance_m ?? "—"} m</b></li>
            <li><span>コスト積分</span><b>{spotResult.metrics.cost_integral}</b></li>
            <li><span>score</span><b>{spotResult.metrics.score}</b></li>
            <li><span>寄り付き誤差</span><b>{spotResult.metrics.approach_error_m} m</b></li>
            <li><span>feasible</span><b className={spotResult.feasible ? "ok" : "bad"}>{spotResult.feasible ? "OK" : "NG"}</b></li>
          </ul>
          {spotResult.reason && (
            <p className="hint" style={{ color: "#fca5a5" }}>不可理由: {spotResult.reason}</p>
          )}
          {spotResult.exit && (
            <p className="hint">
              退出軌道: {spotResult.exit.metrics.length_total_m}m / 切返{spotResult.exit.metrics.n_switchbacks}回 /{" "}
              <b className={spotResult.exit.feasible ? "ok" : "bad"}>{spotResult.exit.feasible ? "OK" : "NG"}</b>
              {spotResult.exit.reason ? `（${spotResult.exit.reason}）` : ""}
            </p>
          )}
          <div className="row" style={{ marginTop: 6, alignItems: "center" }}>
            <button onClick={play}>▶ 再生</button>
            <button onClick={stopPlay}>⏸ 停止</button>
            {cur && <span className="hint" style={{ margin: 0 }}>t={cur.t.toFixed(1)}s ・ {cur.gear === "R" ? "後進" : "前進"}</span>}
          </div>
          <input
            type="range"
            min={0}
            max={Math.max(0, n - 1)}
            value={Math.min(spotIndex, Math.max(0, n - 1))}
            onChange={(e) => { stopPlay(); setSpotIndex(+e.target.value); }}
            style={{ width: "100%" }}
          />
        </>
      )}
    </section>
  );
}
