import { useEffect, useRef, useState } from "react";

import { api } from "@/api/client";
import { useStore } from "@/store/useStore";

// 複数台制御 Phase B: 保存経路ライブラリ(savedRoutes)を「経路集合」として扱い、
// 車幅コリドーの重なり（競合区間/交差）を判定・可視化する。
export function FleetPanel() {
  const savedRoutes = useStore((s) => s.savedRoutes);
  const vehicleId = useStore((s) => s.vehicleId);
  const showSavedRoutes = useStore((s) => s.showSavedRoutes);
  const setShowSavedRoutes = useStore((s) => s.setShowSavedRoutes);
  const fleetConflicts = useStore((s) => s.fleetConflicts);
  const setFleetConflicts = useStore((s) => s.setFleetConflicts);
  const fleetSim = useStore((s) => s.fleetSim);
  const setFleetSim = useStore((s) => s.setFleetSim);
  const fleetSimT = useStore((s) => s.fleetSimT);
  const setFleetSimT = useStore((s) => s.setFleetSimT);
  const fleetBays = useStore((s) => s.fleetBays);
  const setFleetBay = useStore((s) => s.setFleetBay);
  const setStatus = useStore((s) => s.setStatus);
  const [clearance, setClearance] = useState(0.5);
  const [gap, setGap] = useState(2.0);
  const [autoPassing, setAutoPassing] = useState(false);
  const [dispatch, setDispatch] = useState<"simultaneous" | "sequential">("simultaneous");
  const [loops, setLoops] = useState(2);
  const [busy, setBusy] = useState(false);
  const [playing, setPlaying] = useState(false);
  const playRef = useRef<number | null>(null);

  const routeName = (i: number) => savedRoutes[i]?.name ?? `経路${i + 1}`;

  const stopPlay = () => {
    if (playRef.current != null) {
      window.clearInterval(playRef.current);
      playRef.current = null;
    }
    setPlaying(false);
  };
  useEffect(() => stopPlay, []);

  function buildRoutes() {
    return savedRoutes.map((r, i) => ({
      name: r.name,
      points: (r.route?.trajectory?.points ?? []).map((p) => [p.x, p.y] as [number, number]),
      vehicle_id: r.vehicleId ?? vehicleId,
      priority: i, // ライブラリ順＝優先度（上ほど優先）
      bay: fleetBays[r.id] ?? undefined, // すれ違い点（待避所）。設定があれば横退避させる
    }));
  }

  const DEFAULT_BAY = { s_frac: 0.5, offset_m: 8.0, side: 1 as const, ramp_m: 8.0, hold_m: 16.0 };

  async function runSim() {
    const usable = savedRoutes.filter((r) => r.route?.trajectory?.points?.length >= 2);
    if (usable.length < 1) {
      setStatus("経路を保存してください（経路パネルで生成→保存）。", "warn");
      return;
    }
    stopPlay();
    setBusy(true);
    setStatus("簡易シミュレーション中…");
    try {
      const res = await api.fleetSimulate({ routes: buildRoutes(), dt_s: 0.2, gap_m: gap, clearance_m: clearance, auto_passing: autoPassing, dispatch, loops });
      setFleetSim(res);
      setFleetSimT(0);
      setShowSavedRoutes(true);
      const sep = res.min_separation_m != null ? `最接近 ${res.min_separation_m}m` : "";
      const msg =
        res.status === "DEADLOCK"
          ? `デッドロック検出 @${res.deadlock_time_s}s（待避所が必要＝Phase C）`
          : res.status === "COLLISION"
            ? `⚠ 接触の危険（${sep}）。単線対向など＝待避所(Phase C)が必要`
            : res.status === "TIMEOUT"
              ? `時間切れ（${res.makespan_s}s）。経路/優先度を見直してください`
              : `完了: 所要 ${res.makespan_s}s / ${sep}${res.auto_bays?.length ? ` / 自動待避所 ${res.auto_bays.length}個` : ""}`;
      setStatus(msg, res.status === "OK" ? "success" : "warn");
    } catch (e) {
      setStatus(`シミュレーション失敗: ${String(e)}`, "error");
    } finally {
      setBusy(false);
    }
  }

  function togglePlay() {
    if (!fleetSim) return;
    if (playing) {
      stopPlay();
      return;
    }
    const dt = fleetSim.traces[0]?.[1]?.t ?? 0.2; // フレーム間隔（=sim dt）
    setPlaying(true);
    if (fleetSimT >= fleetSim.makespan_s) setFleetSimT(0);
    playRef.current = window.setInterval(() => {
      const s = useStore.getState();
      const next = +(s.fleetSimT + dt).toFixed(2);
      if (next >= (s.fleetSim?.makespan_s ?? 0)) {
        s.setFleetSimT(s.fleetSim?.makespan_s ?? 0);
        stopPlay();
      } else {
        s.setFleetSimT(next);
      }
    }, Math.max(20, dt * 1000)); // 概ね実時間（dt秒ごと）
  }

  async function detect() {
    const usable = savedRoutes.filter((r) => r.route?.trajectory?.points?.length >= 2);
    if (usable.length < 2) {
      setStatus("経路を2本以上保存してください（経路パネルで生成→保存）。", "warn");
      return;
    }
    setBusy(true);
    setStatus("経路の競合を判定中…");
    try {
      const routes = savedRoutes.map((r) => ({
        name: r.name,
        points: (r.route?.trajectory?.points ?? []).map((p) => [p.x, p.y] as [number, number]),
        vehicle_id: r.vehicleId ?? vehicleId,
      }));
      const res = await api.fleetConflicts({ routes, cell_m: 0.5, clearance_m: clearance });
      setFleetConflicts(res.conflicts);
      setShowSavedRoutes(true);
      setStatus(
        res.n_conflicts > 0
          ? `競合 ${res.n_conflicts} 件を検出（地図に赤で表示）`
          : "競合は検出されませんでした",
        res.n_conflicts > 0 ? "warn" : "success",
      );
    } catch (e) {
      setStatus(`競合判定に失敗: ${String(e)}`, "error");
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className="panel">
      <p className="hint" style={{ margin: "0 0 6px" }}>
        保存済みの経路（経路パネルで生成→保存）を集合として、車幅コリドーの重なり（交差・併走/対向）を判定します。
      </p>

      <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center" }}>
        <input type="checkbox" checked={showSavedRoutes} onChange={(e) => setShowSavedRoutes(e.target.checked)} />
        保存経路を地図に重ねて表示
      </label>

      <div className="field-row" style={{ marginTop: 6 }}>
        <label title="車車間の追加余裕[m]。各経路の半幅(車幅/2)に上乗せして重なり判定を厳しめにする">
          車間マージン (m)
          <input type="number" step="0.1" min="0" value={clearance} onChange={(e) => setClearance(Math.max(0, +e.target.value))} />
        </label>
      </div>

      <div style={{ margin: "6px 0" }}>
        <b>経路集合（{savedRoutes.length} 本）</b>
        {savedRoutes.length === 0 ? (
          <p className="hint" style={{ margin: "4px 0" }}>まだありません。「経路」パネルで経路を生成し、名前を付けて保存してください。</p>
        ) : (
          <ul className="route-list">
            {savedRoutes.map((r, i) => {
              const bay = fleetBays[r.id];
              return (
                <li key={r.id} style={{ flexWrap: "wrap" }}>
                  <span style={{ display: "inline-block", width: 12, height: 12, borderRadius: 3, marginRight: 6, verticalAlign: "middle", background: ROUTE_COLORS[i % ROUTE_COLORS.length] }} />
                  <span style={{ flex: 1 }}>{r.name}</span>
                  <span className="hint">{r.vehicleId ?? vehicleId ?? "車両未設定"}</span>
                  <label className="hint" style={{ display: "flex", gap: 4, alignItems: "center", marginLeft: 6 }}
                         title="この経路上にすれ違い用の待避所(横退避)を設ける。対向はどちらかに設定すると捌ける">
                    <input type="checkbox" checked={!!bay}
                           onChange={(e) => setFleetBay(r.id, e.target.checked ? DEFAULT_BAY : null)} />
                    待避所
                  </label>
                  {bay && (
                    <div style={{ flexBasis: "100%", display: "flex", gap: 6, marginTop: 4, paddingLeft: 18, flexWrap: "wrap" }}>
                      <label className="hint" style={{ display: "flex", flexDirection: "column" }}>
                        位置 {Math.round(bay.s_frac * 100)}%
                        <input type="range" min={0.05} max={0.95} step={0.05} value={bay.s_frac}
                               onChange={(e) => setFleetBay(r.id, { ...bay, s_frac: +e.target.value })} style={{ width: 90 }} />
                      </label>
                      <label className="hint" style={{ display: "flex", flexDirection: "column" }}>
                        退避量m
                        <input type="number" step="1" min="1" value={bay.offset_m} style={{ width: 56 }}
                               onChange={(e) => setFleetBay(r.id, { ...bay, offset_m: Math.max(1, +e.target.value) })} />
                      </label>
                      <button className="hint" style={{ alignSelf: "flex-end" }}
                              onClick={() => setFleetBay(r.id, { ...bay, side: (bay.side === 1 ? -1 : 1) })}>
                        {bay.side === 1 ? "左" : "右"}へ
                      </button>
                    </div>
                  )}
                </li>
              );
            })}
          </ul>
        )}
      </div>

      <div className="field-row">
        <button className="primary" disabled={busy || savedRoutes.length < 2} onClick={detect}>
          {busy ? "判定中…" : "競合を判定"}
        </button>
        {fleetConflicts && (
          <button onClick={() => setFleetConflicts(null)}>結果をクリア</button>
        )}
      </div>

      {fleetConflicts && (
        <div style={{ marginTop: 8 }}>
          <b>競合 {fleetConflicts.length} 件</b>
          {fleetConflicts.length === 0 ? (
            <p className="hint" style={{ margin: "4px 0" }}>重なりはありません。</p>
          ) : (
            <ul className="route-list">
              {fleetConflicts.map((c, i) => (
                <li key={i}>
                  <span>{routeName(c.a)} × {routeName(c.b)}</span>
                  <span className="hint">{c.kind === "shared" ? "区間共有" : "交差"} / {c.overlap_area_m2}㎡</span>
                </li>
              ))}
            </ul>
          )}
          <p className="hint" style={{ margin: "4px 0 0" }}>
            赤帯=競合区間（区間Mutex対象）/ 赤枠=重なり範囲。
          </p>
        </div>
      )}

      {/* 簡易シミュレーション（区間予約Mutex＋優先度＋規定減速停止） */}
      <hr style={{ margin: "10px 0", border: 0, borderTop: "1px solid var(--line, #e5e7eb)" }} />
      <b>簡易シミュレーション</b>
      <p className="hint" style={{ margin: "2px 0 6px" }}>
        ライブラリ順＝優先度（上ほど優先）。交差では低優先車が手前で減速停止→相手通過後に再発進します。
      </p>
      <div className="field-row">
        <label title="simultaneous=全車同時に発進 / sequential=1台ずつ。前車がGoal到達で次がStart、全車終わると最初へ戻り周回">
          発進方式
          <select value={dispatch} onChange={(e) => setDispatch(e.target.value as "simultaneous" | "sequential")}>
            <option value="simultaneous">同時発進（競合・待避を見る）</option>
            <option value="sequential">逐次ローテーション（1台ずつ周回）</option>
          </select>
        </label>
        {dispatch === "sequential" && (
          <label title="各車が経路を何周するか（Goal到達で次の車、全車終わると最初へ戻る）">
            周回数
            <input type="number" step="1" min="1" max="50" value={loops} style={{ width: 56 }}
                   onChange={(e) => setLoops(Math.max(1, Math.min(50, Math.round(+e.target.value))))} />
          </label>
        )}
      </div>
      {dispatch === "simultaneous" && (
        <>
          <div className="field-row">
            <label title="占有区間の手前で止まる際の停止マージン[m]（規定減速度に上乗せの安全余裕）">
              停止マージン (m)
              <input type="number" step="0.5" min="0" value={gap} onChange={(e) => setGap(Math.max(0, +e.target.value))} />
            </label>
          </div>
          <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center" }}
                 title="接触(対向単線など)が出たら、低優先側の経路へ自動で待避所(横退避)を入れて解決を試みる">
            <input type="checkbox" checked={autoPassing} onChange={(e) => setAutoPassing(e.target.checked)} />
            接触時に自動で待避所を入れる
          </label>
        </>
      )}
      <div className="field-row">
        <button className="primary" disabled={busy || savedRoutes.length < 1} onClick={runSim}>
          {busy ? "実行中…" : "シミュレーション実行"}
        </button>
        {fleetSim && <button onClick={() => { stopPlay(); setFleetSim(null); }}>クリア</button>}
      </div>

      {fleetSim && (
        <div style={{ marginTop: 6 }}>
          <div className="field-row" style={{ alignItems: "center", gap: 8 }}>
            <button onClick={togglePlay}>{playing ? "⏸ 停止" : "▶ 再生"}</button>
            <span className="hint">
              t={fleetSimT.toFixed(1)}s / {fleetSim.makespan_s}s
              {fleetSim.deadlock ? " ⚠デッドロック" : ""}
            </span>
          </div>
          <input
            type="range" min={0} max={fleetSim.makespan_s} step={0.1} value={Math.min(fleetSimT, fleetSim.makespan_s)}
            onChange={(e) => { stopPlay(); setFleetSimT(+e.target.value); }}
            style={{ width: "100%" }}
          />
          <p className="hint" style={{ margin: "2px 0 0" }}>
            状態: {fleetSim.status === "OK" ? "全車到達" : fleetSim.status === "DEADLOCK" ? "膠着（待避所が必要）"
              : fleetSim.status === "COLLISION" ? "⚠接触の危険（待避所が必要）" : "時間切れ"}
            {fleetSim.min_separation_m != null ? ` / 最接近 ${fleetSim.min_separation_m}m` : ""}
            {fleetSim.total_wait_s != null ? ` / 総待機 ${fleetSim.total_wait_s}s` : ""}。
            地図で●=各車、停止中は赤縁。
          </p>
          {fleetSim.travel_time_s && fleetSim.travel_time_s.length > 0 && (
            <ul className="route-list" style={{ marginTop: 4 }}>
              {fleetSim.travel_time_s.map((tt, i) => (
                <li key={i}>
                  <span style={{ display: "inline-block", width: 10, height: 10, borderRadius: 2, marginRight: 6, background: ROUTE_COLORS[i % ROUTE_COLORS.length] }} />
                  <span style={{ flex: 1 }}>{fleetSim.names[i] ?? `経路${i + 1}`}</span>
                  <span className="hint">所要 {tt}s{fleetSim.wait_time_s?.[i] ? ` / 待機 ${fleetSim.wait_time_s[i]}s` : ""}</span>
                </li>
              ))}
            </ul>
          )}
        </div>
      )}
    </div>
  );
}

// 経路ごとの表示色（MapView と共有）。
export const ROUTE_COLORS = ["#2563eb", "#16a34a", "#d97706", "#9333ea", "#0891b2", "#db2777", "#65a30d", "#dc2626"];
