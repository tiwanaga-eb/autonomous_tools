// 排土（パイル）配置パネル。エリア内に円錐パイル（体積/高さ＋安息角）を均等配置する。
import { api } from "@/api/client";
import { useStore } from "@/store/useStore";
import { runBusy } from "@/ui/busy";

export function PilePanel() {
  const areas = useStore((s) => s.areas);
  const busy = useStore((s) => s.busy);
  const setStatus = useStore((s) => s.setStatus);

  const areaId = useStore((s) => s.pileAreaId);
  const setAreaId = useStore((s) => s.setPileAreaId);
  const sizeMode = useStore((s) => s.pileSizeMode);
  const setSizeMode = useStore((s) => s.setPileSizeMode);
  const volumeM3 = useStore((s) => s.pileVolumeM3);
  const setVolumeM3 = useStore((s) => s.setPileVolumeM3);
  const heightM = useStore((s) => s.pileHeightM);
  const setHeightM = useStore((s) => s.setPileHeightM);
  const reposeDeg = useStore((s) => s.pileReposeDeg);
  const setReposeDeg = useStore((s) => s.setPileReposeDeg);
  const placeMode = useStore((s) => s.pilePlaceMode);
  const setPlaceMode = useStore((s) => s.setPilePlaceMode);
  const dx = useStore((s) => s.pileDx);
  const setDx = useStore((s) => s.setPileDx);
  const dy = useStore((s) => s.pileDy);
  const setDy = useStore((s) => s.setPileDy);
  const stagger = useStore((s) => s.pileStagger);
  const setStagger = useStore((s) => s.setPileStagger);
  const spreadT = useStore((s) => s.pileSpreadT);
  const setSpreadT = useStore((s) => s.setPileSpreadT);
  const edgeMargin = useStore((s) => s.pileEdgeMarginM);
  const setEdgeMargin = useStore((s) => s.setPileEdgeMarginM);
  const plan = useStore((s) => s.pilePlan);
  const setPlan = useStore((s) => s.setPilePlan);

  const area = areas.find((a) => a.id === areaId) ?? null;

  async function run() {
    const a = area ?? areas[0];
    if (!a || a.points.length < 3) {
      setStatus("対象エリアを選んでください（「エリア」機能でポリゴンを作成）", "warn");
      return;
    }
    await runBusy(
      "パイル配置を計算中…",
      async () => {
        const res = await api.earthworksPiles({
          polygon: a.points.map((p) => [p.x, p.y] as [number, number]),
          repose_deg: reposeDeg,
          volume_m3: sizeMode === "volume" ? volumeM3 : null,
          height_m: sizeMode === "height" ? heightM : null,
          dx_m: placeMode === "spacing" ? dx : null,
          dy_m: placeMode === "spacing" && dy > 0 ? dy : null,
          spread_thickness_m: placeMode === "spread" ? spreadT : null,
          stagger,
          edge_margin_m: edgeMargin >= 0 ? edgeMargin : null,
        });
        setPlan(res);
        const th = res.n_theory != null ? `（理論数 ${res.n_theory}）` : "";
        setStatus(
          `パイル ${res.count}個を配置${th}: 高さ${res.pile.height_m}m×基部半径${res.pile.radius_m}m` +
          `・間隔 ${res.spacing.dx_m}×${res.spacing.dy_m}m・合計 ${res.total_volume_m3}m³`,
          "success",
        );
      },
      { failPrefix: "パイル配置に失敗" },
    );
  }

  return (
    <section className="card">
      <h3>排土（パイル）配置</h3>
      <label title="配置先のエリア（「エリア」機能で作成した多角形）">
        対象エリア
        <select value={areaId ?? ""} onChange={(e) => setAreaId(e.target.value || null)}>
          <option value="">（先頭のエリアを使用）</option>
          {areas.map((a) => (
            <option key={a.id} value={a.id}>{a.name}</option>
          ))}
        </select>
      </label>

      <div className="grid2">
        <label title="パイルは安息角の円錐と仮定: 半径 r=h/tanφ, 体積 V=πr²h/3">
          大きさの指定
          <select value={sizeMode} onChange={(e) => setSizeMode(e.target.value as "volume" | "height")}>
            <option value="volume">体積から（ダンプ1杯）</option>
            <option value="height">高さから</option>
          </select>
        </label>
        {sizeMode === "volume" ? (
          <label>
            パイル体積 (m³)
            <input type="number" step="1" min="0.1" value={volumeM3} onChange={(e) => setVolumeM3(Math.max(0.1, +e.target.value))} />
          </label>
        ) : (
          <label>
            パイル高さ (m)
            <input type="number" step="0.1" min="0.1" value={heightM} onChange={(e) => setHeightM(Math.max(0.1, +e.target.value))} />
          </label>
        )}
        <label title="材料の安息角（法面が自然に安定する角度）。土砂 30〜40° 程度">
          安息角 (°)
          <input type="number" step="1" min="5" max="60" value={reposeDeg}
                 onChange={(e) => setReposeDeg(Math.min(60, Math.max(5, +e.target.value)))} />
        </label>
        <label title="間隔指定: 縦横の間隔で端から敷き詰め。撒き出し: 体積と撒き出し厚から間隔・数を自動計算">
          配置方法
          <select value={placeMode} onChange={(e) => setPlaceMode(e.target.value as "spacing" | "spread")}>
            <option value="spacing">間隔を指定</option>
            <option value="spread">撒き出しから計算</option>
          </select>
        </label>
        {placeMode === "spacing" ? (
          <>
            <label>
              間隔 横 (m)
              <input type="number" step="0.5" min="0.5" value={dx} onChange={(e) => setDx(Math.max(0.5, +e.target.value))} />
            </label>
            <label title="0 = 横と同じ間隔">
              間隔 縦 (m)（0=横と同じ）
              <input type="number" step="0.5" min="0" value={dy} onChange={(e) => setDy(Math.max(0, +e.target.value))} />
            </label>
          </>
        ) : (
          <label title="1パイル(V)を厚tで均すと面積V/tをカバー → 推奨間隔√(V/t)・理論数⌊A·t/V⌋">
            撒き出し厚 t (m)
            <input type="number" step="0.1" min="0.1" value={spreadT} onChange={(e) => setSpreadT(Math.max(0.1, +e.target.value))} />
          </label>
        )}
        <label title="エリアの縁から中心までの最小距離。-1=自動（パイル基部がエリア内に収まる=基部半径）">
          縁マージン (m)（-1=自動）
          <input type="number" step="0.5" min="-1" value={edgeMargin} onChange={(e) => setEdgeMargin(+e.target.value)} />
        </label>
      </div>
      <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center" }}>
        <input type="checkbox" checked={stagger} onChange={(e) => setStagger(e.target.checked)} />
        <span>千鳥配置（1行おきに半間隔ずらす）</span>
      </label>

      <div className="row" style={{ marginTop: 6 }}>
        <button className="primary" onClick={run} disabled={busy || areas.length === 0}>
          {busy ? "計算中…" : "配置を計算"}
        </button>
        <button onClick={() => setPlan(null)} disabled={!plan}>クリア</button>
      </div>

      {plan && (
        <ul className="metrics" style={{ marginTop: 8 }}>
          <li><span>配置数</span><b>{plan.count} 個{plan.n_theory != null ? ` / 理論 ${plan.n_theory}` : ""}</b></li>
          <li><span>パイル</span><b>高さ {plan.pile.height_m} m・基部半径 {plan.pile.radius_m} m</b></li>
          <li><span>1パイル体積</span><b>{plan.pile.volume_m3} m³</b></li>
          <li><span>合計体積</span><b>{plan.total_volume_m3} m³</b></li>
          <li><span>間隔</span><b>{plan.spacing.dx_m} × {plan.spacing.dy_m} m{plan.spacing.stagger ? "（千鳥）" : ""}</b></li>
          {plan.suggested_spacing_m != null && (
            <li><span>推奨間隔 √(V/t)</span><b>{plan.suggested_spacing_m} m</b></li>
          )}
          <li><span>エリア面積</span><b>{plan.area_m2} m²</b></li>
        </ul>
      )}
      <p className="hint" style={{ marginTop: 4 }}>
        パイルは安息角の円錐（r=h/tanφ）。格子はエリアの主方向に整列し、縁マージンを除いて端から敷き詰めます。
        地図に基部円（実寸）と中心点を表示します。
      </p>
    </section>
  );
}
