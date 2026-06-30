import { useEffect, useState } from "react";

import { api } from "@/api/client";
import { WORKING_EPSG } from "@/map/proj";
import { useStore } from "@/store/useStore";

// 設計書 §11: LAS → 生cost(float32) + DSM + 表示RGB を生成し、cost レイヤとして重畳。
// LASの読み込みは「データ（レイヤ）」で行い、ここでは登録済みLASを選んで生成する。
export function CostmapPanel() {
  const layers = useStore((s) => s.layers);
  const setLayers = useStore((s) => s.setLayers);
  const setStatus = useStore((s) => s.setStatus);
  const costOpacity = useStore((s) => s.costOpacity);
  const setCostOpacity = useStore((s) => s.setCostOpacity);
  const costVisible = useStore((s) => s.costVisible);
  const setCostVisible = useStore((s) => s.setCostVisible);
  const [busy, setBusy] = useState(false);

  const lasLayers = layers.filter((l) => l.kind === "las");
  const [lasLayerId, setLasLayerId] = useState("");
  // LASレイヤ一覧が変わったら選択を補正（未選択なら先頭、削除されたら先頭へ）
  useEffect(() => {
    if (lasLayers.length === 0) {
      if (lasLayerId) setLasLayerId("");
    } else if (!lasLayers.some((l) => l.id === lasLayerId)) {
      setLasLayerId(lasLayers[0].id);
    }
  }, [lasLayers, lasLayerId]);

  const [gridSize, setGridSize] = useState(0.3);
  const [slopeLimit, setSlopeLimit] = useState(15);
  const [canopyRef, setCanopyRef] = useState(1.0);
  const [srcEpsg, setSrcEpsg] = useState(WORKING_EPSG);
  const [wSlope, setWSlope] = useState(500);
  const [wRough, setWRough] = useState(100);
  const [roughWindow, setRoughWindow] = useState(1.0);

  async function build() {
    if (!lasLayerId) {
      setStatus("「データ」でLASを読み込み、ここで選択してください", "warn");
      return;
    }
    setBusy(true);
    const gBusy = useStore.getState().setBusy;
    gBusy(true, "コストマップ生成中…");
    try {
      setStatus("コストマップ生成中（傾斜/粗さ → cost + DSM）…");
      const cost = await api.generateCostmap({
        las_layer_id: lasLayerId,
        src_epsg: srcEpsg,
        target_epsg: WORKING_EPSG,
        params: {
          grid_size_m: gridSize,
          slope_limit_deg: slopeLimit,
          canopy_ref_m: canopyRef,
          w_slope: wSlope,
          w_rough: wRough,
          rough_window_m: roughWindow,
        },
      });
      setLayers(await api.listLayers());
      if (cost.warning) {
        setStatus(`⚠ ${cost.warning}`, "warn");
      } else {
        setStatus(
          `コストマップ生成完了: ${cost.id} ・ ${cost.density_pts_m2 ?? "?"} pts/m² ・ ${cost.pts_per_cell ?? "?"} 点/セル`,
          "success",
        );
      }
    } catch (e) {
      setStatus(`コストマップ生成に失敗: ${String(e)}`, "error");
    } finally {
      gBusy(false);
      setBusy(false);
    }
  }

  return (
    <section className="card">
      <h3>コストマップ（LASから）</h3>
      <label>
        LASレイヤ
        <select value={lasLayerId} onChange={(e) => setLasLayerId(e.target.value)}>
          {lasLayers.length === 0 && <option value="">（LAS未登録 — 「データ」で読み込み）</option>}
          {lasLayers.map((l) => (
            <option key={l.id} value={l.id}>
              {l.filename ?? l.id}
            </option>
          ))}
        </select>
      </label>
      <div className="grid2">
        <label>
          grid (m)
          <input type="number" step="0.1" value={gridSize} onChange={(e) => setGridSize(+e.target.value)} />
        </label>
        <label>
          slope limit (°)
          <input type="number" step="1" value={slopeLimit} onChange={(e) => setSlopeLimit(+e.target.value)} />
        </label>
        <label>
          veg ref (m)
          <input type="number" step="0.5" value={canopyRef} onChange={(e) => setCanopyRef(+e.target.value)} />
        </label>
        <label>
          LAS EPSG
          <input type="number" step="1" value={srcEpsg} onChange={(e) => setSrcEpsg(+e.target.value)} />
        </label>
        <label>
          w_slope（傾斜重み）
          <input type="number" step="50" value={wSlope} onChange={(e) => setWSlope(+e.target.value)} />
        </label>
        <label>
          w_rough（粗さ重み）
          <input type="number" step="25" value={wRough} onChange={(e) => setWRough(+e.target.value)} />
        </label>
        <label>
          rough window (m)
          <input type="number" step="0.5" value={roughWindow} onChange={(e) => setRoughWindow(+e.target.value)} />
        </label>
      </div>
      <div className="row" style={{ marginTop: 6 }}>
        <button className="primary" onClick={build} disabled={busy || !lasLayerId}>
          Build Costmap
        </button>
      </div>
      <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center" }}>
        <input type="checkbox" checked={costVisible} onChange={(e) => setCostVisible(e.target.checked)} />
        <span>cost を表示</span>
      </label>
      <label className="slider">
        <span>cost 透明度: {Math.round(costOpacity * 100)}%</span>
        <input
          type="range"
          min={0}
          max={1}
          step={0.05}
          value={costOpacity}
          disabled={!costVisible}
          onChange={(e) => setCostOpacity(+e.target.value)}
        />
      </label>
      <p className="hint">生cost(float32)＋DSM＋表示RGB(穴埋め/範囲外透明) を生成。cost は走行可能領域・勾配解析の入力（§12/§10）</p>
    </section>
  );
}
