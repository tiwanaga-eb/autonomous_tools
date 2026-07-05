import { useEffect, useState } from "react";

import { api } from "@/api/client";
import { WORKING_EPSG } from "@/map/proj";
import { useStore } from "@/store/useStore";
import { NumberField } from "@/ui/NumberField";

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
  // LAS の CRS。0=自動（ヘッダ検出→無ければ経緯度判定でWGS84推定→作業ゾーン）。
  const [srcEpsg, setSrcEpsg] = useState(0);
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
        ...(srcEpsg > 0 ? { src_epsg: srcEpsg } : {}),  // 0=自動（ヘッダ/経緯度推定に委ねる）
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
          <NumberField value={gridSize} onCommit={setGridSize} step={0.1} min={0.05} max={10} />
        </label>
        <label>
          slope limit (°)
          <NumberField value={slopeLimit} onCommit={setSlopeLimit} step={1} min={1} max={45} />
        </label>
        <label>
          veg ref (m)
          <NumberField value={canopyRef} onCommit={setCanopyRef} step={0.5} min={0} max={20} />
        </label>
        <label title="LAS座標のCRS。自動=ヘッダのCRSを使用（無ければ経緯度らしき値ならWGS84と推定、それ以外は作業ゾーンのまま）。WGS84のLASもここが「自動」か「WGS84」なら作業ゾーンへ変換されます">
          LASのCRS
          <select
            value={srcEpsg === 0 || srcEpsg === 4326 || srcEpsg === WORKING_EPSG ? String(srcEpsg) : "custom"}
            onChange={(e) => {
              const v = e.target.value;
              if (v === "custom") setSrcEpsg(6677 === WORKING_EPSG ? 6675 : 6677);
              else setSrcEpsg(+v);
            }}
          >
            <option value="0">自動（ヘッダ / 経緯度は WGS84 と推定）</option>
            <option value="4326">WGS84 経緯度 (EPSG:4326)</option>
            <option value={String(WORKING_EPSG)}>作業ゾーン (EPSG:{WORKING_EPSG})</option>
            <option value="custom">その他（EPSG直接指定）</option>
          </select>
        </label>
        {srcEpsg !== 0 && srcEpsg !== 4326 && srcEpsg !== WORKING_EPSG && (
          <label>
            EPSG コード
            <NumberField value={srcEpsg} onCommit={setSrcEpsg} step={1} min={1000} max={99999} />
          </label>
        )}
        <label>
          w_slope（傾斜重み）
          <NumberField value={wSlope} onCommit={setWSlope} step={50} min={0} max={10000} />
        </label>
        <label>
          w_rough（粗さ重み）
          <NumberField value={wRough} onCommit={setWRough} step={25} min={0} max={10000} />
        </label>
        <label>
          rough window (m)
          <NumberField value={roughWindow} onCommit={setRoughWindow} step={0.5} min={0.1} max={20} />
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
