import { useRef, useState } from "react";

import { api } from "@/api/client";
import { JGD2011_ZONE_LIST, WORKING_EPSG } from "@/map/proj";
import { useStore } from "@/store/useStore";
import type { Layer } from "@/types/api";

export function LayerPanel() {
  const layers = useStore((s) => s.layers);
  const setLayers = useStore((s) => s.setLayers);
  const setStatus = useStore((s) => s.setStatus);
  const fileRef = useRef<HTMLInputElement>(null);
  const lasRef = useRef<HTMLInputElement>(null);
  const [busy, setBusy] = useState(false);
  const [makeOrtho, setMakeOrtho] = useState(true);
  const [crsEditId, setCrsEditId] = useState<string | null>(null);

  // 作業ゾーンと異なるゾーンで取り込まれたラスタ（地図に表示できない＝要再生成）
  const otherZone = layers.filter(
    (l) =>
      (l.kind === "ortho" || l.kind === "cost" || l.kind === "drivable") &&
      l.epsg != null &&
      l.epsg !== WORKING_EPSG,
  );

  async function upload(kind: string, f: File | undefined) {
    if (!f) {
      setStatus(kind === "las" ? "LASファイルを選択してください" : "GeoTIFFを選択してください", "warn");
      return;
    }
    setBusy(true);
    const gBusy = useStore.getState().setBusy;
    gBusy(true, `アップロード中… (${f.name})${kind === "las" && makeOrtho ? " ＋オルソ生成" : ""}`);
    setStatus(`アップロード中 ${f.name} → ${kind} …`);
    try {
      const layer = await api.uploadLayer(kind, f, kind === "las" ? { makeOrtho } : undefined);
      setLayers(await api.listLayers());
      if (kind === "las") {
        const ortho = layer.auto_ortho_id ? "・オルソ自動生成済み（2D地図に表示）" : "";
        const err = layer.ortho_error ? ` ⚠${layer.ortho_error}` : "";
        const noCrs = layer.epsg == null ? " ⚠CRS未指定（下のレイヤ一覧で座標系を指定できます）" : "";
        setStatus(`LAS登録: ${layer.id}${ortho}${err}${noCrs}`, layer.ortho_error || layer.epsg == null ? "warn" : "success");
      } else {
        setStatus(`登録: ${layer.id} (EPSG:${layer.epsg}, ${layer.crs_source})`, "success");
      }
    } catch (e) {
      setStatus(`アップロード失敗: ${String(e)}`, "error");
    } finally {
      gBusy(false);
      setBusy(false);
    }
  }

  async function remove(id: string) {
    if (!window.confirm(`レイヤ「${id}」を削除します。よろしいですか？`)) return;
    await api.deleteLayer(id);
    setLayers(await api.listLayers());
  }

  // LAS の座標系を後付け指定 → 以後の costmap/3D点群/オルソが指定 CRS で解釈される
  async function setLasEpsg(id: string, epsg: number | null) {
    setBusy(true);
    try {
      await api.setLayerEpsg(id, epsg);
      setLayers(await api.listLayers());
      setStatus(
        epsg
          ? `LAS ${id} の座標系を EPSG:${epsg} に設定しました。コストマップ/オルソは再生成してください。`
          : `LAS ${id} の座標系指定を解除しました（ヘッダ/自動推定に戻ります）。`,
        "success",
      );
    } catch (e) {
      setStatus(`座標系の設定に失敗: ${String(e)}`, "error");
    } finally {
      setBusy(false);
      setCrsEditId(null);
    }
  }

  async function genOrtho(id: string) {
    setBusy(true);
    const gBusy = useStore.getState().setBusy;
    gBusy(true, "LASからオルソを生成中…");
    try {
      const o = await api.makeOrthoFromLas(id);
      setLayers(await api.listLayers());
      setStatus(`オルソ生成: ${o.id}（${o.width}×${o.height}px, ${o.res_m}m/px）`, "success");
    } catch (e) {
      setStatus(`オルソ生成に失敗: ${String(e)}`, "error");
    } finally {
      gBusy(false);
      setBusy(false);
    }
  }

  function lasCrsLabel(l: Layer): string {
    if (l.epsg == null) return "CRSなし⚠";
    const src = l.crs_source === "user" ? "指定" : l.crs_source === "detected" ? "ヘッダ" : l.crs_source ?? "";
    return `EPSG:${l.epsg}(${src})`;
  }

  return (
    <section className="card">
      <h3>レイヤ</h3>
      {otherZone.length > 0 && (
        <p className="hint" style={{ color: "var(--warn)" }}>
          ⚠ 作業ゾーン(EPSG:{WORKING_EPSG})と異なるゾーンのラスタが {otherZone.length} 件あり、地図に表示されません。
          現ゾーンで再生成（再アップロード）してください。
        </p>
      )}
      <h4 style={{ margin: "4px 0" }}>GeoTIFF（Ortho / Cost）</h4>
      <input ref={fileRef} type="file" accept=".tif,.tiff,image/tiff" />
      <div className="row">
        <button disabled={busy} onClick={() => upload("ortho", fileRef.current?.files?.[0])}>
          オルソを取込
        </button>
        <button disabled={busy} onClick={() => upload("cost", fileRef.current?.files?.[0])}>
          コストを取込
        </button>
      </div>

      <h4 style={{ margin: "10px 0 4px" }}>点群（LAS / LAZ）</h4>
      <input ref={lasRef} type="file" accept=".las,.laz" />
      <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center" }}>
        <input type="checkbox" checked={makeOrtho} onChange={(e) => setMakeOrtho(e.target.checked)} />
        <span title="点群の平均色（RGBが無ければ標高グレー）から真上視のオルソ画像を作り、2D地図に表示します">
          取り込み時にオルソを自動生成
        </span>
      </label>
      <div className="row">
        <button disabled={busy} onClick={() => upload("las", lasRef.current?.files?.[0])}>
          LASを取込
        </button>
      </div>
      <p className="hint">
        LAS点群は<b>3Dビュー＋「点群」モード</b>で表示されます。オルソを自動生成すると<b>2D地図にも</b>表示されます。
        「マップ生成」でこのLASからコストマップを作成できます。
        ヘッダにCRSが無いLAS（国内の測量データに多い）は、下の一覧で<b>座標系を指定</b>してください。
      </p>

      <ul className="list">
        {layers.map((l) => (
          <li key={l.id}>
            <span>
              {l.kind === "las"
                ? `las · ${l.filename ?? l.id} · ${lasCrsLabel(l)}`
                : `${l.kind} · EPSG${l.epsg ?? "?"} · ${l.width ?? "?"}×${l.height ?? "?"}`}
            </span>
            <span className="row" style={{ gap: 4 }}>
              {l.kind === "las" && crsEditId !== l.id && (
                <>
                  <button
                    disabled={busy}
                    onClick={() => setCrsEditId(l.id)}
                    title="LASの座標系(EPSG)を指定/変更（ヘッダCRS欠落時に位置ズレを直す）"
                  >
                    座標系
                  </button>
                  <button disabled={busy} onClick={() => genOrtho(l.id)} title="この LAS からオルソを（再）生成">
                    オルソ生成
                  </button>
                </>
              )}
              {l.kind === "las" && crsEditId === l.id && (
                <select
                  disabled={busy}
                  defaultValue={l.epsg ?? 0}
                  onChange={(e) => {
                    const v = Number(e.target.value);
                    void setLasEpsg(l.id, v === 0 ? null : v);
                  }}
                >
                  <option value={0}>自動（ヘッダ/推定）</option>
                  <option value={4326}>WGS84 経緯度 (4326)</option>
                  {JGD2011_ZONE_LIST.map((z) => (
                    <option key={z.epsg} value={z.epsg}>
                      {z.label}
                    </option>
                  ))}
                </select>
              )}
              <button className="del" onClick={() => remove(l.id)}>
                ✕
              </button>
            </span>
          </li>
        ))}
        {layers.length === 0 && <li className="hint">レイヤがありません</li>}
      </ul>
    </section>
  );
}
