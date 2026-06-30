import { useRef, useState } from "react";

import { api } from "@/api/client";
import { WORKING_EPSG } from "@/map/proj";
import { useStore } from "@/store/useStore";

export function LayerPanel() {
  const layers = useStore((s) => s.layers);
  const setLayers = useStore((s) => s.setLayers);
  const setStatus = useStore((s) => s.setStatus);
  const fileRef = useRef<HTMLInputElement>(null);
  const lasRef = useRef<HTMLInputElement>(null);
  const [busy, setBusy] = useState(false);

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
    gBusy(true, `アップロード中… (${f.name})`);
    setStatus(`アップロード中 ${f.name} → ${kind} …`);
    try {
      const layer = await api.uploadLayer(kind, f);
      setLayers(await api.listLayers());
      setStatus(
        kind === "las"
          ? `LAS登録: ${layer.id}（3Dビューの「点群」で表示／「マップ生成」でコストマップ作成）`
          : `登録: ${layer.id} (EPSG:${layer.epsg}, ${layer.crs_source})`,
        "success",
      );
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
          Upload Ortho
        </button>
        <button disabled={busy} onClick={() => upload("cost", fileRef.current?.files?.[0])}>
          Upload Cost
        </button>
      </div>

      <h4 style={{ margin: "10px 0 4px" }}>点群（LAS / LAZ）</h4>
      <input ref={lasRef} type="file" accept=".las,.laz" />
      <div className="row">
        <button disabled={busy} onClick={() => upload("las", lasRef.current?.files?.[0])}>
          Upload LAS
        </button>
      </div>
      <p className="hint">
        LAS点群は<b>3Dビュー＋「点群」モード</b>で表示されます（2D地図には出ません）。
        「マップ生成」でこのLASからコストマップを作成できます。
      </p>

      <ul className="list">
        {layers.map((l) => (
          <li key={l.id}>
            <span>
              {l.kind === "las"
                ? `las · ${l.filename ?? l.id}`
                : `${l.kind} · EPSG${l.epsg ?? "?"} · ${l.width ?? "?"}×${l.height ?? "?"}`}
            </span>
            <button className="del" onClick={() => remove(l.id)}>
              ✕
            </button>
          </li>
        ))}
        {layers.length === 0 && <li className="hint">no layers</li>}
      </ul>
    </section>
  );
}
