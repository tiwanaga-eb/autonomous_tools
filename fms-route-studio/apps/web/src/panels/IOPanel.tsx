import { useRef, useState } from "react";

import {
  exportAllRoutesGeoJson,
  exportAllRoutesSeparate,
  exportRouteCsv,
  exportRouteGeoJson,
  exportSpottingCsv,
  exportSpottingGeoJson,
} from "@/exporters";
import { exportProjectJson, importProjectJson } from "@/io";
import { useStore } from "@/store/useStore";

export function IOPanel() {
  const setStatus = useStore((s) => s.setStatus);
  const route = useStore((s) => s.route);
  const spotResult = useStore((s) => s.spotResult);
  const savedRoutes = useStore((s) => s.savedRoutes);
  const fileRef = useRef<HTMLInputElement>(null);
  const [busy, setBusy] = useState(false);

  async function exportAllRoutes(separate: boolean) {
    setBusy(true);
    try {
      const n = separate ? await exportAllRoutesSeparate() : await exportAllRoutesGeoJson();
      setStatus(n > 0 ? `保存ルート ${n} 本をエクスポート（${separate ? "個別ファイル" : "1ファイル"}）` : "保存ルートがありません", n > 0 ? "success" : "warn");
    } catch (e) {
      setStatus(`エクスポート失敗: ${String(e)}`, "error");
    } finally {
      setBusy(false);
    }
  }

  const guard = (ok: boolean, what: string) => {
    if (!ok) setStatus(`${what} がありません（先に生成してください）`);
    else setStatus(`${what} をエクスポートしました`);
  };

  async function save() {
    setBusy(true);
    try {
      await exportProjectJson();
      setStatus("exported JSON");
    } catch (e) {
      setStatus(`export failed: ${String(e)}`);
    } finally {
      setBusy(false);
    }
  }

  async function load() {
    const f = fileRef.current?.files?.[0];
    if (!f) {
      setStatus("choose a JSON file first");
      return;
    }
    setBusy(true);
    try {
      const r = await importProjectJson(f);
      setStatus(`imported: ${r.areas} areas, ${r.routes} routes`);
    } catch (e) {
      setStatus(`import failed: ${String(e)}`);
    } finally {
      setBusy(false);
    }
  }

  return (
    <section className="card">
      <h3>入出力（JSON）</h3>
      <div className="row">
        <button onClick={save} disabled={busy}>
          Save JSON
        </button>
      </div>
      <div className="row" style={{ marginTop: 6 }}>
        <input ref={fileRef} type="file" accept="application/json,.json" />
        <button onClick={load} disabled={busy}>
          Load JSON
        </button>
      </div>
      <p className="hint">haulRoutes / polygons 形式（WGS84 + points_xy）で後方互換</p>

      <h3 style={{ marginTop: 12 }}>Export</h3>
      <p className="hint" style={{ marginTop: 0 }}>保存ルート一括（プロジェクトの全ルート / 元の名前を引き継ぐ）</p>
      <div className="row wrap">
        <button onClick={() => exportAllRoutes(false)} disabled={busy || savedRoutes.length === 0}>
          全ルート（1ファイル）
        </button>
        <button onClick={() => exportAllRoutes(true)} disabled={busy || savedRoutes.length === 0}>
          全ルート（個別ファイル）
        </button>
      </div>
      <p className="hint" style={{ marginTop: 6 }}>経路（解析軌跡）</p>
      <div className="row wrap">
        <button onClick={() => guard(exportRouteCsv(), "経路CSV")} disabled={!route}>
          Route CSV
        </button>
        <button onClick={async () => guard(await exportRouteGeoJson(), "経路GeoJSON")} disabled={!route}>
          Route GeoJSON
        </button>
      </div>
      <p className="hint" style={{ marginTop: 6 }}>寄り付き（spotting）</p>
      <div className="row wrap">
        <button onClick={() => guard(exportSpottingCsv(), "寄り付きCSV")} disabled={!spotResult}>
          Spotting CSV
        </button>
        <button onClick={async () => guard(await exportSpottingGeoJson(), "寄り付きGeoJSON")} disabled={!spotResult}>
          Spotting GeoJSON
        </button>
      </div>
      <p className="hint">CSV=6677メートル座標 / GeoJSON=WGS84(lng,lat)。</p>
    </section>
  );
}
