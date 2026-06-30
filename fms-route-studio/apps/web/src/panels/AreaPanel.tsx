import { useState } from "react";

import { dispatch } from "@/commandBus";
import { useStore } from "@/store/useStore";

const PRESETS = ["", "ParkingZone", "LoadingZone", "DumpingZone", "AutonomousOperationZone", "NoGoZone"];

export function AreaPanel() {
  const mode = useStore((s) => s.mode);
  const areas = useStore((s) => s.areas);
  const active = useStore((s) => s.activePolygon);
  const imported = useStore((s) => s.importedRoutes);
  const [preset, setPreset] = useState("");
  const [name, setName] = useState("");

  const finalName = name.trim() || preset || `Area_${areas.length + 1}`;

  return (
    <section className="card">
      <h3>エリア</h3>
      <div className="row wrap">
        <button data-active={mode === "polygon"} onClick={() => dispatch({ type: "SET_MODE", mode: "polygon" })}>
          Polygon
        </button>
        <button onClick={() => dispatch({ type: "FINISH_POLYGON", name: finalName })} disabled={active.length < 3}>
          Finish ({active.length})
        </button>
        <button onClick={() => dispatch({ type: "UNDO_POLY_VERTEX" })} disabled={active.length === 0}>
          1つ戻す
        </button>
        <button onClick={() => dispatch({ type: "CANCEL_POLYGON" })} disabled={active.length === 0}>
          Cancel
        </button>
        <button data-active={mode === "edit"} onClick={() => dispatch({ type: "SET_MODE", mode: mode === "edit" ? "pan" : "edit" })}>
          頂点編集
        </button>
      </div>
      {mode === "edit" && (
        <p className="hint">
          頂点をドラッグで移動／辺をクリックで頂点追加／Alt+クリックで頂点削除（経路の中継点も同様に編集できます）。
        </p>
      )}
      <div className="row wrap" style={{ marginTop: 6 }}>
        <select value={preset} onChange={(e) => setPreset(e.target.value)}>
          {PRESETS.map((p) => (
            <option key={p} value={p}>
              {p || "(custom)"}
            </option>
          ))}
        </select>
        <input type="text" placeholder="area name" value={name} onChange={(e) => setName(e.target.value)} />
      </div>
      <ul className="list">
        {areas.map((a) => (
          <li key={a.id}>
            <span>
              {a.name} · {a.points.length}pt
            </span>
            <button className="del" onClick={() => dispatch({ type: "DELETE_AREA", id: a.id })}>
              ✕
            </button>
          </li>
        ))}
        {areas.length === 0 && <li className="hint">no areas</li>}
      </ul>
      {imported.length > 0 && <p className="hint">imported routes: {imported.length}</p>}
    </section>
  );
}
