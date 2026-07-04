import { useState } from "react";

import { dispatch } from "@/commandBus";
import { fmtArea, polygonArea } from "@/measure";
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
          ポリゴン描画
        </button>
        <button onClick={() => dispatch({ type: "FINISH_POLYGON", name: finalName })} disabled={active.length < 3}>
          確定 ({active.length})
        </button>
        <button onClick={() => dispatch({ type: "UNDO_POLY_VERTEX" })} disabled={active.length === 0}>
          1つ戻す
        </button>
        <button onClick={() => dispatch({ type: "CANCEL_POLYGON" })} disabled={active.length === 0}>
          取消
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
      {active.length >= 3 && (
        <p className="hint">作図中の面積: {fmtArea(polygonArea(active))}</p>
      )}
      <div className="row wrap" style={{ marginTop: 6 }}>
        <select value={preset} onChange={(e) => setPreset(e.target.value)}>
          {PRESETS.map((p) => (
            <option key={p} value={p}>
              {p || "(カスタム名)"}
            </option>
          ))}
        </select>
        <input type="text" placeholder="エリア名" value={name} onChange={(e) => setName(e.target.value)} />
      </div>
      <ul className="list">
        {areas.map((a) => (
          <li key={a.id}>
            <span>
              {a.name} · {a.points.length}頂点 · <b>{fmtArea(polygonArea(a.points))}</b>
            </span>
            <button className="del" onClick={() => dispatch({ type: "DELETE_AREA", id: a.id })}>
              ✕
            </button>
          </li>
        ))}
        {areas.length === 0 && <li className="hint">エリアがありません</li>}
      </ul>
      {imported.length > 0 && <p className="hint">インポート済みルート: {imported.length}</p>}
    </section>
  );
}
