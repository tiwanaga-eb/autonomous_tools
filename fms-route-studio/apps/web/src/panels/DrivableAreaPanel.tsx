import { useState } from "react";

import { api } from "@/api/client";
import type { DrivableParams } from "@/api/client";
import { dispatch } from "@/commandBus";
import { pickLayer } from "@/layerSelect";
import { useStore } from "@/store/useStore";
import { NumberField } from "@/ui/NumberField";

// 設計書 §12: コストマップ → 走行可能領域。人が Include/Exclude で非破壊微修正。
export function DrivableAreaPanel() {
  const layers = useStore((s) => s.layers);
  const setLayers = useStore((s) => s.setLayers);
  const setStatus = useStore((s) => s.setStatus);
  const activePolygon = useStore((s) => s.activePolygon);
  const mode = useStore((s) => s.mode);
  const drivableOpacity = useStore((s) => s.drivableOpacity);
  const setDrivableOpacity = useStore((s) => s.setDrivableOpacity);
  const drivableVisible = useStore((s) => s.drivableVisible);
  const setDrivableVisible = useStore((s) => s.setDrivableVisible);

  const [drivableId, setDrivableId] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [p, setP] = useState<DrivableParams>({
    threshold: 150,
    close_m: 1.0,
    open_m: 0.5,
    min_area_m2: 50,
    clearance_m: 0,
    smooth_m: 0,
    max_hole_m2: 0,
    keep_largest: false,
    method: "threshold",
  });

  const costLayerId = useStore((s) => s.costLayerId);
  const costLayer = pickLayer(layers, "cost", costLayerId);
  const drivable = layers.find((l) => l.id === drivableId);
  const stats = drivable?.stats;

  const set = (k: keyof DrivableParams) => (v: number) => setP((prev) => ({ ...prev, [k]: v }));

  async function build() {
    if (!costLayer) {
      setStatus("先にコストマップを生成してください", "warn");
      return;
    }
    setBusy(true);
    const gBusy = useStore.getState().setBusy;
    gBusy(true, drivableId ? "走行可能領域を再生成中…" : "走行可能領域を生成中…");
    setStatus(drivableId ? "走行可能領域を再生成中…" : "走行可能領域を生成中…");
    try {
      const m = drivableId
        ? await api.regenerateDrivable(drivableId, p)
        : await api.generateDrivable(costLayer.id, p);
      setDrivableId(m.id);
      setLayers(await api.listLayers());
      setStatus(
        `走行可能領域: ${m.stats?.area_m2} m² / ${m.stats?.island_count} 島 / 道幅中央 ${m.stats?.width_median_m ?? "?"} m`,
        "success",
      );
    } catch (e) {
      setStatus(`走行可能領域の生成に失敗: ${String(e)}`, "error");
    } finally {
      gBusy(false);
      setBusy(false);
    }
  }

  async function removeEdit(index: number) {
    if (!drivableId) return;
    setBusy(true);
    try {
      await api.deleteDrivableEdit(drivableId, index);
      setLayers(await api.listLayers());
      setStatus("編集を削除しました");
    } catch (e) {
      setStatus(`削除に失敗: ${String(e)}`);
    } finally {
      setBusy(false);
    }
  }

  async function clearEdits() {
    if (!drivableId) return;
    if (!window.confirm("手修正（include/exclude）を全て消去します。よろしいですか？")) return;
    setBusy(true);
    try {
      await api.clearDrivableEdits(drivableId);
      setLayers(await api.listLayers());
      setStatus("全編集をクリアしました");
    } catch (e) {
      setStatus(`クリアに失敗: ${String(e)}`);
    } finally {
      setBusy(false);
    }
  }

  async function edit(op: "include" | "exclude") {
    if (!drivableId) {
      setStatus("先に走行可能領域を生成してください", "warn");
      return;
    }
    if (activePolygon.length < 3) {
      setStatus("先にポリゴンを描いてください（ポリゴンモード）", "warn");
      return;
    }
    setBusy(true);
    setStatus(`走行可能領域を${op === "include" ? "追加" : "除外"}中…`);
    try {
      const poly = activePolygon.map((pt) => [pt.x, pt.y] as [number, number]);
      const m = await api.editDrivable(drivableId, op, poly);
      dispatch({ type: "CANCEL_POLYGON" });
      setLayers(await api.listLayers());
      setStatus(`走行可能領域 ${op}: ${m.stats?.area_m2} m²`);
    } catch (e) {
      setStatus(`編集に失敗: ${String(e)}`);
    } finally {
      setBusy(false);
    }
  }

  return (
    <section className="card">
      <h3>走行可能領域</h3>
      {!costLayer && <p className="hint">先にコストマップを生成してください</p>}
      <label>
        生成方法
        <select
          value={p.method}
          onChange={(e) => setP((prev) => ({ ...prev, method: e.target.value as DrivableParams["method"] }))}
        >
          <option value="threshold">閾値（手動しきい値）</option>
          <option value="otsu">OpenCV Otsu（自動・大域）</option>
          <option value="adaptive">OpenCV 適応（局所・地形変化に強い）</option>
        </select>
      </label>
      <div className="grid2">
        <label title="コスト値がこの値以下のセルを走行可とみなす（threshold 法のみ）">
          しきい値{p.method !== "threshold" ? "（otsu/適応では無視）" : ""}
          <NumberField value={p.threshold} onCommit={set("threshold")} step={10} min={0} max={255} disabled={p.method !== "threshold"} />
        </label>
        <label title="境界から車体半幅ぶん内側へ縮める安全マージン">
          離隔 (m)
          <NumberField value={p.clearance_m} onCommit={set("clearance_m")} step={0.5} min={0} />
        </label>
        <label title="モルフォロジー closing。小さな切れ目・溝を接続する">
          隙間接続 close (m)
          <NumberField value={p.close_m} onCommit={set("close_m")} step={0.5} min={0} />
        </label>
        <label title="モルフォロジー opening。細いヒゲ・ノイズを除去する">
          ノイズ除去 open (m)
          <NumberField value={p.open_m} onCommit={set("open_m")} step={0.5} min={0} />
        </label>
        <label title="この面積未満の孤立領域（島）を除去">
          最小面積 (m²)
          <NumberField value={p.min_area_m2} onCommit={set("min_area_m2")} step={10} min={0} />
        </label>
        <label title="輪郭の平滑化半径">
          輪郭平滑 (m)
          <NumberField value={p.smooth_m} onCommit={set("smooth_m")} step={0.5} min={0} />
        </label>
        <label title="この面積以下の穴（走行不可の孤立点）を埋める">
          穴埋め ≤ (m²)
          <NumberField value={p.max_hole_m2} onCommit={set("max_hole_m2")} step={10} min={0} />
        </label>
      </div>
      <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center", marginTop: 6 }}>
        <input type="checkbox" checked={p.keep_largest} onChange={(e) => setP((prev) => ({ ...prev, keep_largest: e.target.checked }))} />
        <span>最大の連結領域のみ残す（島ノイズ除去）</span>
      </label>
      <div className="row" style={{ marginTop: 6 }}>
        <button className="primary" onClick={build} disabled={busy || !costLayer}>
          {busy ? "生成中…" : drivableId ? "再生成" : "生成"}
        </button>
        {drivableId && (
          <button onClick={() => setDrivableId(null)} disabled={busy} title="既存を残したまま別の走行可能領域を新規生成">
            新規
          </button>
        )}
      </div>

      {/* 表示制御（cost と重なって見づらい時に独立トグル/透明度） */}
      <label className="slider" style={{ flexDirection: "row", gap: 6, alignItems: "center", marginTop: 6 }}>
        <input type="checkbox" checked={drivableVisible} onChange={(e) => setDrivableVisible(e.target.checked)} />
        <span>走行可能領域を表示</span>
      </label>
      <label className="slider">
        <span>走行可能領域 透明度: {Math.round(drivableOpacity * 100)}%</span>
        <input
          type="range"
          min={0}
          max={1}
          step={0.05}
          value={drivableOpacity}
          disabled={!drivableVisible}
          onChange={(e) => setDrivableOpacity(+e.target.value)}
        />
      </label>

      {drivableId && (
        <>
          <p className="hint" style={{ marginTop: 6 }}>
            手修正: <b>ポリゴンを描く</b>→<b>走行可に追加</b>で既存領域に<b>マージ（合体）</b> / <b>走行不可に除外</b>で穴あけ。
            非破壊（再生成しても保持）。{mode === "polygon" ? `　頂点 ${activePolygon.length} 点` : ""}
          </p>
          <div className="row wrap">
            <button data-active={mode === "polygon"} onClick={() => dispatch({ type: "SET_MODE", mode: "polygon" })}>
              {mode === "polygon" ? "描画中…クリックで頂点" : "ポリゴンを描く"}
            </button>
            <button
              className="primary"
              onClick={() => edit("include")}
              disabled={busy || activePolygon.length < 3}
              title="描いたポリゴンを走行可能領域にマージ（合体）"
            >
              走行可に追加（マージ）
            </button>
            <button
              onClick={() => edit("exclude")}
              disabled={busy || activePolygon.length < 3}
              title="描いたポリゴンを走行可能領域から除外（穴あけ）"
            >
              走行不可に除外
            </button>
            <button onClick={() => dispatch({ type: "UNDO_POLY_VERTEX" })} disabled={activePolygon.length === 0}>
              1つ戻す
            </button>
            <button onClick={() => dispatch({ type: "CANCEL_POLYGON" })} disabled={activePolygon.length === 0}>
              取消
            </button>
          </div>
          {drivable?.edits && drivable.edits.length > 0 && (
            <>
              <div className="row" style={{ justifyContent: "space-between", marginTop: 6 }}>
                <span className="hint" style={{ margin: 0 }}>編集 {drivable.edits.length} 件</span>
                <button className="del" onClick={clearEdits} disabled={busy}>全消去</button>
              </div>
              <ul className="list">
                {drivable.edits.map((e, i) => (
                  <li key={i}>
                    <span>
                      {i + 1}. {e.op === "include" ? "走行可に追加（マージ）" : "走行不可に除外"} ・ {e.polygon.length}頂点
                    </span>
                    <button className="del" onClick={() => removeEdit(i)} disabled={busy}>削除</button>
                  </li>
                ))}
              </ul>
            </>
          )}
        </>
      )}
      {stats && (
        <ul className="metrics" style={{ marginTop: 6 }}>
          <li>
            <span>面積</span>
            <b>{stats.area_m2} m²</b>
          </li>
          <li>
            <span>島 / 穴</span>
            <b>{stats.island_count} / {stats.hole_count ?? 0}</b>
          </li>
          <li>
            <span>最大領域</span>
            <b>{stats.largest_island_m2} m²</b>
          </li>
          <li>
            <span>道幅 median</span>
            <b>{stats.width_median_m ?? "?"} m</b>
          </li>
          <li>
            <span>道幅 max</span>
            <b>{stats.width_max_m ?? "?"} m</b>
          </li>
        </ul>
      )}
    </section>
  );
}
