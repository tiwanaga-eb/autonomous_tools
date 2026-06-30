import { useEffect, useState } from "react";

import { api } from "@/api/client";
import { applyProject, serializeProject } from "@/projectState";
import { useStore } from "@/store/useStore";

type Summary = { id: string; name: string; updated_at: string | null };

// 設計書 §15 / Phase 6: ワーキング状態（waypoints/姿勢/パラメータ等）を名前付きでサーバ保存・読込。
export function ProjectPanel() {
  const setStatus = useStore((s) => s.setStatus);
  const [items, setItems] = useState<Summary[]>([]);
  const [name, setName] = useState("");
  const [currentId, setCurrentId] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  const refresh = () => api.listProjects().then(setItems).catch(() => undefined);
  useEffect(() => {
    refresh();
  }, []);

  async function saveNew() {
    if (!name.trim()) {
      setStatus("プロジェクト名を入力してください");
      return;
    }
    setBusy(true);
    try {
      const r = await api.createProject(name.trim(), serializeProject());
      setCurrentId(r.id);
      await refresh();
      setStatus(`プロジェクト保存: ${r.name}`);
    } catch (e) {
      setStatus(`保存失敗: ${String(e)}`);
    } finally {
      setBusy(false);
    }
  }

  async function update() {
    if (!currentId) return;
    setBusy(true);
    try {
      const r = await api.updateProject(currentId, name.trim() || "project", serializeProject());
      await refresh();
      setStatus(`上書き保存: ${r.name}`);
    } catch (e) {
      setStatus(`更新失敗: ${String(e)}`);
    } finally {
      setBusy(false);
    }
  }

  async function load(id: string) {
    setBusy(true);
    try {
      const p = await api.getProject(id);
      applyProject(p.state);
      setCurrentId(p.id);
      setName(p.name);
      const hasRoute = !!(p.state as { route?: unknown }).route;
      const nAreas = ((p.state as { areas?: unknown[] }).areas ?? []).length;
      setStatus(`読込: ${p.name}（経路${hasRoute ? "✓" : "—"} / エリア${nAreas}）`);
    } catch (e) {
      setStatus(`読込失敗: ${String(e)}`);
    } finally {
      setBusy(false);
    }
  }

  async function remove(id: string) {
    if (!window.confirm("このプロジェクトを削除します。よろしいですか？")) return;
    setBusy(true);
    try {
      await api.deleteProject(id);
      if (currentId === id) setCurrentId(null);
      await refresh();
      setStatus("プロジェクト削除");
    } catch (e) {
      setStatus(`削除失敗: ${String(e)}`);
    } finally {
      setBusy(false);
    }
  }

  return (
    <section className="card">
      <h3>プロジェクト</h3>
      <div className="row">
        <input
          type="text"
          placeholder="プロジェクト名"
          value={name}
          onChange={(e) => setName(e.target.value)}
          style={{ flex: 1 }}
        />
      </div>
      <div className="row" style={{ marginTop: 6 }}>
        <button className="primary" onClick={saveNew} disabled={busy}>
          新規保存
        </button>
        <button onClick={update} disabled={busy || !currentId}>
          上書き保存
        </button>
      </div>
      {items.length > 0 && (
        <ul className="list" style={{ marginTop: 8 }}>
          {items.map((p) => (
            <li key={p.id} data-active={p.id === currentId}>
              <span style={{ overflow: "hidden", textOverflow: "ellipsis" }}>{p.name}</span>
              <span className="row" style={{ gap: 4 }}>
                <button onClick={() => load(p.id)} disabled={busy}>
                  読込
                </button>
                <button onClick={() => remove(p.id)} disabled={busy}>
                  ×
                </button>
              </span>
            </li>
          ))}
        </ul>
      )}
      <p className="hint">経路（軌跡）・エリア・waypoints/姿勢/車両/パラメータをサーバに保存し、呼び出して編集できます。読込後はそのまま表示され、waypoint編集→再生成やエリア編集が可能。レイヤ（点群/コスト）は別管理で常駐。</p>
    </section>
  );
}
