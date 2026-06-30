import { useEffect, useRef, useState } from "react";

import { api } from "@/api/client";
import type { AgentStep } from "@/api/client";
import { applyAgentActions } from "@/agentActions";
import { useStore } from "@/store/useStore";

interface Turn {
  role: "user" | "assistant";
  content: string;
  steps?: AgentStep[];
}

// ストアから現在状態のスナップショット（AI が読むコンテキスト）を組み立てる。
function buildContext(): Record<string, unknown> {
  const s = useStore.getState();
  return {
    vehicle_id: s.vehicleId,
    plan: {
      mode: s.planMode,
      algorithm: s.algorithm,
      spacing_m: s.routeSpacing,
      road_width_m: s.roadWidthM,
      enforce_footprint: s.enforceFootprint,
      enforce_min_radius: s.enforceMinRadius,
      allow_reverse: s.allowReverse,
      refine_elastic_band: s.refineElasticBand,
    },
    waypoints: s.waypoints.map((w) => ({
      x: w.xy.x,
      y: w.xy.y,
      role: w.role,
      heading_deg: w.heading_deg ?? null,
    })),
    areas: s.areas.map((a) => a.points.map((p) => [p.x, p.y])),
    spot: {
      start: s.spotStart,
      target: s.spotTarget,
      max_switchbacks: s.spotMaxSwitch,
      require_switchback: s.spotMaxSwitch === 1,
      method: s.spotMethod,
      smooth_path: s.spotSmooth,
      road_width_m: s.spotRoadWidthM,
      weights: s.spotWeights,
    },
  };
}

const SUGGESTIONS = [
  "今の状態を教えて",
  "最新のLASからコストマップを作って",
  "走行可能領域を生成して",
  "今のwaypointでHybrid A*の経路を作って",
];

export function ChatPanel() {
  const [turns, setTurns] = useState<Turn[]>([]);
  const [input, setInput] = useState("");
  const [busy, setBusy] = useState(false);
  const [available, setAvailable] = useState<boolean | null>(null);
  const [label, setLabel] = useState<string>("");
  const [detail, setDetail] = useState<string>("");
  const logRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    api
      .agentStatus()
      .then((r) => {
        setAvailable(r.available);
        setLabel([r.provider, r.model].filter(Boolean).join(" · "));
        setDetail(r.detail ?? "");
      })
      .catch(() => setAvailable(false));
  }, []);

  useEffect(() => {
    logRef.current?.scrollTo({ top: logRef.current.scrollHeight });
  }, [turns, busy]);

  async function send(text: string) {
    const msg = text.trim();
    if (!msg || busy) return;
    setInput("");
    const history = turns.map((t) => ({ role: t.role, content: t.content }));
    const nextTurns: Turn[] = [...turns, { role: "user", content: msg }];
    setTurns(nextTurns);
    setBusy(true);
    try {
      const res = await api.agentChat([...history, { role: "user", content: msg }], buildContext());
      await applyAgentActions(res.actions);
      setTurns([...nextTurns, { role: "assistant", content: res.reply, steps: res.steps }]);
    } catch (e) {
      setTurns([...nextTurns, { role: "assistant", content: `エラー: ${String(e)}` }]);
    } finally {
      setBusy(false);
    }
  }

  return (
    <section className="card">
      <h3>AI アシスタント{label ? ` · ${label}` : ""}</h3>

      {available === false && (
        <p className="chat-disabled">
          AI プロバイダが未設定です{detail ? `（${detail}）` : ""}。サーバを起動するシェルで以下のいずれかを設定し再起動してください:
          <br />・<code>GROQ_API_KEY</code>（無料）/ <code>OPENROUTER_API_KEY</code>（無料モデル有）
          <br />・<code>FRS_AI_BASE_URL=http://localhost:11434/v1</code>＋<code>FRS_AI_MODEL</code>（Ollama・鍵不要）
          <br />・<code>ANTHROPIC_API_KEY</code>（Claude）
        </p>
      )}

      <div className="chat-log" ref={logRef}>
        {turns.length === 0 && (
          <div className="chat-empty">経路計画を手伝います。やりたいことを日本語で入力してください。</div>
        )}
        {turns.map((t, i) => (
          <div key={i} style={{ display: "contents" }}>
            {t.steps && t.steps.length > 0 && (
              <div className="chat-steps">
                {t.steps.map((s, j) => (
                  <span key={j} className={s.ok ? "chat-step-ok" : "chat-step-err"}>
                    🔧 {s.tool} {s.ok ? "✓" : "✗"}
                  </span>
                ))}
              </div>
            )}
            <div className={`chat-msg ${t.role}`}>{t.content}</div>
          </div>
        ))}
        {busy && <div className="chat-msg assistant">…考え中</div>}
      </div>

      {turns.length === 0 && available !== false && (
        <div className="row wrap" style={{ marginBottom: 8 }}>
          {SUGGESTIONS.map((s) => (
            <button key={s} onClick={() => send(s)} disabled={busy} style={{ fontSize: 12 }}>
              {s}
            </button>
          ))}
        </div>
      )}

      <div className="chat-input-row">
        <textarea
          value={input}
          placeholder="例: 最新のLASからコストマップ→走行可能領域→経路まで作って"
          onChange={(e) => setInput(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === "Enter" && (e.metaKey || e.ctrlKey)) {
              e.preventDefault();
              void send(input);
            }
          }}
          disabled={busy || available === false}
        />
        <button className="primary" onClick={() => send(input)} disabled={busy || !input.trim() || available === false}>
          送信
        </button>
      </div>
      <p className="hint">⌘/Ctrl+Enter で送信。AI は地図・パネルを直接操作します（Undo可）。</p>
    </section>
  );
}
