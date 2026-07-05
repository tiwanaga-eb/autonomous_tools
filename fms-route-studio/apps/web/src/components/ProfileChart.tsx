import { useMemo, useRef } from "react";

// 依存なしの軽量SVG折れ線チャート。弧長 s[m] を横軸に、1系列の縦軸プロファイルを描く。
// 設計書 §10/§21: κ(s) / dκ/ds(s) / steer(s) / grade(s) の可視化。limit線・violation帯・hoverに対応。

export interface ProfileBand {
  s0: number;
  s1: number;
}

interface Props {
  title: string;
  unit: string;
  color: string;
  xs: number[]; // 弧長 s [m]（昇順）
  ys: number[]; // 値（xs と同長）
  limit?: number | null; // 水平の制限線（任意）
  limitLabel?: string;
  bands?: ProfileBand[]; // violation 区間ハイライト（任意）
  fmt?: (v: number) => string; // 値の表示フォーマット
  hoverExtra?: (v: number) => string | null; // ホバー値の併記（例: κ→半径R[m], m/s→km/h）。軸には出さない
  height?: number;
  hoverIndex?: number | null; // 外部から制御されるホバー点index（全チャート/地図で共有）
  onHover?: (i: number | null) => void; // ホバー点index通知（地図ハイライト同期用）
}

const VBW = 320; // viewBox 幅（width:100% で実サイズへスケール）
const PAD = { l: 40, r: 8, t: 10, b: 20 };

function niceFmt(v: number): string {
  const a = Math.abs(v);
  if (a !== 0 && (a < 1e-3 || a >= 1e4)) return v.toExponential(1);
  if (a >= 100) return v.toFixed(0);
  if (a >= 1) return v.toFixed(2);
  return v.toFixed(4);
}

export function ProfileChart({
  title,
  unit,
  color,
  xs,
  ys,
  limit = null,
  limitLabel,
  bands = [],
  fmt = niceFmt,
  hoverExtra,
  height = 96,
  hoverIndex = null,
  onHover,
}: Props) {
  const svgRef = useRef<SVGSVGElement>(null);
  const n0 = Math.min(xs.length, ys.length);
  const hoverI = hoverIndex != null && hoverIndex >= 0 && hoverIndex < n0 ? hoverIndex : null;
  const setHoverI = (i: number | null) => onHover?.(i);

  const model = useMemo(() => {
    const n = Math.min(xs.length, ys.length);
    if (n < 2) return null;
    const sMin = xs[0];
    const sMax = xs[n - 1] || sMin + 1;
    let yMin = Infinity;
    let yMax = -Infinity;
    for (let i = 0; i < n; i++) {
      if (!Number.isFinite(ys[i])) continue;
      if (ys[i] < yMin) yMin = ys[i];
      if (ys[i] > yMax) yMax = ys[i];
    }
    if (limit != null && Number.isFinite(limit)) {
      yMin = Math.min(yMin, limit);
      yMax = Math.max(yMax, limit);
    }
    if (!Number.isFinite(yMin) || !Number.isFinite(yMax)) return null;
    if (yMin === yMax) {
      yMin -= 1;
      yMax += 1;
    } else {
      const pad = (yMax - yMin) * 0.08;
      yMin -= pad;
      yMax += pad;
    }
    const plotW = VBW - PAD.l - PAD.r;
    const plotH = height - PAD.t - PAD.b;
    const sx = (s: number) => PAD.l + ((s - sMin) / (sMax - sMin || 1)) * plotW;
    const sy = (y: number) => PAD.t + (1 - (y - yMin) / (yMax - yMin || 1)) * plotH;
    let d = "";
    for (let i = 0; i < n; i++) {
      const yv = Number.isFinite(ys[i]) ? ys[i] : yMin;
      d += `${i === 0 ? "M" : "L"}${sx(xs[i]).toFixed(1)},${sy(yv).toFixed(1)} `;
    }
    return { n, sMin, sMax, yMin, yMax, plotH, sx, sy, d };
  }, [xs, ys, limit, height]);

  if (!model) {
    return (
      <div className="profile-chart">
        <div className="profile-head">
          <span>{title}</span>
        </div>
        <p className="hint" style={{ margin: "2px 0" }}>
          データ不足
        </p>
      </div>
    );
  }

  const { sMin, sMax, yMin, yMax, sx, sy, d } = model;
  const baselineY = yMin <= 0 && yMax >= 0 ? sy(0) : null;

  const onMove = (e: React.MouseEvent<SVGSVGElement>) => {
    const rect = svgRef.current?.getBoundingClientRect();
    if (!rect) return;
    const vbX = ((e.clientX - rect.left) / rect.width) * VBW;
    // vbX → 最近傍の s インデックス
    const frac = (vbX - PAD.l) / (VBW - PAD.l - PAD.r);
    const sTarget = sMin + frac * (sMax - sMin);
    let best = 0;
    let bestD = Infinity;
    for (let i = 0; i < xs.length; i++) {
      const dd = Math.abs(xs[i] - sTarget);
      if (dd < bestD) {
        bestD = dd;
        best = i;
      }
    }
    setHoverI(best);
  };

  const hx = hoverI != null ? sx(xs[hoverI]) : null;
  const hy = hoverI != null && Number.isFinite(ys[hoverI]) ? sy(ys[hoverI]) : null;

  return (
    <div className="profile-chart">
      <div className="profile-head">
        <span style={{ color }}>{title}</span>
        <span className="profile-sub">
          {hoverI != null
            ? `s=${xs[hoverI].toFixed(1)}m  ${fmt(ys[hoverI])} ${unit}` +
              (() => {
                const ex = Number.isFinite(ys[hoverI]) ? hoverExtra?.(ys[hoverI]) : null;
                return ex ? `（${ex}）` : "";
              })()
            : `max ${fmt(yMax)} / min ${fmt(yMin)} ${unit}`}
        </span>
      </div>
      <svg
        ref={svgRef}
        viewBox={`0 0 ${VBW} ${height}`}
        width="100%"
        height={height}
        preserveAspectRatio="none"
        onMouseMove={onMove}
        onMouseLeave={() => setHoverI(null)}
        style={{ display: "block", cursor: "crosshair" }}
      >
        {/* violation 帯 */}
        {bands.map((b, i) => {
          const x0 = sx(Math.max(b.s0, sMin));
          const x1 = sx(Math.min(b.s1, sMax));
          return (
            <rect
              key={i}
              x={x0}
              y={PAD.t}
              width={Math.max(1, x1 - x0)}
              height={height - PAD.t - PAD.b}
              fill="#ef444433"
            />
          );
        })}
        {/* 枠 */}
        <rect
          x={PAD.l}
          y={PAD.t}
          width={VBW - PAD.l - PAD.r}
          height={height - PAD.t - PAD.b}
          fill="none"
          stroke="#33415544"
        />
        {/* y=0 基準線 */}
        {baselineY != null && (
          <line x1={PAD.l} y1={baselineY} x2={VBW - PAD.r} y2={baselineY} stroke="#94a3b855" strokeDasharray="2 3" />
        )}
        {/* limit 線 */}
        {limit != null && Number.isFinite(limit) && (
          <line
            x1={PAD.l}
            y1={sy(limit)}
            x2={VBW - PAD.r}
            y2={sy(limit)}
            stroke="#ef4444"
            strokeWidth={1}
            strokeDasharray="4 3"
          />
        )}
        {/* 折れ線 */}
        <path d={d} fill="none" stroke={color} strokeWidth={1.4} />
        {/* hover カーソル */}
        {hx != null && (
          <line x1={hx} y1={PAD.t} x2={hx} y2={height - PAD.b} stroke="#cbd5e1aa" strokeWidth={1} />
        )}
        {hx != null && hy != null && <circle cx={hx} cy={hy} r={2.8} fill={color} stroke="#0f172a" strokeWidth={0.8} />}
        {/* y 軸ラベル（最大/最小） */}
        <text x={PAD.l - 3} y={PAD.t + 7} textAnchor="end" fontSize="8" fill="#94a3b8">
          {fmt(yMax)}
        </text>
        <text x={PAD.l - 3} y={height - PAD.b} textAnchor="end" fontSize="8" fill="#94a3b8">
          {fmt(yMin)}
        </text>
        {/* x 軸ラベル */}
        <text x={PAD.l} y={height - 4} fontSize="8" fill="#94a3b8">
          0
        </text>
        <text x={VBW - PAD.r} y={height - 4} textAnchor="end" fontSize="8" fill="#94a3b8">
          {sMax.toFixed(0)}m
        </text>
        {limit != null && Number.isFinite(limit) && limitLabel && (
          <text x={VBW - PAD.r - 2} y={sy(limit) - 2} textAnchor="end" fontSize="8" fill="#ef4444">
            {limitLabel}
          </text>
        )}
      </svg>
    </div>
  );
}
