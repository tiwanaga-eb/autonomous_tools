import type { Path } from '../types';

type Props = {
  path: Path | null;
  color: string;
  label?: string;
};

/**
 * Renders a planned path as a polyline whose segment colour depends on the
 * direction (forward / reverse) and validity (invalid → red regardless).
 */
export default function PathLayer({ path, color }: Props) {
  if (!path) return null;
  const segments: Array<{ x1: number; y1: number; x2: number; y2: number; stroke: string; width: number }> = [];
  for (let i = 1; i < path.samples.length; i++) {
    const a = path.samples[i - 1];
    const b = path.samples[i];
    const invalid = a.invalid || b.invalid;
    const stroke = invalid ? '#f85149' : b.direction < 0 ? '#d29922' : color;
    segments.push({ x1: a.x, y1: a.y, x2: b.x, y2: b.y, stroke, width: invalid ? 0.55 : 0.35 });
  }
  return (
    <g className="path-layer">
      {segments.map((s, i) => (
        <line
          key={i}
          x1={s.x1}
          y1={s.y1}
          x2={s.x2}
          y2={s.y2}
          stroke={s.stroke}
          strokeWidth={s.width}
          strokeLinecap="round"
        />
      ))}
      {path.cuspIndices.map((i) => {
        const s = path.samples[i];
        return (
          <circle
            key={`cusp-${i}`}
            cx={s.x}
            cy={s.y}
            r={0.7}
            fill="#ffffff"
            stroke="#d29922"
            strokeWidth={0.2}
          />
        );
      })}
    </g>
  );
}
