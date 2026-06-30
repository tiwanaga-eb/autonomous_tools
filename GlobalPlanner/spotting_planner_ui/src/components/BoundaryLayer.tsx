import type { Polygon } from '../types';

type Props = {
  authorized: Polygon;
  traversable: Polygon;
};

function toPoints(poly: Polygon): string {
  return poly.map((p) => `${p.x},${p.y}`).join(' ');
}

export default function BoundaryLayer({ authorized, traversable }: Props) {
  return (
    <g className="boundary-layer">
      <polygon
        points={toPoints(authorized)}
        fill="none"
        stroke="#58a6ff"
        strokeWidth={0.25}
        strokeDasharray="1.0 0.6"
      />
      <polygon
        points={toPoints(traversable)}
        fill="#1f2937"
        fillOpacity={0.45}
        stroke="#3fb950"
        strokeWidth={0.2}
      />
    </g>
  );
}
