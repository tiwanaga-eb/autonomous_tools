import type { Obstacle } from '../types';

type Props = { obstacles: Obstacle[] };

export default function ObstacleLayer({ obstacles }: Props) {
  return (
    <g className="obstacle-layer">
      {obstacles.map((o) => (
        <polygon
          key={o.id}
          points={o.polygon.map((p) => `${p.x},${p.y}`).join(' ')}
          fill="#f85149"
          fillOpacity={0.3}
          stroke="#f85149"
          strokeWidth={0.2}
        />
      ))}
    </g>
  );
}
