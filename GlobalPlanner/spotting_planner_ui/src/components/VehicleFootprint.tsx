import type { Path, Pose } from '../types';
import { truckFootprint } from '../geometry/footprint';

type Props = {
  pose?: Pose;
  path?: Path | null;
  length: number;
  width: number;
  inflate?: number;
  every?: number;
  stroke?: string;
  fillOpacity?: number;
  showInflated?: boolean;
};

function polyPoints(pose: Pose, length: number, width: number, inflate: number): string {
  return truckFootprint(pose, length, width, inflate)
    .map((p) => `${p.x},${p.y}`)
    .join(' ');
}

/**
 * Renders truck footprints either as a single static pose or sampled along a path.
 * If `showInflated` is true, the inflated (safety + localization + tracking) footprint
 * is overlaid faintly so the user can see why the path is shaped the way it is.
 */
export default function VehicleFootprint({
  pose,
  path,
  length,
  width,
  inflate = 0,
  every = 8,
  stroke = '#e6edf3',
  fillOpacity = 0.08,
  showInflated = false,
}: Props) {
  const poses: Pose[] = [];
  if (pose) poses.push(pose);
  if (path) {
    for (let i = 0; i < path.samples.length; i += every) {
      poses.push(path.samples[i]);
    }
    poses.push(path.samples[path.samples.length - 1]);
  }
  return (
    <g className="vehicle-footprint">
      {poses.map((p, i) => (
        <g key={i}>
          {showInflated && (
            <polygon
              points={polyPoints(p, length, width, inflate)}
              fill="none"
              stroke="#58a6ff"
              strokeOpacity={0.25}
              strokeWidth={0.15}
              strokeDasharray="0.5 0.5"
            />
          )}
          <polygon
            points={polyPoints(p, length, width, 0)}
            fill={stroke}
            fillOpacity={fillOpacity}
            stroke={stroke}
            strokeWidth={0.12}
          />
          {/* heading tick */}
          <line
            x1={p.x}
            y1={p.y}
            x2={p.x + (length * 0.6) * Math.cos(p.heading)}
            y2={p.y + (length * 0.6) * Math.sin(p.heading)}
            stroke={stroke}
            strokeWidth={0.18}
          />
        </g>
      ))}
    </g>
  );
}
