import { useMemo, useRef, useState, type WheelEvent, type MouseEvent } from 'react';
import { useStore } from '../state/store';
import BoundaryLayer from './BoundaryLayer';
import ObstacleLayer from './ObstacleLayer';
import PathLayer from './PathLayer';
import VehicleFootprint from './VehicleFootprint';

/**
 * SVG-based map canvas. Y-axis is flipped (north up) by using a parent
 * `transform=scale(1,-1)` so children can use world XY directly.
 *
 * Zoom: ctrl + wheel. Pan: drag the background.
 */
export default function MapCanvas() {
  const { scenario, params, planResult } = useStore();
  const svgRef = useRef<SVGSVGElement>(null);
  const [view, setView] = useState({ x: -5, y: -5, w: 90, h: 70 });
  const [dragging, setDragging] = useState<{ x: number; y: number } | null>(null);

  const inflate =
    params.safetyBuffer + params.localizationErrorMargin + params.pathFollowingErrorMargin;

  const cuspMarker = useMemo(() => planResult?.cuspPose ?? null, [planResult]);

  const onWheel = (e: WheelEvent<SVGSVGElement>) => {
    if (!e.ctrlKey && !e.metaKey) return;
    e.preventDefault();
    const scale = e.deltaY > 0 ? 1.1 : 1 / 1.1;
    setView((v) => ({
      x: v.x + (v.w * (1 - scale)) / 2,
      y: v.y + (v.h * (1 - scale)) / 2,
      w: v.w * scale,
      h: v.h * scale,
    }));
  };

  const onMouseDown = (e: MouseEvent) => {
    setDragging({ x: e.clientX, y: e.clientY });
  };
  const onMouseMove = (e: MouseEvent) => {
    if (!dragging || !svgRef.current) return;
    const r = svgRef.current.getBoundingClientRect();
    const dx = ((e.clientX - dragging.x) / r.width) * view.w;
    const dy = ((e.clientY - dragging.y) / r.height) * view.h;
    setView((v) => ({ ...v, x: v.x - dx, y: v.y + dy }));
    setDragging({ x: e.clientX, y: e.clientY });
  };
  const onMouseUp = () => setDragging(null);

  const viewBox = `${view.x} ${view.y} ${view.w} ${view.h}`;

  return (
    <div className="canvas-wrap">
      <svg
        ref={svgRef}
        viewBox={viewBox}
        preserveAspectRatio="xMidYMid meet"
        onWheel={onWheel}
        onMouseDown={onMouseDown}
        onMouseMove={onMouseMove}
        onMouseUp={onMouseUp}
        onMouseLeave={onMouseUp}
      >
        <g transform={`translate(0 ${view.h + 2 * view.y}) scale(1 -1)`}>
          {/* grid */}
          <g stroke="#1f2937" strokeWidth={0.05}>
            {Array.from({ length: 20 }, (_, i) => i * 5).map((v) => (
              <g key={`g-${v}`}>
                <line x1={v} y1={view.y - 10} x2={v} y2={view.y + view.h + 10} />
                <line x1={view.x - 10} y1={v} x2={view.x + view.w + 10} y2={v} />
              </g>
            ))}
          </g>

          <BoundaryLayer
            authorized={scenario.authorizedBoundary}
            traversable={scenario.traversableArea}
          />
          <ObstacleLayer obstacles={scenario.obstacles} />

          {/* paths */}
          {planResult?.spottingPath && (
            <>
              <PathLayer path={planResult.spottingPath} color="#3fb950" />
              <VehicleFootprint
                path={planResult.spottingPath}
                length={params.truckLength}
                width={params.truckWidth}
                inflate={inflate}
                every={6}
                stroke="#3fb950"
                showInflated={false}
              />
            </>
          )}
          {planResult?.exitPath && (
            <>
              <PathLayer path={planResult.exitPath} color="#58a6ff" />
              <VehicleFootprint
                path={planResult.exitPath}
                length={params.truckLength}
                width={params.truckWidth}
                inflate={inflate}
                every={6}
                stroke="#58a6ff"
                showInflated={false}
              />
            </>
          )}

          {/* fixed pose markers */}
          <VehicleFootprint
            pose={scenario.queuePose}
            length={params.truckLength}
            width={params.truckWidth}
            inflate={inflate}
            stroke="#8b949e"
            fillOpacity={0.15}
            showInflated
          />
          <VehicleFootprint
            pose={scenario.dumpTargetPose}
            length={params.truckLength}
            width={params.truckWidth}
            inflate={inflate}
            stroke="#d29922"
            fillOpacity={0.18}
            showInflated
          />
          <VehicleFootprint
            pose={scenario.exitLanePose}
            length={params.truckLength}
            width={params.truckWidth}
            inflate={inflate}
            stroke="#58a6ff"
            fillOpacity={0.15}
            showInflated
          />

          {/* cusp candidates */}
          {planResult?.cuspCandidates.map((c, i) => (
            <circle
              key={`cand-${i}`}
              cx={c.x}
              cy={c.y}
              r={0.4}
              fill="none"
              stroke="#8b949e"
              strokeWidth={0.1}
              strokeDasharray="0.3 0.2"
            />
          ))}
          {cuspMarker && (
            <circle
              cx={cuspMarker.x}
              cy={cuspMarker.y}
              r={1.0}
              fill="none"
              stroke="#ffffff"
              strokeWidth={0.25}
            />
          )}

          {/* pose labels (text needs un-flipping) */}
          <g transform={`translate(0 ${view.h + 2 * view.y}) scale(1 -1)`} fontSize={1.6} fill="#8b949e" fontFamily="Menlo, monospace">
            <text x={scenario.queuePose.x + 1.5} y={view.h + 2 * view.y - scenario.queuePose.y}>
              QUEUE
            </text>
            <text
              x={scenario.dumpTargetPose.x + 1.5}
              y={view.h + 2 * view.y - scenario.dumpTargetPose.y}
            >
              DUMP
            </text>
            <text
              x={scenario.exitLanePose.x + 1.5}
              y={view.h + 2 * view.y - scenario.exitLanePose.y}
            >
              EXIT
            </text>
          </g>
        </g>
      </svg>
    </div>
  );
}
