import { useStore } from '../state/store';

function fmt(v: number, digits = 2, unit = ''): string {
  if (!isFinite(v)) return '—';
  return `${v.toFixed(digits)}${unit ? ' ' + unit : ''}`;
}

export default function PlannerStatusPanel() {
  const planResult = useStore((s) => s.planResult);

  if (!planResult) {
    return (
      <div className="panel">
        <h2>Planner Status</h2>
        <p style={{ color: 'var(--muted)', fontSize: 12 }}>
          Click <b>Run Planner</b> to generate spotting and exit paths for the current scenario.
        </p>
        <h3>Legend</h3>
        <ul style={{ fontSize: 12, color: 'var(--muted)', paddingLeft: 16 }}>
          <li>
            <span style={{ color: '#3fb950' }}>green</span> — spotting forward
          </li>
          <li>
            <span style={{ color: '#d29922' }}>amber</span> — reverse segments
          </li>
          <li>
            <span style={{ color: '#58a6ff' }}>blue</span> — exit path
          </li>
          <li>
            <span style={{ color: '#f85149' }}>red</span> — invalid path segment
          </li>
          <li>white circle — chosen cusp pose</li>
          <li>dashed blue — inflated footprint (safety + localization + tracking margins)</li>
        </ul>
        <h3>Controls</h3>
        <p style={{ fontSize: 12, color: 'var(--muted)' }}>
          Ctrl/⌘ + scroll = zoom. Click + drag = pan.
        </p>
      </div>
    );
  }

  return (
    <div className="panel">
      <h2>Planner Status</h2>
      <div className="status-row">
        <span className="label">Feasibility</span>
        <span className="value">
          {planResult.feasible ? (
            <span className="badge ok">FEASIBLE</span>
          ) : (
            <span className="badge fail">NO FEASIBLE PATH</span>
          )}
        </span>
      </div>
      <div className="status-row">
        <span className="label">Planning time</span>
        <span className="value">{fmt(planResult.planningResponseMs, 1, 'ms')}</span>
      </div>
      <div className="status-row">
        <span className="label">Spotting leg</span>
        <span className="value">{planResult.spottingLegStatus}</span>
      </div>
      <div className="status-row">
        <span className="label">Exit leg</span>
        <span className="value">{planResult.exitLegStatus}</span>
      </div>

      <h3>Metrics</h3>
      <div className="status-row">
        <span className="label">Total length</span>
        <span className="value">{fmt(planResult.totalPathLength, 1, 'm')}</span>
      </div>
      <div className="status-row">
        <span className="label">Reverse length</span>
        <span className="value">{fmt(planResult.totalReverseDistance, 1, 'm')}</span>
      </div>
      <div className="status-row">
        <span className="label">Number of cusps</span>
        <span className="value">{planResult.numCusps}</span>
      </div>
      <div className="status-row">
        <span className="label">Min boundary clr.</span>
        <span className="value">{fmt(planResult.minBoundaryClearance, 2, 'm')}</span>
      </div>
      <div className="status-row">
        <span className="label">Min obstacle clr.</span>
        <span className="value">{fmt(planResult.minObstacleClearance, 2, 'm')}</span>
      </div>
      <div className="status-row">
        <span className="label">Tracking margin</span>
        <span className="value">{fmt(planResult.estimatedPathFollowingErrorM, 2, 'm')}</span>
      </div>

      {planResult.cuspPose && (
        <>
          <h3>Cusp</h3>
          <div className="status-row">
            <span className="label">X / Y</span>
            <span className="value">
              {planResult.cuspPose.x.toFixed(1)} / {planResult.cuspPose.y.toFixed(1)}
            </span>
          </div>
          <div className="status-row">
            <span className="label">Heading</span>
            <span className="value">
              {((planResult.cuspPose.heading * 180) / Math.PI).toFixed(1)}°
            </span>
          </div>
          <div className="status-row">
            <span className="label">Candidates</span>
            <span className="value">{planResult.cuspCandidates.length}</span>
          </div>
        </>
      )}

      {planResult.failureReason && (
        <div className="failure-reason">
          <b>Failure reason</b>
          <div style={{ marginTop: 4 }}>{planResult.failureReason}</div>
        </div>
      )}
    </div>
  );
}
