import { useStore } from '../state/store';
import type { PlannerParams } from '../types';

type NumKey = {
  [K in keyof PlannerParams]: PlannerParams[K] extends number ? K : never;
}[keyof PlannerParams];

function NumberRow({
  k,
  label,
  step = 0.1,
  min,
  max,
}: {
  k: NumKey;
  label: string;
  step?: number;
  min?: number;
  max?: number;
}) {
  const value = useStore((s) => s.params[k]);
  const setParam = useStore((s) => s.setParam);
  return (
    <div className="row">
      <label htmlFor={`p-${String(k)}`}>{label}</label>
      <input
        id={`p-${String(k)}`}
        type="number"
        step={step}
        min={min}
        max={max}
        value={Number.isFinite(value) ? value : ''}
        onChange={(e) => setParam(k, parseFloat(e.target.value))}
      />
    </div>
  );
}

export default function ParameterPanel() {
  const params = useStore((s) => s.params);
  const setParam = useStore((s) => s.setParam);
  const setManualCuspXY = useStore((s) => s.setManualCuspXY);
  const setManualCuspHeading = useStore((s) => s.setManualCuspHeading);
  const planning = useStore((s) => s.planning);
  const runPlanner = useStore((s) => s.runPlanner);
  const resetParams = useStore((s) => s.resetParams);

  return (
    <div className="panel">
      <h2>Planner Parameters</h2>

      <h3>Vehicle</h3>
      <NumberRow k="truckLength" label="Truck length (m)" step={0.1} min={3} />
      <NumberRow k="truckWidth" label="Truck width (m)" step={0.1} min={1.5} />
      <NumberRow k="minTurningRadius" label="Min turning radius (m)" step={0.5} min={2} />
      <NumberRow k="maxSteeringAngleDeg" label="Max steering (deg)" step={1} min={5} max={60} />

      <h3>Safety</h3>
      <NumberRow k="safetyBuffer" label="Safety buffer (m)" step={0.1} min={0} />
      <NumberRow k="localizationErrorMargin" label="Localization err (m)" step={0.05} min={0} />
      <NumberRow k="pathFollowingErrorMargin" label="Path-following err (m)" step={0.05} min={0} />

      <h3>Cost weights</h3>
      <NumberRow k="reversePenalty" label="Reverse penalty" step={0.1} min={0} />
      <NumberRow k="cuspPenalty" label="Cusp penalty" step={0.5} min={0} />
      <NumberRow k="boundaryClearanceWeight" label="Boundary clearance" step={0.1} min={0} />
      <NumberRow k="steeringSmoothnessWeight" label="Steering smoothness" step={0.05} min={0} />

      <h3>Goal tolerance</h3>
      <NumberRow k="finalPoseToleranceM" label="Final pose tol (m)" step={0.1} min={0.1} />
      <NumberRow k="finalHeadingToleranceDeg" label="Final heading tol (deg)" step={1} min={1} />
      <NumberRow k="maxReverseDistance" label="Max reverse dist (m)" step={1} min={1} />

      <h3>Search</h3>
      <NumberRow k="gridResolution" label="Grid resolution (m)" step={0.1} min={0.2} />
      <NumberRow k="headingResolution" label="Heading res (rad)" step={0.01} min={0.05} />
      <NumberRow k="planningTimeoutMs" label="Timeout (ms)" step={100} min={100} />
      <NumberRow k="maxExpansions" label="Max expansions" step={1000} min={1000} />

      <h3>Cusp</h3>
      <div className="row">
        <label htmlFor="auto-cusp">Auto cusp</label>
        <input
          id="auto-cusp"
          type="checkbox"
          checked={params.enableAutoCusp}
          onChange={(e) => setParam('enableAutoCusp', e.target.checked)}
        />
      </div>
      <NumberRow k="numCuspCandidates" label="Cusp candidates" step={1} min={1} max={32} />

      {!params.enableAutoCusp && (
        <>
          <div className="row">
            <label>Manual cusp X</label>
            <input
              type="number"
              step={0.5}
              value={params.manualCusp.x}
              onChange={(e) => setManualCuspXY(parseFloat(e.target.value), params.manualCusp.y)}
            />
          </div>
          <div className="row">
            <label>Manual cusp Y</label>
            <input
              type="number"
              step={0.5}
              value={params.manualCusp.y}
              onChange={(e) => setManualCuspXY(params.manualCusp.x, parseFloat(e.target.value))}
            />
          </div>
          <div className="row">
            <label>Manual cusp heading (deg)</label>
            <input
              type="number"
              step={1}
              value={(params.manualCusp.heading * 180) / Math.PI}
              onChange={(e) =>
                setManualCuspHeading((parseFloat(e.target.value) * Math.PI) / 180)
              }
            />
          </div>
        </>
      )}

      <button className="primary" onClick={runPlanner} disabled={planning}>
        {planning ? 'Planning…' : 'Run Planner'}
      </button>
      <button
        className="secondary"
        onClick={resetParams}
        style={{ marginTop: 6, width: '100%' }}
      >
        Reset to defaults
      </button>
    </div>
  );
}
