import { describe, it, expect } from 'vitest';
import { plan } from '../src/planner/plannerOrchestrator';
import { hybridAstar } from '../src/planner/hybridAstar';
import { defaultScenario, defaultParams } from '../src/scenario/defaultScenario';
import type { Scenario } from '../src/types';

describe('hybridAstar straight-line case', () => {
  it('plans a forward straight path in an empty corridor', () => {
    const scenario: Scenario = {
      authorizedBoundary: [
        { x: -2, y: -10 },
        { x: 60, y: -10 },
        { x: 60, y: 10 },
        { x: -2, y: 10 },
      ],
      traversableArea: [
        { x: 0, y: -8 },
        { x: 58, y: -8 },
        { x: 58, y: 8 },
        { x: 0, y: 8 },
      ],
      obstacles: [],
      queuePose: { x: 5, y: 0, heading: 0 },
      dumpTargetPose: { x: 50, y: 0, heading: 0 },
      exitLanePose: { x: 0, y: 0, heading: 0 },
    };
    const params = {
      ...defaultParams,
      truckLength: 6,
      truckWidth: 2.5,
      minTurningRadius: 5,
      gridResolution: 1.0,
      headingResolution: (10 * Math.PI) / 180,
      planningTimeoutMs: 3000,
      maxExpansions: 60_000,
      finalPoseToleranceM: 1.0,
      finalHeadingToleranceDeg: 10,
    };
    const r = hybridAstar({
      start: scenario.queuePose,
      goal: scenario.dumpTargetPose,
      scenario,
      params,
    });
    expect(r.ok).toBe(true);
    if (r.ok) {
      expect(r.path.totalLength).toBeGreaterThan(40);
      expect(r.path.totalLength).toBeLessThan(80);
    }
  });
});

describe('plan() with default scenario', () => {
  it('produces a feasible result in auto cusp mode within the timeout', () => {
    const result = plan(defaultScenario, {
      ...defaultParams,
      planningTimeoutMs: 4000,
      maxExpansions: 80_000,
    });
    // The default scenario is hand-tuned to be solvable. We do not assert exact metrics,
    // but feasibility AND that a cusp was selected AND that the spotting path has cusps.
    if (!result.feasible) {
      // helpful diagnostic when this test fails
      console.error('Plan failed:', result.failureReason, result);
    }
    expect(result.feasible).toBe(true);
    expect(result.cuspPose).not.toBeNull();
    expect(result.spottingPath).not.toBeNull();
    expect(result.exitPath).not.toBeNull();
    expect(result.numCusps).toBeGreaterThanOrEqual(1);
  });
});

describe('plan() with infeasible scenario', () => {
  it('reports infeasibility cleanly when the dump target is walled off', () => {
    const blocked: Scenario = {
      ...defaultScenario,
      obstacles: [
        ...defaultScenario.obstacles,
        // Wall fully across the pad in front of the dump target
        {
          id: 'big-wall',
          polygon: [
            { x: 4, y: 40 },
            { x: 76, y: 40 },
            { x: 76, y: 44 },
            { x: 4, y: 44 },
          ],
        },
      ],
    };
    const result = plan(blocked, {
      ...defaultParams,
      planningTimeoutMs: 1500,
      maxExpansions: 20_000,
    });
    expect(result.feasible).toBe(false);
    expect(result.failureReason).toBeTruthy();
  });
});
