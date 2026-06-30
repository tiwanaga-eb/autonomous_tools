import type { Scenario, PlannerParams } from '../types';

/**
 * Default scenario approximates a dump bench at a quarry:
 *  - authorized boundary: large pad
 *  - traversable area: slightly inside, leaving a buffer
 *  - obstacles: a stockpile column and a wall stub
 *  - queue pose: south side facing north
 *  - dump target: north edge facing south (truck reverses in)
 *  - exit lane: east side facing east
 *
 * All units in metres / radians.
 */
const DEG = Math.PI / 180;

export const defaultScenario: Scenario = {
  authorizedBoundary: [
    { x: 0, y: 0 },
    { x: 80, y: 0 },
    { x: 80, y: 60 },
    { x: 0, y: 60 },
  ],
  traversableArea: [
    { x: 4, y: 4 },
    { x: 76, y: 4 },
    { x: 76, y: 56 },
    { x: 4, y: 56 },
  ],
  obstacles: [
    {
      id: 'stockpile-1',
      polygon: [
        { x: 30, y: 35 },
        { x: 42, y: 35 },
        { x: 42, y: 45 },
        { x: 30, y: 45 },
      ],
    },
    {
      id: 'wall-stub',
      polygon: [
        { x: 55, y: 18 },
        { x: 60, y: 18 },
        { x: 60, y: 28 },
        { x: 55, y: 28 },
      ],
    },
  ],
  // Truck must back up to the dump target with heading pointing south (-y), i.e. heading = -π/2.
  dumpTargetPose: { x: 20, y: 50, heading: -90 * DEG },
  queuePose: { x: 12, y: 10, heading: 90 * DEG },
  exitLanePose: { x: 60, y: 12, heading: 0 * DEG },
};

export const defaultParams: PlannerParams = {
  truckLength: 11.5,
  truckWidth: 5.0,
  minTurningRadius: 10.0,
  maxSteeringAngleDeg: 35,

  safetyBuffer: 0.5,
  localizationErrorMargin: 0.3,
  pathFollowingErrorMargin: 0.4,

  reversePenalty: 1.5,
  cuspPenalty: 8.0,
  boundaryClearanceWeight: 2.0,
  steeringSmoothnessWeight: 0.4,

  finalPoseToleranceM: 0.8,
  finalHeadingToleranceDeg: 8,
  maxReverseDistance: 40,

  gridResolution: 1.0,
  // Must be at least as fine as finalHeadingToleranceDeg so the closed-set heading bins
  // can resolve goal alignment. Each ±max-steer arc at grid_resolution = 1 m steps the
  // heading by ~5.7° (= 1/R_min), which matches a 5° bin well.
  headingResolution: 5 * DEG,
  planningTimeoutMs: 2000,
  maxExpansions: 40_000,

  enableAutoCusp: true,
  numCuspCandidates: 9,
  manualCusp: { x: 40, y: 30, heading: 90 * DEG },
};
