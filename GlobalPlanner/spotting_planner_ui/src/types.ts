/**
 * Shared types for the spotting/exit planner prototype.
 *
 * Coordinate convention: world frame, X=east, Y=north, heading θ in radians, CCW from +X.
 * Distance unit: metres. Time unit: seconds.
 */

export type Vec2 = { x: number; y: number };

export type Pose = { x: number; y: number; heading: number };

export type Polygon = Vec2[]; // CCW vertex list, implicitly closed (last → first)

export type Obstacle = {
  id: string;
  polygon: Polygon;
};

export type Scenario = {
  /** Outer authorized planning boundary the truck must remain inside. */
  authorizedBoundary: Polygon;
  /** Physically traversable area (e.g. flat compacted ground). */
  traversableArea: Polygon;
  /** No-go zones inside the traversable area. */
  obstacles: Obstacle[];
  /** Pose of the dump target (truck must arrive here aligned for spotting). */
  dumpTargetPose: Pose;
  /** Pose where the truck waits in the queue. */
  queuePose: Pose;
  /** Pose at which the exit path terminates (joins exit lane). */
  exitLanePose: Pose;
};

export type PlannerParams = {
  // Vehicle
  truckLength: number;
  truckWidth: number;
  minTurningRadius: number;
  maxSteeringAngleDeg: number;

  // Safety margins
  safetyBuffer: number;
  localizationErrorMargin: number;
  pathFollowingErrorMargin: number;

  // Cost weights
  reversePenalty: number;
  cuspPenalty: number;
  boundaryClearanceWeight: number;
  steeringSmoothnessWeight: number;

  // Goal tolerance
  finalPoseToleranceM: number;
  finalHeadingToleranceDeg: number;
  maxReverseDistance: number;

  // Discretisation
  gridResolution: number;       // m per cell for closed-set discretisation
  headingResolution: number;    // rad per heading bin
  planningTimeoutMs: number;
  maxExpansions: number;

  // Cusp control
  enableAutoCusp: boolean;
  numCuspCandidates: number;
  manualCusp: Pose;             // used when enableAutoCusp = false
};

/** A pose sample on a generated path, with direction of travel. */
export type PathSample = Pose & {
  direction: 1 | -1;            // +1 forward, -1 reverse
  steer: number;                // signed steering angle (rad) of segment leading to this sample
  invalid?: boolean;            // true if this sample's footprint violates boundary / obstacle
};

export type Path = {
  samples: PathSample[];
  cuspIndices: number[];        // indices where direction flips
  totalLength: number;
  reverseLength: number;
};

export type PlanResultLegStatus =
  | 'ok'
  | 'timeout'
  | 'no_expansion'
  | 'cusp_invalid'
  | 'goal_unreachable'
  | 'start_invalid';

export type PlanResult = {
  feasible: boolean;
  spottingPath: Path | null;
  exitPath: Path | null;
  /** Effective cusp pose used (auto-selected or manual). null = couldn't even pick one. */
  cuspPose: Pose | null;
  /** Cusp candidates considered (auto mode only), for visualisation/debug. */
  cuspCandidates: Pose[];

  // Metrics
  planningResponseMs: number;
  minBoundaryClearance: number;
  minObstacleClearance: number;
  totalPathLength: number;
  totalReverseDistance: number;
  numCusps: number;
  estimatedPathFollowingErrorM: number;

  // Diagnostics
  spottingLegStatus: PlanResultLegStatus;
  exitLegStatus: PlanResultLegStatus;
  failureReason: string | null;
};
