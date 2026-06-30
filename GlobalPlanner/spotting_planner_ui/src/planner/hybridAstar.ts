import type { Pose, PathSample, Path, PlannerParams, Scenario } from '../types';
import { bicycleStep, effectiveWheelBase } from '../vehicle/bicycle';
import { poseIsFree, poseClearance, type CollisionContext } from './collision';
import { heuristic } from './heuristic';
import { buildPrimitives } from './motionPrimitives';
import { edgeCost } from './costs';
import { MinHeap } from './heap';
import { angleDiff } from '../geometry/primitives';

export type HybridAstarRequest = {
  start: Pose;
  goal: Pose;
  scenario: Scenario;
  params: PlannerParams;
};

export type HybridAstarResult =
  | { ok: true; path: Path; expansions: number; elapsedMs: number }
  | {
      ok: false;
      reason: 'timeout' | 'no_expansion' | 'start_invalid' | 'goal_unreachable';
      expansions: number;
      elapsedMs: number;
    };

type Node = {
  pose: Pose;
  steer: number;
  direction: 1 | -1;
  g: number;
  priority: number;
  parent: Node | null;
};

function discKey(pose: Pose, gridRes: number, headingRes: number): string {
  const ix = Math.round(pose.x / gridRes);
  const iy = Math.round(pose.y / gridRes);
  // Wrap heading to [0, 2π) before binning so that θ ≈ 0 and θ ≈ 2π collide.
  let th = pose.heading;
  while (th < 0) th += 2 * Math.PI;
  while (th >= 2 * Math.PI) th -= 2 * Math.PI;
  const ih = Math.round(th / headingRes);
  return `${ix}|${iy}|${ih}`;
}

function goalReached(pose: Pose, goal: Pose, posTolM: number, headingTolRad: number): boolean {
  const d = Math.hypot(pose.x - goal.x, pose.y - goal.y);
  if (d > posTolM) return false;
  return Math.abs(angleDiff(pose.heading, goal.heading)) <= headingTolRad;
}

/**
 * Reconstruct a Path from a goal node by walking parent pointers.
 *
 * Each parent→child segment is sub-sampled at the planner's grid resolution so the
 * caller can render footprints and run final boundary re-validation on densified
 * samples (catches any pathological discretisation gap).
 */
function reconstruct(goalNode: Node, wheelBase: number, gridRes: number): Path {
  const chain: Node[] = [];
  let cur: Node | null = goalNode;
  while (cur !== null) {
    chain.push(cur);
    cur = cur.parent;
  }
  chain.reverse();

  const samples: PathSample[] = [];
  const cuspIndices: number[] = [];
  let totalLength = 0;
  let reverseLength = 0;

  // First sample = start pose
  samples.push({ ...chain[0].pose, direction: chain[0].direction, steer: chain[0].steer });

  for (let i = 1; i < chain.length; i++) {
    const parent = chain[i - 1];
    const node = chain[i];
    const direction = node.direction;
    const steer = node.steer;
    const ds = gridRes;
    // Densify the segment for rendering / re-validation
    let p = parent.pose;
    for (let s = 0; s < ds - 1e-6; s += ds) {
      p = bicycleStep(p, steer, direction, ds, wheelBase);
      samples.push({ ...p, direction, steer });
      totalLength += ds;
      if (direction < 0) reverseLength += ds;
    }
    // Record cusp if direction flipped from previous segment
    if (i > 1 && chain[i - 1].direction !== direction) {
      cuspIndices.push(samples.length - 1);
    }
  }

  return { samples, cuspIndices, totalLength, reverseLength };
}

export function hybridAstar(req: HybridAstarRequest): HybridAstarResult {
  const t0 = performance.now();
  const { start, goal, scenario, params } = req;
  const startedAt = t0;
  const deadline = t0 + params.planningTimeoutMs;

  const maxSteer = (params.maxSteeringAngleDeg * Math.PI) / 180;
  const wheelBase = effectiveWheelBase(params.minTurningRadius, maxSteer);
  const inflate =
    params.safetyBuffer + params.localizationErrorMargin + params.pathFollowingErrorMargin;
  const colCtx: CollisionContext = {
    scenario,
    truckLength: params.truckLength,
    truckWidth: params.truckWidth,
    inflate,
  };

  if (!poseIsFree(start, colCtx)) {
    return { ok: false, reason: 'start_invalid', expansions: 0, elapsedMs: performance.now() - startedAt };
  }

  const primitives = buildPrimitives(maxSteer);
  const open = new MinHeap<Node>();
  const closed = new Map<string, number>(); // key → g cost

  const startNode: Node = {
    pose: start,
    steer: 0,
    direction: 1,
    g: 0,
    priority: heuristic(start, goal, params.minTurningRadius),
    parent: null,
  };
  open.push(startNode);

  const headingTolRad = (params.finalHeadingToleranceDeg * Math.PI) / 180;

  let expansions = 0;
  while (open.size > 0) {
    if (expansions % 256 === 0 && performance.now() > deadline) {
      return { ok: false, reason: 'timeout', expansions, elapsedMs: performance.now() - startedAt };
    }
    if (expansions >= params.maxExpansions) {
      return { ok: false, reason: 'timeout', expansions, elapsedMs: performance.now() - startedAt };
    }
    const cur = open.pop()!;
    const key = discKey(cur.pose, params.gridResolution, params.headingResolution);
    const existing = closed.get(key);
    if (existing !== undefined && existing <= cur.g) continue;
    closed.set(key, cur.g);
    expansions++;

    if (goalReached(cur.pose, goal, params.finalPoseToleranceM, headingTolRad)) {
      const path = reconstruct(cur, wheelBase, params.gridResolution);
      return { ok: true, path, expansions, elapsedMs: performance.now() - startedAt };
    }

    for (const prim of primitives) {
      const child = bicycleStep(cur.pose, prim.steer, prim.direction, params.gridResolution, wheelBase);
      if (!poseIsFree(child, colCtx)) continue;
      const clearance = poseClearance(child, colCtx);
      const minClear = Math.min(clearance.boundary, clearance.obstacle);
      const stepCost = edgeCost(
        params.gridResolution,
        prim.direction,
        prim.steer,
        cur.steer,
        cur.direction,
        minClear,
        {
          reversePenalty: params.reversePenalty,
          steeringSmoothnessWeight: params.steeringSmoothnessWeight,
          boundaryClearanceWeight: params.boundaryClearanceWeight,
          clearanceFloor: 1.5,
        },
        params.cuspPenalty
      );
      const childG = cur.g + stepCost;
      const childKey = discKey(child, params.gridResolution, params.headingResolution);
      const existingG = closed.get(childKey);
      if (existingG !== undefined && existingG <= childG) continue;
      open.push({
        pose: child,
        steer: prim.steer,
        direction: prim.direction,
        g: childG,
        priority: childG + heuristic(child, goal, params.minTurningRadius),
        parent: cur,
      });
    }
  }

  return { ok: false, reason: 'goal_unreachable', expansions, elapsedMs: performance.now() - startedAt };
}
