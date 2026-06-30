import type { PlannerParams, Scenario, PlanResult, Pose, Path, PlanResultLegStatus } from '../types';
import { hybridAstar, type HybridAstarResult } from './hybridAstar';
import { generateCuspCandidates } from './cuspSampler';
import { validatePath } from './validator';
import { poseIsFree } from './collision';

/**
 * Two-stage orchestrator:
 *   1. Spotting:  queuePose → cuspPose → dumpTargetPose  (queue→cusp forward, cusp→dump reverse)
 *   2. Exit:      dumpTargetPose → exitLanePose          (typically forward)
 *
 * Cusp pose is either chosen automatically (best of N candidates by total spotting
 * cost) or supplied manually by the user.
 *
 * We split spotting into two A* calls because Hybrid A* naturally produces cusps
 * inside a single search, but enforcing the cusp to land at a SPECIFIC point is
 * easier by composing two single-direction-dominant searches with a shared
 * intermediate pose.
 *
 * For the forward leg (queue → cusp) we use the unmodified Hybrid A* (it can use
 * both directions but in practice picks forward). For the reverse leg (cusp →
 * dump) we apply a heavy forward-direction penalty by temporarily flipping the
 * reversePenalty into a forward penalty — easiest way is to plan in the reversed
 * geometry: search "forward" from dump → cusp and then reverse the path.
 */

type HybridFailReason = 'timeout' | 'no_expansion' | 'start_invalid' | 'goal_unreachable';
function reasonToReason(reason: HybridFailReason): PlanResultLegStatus {
  return reason;
}

/**
 * Re-interpret a forward-planned path as the *reverse* traversal of the same poses.
 * The heading at each pose is unchanged — a truck following the pose sequence in
 * reverse simply flips its direction-of-travel flag (forward ↔ reverse) at each
 * step. We deliberately do NOT add π to the heading: the truck's body orientation
 * is the same; only its velocity sign flips.
 */
function reversePath(path: Path): Path {
  const samples = [...path.samples].reverse().map((s) => ({
    ...s,
    direction: -s.direction as 1 | -1,
  }));
  return {
    samples,
    cuspIndices: [],
    totalLength: path.totalLength,
    // forward segments in the original become reverse segments in the flipped path
    reverseLength: path.totalLength - path.reverseLength,
  };
}

function concatPaths(a: Path, b: Path): Path {
  // The last sample of `a` should match the first sample of `b`. We drop one copy.
  const samples = [...a.samples, ...b.samples.slice(1)];
  const cuspIndices = [...a.cuspIndices];
  // The junction between a and b is itself a cusp if the directions flip there.
  const aEnd = a.samples[a.samples.length - 1];
  const bStart = b.samples[0];
  if (aEnd && bStart && aEnd.direction !== bStart.direction) {
    cuspIndices.push(a.samples.length - 1);
  }
  cuspIndices.push(...b.cuspIndices.map((i) => i + a.samples.length - 1));
  return {
    samples,
    cuspIndices,
    totalLength: a.totalLength + b.totalLength,
    reverseLength: a.reverseLength + b.reverseLength,
  };
}

function planLeg(
  start: Pose,
  goal: Pose,
  scenario: Scenario,
  params: PlannerParams
): HybridAstarResult {
  return hybridAstar({ start, goal, scenario, params });
}

export function plan(scenario: Scenario, params: PlannerParams): PlanResult {
  const t0 = performance.now();

  const colCtx = {
    scenario,
    truckLength: params.truckLength,
    truckWidth: params.truckWidth,
    inflate:
      params.safetyBuffer + params.localizationErrorMargin + params.pathFollowingErrorMargin,
  };

  // Up-front validation of fixed poses
  const queueOk = poseIsFree(scenario.queuePose, colCtx);
  const dumpOk = poseIsFree(scenario.dumpTargetPose, colCtx);
  const exitOk = poseIsFree(scenario.exitLanePose, colCtx);
  if (!queueOk || !dumpOk || !exitOk) {
    const which = !queueOk ? 'queuePose' : !dumpOk ? 'dumpTargetPose' : 'exitLanePose';
    return {
      feasible: false,
      spottingPath: null,
      exitPath: null,
      cuspPose: null,
      cuspCandidates: [],
      planningResponseMs: performance.now() - t0,
      minBoundaryClearance: 0,
      minObstacleClearance: 0,
      totalPathLength: 0,
      totalReverseDistance: 0,
      numCusps: 0,
      estimatedPathFollowingErrorM: 0,
      spottingLegStatus: 'start_invalid',
      exitLegStatus: 'start_invalid',
      failureReason: `Fixed pose ${which} is not feasible with current truck footprint and clearances.`,
    };
  }

  // === Spotting leg ===
  let candidates: Pose[] = [];
  let chosenCusp: Pose | null = null;
  let spottingPath: Path | null = null;
  let spottingStatus: PlanResultLegStatus = 'no_expansion';

  if (params.enableAutoCusp) {
    candidates = generateCuspCandidates(scenario.dumpTargetPose, scenario, params);
    if (candidates.length === 0) {
      spottingStatus = 'cusp_invalid';
    } else {
      let bestScore = Infinity;
      for (const cand of candidates) {
        const fwd = planLeg(scenario.queuePose, cand, scenario, params);
        if (!fwd.ok) continue;
        // Plan dump → cand forward, then reverse so it becomes cand → dump in reverse.
        const back = planLeg(scenario.dumpTargetPose, cand, scenario, params);
        if (!back.ok) continue;
        const reverseLeg = reversePath(back.path);
        const combined = concatPaths(fwd.path, reverseLeg);
        const reverseRatioScore =
          combined.reverseLength > params.maxReverseDistance ? Infinity : 0;
        const score = combined.totalLength + reverseRatioScore;
        if (score < bestScore) {
          bestScore = score;
          chosenCusp = cand;
          spottingPath = combined;
        }
      }
      if (!spottingPath) {
        spottingStatus = 'goal_unreachable';
      } else {
        spottingStatus = 'ok';
      }
    }
  } else {
    chosenCusp = params.manualCusp;
    if (!poseIsFree(chosenCusp, colCtx)) {
      spottingStatus = 'cusp_invalid';
    } else {
      const fwd = planLeg(scenario.queuePose, chosenCusp, scenario, params);
      if (!fwd.ok) {
        spottingStatus = reasonToReason(fwd.reason);
      } else {
        const back = planLeg(scenario.dumpTargetPose, chosenCusp, scenario, params);
        if (!back.ok) {
          spottingStatus = reasonToReason(back.reason);
        } else {
          const reverseLeg = reversePath(back.path);
          spottingPath = concatPaths(fwd.path, reverseLeg);
          spottingStatus = 'ok';
        }
      }
    }
  }

  // === Exit leg ===
  let exitPath: Path | null = null;
  let exitStatus: PlanResultLegStatus = 'no_expansion';
  const exitResult = planLeg(scenario.dumpTargetPose, scenario.exitLanePose, scenario, params);
  if (exitResult.ok) {
    exitPath = exitResult.path;
    exitStatus = 'ok';
  } else {
    exitStatus = reasonToReason(exitResult.reason);
  }

  // === Aggregate validation + metrics ===
  let minBoundary = Infinity;
  let minObstacle = Infinity;
  let totalLength = 0;
  let totalReverse = 0;
  let numCusps = 0;
  let anyInvalid = false;

  if (spottingPath) {
    const v = validatePath(spottingPath, scenario, params);
    if (v.invalidIndices.length > 0) anyInvalid = true;
    minBoundary = Math.min(minBoundary, v.minBoundary);
    minObstacle = Math.min(minObstacle, v.minObstacle);
    totalLength += spottingPath.totalLength;
    totalReverse += spottingPath.reverseLength;
    numCusps += spottingPath.cuspIndices.length;
  }
  if (exitPath) {
    const v = validatePath(exitPath, scenario, params);
    if (v.invalidIndices.length > 0) anyInvalid = true;
    minBoundary = Math.min(minBoundary, v.minBoundary);
    minObstacle = Math.min(minObstacle, v.minObstacle);
    totalLength += exitPath.totalLength;
    totalReverse += exitPath.reverseLength;
    numCusps += exitPath.cuspIndices.length;
  }

  const feasible = !anyInvalid && spottingStatus === 'ok' && exitStatus === 'ok';

  let failureReason: string | null = null;
  if (!feasible) {
    const parts: string[] = [];
    if (spottingStatus !== 'ok') parts.push(`spotting leg: ${spottingStatus}`);
    if (exitStatus !== 'ok') parts.push(`exit leg: ${exitStatus}`);
    if (anyInvalid) parts.push('post-validation found boundary violations (raise grid resolution or buffer)');
    failureReason = parts.join('; ') || 'unknown';
  }

  return {
    feasible,
    spottingPath,
    exitPath,
    cuspPose: chosenCusp,
    cuspCandidates: candidates,
    planningResponseMs: performance.now() - t0,
    minBoundaryClearance: isFinite(minBoundary) ? minBoundary : 0,
    minObstacleClearance: isFinite(minObstacle) ? minObstacle : 0,
    totalPathLength: totalLength,
    totalReverseDistance: totalReverse,
    numCusps,
    estimatedPathFollowingErrorM: params.pathFollowingErrorMargin,
    spottingLegStatus: spottingStatus,
    exitLegStatus: exitStatus,
    failureReason,
  };
}
