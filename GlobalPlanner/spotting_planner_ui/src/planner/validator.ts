import type { Path, PlannerParams, Scenario, PathSample } from '../types';
import { poseClearance, poseIsFree, type CollisionContext } from './collision';

/**
 * Re-validate a path by re-running the inflated collision check on every densified
 * sample and computing the minimum boundary / obstacle clearance over the path.
 *
 * Returns invalid sample indices so the renderer can highlight them in red. In a
 * correct planner this should always come back empty; we run it anyway as a
 * defence-in-depth check against discretisation gaps.
 */
export function validatePath(
  path: Path,
  scenario: Scenario,
  params: PlannerParams
): { invalidIndices: number[]; minBoundary: number; minObstacle: number } {
  const ctx: CollisionContext = {
    scenario,
    truckLength: params.truckLength,
    truckWidth: params.truckWidth,
    inflate:
      params.safetyBuffer + params.localizationErrorMargin + params.pathFollowingErrorMargin,
  };
  const invalidIndices: number[] = [];
  let minBoundary = Infinity;
  let minObstacle = Infinity;
  path.samples.forEach((s: PathSample, idx) => {
    if (!poseIsFree(s, ctx)) {
      invalidIndices.push(idx);
      s.invalid = true;
    }
    const c = poseClearance(s, ctx);
    if (c.boundary < minBoundary) minBoundary = c.boundary;
    if (c.obstacle < minObstacle) minObstacle = c.obstacle;
  });
  return { invalidIndices, minBoundary, minObstacle };
}
