import type { Pose, Scenario } from '../types';
import { truckFootprint } from '../geometry/footprint';
import {
  polygonContains,
  polygonsOverlap,
  distancePointToPolygonEdge,
} from '../geometry/polygon';

export type CollisionContext = {
  scenario: Scenario;
  truckLength: number;
  truckWidth: number;
  inflate: number;
};

/**
 * A pose is collision-free iff:
 *   - the inflated truck footprint lies fully inside the authorized boundary,
 *   - the inflated footprint lies fully inside the traversable area,
 *   - the inflated footprint does not overlap any obstacle.
 */
export function poseIsFree(pose: Pose, ctx: CollisionContext): boolean {
  const fp = truckFootprint(pose, ctx.truckLength, ctx.truckWidth, ctx.inflate);
  if (!polygonContains(ctx.scenario.authorizedBoundary, fp)) return false;
  if (!polygonContains(ctx.scenario.traversableArea, fp)) return false;
  for (const obs of ctx.scenario.obstacles) {
    if (polygonsOverlap(fp, obs.polygon)) return false;
  }
  return true;
}

/**
 * Minimum clearance from the (non-inflated) footprint to the nearest boundary or obstacle.
 * Returns positive metres. Used for telemetry, not for collision rejection.
 */
export function poseClearance(
  pose: Pose,
  ctx: CollisionContext
): { boundary: number; obstacle: number } {
  const fp = truckFootprint(pose, ctx.truckLength, ctx.truckWidth, 0);
  let boundary = Infinity;
  for (const v of fp) {
    boundary = Math.min(boundary, distancePointToPolygonEdge(v, ctx.scenario.traversableArea));
    boundary = Math.min(boundary, distancePointToPolygonEdge(v, ctx.scenario.authorizedBoundary));
  }
  let obstacle = Infinity;
  for (const obs of ctx.scenario.obstacles) {
    for (const v of fp) {
      obstacle = Math.min(obstacle, distancePointToPolygonEdge(v, obs.polygon));
    }
  }
  return { boundary, obstacle };
}
