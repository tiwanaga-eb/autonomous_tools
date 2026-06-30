/**
 * Edge cost for one motion primitive expansion.
 *
 *   c = ds * (1 + reversePenalty if reverse) * curvaturePenalty
 *
 * Plus, applied externally by the search:
 *   + cuspPenalty when direction flips between parent and child
 *   + boundaryClearanceWeight * max(0, 1/clearance - 1/clearanceFloor) for tight passes
 *   + steeringSmoothnessWeight * |Δsteer|
 */

export type EdgeCostParams = {
  reversePenalty: number;
  steeringSmoothnessWeight: number;
  boundaryClearanceWeight: number;
  clearanceFloor: number; // clearance below which we start charging the weight
};

export function edgeCost(
  ds: number,
  direction: 1 | -1,
  steer: number,
  parentSteer: number,
  parentDir: 1 | -1,
  minBoundaryClearance: number,
  params: EdgeCostParams,
  cuspPenalty: number
): number {
  let c = ds;
  if (direction < 0) c *= 1 + params.reversePenalty;
  if (parentDir !== direction) c += cuspPenalty;
  c += params.steeringSmoothnessWeight * Math.abs(steer - parentSteer);
  if (minBoundaryClearance < params.clearanceFloor) {
    const deficit = 1 / Math.max(minBoundaryClearance, 0.01) - 1 / params.clearanceFloor;
    c += params.boundaryClearanceWeight * deficit;
  }
  return c;
}
