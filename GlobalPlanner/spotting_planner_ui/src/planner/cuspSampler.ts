import type { Pose, PlannerParams, Scenario } from '../types';
import { poseIsFree } from './collision';

/**
 * Auto-generate cusp pose candidates for the spotting maneuver.
 *
 * A spotting cusp is the pose at which the truck stops going forward and switches to
 * reversing toward the dump target. Good cusps satisfy:
 *   - heading roughly opposite the final dump heading (so the reverse leg backs in
 *     straight),
 *   - positioned in a forward arc in front of the dump target (i.e. on the side
 *     opposite the truck's final heading direction),
 *   - footprint clear of boundary and obstacles.
 *
 * We sample over a polar fan in front of the dump target:
 *   distance  ∈ [1.2 R, 2.5 R] where R = min turning radius
 *   bearing   ∈ [-60°, +60°]   relative to the "front" direction of the dump target
 *   heading   = dumpHeading + 180° (with small jitter)
 *
 * Output is filtered to free poses only and sorted by distance from dump target.
 */
export function generateCuspCandidates(
  dump: Pose,
  scenario: Scenario,
  params: PlannerParams
): Pose[] {
  const candidates: Pose[] = [];
  const r = params.minTurningRadius;
  const minD = 1.2 * r;
  const maxD = 2.5 * r;

  const N = Math.max(3, params.numCuspCandidates);
  // Geometric distribution over distance × bearing.
  const distSteps = 3;
  const bearingSteps = Math.max(2, Math.ceil(N / distSteps));

  // The truck approaches the dump while reversing, so its heading at the cusp matches
  // the dump heading (NOT the opposite). The cusp itself sits *in front of* the dump
  // along the dump's own forward (+x_local) direction so that reversing from the cusp
  // moves the truck backward — i.e. toward the dump in world space.
  const cuspHeading = dump.heading;

  const colCtx = {
    scenario,
    truckLength: params.truckLength,
    truckWidth: params.truckWidth,
    inflate:
      params.safetyBuffer + params.localizationErrorMargin + params.pathFollowingErrorMargin,
  };

  for (let di = 0; di < distSteps; di++) {
    const d = minD + (di / Math.max(1, distSteps - 1)) * (maxD - minD);
    for (let bi = 0; bi < bearingSteps; bi++) {
      const bFrac = bearingSteps === 1 ? 0.5 : bi / (bearingSteps - 1);
      const bearing = (bFrac - 0.5) * (120 * Math.PI) / 180; // ±60°
      // Place the candidate in front of the dump target.
      const angleFromDump = dump.heading + bearing;
      const cx = dump.x + d * Math.cos(angleFromDump);
      const cy = dump.y + d * Math.sin(angleFromDump);
      // Small jitter on heading so off-axis candidates point slightly back toward dump,
      // smoothing the entry of the reverse leg.
      const headingJitter = bearing * 0.3;
      const pose: Pose = { x: cx, y: cy, heading: cuspHeading + headingJitter };
      if (poseIsFree(pose, colCtx)) candidates.push(pose);
    }
  }

  // Sort by distance from dump target (closer cusps tend to give shorter reverse legs).
  candidates.sort(
    (a, b) =>
      Math.hypot(a.x - dump.x, a.y - dump.y) - Math.hypot(b.x - dump.x, b.y - dump.y)
  );
  return candidates.slice(0, N);
}
