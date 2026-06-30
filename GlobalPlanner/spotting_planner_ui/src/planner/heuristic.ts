import type { Pose } from '../types';

/**
 * Admissible heuristic for Hybrid A*.
 *
 * The optimal cost-to-go for a non-holonomic vehicle is bounded below by the
 * Reeds-Shepp (RS) length, which itself is bounded below by the Euclidean distance.
 * We approximate the lower bound with a cheap-to-compute mixture:
 *
 *   h(p, g) = max(
 *     euclidean(p, g),
 *     turnLowerBound(p, g, R_min)
 *   )
 *
 * `turnLowerBound` accounts for the minimum sweep needed to align the heading at
 * the goal by adding a fraction of (R_min * |Δθ|) when the goal is close.
 *
 * This is provably ≤ the true RS length, so A* remains optimal w.r.t. the discrete
 * primitive set. We do not include reverse-penalty terms in the heuristic since
 * they would break admissibility in general.
 */
export function heuristic(p: Pose, g: Pose, minTurningRadius: number): number {
  const dx = g.x - p.x;
  const dy = g.y - p.y;
  const eucl = Math.hypot(dx, dy);
  let dh = g.heading - p.heading;
  while (dh > Math.PI) dh -= 2 * Math.PI;
  while (dh <= -Math.PI) dh += 2 * Math.PI;
  const turn = Math.abs(dh) * minTurningRadius;
  // Blend: when far away, distance dominates; close in, heading correction dominates.
  return Math.max(eucl, 0.5 * (eucl + turn));
}
