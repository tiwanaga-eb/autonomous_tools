import type { Pose } from '../types';
import { normalizeAngle } from '../geometry/primitives';

/**
 * Kinematic bicycle model.
 *
 *   ẋ = v cos θ
 *   ẏ = v sin θ
 *   θ̇ = (v / L) tan δ
 *
 * v is signed (negative for reverse). δ is steering angle.
 *
 * We integrate over arc length ds (= |v|·dt). For prototype use we treat each motion
 * primitive as a constant-curvature arc and compute the closed-form pose update.
 *
 * The "wheelbase" parameter is supplied directly (caller derives it from truck length).
 */
export function bicycleStep(pose: Pose, steer: number, direction: 1 | -1, ds: number, wheelBase: number): Pose {
  const signedDs = direction * ds;
  const curvature = Math.tan(steer) / wheelBase;
  if (Math.abs(curvature) < 1e-9) {
    return {
      x: pose.x + signedDs * Math.cos(pose.heading),
      y: pose.y + signedDs * Math.sin(pose.heading),
      heading: pose.heading,
    };
  }
  const r = 1 / curvature;
  const dtheta = signedDs * curvature;
  const newHeading = normalizeAngle(pose.heading + dtheta);
  // Pivot around the instantaneous turning centre.
  const cx = pose.x - r * Math.sin(pose.heading);
  const cy = pose.y + r * Math.cos(pose.heading);
  return {
    x: cx + r * Math.sin(newHeading),
    y: cy - r * Math.cos(newHeading),
    heading: newHeading,
  };
}

/**
 * Effective wheelbase that yields the configured minimum turning radius at full steer.
 *
 *   R_min = L_wb / tan(δ_max)
 *   ⇒  L_wb = R_min * tan(δ_max)
 *
 * Using this wheelbase keeps the bicycle model self-consistent with the user-facing
 * "min turning radius" and "max steering angle" parameters.
 */
export function effectiveWheelBase(minTurningRadius: number, maxSteeringAngleRad: number): number {
  const t = Math.tan(maxSteeringAngleRad);
  if (Math.abs(t) < 1e-6) return minTurningRadius;
  return minTurningRadius * t;
}
