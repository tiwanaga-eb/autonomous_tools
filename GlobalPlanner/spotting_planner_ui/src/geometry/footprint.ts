import type { Pose, Polygon } from '../types';

/**
 * Truck footprint as a CCW polygon at pose (x, y, θ).
 *
 * Model assumption: the rear axle of the truck is the kinematic reference point (x, y).
 * The footprint rectangle extends:
 *   - rearOverhang behind the rear axle
 *   - (length - rearOverhang) in front of the rear axle
 *   - ±width/2 to the sides
 *
 * For prototype purposes we approximate rearOverhang = 0.15 * length (typical haul truck).
 */
export function truckFootprint(pose: Pose, length: number, width: number, inflate = 0): Polygon {
  const rearOverhang = 0.15 * length;
  const halfW = width / 2 + inflate;
  const front = length - rearOverhang + inflate;
  const rear = rearOverhang + inflate;

  // Corners in the truck's local frame (forward = +x_local, left = +y_local).
  const local: [number, number][] = [
    [-rear, -halfW],
    [+front, -halfW],
    [+front, +halfW],
    [-rear, +halfW],
  ];

  const cos = Math.cos(pose.heading);
  const sin = Math.sin(pose.heading);

  return local.map(([lx, ly]) => ({
    x: pose.x + lx * cos - ly * sin,
    y: pose.y + lx * sin + ly * cos,
  }));
}

/** Convenience: returns just the front-axle position (for forward-arrow rendering). */
export function frontAxleAnchor(pose: Pose, length: number): { x: number; y: number } {
  const rearOverhang = 0.15 * length;
  const wheelBase = length - 2 * rearOverhang; // crude estimate; OK for visuals only
  return {
    x: pose.x + wheelBase * Math.cos(pose.heading),
    y: pose.y + wheelBase * Math.sin(pose.heading),
  };
}
