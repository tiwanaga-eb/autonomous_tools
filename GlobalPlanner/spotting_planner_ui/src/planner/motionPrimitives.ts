/**
 * Six motion primitives at constant curvature:
 *   (steer ∈ {-δ_max, 0, +δ_max}) × (direction ∈ {forward, reverse})
 *
 * Each primitive is an arc of length `ds` integrated by the bicycle model.
 * `ds` is set to the planner's grid resolution so successive expansions visit
 * adjacent discretised cells.
 */
export type Primitive = {
  steer: number;        // rad
  direction: 1 | -1;
};

export function buildPrimitives(maxSteerRad: number): Primitive[] {
  return [
    { steer: -maxSteerRad, direction: +1 },
    { steer: 0, direction: +1 },
    { steer: +maxSteerRad, direction: +1 },
    { steer: -maxSteerRad, direction: -1 },
    { steer: 0, direction: -1 },
    { steer: +maxSteerRad, direction: -1 },
  ];
}
