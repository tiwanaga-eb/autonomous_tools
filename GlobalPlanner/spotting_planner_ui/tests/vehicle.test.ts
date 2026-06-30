import { describe, it, expect } from 'vitest';
import { bicycleStep, effectiveWheelBase } from '../src/vehicle/bicycle';

describe('bicycleStep', () => {
  it('straight forward step advances along heading', () => {
    const p = bicycleStep({ x: 0, y: 0, heading: 0 }, 0, 1, 1, 5);
    expect(p.x).toBeCloseTo(1);
    expect(p.y).toBeCloseTo(0);
    expect(p.heading).toBeCloseTo(0);
  });
  it('straight reverse step retreats', () => {
    const p = bicycleStep({ x: 0, y: 0, heading: 0 }, 0, -1, 1, 5);
    expect(p.x).toBeCloseTo(-1);
    expect(p.y).toBeCloseTo(0);
  });
  it('positive steer turns left', () => {
    // After a full quarter circle of arc length (πR/2), heading should advance by π/2.
    const R = 10;
    const wb = R * Math.tan((30 * Math.PI) / 180); // wheelBase that yields R at δ=30°
    const arc = (Math.PI * R) / 2;
    const steer = (30 * Math.PI) / 180;
    let p = { x: 0, y: 0, heading: 0 };
    const N = 200;
    for (let i = 0; i < N; i++) p = bicycleStep(p, steer, 1, arc / N, wb);
    expect(p.heading).toBeCloseTo(Math.PI / 2, 1);
  });
});

describe('effectiveWheelBase', () => {
  it('yields configured R_min at full steer', () => {
    const R = 8;
    const delta = (35 * Math.PI) / 180;
    const wb = effectiveWheelBase(R, delta);
    // R = wb / tan(δ) ⇒ R = (R tan δ) / tan δ = R  ✓
    expect(wb / Math.tan(delta)).toBeCloseTo(R);
  });
});
