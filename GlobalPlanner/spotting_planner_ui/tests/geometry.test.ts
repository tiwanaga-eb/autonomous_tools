import { describe, it, expect } from 'vitest';
import {
  pointInPolygon,
  polygonContains,
  polygonsOverlap,
  signedDistanceToPolygon,
  segmentsIntersect,
} from '../src/geometry/polygon';
import { truckFootprint } from '../src/geometry/footprint';
import { normalizeAngle, angleDiff } from '../src/geometry/primitives';

const square = [
  { x: 0, y: 0 },
  { x: 10, y: 0 },
  { x: 10, y: 10 },
  { x: 0, y: 10 },
];

describe('pointInPolygon', () => {
  it('detects interior point', () => {
    expect(pointInPolygon({ x: 5, y: 5 }, square)).toBe(true);
  });
  it('rejects exterior point', () => {
    expect(pointInPolygon({ x: 15, y: 5 }, square)).toBe(false);
  });
});

describe('segmentsIntersect', () => {
  it('detects crossing X', () => {
    expect(
      segmentsIntersect({ x: 0, y: 0 }, { x: 10, y: 10 }, { x: 0, y: 10 }, { x: 10, y: 0 })
    ).toBe(true);
  });
  it('parallel non-crossing', () => {
    expect(
      segmentsIntersect({ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 0, y: 1 }, { x: 10, y: 1 })
    ).toBe(false);
  });
});

describe('polygonContains', () => {
  it('inner box inside outer box', () => {
    const inner = [
      { x: 2, y: 2 },
      { x: 4, y: 2 },
      { x: 4, y: 4 },
      { x: 2, y: 4 },
    ];
    expect(polygonContains(square, inner)).toBe(true);
  });
  it('rejects overlapping box', () => {
    const inner = [
      { x: 9, y: 9 },
      { x: 12, y: 9 },
      { x: 12, y: 12 },
      { x: 9, y: 12 },
    ];
    expect(polygonContains(square, inner)).toBe(false);
  });
});

describe('polygonsOverlap', () => {
  it('disjoint boxes', () => {
    const b = [
      { x: 20, y: 20 },
      { x: 25, y: 20 },
      { x: 25, y: 25 },
      { x: 20, y: 25 },
    ];
    expect(polygonsOverlap(square, b)).toBe(false);
  });
  it('overlapping boxes', () => {
    const b = [
      { x: 8, y: 8 },
      { x: 12, y: 8 },
      { x: 12, y: 12 },
      { x: 8, y: 12 },
    ];
    expect(polygonsOverlap(square, b)).toBe(true);
  });
});

describe('signedDistanceToPolygon', () => {
  it('positive inside', () => {
    expect(signedDistanceToPolygon({ x: 5, y: 5 }, square)).toBeGreaterThan(0);
  });
  it('negative outside', () => {
    expect(signedDistanceToPolygon({ x: 15, y: 5 }, square)).toBeLessThan(0);
  });
});

describe('truckFootprint', () => {
  it('axis-aligned footprint at origin', () => {
    const fp = truckFootprint({ x: 0, y: 0, heading: 0 }, 10, 4);
    expect(fp).toHaveLength(4);
    // x range should span roughly [-1.5, 8.5] (rearOverhang = 1.5)
    const xs = fp.map((p) => p.x);
    expect(Math.min(...xs)).toBeCloseTo(-1.5);
    expect(Math.max(...xs)).toBeCloseTo(8.5);
    // y range should be [-2, 2]
    const ys = fp.map((p) => p.y);
    expect(Math.min(...ys)).toBeCloseTo(-2);
    expect(Math.max(...ys)).toBeCloseTo(2);
  });
  it('inflated footprint grows in all directions', () => {
    const fp0 = truckFootprint({ x: 0, y: 0, heading: 0 }, 10, 4, 0);
    const fp1 = truckFootprint({ x: 0, y: 0, heading: 0 }, 10, 4, 0.5);
    const w0 = Math.max(...fp0.map((p) => p.y)) - Math.min(...fp0.map((p) => p.y));
    const w1 = Math.max(...fp1.map((p) => p.y)) - Math.min(...fp1.map((p) => p.y));
    expect(w1 - w0).toBeCloseTo(1.0);
  });
});

describe('angle utilities', () => {
  it('normalizeAngle wraps to (-π, π]', () => {
    expect(normalizeAngle(3 * Math.PI)).toBeCloseTo(Math.PI);
    expect(normalizeAngle(-3 * Math.PI)).toBeCloseTo(Math.PI);
  });
  it('angleDiff is shortest signed', () => {
    expect(angleDiff(0, Math.PI / 2)).toBeCloseTo(Math.PI / 2);
    expect(angleDiff(Math.PI - 0.1, -Math.PI + 0.1)).toBeCloseTo(0.2, 5);
  });
});
