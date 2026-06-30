import { describe, it, expect } from 'vitest';
import { poseIsFree, poseClearance } from '../src/planner/collision';
import type { Scenario } from '../src/types';

const scenario: Scenario = {
  authorizedBoundary: [
    { x: 0, y: 0 },
    { x: 30, y: 0 },
    { x: 30, y: 30 },
    { x: 0, y: 30 },
  ],
  traversableArea: [
    { x: 2, y: 2 },
    { x: 28, y: 2 },
    { x: 28, y: 28 },
    { x: 2, y: 28 },
  ],
  obstacles: [
    {
      id: 'block',
      polygon: [
        { x: 15, y: 15 },
        { x: 20, y: 15 },
        { x: 20, y: 20 },
        { x: 15, y: 20 },
      ],
    },
  ],
  dumpTargetPose: { x: 0, y: 0, heading: 0 },
  queuePose: { x: 0, y: 0, heading: 0 },
  exitLanePose: { x: 0, y: 0, heading: 0 },
};

const ctx = { scenario, truckLength: 6, truckWidth: 3, inflate: 0.2 };

describe('poseIsFree', () => {
  it('free pose in open area', () => {
    expect(poseIsFree({ x: 10, y: 10, heading: 0 }, ctx)).toBe(true);
  });
  it('rejects pose intersecting obstacle', () => {
    expect(poseIsFree({ x: 17, y: 17, heading: 0 }, ctx)).toBe(false);
  });
  it('rejects pose hanging outside traversable area', () => {
    expect(poseIsFree({ x: 2.1, y: 10, heading: 0 }, ctx)).toBe(false);
  });
});

describe('poseClearance', () => {
  it('returns finite boundary and obstacle clearances', () => {
    const c = poseClearance({ x: 10, y: 10, heading: 0 }, ctx);
    expect(c.boundary).toBeGreaterThan(0);
    expect(c.obstacle).toBeGreaterThan(0);
    expect(Number.isFinite(c.boundary)).toBe(true);
    expect(Number.isFinite(c.obstacle)).toBe(true);
  });
});
