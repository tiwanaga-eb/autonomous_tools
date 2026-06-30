import type { Vec2 } from '../types';

export const TWO_PI = Math.PI * 2;

export function add(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x + b.x, y: a.y + b.y };
}

export function sub(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x - b.x, y: a.y - b.y };
}

export function scale(a: Vec2, k: number): Vec2 {
  return { x: a.x * k, y: a.y * k };
}

export function dot(a: Vec2, b: Vec2): number {
  return a.x * b.x + a.y * b.y;
}

export function cross(a: Vec2, b: Vec2): number {
  return a.x * b.y - a.y * b.x;
}

export function length(a: Vec2): number {
  return Math.hypot(a.x, a.y);
}

export function distance(a: Vec2, b: Vec2): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

/** Wrap angle to (-π, π]. */
export function normalizeAngle(theta: number): number {
  let t = theta % TWO_PI;
  if (t > Math.PI) t -= TWO_PI;
  if (t <= -Math.PI) t += TWO_PI;
  return t;
}

/** Shortest signed angular distance from a to b in (-π, π]. */
export function angleDiff(a: number, b: number): number {
  return normalizeAngle(b - a);
}

export function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}
