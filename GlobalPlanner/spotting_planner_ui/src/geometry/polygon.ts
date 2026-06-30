import type { Polygon, Vec2 } from '../types';
import { distance } from './primitives';

/**
 * Ray-casting point-in-polygon test. Works for both CW and CCW polygons.
 * Edges are treated as closed-open to avoid double counts on vertex hits.
 */
export function pointInPolygon(p: Vec2, poly: Polygon): boolean {
  let inside = false;
  const n = poly.length;
  for (let i = 0, j = n - 1; i < n; j = i++) {
    const xi = poly[i].x;
    const yi = poly[i].y;
    const xj = poly[j].x;
    const yj = poly[j].y;
    const intersect =
      yi > p.y !== yj > p.y && p.x < ((xj - xi) * (p.y - yi)) / (yj - yi + 1e-12) + xi;
    if (intersect) inside = !inside;
  }
  return inside;
}

function onSegment(p: Vec2, q: Vec2, r: Vec2): boolean {
  return (
    q.x <= Math.max(p.x, r.x) + 1e-9 &&
    q.x >= Math.min(p.x, r.x) - 1e-9 &&
    q.y <= Math.max(p.y, r.y) + 1e-9 &&
    q.y >= Math.min(p.y, r.y) - 1e-9
  );
}

function orientation(p: Vec2, q: Vec2, r: Vec2): number {
  const v = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
  if (Math.abs(v) < 1e-12) return 0;
  return v > 0 ? 1 : 2;
}

/** True iff segments (p1,p2) and (p3,p4) intersect (including touching). */
export function segmentsIntersect(p1: Vec2, p2: Vec2, p3: Vec2, p4: Vec2): boolean {
  const o1 = orientation(p1, p2, p3);
  const o2 = orientation(p1, p2, p4);
  const o3 = orientation(p3, p4, p1);
  const o4 = orientation(p3, p4, p2);
  if (o1 !== o2 && o3 !== o4) return true;
  if (o1 === 0 && onSegment(p1, p3, p2)) return true;
  if (o2 === 0 && onSegment(p1, p4, p2)) return true;
  if (o3 === 0 && onSegment(p3, p1, p4)) return true;
  if (o4 === 0 && onSegment(p3, p2, p4)) return true;
  return false;
}

/** True iff any edge of polyA intersects any edge of polyB. */
export function polygonsEdgeIntersect(polyA: Polygon, polyB: Polygon): boolean {
  for (let i = 0, ai = polyA.length - 1; i < polyA.length; ai = i++) {
    for (let j = 0, bj = polyB.length - 1; j < polyB.length; bj = j++) {
      if (segmentsIntersect(polyA[ai], polyA[i], polyB[bj], polyB[j])) return true;
    }
  }
  return false;
}

/**
 * True iff polyA overlaps polyB (intersects edges OR is fully contained in either direction).
 */
export function polygonsOverlap(polyA: Polygon, polyB: Polygon): boolean {
  if (polygonsEdgeIntersect(polyA, polyB)) return true;
  if (pointInPolygon(polyA[0], polyB)) return true;
  if (pointInPolygon(polyB[0], polyA)) return true;
  return false;
}

/**
 * True iff polyInner lies fully inside polyOuter (no edge crossings, all vertices inside).
 */
export function polygonContains(polyOuter: Polygon, polyInner: Polygon): boolean {
  if (polygonsEdgeIntersect(polyOuter, polyInner)) return false;
  for (const v of polyInner) {
    if (!pointInPolygon(v, polyOuter)) return false;
  }
  return true;
}

/** Closest distance from a point to a polygon edge (regardless of inside/outside). */
export function distancePointToPolygonEdge(p: Vec2, poly: Polygon): number {
  let best = Infinity;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const d = distancePointToSegment(p, poly[j], poly[i]);
    if (d < best) best = d;
  }
  return best;
}

function distancePointToSegment(p: Vec2, a: Vec2, b: Vec2): number {
  const vx = b.x - a.x;
  const vy = b.y - a.y;
  const wx = p.x - a.x;
  const wy = p.y - a.y;
  const c1 = vx * wx + vy * wy;
  if (c1 <= 0) return distance(p, a);
  const c2 = vx * vx + vy * vy;
  if (c2 <= c1) return distance(p, b);
  const t = c1 / c2;
  const proj: Vec2 = { x: a.x + t * vx, y: a.y + t * vy };
  return distance(p, proj);
}

/**
 * Signed distance: positive if point is inside polygon, negative outside.
 * Magnitude is distance to nearest edge.
 */
export function signedDistanceToPolygon(p: Vec2, poly: Polygon): number {
  const d = distancePointToPolygonEdge(p, poly);
  return pointInPolygon(p, poly) ? d : -d;
}
