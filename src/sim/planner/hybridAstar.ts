import { footprint } from "../agents";
import { type Vec2, angleOf, dist, wrapAngle } from "../geometry";
import type { MapGrid } from "../map/network";
import { MinHeap } from "./route";

export interface Pose {
  x: number;
  y: number;
  h: number;
}

export interface FreePathPoint extends Pose {
  /** 1 forward, -1 reverse. */
  dir: number;
}

export interface FreeSpaceQuery {
  start: Pose;
  goals: Pose[];
  grid: MapGrid;
  /** Stationary obstacles (circle centres and radii). */
  obstacles: { p: Vec2; r: number }[];
  length: number;
  width: number;
  maxCurvature: number;
  maxExpansions: number;
}

export interface FreeSpaceResult {
  path: FreePathPoint[];
  expansions: number;
  goal: Pose;
}

const STEP = 1.6;
const HEADING_BINS = 72;

interface Node extends Pose {
  g: number;
  dir: number;
  kappa: number;
  parent: Node | null;
}

/**
 * Hybrid A* over (x, y, heading) with forward and reverse arcs. Used when the vehicle is not on a
 * lane it can follow: off the carriageway, facing against traffic, or inside a plaza.
 */
export function hybridAStar(q: FreeSpaceQuery): FreeSpaceResult | null {
  const { grid, goals, length, width } = q;
  if (goals.length === 0) return null;
  const curvatures = [-q.maxCurvature, -q.maxCurvature / 2, 0, q.maxCurvature / 2, q.maxCurvature];
  const radius = width / 2 + 0.2;

  const collides = (x: number, y: number, h: number): boolean => {
    for (const c of footprint({ x, y }, h, length, width)) {
      if (!grid.inside(c, 1)) return true;
      if (grid.clearanceAt(c) < radius) return true;
      for (const o of q.obstacles) {
        if (dist(c, o.p) < radius + o.r) return true;
      }
    }
    return false;
  };

  const heuristic = (x: number, y: number, h: number): number => {
    let best = Infinity;
    for (const g of goals) {
      const d = Math.hypot(g.x - x, g.y - y);
      const dh = Math.abs(wrapAngle(g.h - h));
      const v = d + 2.2 * dh;
      if (v < best) best = v;
    }
    return best;
  };

  const isGoal = (n: Pose): Pose | null => {
    for (const g of goals) {
      if (Math.hypot(g.x - n.x, g.y - n.y) < 1.8 && Math.abs(wrapAngle(g.h - n.h)) < 0.35) return g;
    }
    return null;
  };

  const key = (x: number, y: number, h: number): number => {
    const ix = Math.floor(x / 1.0) + 2048;
    const iy = Math.floor(y / 1.0) + 2048;
    const ih = ((Math.floor((wrapAngle(h) + Math.PI) / ((2 * Math.PI) / HEADING_BINS)) % HEADING_BINS) + HEADING_BINS) % HEADING_BINS;
    return (ix * 4096 + iy) * HEADING_BINS + ih;
  };

  const start: Node = { ...q.start, g: 0, dir: 1, kappa: 0, parent: null };
  const open = new MinHeap<Node>();
  const closed = new Set<number>();
  const bestG = new Map<number, number>();
  open.push(heuristic(start.x, start.y, start.h), start);
  let expansions = 0;

  while (open.size > 0 && expansions < q.maxExpansions) {
    const top = open.pop();
    if (!top) break;
    const n = top.v;
    const k = key(n.x, n.y, n.h);
    if (closed.has(k)) continue;
    closed.add(k);
    expansions++;
    const reached = isGoal(n);
    if (reached) return { path: reconstruct(n), expansions, goal: reached };

    for (const dir of [1, -1]) {
      for (const kappa of curvatures) {
        // Integrate the arc in small sub-steps so collisions between nodes are not missed.
        let x = n.x;
        let y = n.y;
        let h = n.h;
        let ok = true;
        const sub = 4;
        for (let i = 0; i < sub; i++) {
          const ds = (STEP / sub) * dir;
          h = h + ds * kappa;
          x += Math.cos(h) * ds;
          y += Math.sin(h) * ds;
          if (collides(x, y, h)) {
            ok = false;
            break;
          }
        }
        if (!ok) continue;
        const nk = key(x, y, h);
        if (closed.has(nk)) continue;
        const offRoad = grid.onRoad({ x, y }) ? 1 : 1.35;
        let cost = STEP * offRoad * (dir < 0 ? 2.2 : 1) + Math.abs(kappa) * 0.6;
        if (dir !== n.dir && n.parent) cost += 4.5;
        if (kappa !== n.kappa) cost += 0.25;
        const g = n.g + cost;
        const prev = bestG.get(nk);
        if (prev !== undefined && prev <= g) continue;
        bestG.set(nk, g);
        open.push(g + heuristic(x, y, h), { x, y, h, g, dir, kappa, parent: n });
      }
    }
  }
  return null;
}

function reconstruct(n: Node): FreePathPoint[] {
  const out: FreePathPoint[] = [];
  let cur: Node | null = n;
  while (cur) {
    out.push({ x: cur.x, y: cur.y, h: cur.h, dir: cur.dir });
    cur = cur.parent;
  }
  out.reverse();
  // The direction stored on a node describes the motion that reached it; shift it to the segment start.
  for (let i = 0; i < out.length - 1; i++) out[i].dir = out[i + 1].dir;
  return out;
}

/** Splits a free-space path into runs of constant driving direction. */
export function splitByDirection(path: FreePathPoint[]): FreePathPoint[][] {
  const runs: FreePathPoint[][] = [];
  let cur: FreePathPoint[] = [];
  for (const p of path) {
    if (cur.length > 0 && p.dir !== cur[cur.length - 1].dir) {
      cur.push(p);
      runs.push(cur);
      cur = [{ ...p }];
      continue;
    }
    cur.push(p);
  }
  if (cur.length > 1) runs.push(cur);
  return runs;
}

export function poseHeading(a: Vec2, b: Vec2): number {
  return angleOf({ x: b.x - a.x, y: b.y - a.y });
}
