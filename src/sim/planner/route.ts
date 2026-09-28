import type { Connector, Lane } from "../map/network";

export interface RouteStep {
  lane: Lane;
  /** Arc length at which the route enters this lane. */
  sIn: number;
  via: "start" | "conn" | "lc";
  conn: Connector | null;
}

export interface RouteGoal {
  /** Goal arc length for each lane of the destination road. */
  lanes: Map<Lane, number>;
}

const TURN_COST = { straight: 0, right: 4, left: 10, uturn: 90 } as const;
const LC_COST = 9;
const LC_DIST = 18;

class MinHeap<T> {
  private readonly items: { k: number; v: T }[] = [];
  get size(): number {
    return this.items.length;
  }
  push(k: number, v: T): void {
    const a = this.items;
    a.push({ k, v });
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (a[p].k <= a[i].k) break;
      [a[p], a[i]] = [a[i], a[p]];
      i = p;
    }
  }
  pop(): { k: number; v: T } | undefined {
    const a = this.items;
    if (a.length === 0) return undefined;
    const top = a[0];
    const last = a.pop() as { k: number; v: T };
    if (a.length > 0) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        if (l < a.length && a[l].k < a[m].k) m = l;
        if (r < a.length && a[r].k < a[m].k) m = r;
        if (m === i) break;
        [a[m], a[i]] = [a[i], a[m]];
        i = m;
      }
    }
    return top;
  }
}

export { MinHeap };

interface Label {
  cost: number;
  sIn: number;
  prev: Lane | null;
  via: RouteStep["via"];
  conn: Connector | null;
}

/** Shortest lane-level route; lane changes are edges with a fixed cost and minimum distance. */
export function planRoute(start: Lane, s0: number, goal: RouteGoal): RouteStep[] | null {
  const labels = new Map<Lane, Label>();
  const heap = new MinHeap<{ lane: Lane; goal: boolean }>();
  labels.set(start, { cost: 0, sIn: s0, prev: null, via: "start", conn: null });
  heap.push(0, { lane: start, goal: false });
  const done = new Set<Lane>();
  let goalLane: Lane | null = null;
  let bestGoal = Infinity;

  while (heap.size > 0) {
    const top = heap.pop();
    if (!top) break;
    const { lane, goal: isGoal } = top.v;
    if (isGoal) {
      if (top.k <= bestGoal) {
        goalLane = lane;
        break;
      }
      continue;
    }
    if (done.has(lane)) continue;
    done.add(lane);
    const lab = labels.get(lane) as Label;
    const gs = goal.lanes.get(lane);
    if (gs !== undefined && lab.sIn <= gs + 0.5) {
      const total = lab.cost + Math.max(0, gs - lab.sIn);
      if (total < bestGoal) {
        bestGoal = total;
        heap.push(total, { lane, goal: true });
      }
    }
    const relax = (to: Lane, cost: number, sIn: number, via: RouteStep["via"], conn: Connector | null): void => {
      if (done.has(to)) return;
      const cur = labels.get(to);
      if (cur && cur.cost <= cost) return;
      labels.set(to, { cost, sIn, prev: lane, via, conn });
      heap.push(cost, { lane: to, goal: false });
    };
    const remaining = Math.max(0, lane.poly.length - lab.sIn);
    for (const c of lane.out) {
      const signal = c.junction.controller ? 4 : 0;
      relax(c.to, lab.cost + remaining + c.poly.length + TURN_COST[c.turn] + signal, 0, "conn", c);
    }
    for (const nb of [lane.left, lane.right]) {
      if (!nb) continue;
      const sIn = lab.sIn + LC_DIST;
      if (remaining < LC_DIST + 6 || sIn > nb.poly.length - 6) continue;
      relax(nb, lab.cost + LC_DIST + LC_COST, sIn, "lc", null);
    }
  }
  if (!goalLane) return null;
  const steps: RouteStep[] = [];
  let cur: Lane | null = goalLane;
  for (let guard = 0; cur && guard < 5000; guard++) {
    const lab = labels.get(cur) as Label;
    steps.push({ lane: cur, sIn: lab.sIn, via: lab.via, conn: lab.conn });
    cur = lab.prev;
  }
  return steps.reverse();
}

/** Goal specification for a destination point: every lane of the road it lies on. */
export function goalForPoint(lane: Lane, s: number): RouteGoal {
  const lanes = new Map<Lane, number>();
  const p = lane.poly.sampleAt(s).p;
  const seg = lane.road.seg;
  // Either carriageway of the street counts: the destination can be reached from both sides.
  for (const road of [seg.forward, seg.backward]) {
    if (!road) continue;
    for (const l of road.lanes) {
      const pr = l.poly.project(p);
      if (pr.s > 1 && pr.s < l.poly.length - 1) lanes.set(l, pr.s);
    }
  }
  if (lanes.size === 0) lanes.set(lane, s);
  return { lanes };
}
