import type { Connector, Lane } from "../map/network";

export interface RouteStep {
  lane: Lane;
  /** Arc length at which the route enters this lane. */
  sIn: number;
  via: "start" | "conn" | "lc";
  conn: Connector | null;
}

export interface RouteGoal {
  /** Goal arc length for each lane that counts as arriving. */
  lanes: Map<Lane, number>;
}

const TURN_COST = { straight: 0, right: 4, left: 10, uturn: 90 } as const;
const LC_COST = 9;
const LC_DIST = 18;

export class MinHeap<T> {
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

interface Label {
  cost: number;
  sIn: number;
  lane: Lane;
  prev: number;
  via: RouteStep["via"];
  conn: Connector | null;
}

/**
 * Shortest lane-level route; lane changes are edges with a cost and a minimum distance.
 * The search state is (lane, entry mode): a lane entered at its start through a connector offers
 * different onward lane changes than the same lane entered mid-way, so both are kept.
 * Arrivals at goal lanes are labelled separately so a route may loop back to a lane it has
 * already passed (for example when the destination is behind the vehicle).
 */
export function planRoute(start: Lane, s0: number, goal: RouteGoal): RouteStep[] | null {
  const key = (lane: Lane, mid: boolean): number => lane.id * 2 + (mid ? 1 : 0);
  const labels = new Map<number, Label>();
  const goalLabels = new Map<Lane, Label>();
  const heap = new MinHeap<{ key: number; goal: Lane | null }>();
  const done = new Set<number>();
  let bestGoal = Infinity;

  const offerGoal = (lab: Label): void => {
    const gs = goal.lanes.get(lab.lane);
    if (gs === undefined || lab.sIn > gs + 0.5) return;
    const total = lab.cost + Math.max(0, gs - lab.sIn);
    if (total >= bestGoal) return;
    bestGoal = total;
    goalLabels.set(lab.lane, lab);
    heap.push(total, { key: -1, goal: lab.lane });
  };

  const k0 = key(start, s0 > 0.5);
  const startLabel: Label = { cost: 0, sIn: s0, lane: start, prev: -1, via: "start", conn: null };
  labels.set(k0, startLabel);
  offerGoal(startLabel);
  heap.push(0, { key: k0, goal: null });

  let arrived: Lane | null = null;
  while (heap.size > 0) {
    const top = heap.pop();
    if (!top) break;
    if (top.v.goal) {
      if (top.k <= bestGoal + 1e-9) {
        arrived = top.v.goal;
        break;
      }
      continue;
    }
    const k = top.v.key;
    if (done.has(k)) continue;
    done.add(k);
    const lab = labels.get(k) as Label;
    const lane = lab.lane;
    const remaining = Math.max(0, lane.poly.length - lab.sIn);
    const relax = (to: Lane, cost: number, sIn: number, via: RouteStep["via"], conn: Connector | null): void => {
      const next: Label = { cost, sIn, lane: to, prev: k, via, conn };
      offerGoal(next);
      const nk = key(to, via === "lc");
      if (done.has(nk)) return;
      const cur = labels.get(nk);
      if (cur && cur.cost <= cost) return;
      labels.set(nk, next);
      heap.push(cost, { key: nk, goal: null });
    };
    for (const c of lane.out) {
      const signal = c.junction.controller ? 4 : 0;
      relax(c.to, lab.cost + remaining + c.poly.length + TURN_COST[c.turn] + signal, 0, "conn", c);
    }
    for (const nb of [lane.left, lane.right]) {
      if (!nb) continue;
      // Map data splits roads into short pieces; a lane change may start on one and finish on the
      // next, so short lanes allow a compressed lane change at a higher price.
      const lcLen = Math.max(0, Math.min(LC_DIST, remaining - 1));
      const sIn = Math.max(0, Math.min(lab.sIn + lcLen / 2, nb.poly.length - 0.5));
      relax(nb, lab.cost + lcLen + LC_COST + (LC_DIST - lcLen) * 3, sIn, "lc", null);
    }
  }
  if (!arrived) return null;
  const steps: RouteStep[] = [];
  let cur: Label | undefined = goalLabels.get(arrived);
  for (let guard = 0; cur && guard < 5000; guard++) {
    steps.push({ lane: cur.lane, sIn: cur.sIn, via: cur.via, conn: cur.conn });
    cur = cur.prev >= 0 ? labels.get(cur.prev) : undefined;
  }
  return steps.reverse();
}

/** Goal specification for a destination point: every lane of the street it lies on, both directions. */
export function goalForPoint(lane: Lane, s: number): RouteGoal {
  const lanes = new Map<Lane, number>();
  const p = lane.poly.sampleAt(s).p;
  const seg = lane.road.seg;
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
