import { Polyline, type Vec2, dist, lerp, smoothstep } from "../geometry";
import { type Connector, type Lane, LANE_W, type RoadNetwork } from "../map/network";
import type { RouteStep } from "./route";

export interface Gate {
  s: number;
  kind: "signal";
  lane: Lane;
}

export interface ConnSpan {
  s0: number;
  s1: number;
  conn: Connector;
}

/**
 * The route expressed as a continuous guide line. Per-point metadata describes the drivable
 * corridor around it; the trajectory planner decides how closely to follow it.
 */
export class Reference {
  readonly poly: Polyline;
  /** Lane at each point (-1 inside junctions). */
  readonly lane: Int32Array;
  /** Distance from the guide line to the left/right edge of the same-direction carriageway. */
  readonly leftEdge: Float32Array;
  readonly rightEdge: Float32Array;
  /** Width of oncoming lanes directly to the left (0 for one-way roads and junctions). */
  readonly oppWidth: Float32Array;
  readonly speed: Float32Array;
  readonly inJunction: Uint8Array;
  readonly gates: Gate[];
  readonly spans: ConnSpan[];
  readonly destS: number;
  readonly lanesUsed: Set<Lane>;

  constructor(
    pts: Vec2[],
    meta: { lane: number; left: number; right: number; opp: number; speed: number; junction: boolean }[],
    gateIdx: { idx: number; lane: Lane }[],
    spanIdx: { i0: number; i1: number; conn: Connector }[],
    destPoint: Vec2,
    lanesUsed: Set<Lane>,
  ) {
    // Deduplicate while keeping metadata aligned with the polyline vertices.
    const keepPts: Vec2[] = [];
    const keepIdx: number[] = [];
    const remap = new Int32Array(pts.length);
    for (let i = 0; i < pts.length; i++) {
      const last = keepPts[keepPts.length - 1];
      if (!last || dist(last, pts[i]) > 0.05) {
        keepPts.push(pts[i]);
        keepIdx.push(i);
      }
      remap[i] = keepPts.length - 1;
    }
    this.poly = new Polyline(keepPts);
    const n = this.poly.pts.length;
    this.lane = new Int32Array(n);
    this.leftEdge = new Float32Array(n);
    this.rightEdge = new Float32Array(n);
    this.oppWidth = new Float32Array(n);
    this.speed = new Float32Array(n);
    this.inJunction = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      const m = meta[keepIdx[Math.min(i, keepIdx.length - 1)]];
      this.lane[i] = m.lane;
      this.leftEdge[i] = m.left;
      this.rightEdge[i] = m.right;
      this.oppWidth[i] = m.opp;
      this.speed[i] = m.speed;
      this.inJunction[i] = m.junction ? 1 : 0;
    }
    this.gates = gateIdx.map((g) => ({ s: this.poly.cum[remap[g.idx]], kind: "signal" as const, lane: g.lane }));
    this.spans = spanIdx.map((sp) => ({ s0: this.poly.cum[remap[sp.i0]], s1: this.poly.cum[remap[sp.i1]], conn: sp.conn }));
    // The destination is the closest approach to the goal point near the end of the route.
    const tail = Math.max(0, this.poly.pts.length - 1 - 160);
    this.destS = this.poly.project(destPoint, this.poly.pts.length - 1, this.poly.pts.length - 1 - tail).s;
    this.lanesUsed = lanesUsed;
  }

  /** Index of the vertex at or before arc length s. */
  index(s: number): number {
    return this.poly.segmentAt(s);
  }
}

function laneMeta(l: Lane): { lane: number; left: number; right: number; opp: number; speed: number; junction: boolean } {
  const n = l.road.lanes.length;
  const seg = l.road.seg;
  const other = seg.forward === l.road ? seg.backward : seg.forward;
  return {
    lane: l.id,
    left: (n - 1 - l.k) * LANE_W + LANE_W / 2,
    right: l.k * LANE_W + LANE_W / 2,
    opp: other ? other.lanes.length * LANE_W : 0,
    speed: l.road.speed,
    junction: false,
  };
}

/** Builds the guide line for a lane-level route starting at the ego's position on the first lane. */
export function buildReference(net: RoadNetwork, route: RouteStep[], startS: number, v: number, dest: { lane: Lane; s: number }): Reference {
  const pts: Vec2[] = [];
  const meta: ReturnType<typeof laneMeta>[] = [];
  const gates: { idx: number; lane: Lane }[] = [];
  const spans: { i0: number; i1: number; conn: Connector }[] = [];
  const used = new Set<Lane>();
  const pushLane = (l: Lane, s0: number, s1: number): void => {
    const m = laneMeta(l);
    used.add(l);
    for (let s = s0; s < s1; s += 1) {
      pts.push(l.poly.sampleAt(s).p);
      meta.push(m);
    }
    pts.push(l.poly.sampleAt(s1).p);
    meta.push(m);
  };

  let s = Math.max(0, Math.min(startS, route[0].lane.poly.length));
  for (let i = 0; i < route.length; i++) {
    const step = route[i];
    const lane = step.lane;
    const next = route[i + 1];
    const isLast = i === route.length - 1;
    if (isLast) {
      const end = Math.min(lane.poly.length, dest.s + 12);
      pushLane(lane, s, Math.max(s, end));
      break;
    }
    if (next.via === "lc") {
      // Lane change: blend from this lane into the neighbour.
      const target = next.lane;
      const L = Math.max(8, Math.min(28, 2.2 * v + 10, lane.poly.length - s - 2, target.poly.length - next.sIn + 10));
      const begin = s + 1;
      pushLane(lane, s, begin);
      const mt = laneMeta(target);
      used.add(target);
      let endT = 0;
      for (let du = 0.5; du <= L; du += 0.5) {
        const pa = lane.poly.sampleAt(begin + du).p;
        const tS = target.poly.project(pa).s;
        const pb = target.poly.sampleAt(tS).p;
        pts.push(lerp(pa, pb, smoothstep(du / L)));
        meta.push(mt);
        endT = tS;
      }
      s = endT + 0.5;
      continue;
    }
    // Connector to the next lane.
    const end = lane.poly.length;
    pushLane(lane, s, Math.max(s, end));
    const conn = next.conn as Connector;
    if (net.laneSignal(lane, 0) !== null) gates.push({ idx: pts.length - 1, lane });
    const i0 = pts.length - 1;
    const connMeta = { lane: -1, left: 1.6, right: 1.6, opp: 0, speed: lane.road.speed, junction: true };
    for (let k = 1; k < conn.poly.pts.length; k++) {
      pts.push(conn.poly.pts[k]);
      meta.push(connMeta);
    }
    spans.push({ i0, i1: pts.length - 1, conn });
    s = 0.5;
  }
  const destPoint = dest.lane.poly.sampleAt(dest.s).p;
  return new Reference(smoothGuide(pts), meta, gates, spans, destPoint, used);
}

/**
 * Map data splits roads at every node, so a single turn can span several short lanes and
 * connectors with a curvature spike at each joint. A moving average of about 3 m removes the
 * spikes while keeping vertex indices (and therefore the metadata) aligned.
 */
function smoothGuide(pts: Vec2[]): Vec2[] {
  let cur = pts;
  for (let pass = 0; pass < 3; pass++) {
    const next: Vec2[] = new Array(cur.length);
    for (let i = 0; i < cur.length; i++) {
      const r = Math.min(3, i, cur.length - 1 - i);
      let x = 0;
      let y = 0;
      for (let k = -r; k <= r; k++) {
        x += cur[i + k].x;
        y += cur[i + k].y;
      }
      next[i] = { x: x / (2 * r + 1), y: y / (2 * r + 1) };
    }
    cur = next;
  }
  return cur;
}
