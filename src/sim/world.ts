import {
  Polyline,
  type Vec2,
  add,
  cubicBezier,
  dot,
  right,
  scale,
  sub,
  vec,
} from "./geometry";

export const GRID = 5;
export const BLOCK = 80;
export const LANE_W = 3.5;
export const ROAD_HALF = 7;
/** Distance from node centre to the stop line. */
export const STOP_OFF = 11.5;
/** Crosswalk band, measured from node centre along the arm. */
export const CROSS_IN = 7.5;
export const CROSS_OUT = 10.5;
/** Pedestrian corner position (both axes) relative to node centre. */
export const CORNER = 9.8;
/** Sidewalk centre offset from a road centreline. */
export const SIDEWALK = 9.2;
export const SPEED_LIMIT = 50 / 3.6;

/** E, S, W, N in the simulation frame (y grows southwards). */
export const DIRS: readonly Vec2[] = [vec(1, 0), vec(0, 1), vec(-1, 0), vec(0, -1)];
export type Turn = "straight" | "left" | "right";

export const angleOfDir = (d: number): number => Math.atan2((DIRS[d] as Vec2).y, (DIRS[d] as Vec2).x);

export function turnOf(dirIn: number, dirOut: number): Turn | "uturn" {
  const d = (dirOut - dirIn + 4) % 4;
  if (d === 0) return "straight";
  if (d === 1) return "right";
  if (d === 3) return "left";
  return "uturn";
}

/** Lane 0 is the outer (kerb-side) lane, lane 1 is next to the centreline. */
export const laneOffset = (lane: number): number => (lane === 0 ? 1.5 : 0.5) * LANE_W;

/** Lane that a turn must start from, or null when either lane works. */
export function requiredLane(turn: Turn): number | null {
  if (turn === "left") return 1;
  if (turn === "right") return 0;
  return null;
}

export interface GraphNode {
  id: number;
  i: number;
  j: number;
  pos: Vec2;
  /** Outgoing edge id per direction, -1 when the arm does not exist. */
  out: number[];
  /** Signal cycle offset in seconds. */
  phaseOffset: number;
}

export interface Edge {
  id: number;
  from: number;
  to: number;
  dir: number;
  /** Start of the lane segments (just past the exit crosswalk). */
  start: Vec2;
  length: number;
  lanes: [Polyline, Polyline];
}

export type SignalColor = "G" | "Y" | "R";

interface PhaseDef {
  name: "NS_THRU" | "NS_THRU_Y" | "NS_LEFT" | "NS_LEFT_Y" | "EW_THRU" | "EW_THRU_Y" | "EW_LEFT" | "EW_LEFT_Y" | "ALL_RED" | "PED";
  dur: number;
}

export const PHASES: readonly PhaseDef[] = [
  { name: "NS_THRU", dur: 8 },
  { name: "NS_THRU_Y", dur: 2.5 },
  { name: "NS_LEFT", dur: 4.5 },
  { name: "NS_LEFT_Y", dur: 2 },
  { name: "ALL_RED", dur: 1 },
  { name: "EW_THRU", dur: 8 },
  { name: "EW_THRU_Y", dur: 2.5 },
  { name: "EW_LEFT", dur: 4.5 },
  { name: "EW_LEFT_Y", dur: 2 },
  { name: "ALL_RED", dur: 1 },
  { name: "PED", dur: 9 },
  { name: "ALL_RED", dur: 1.5 },
];
export const CYCLE = PHASES.reduce((a, p) => a + p.dur, 0);

export interface PhaseState {
  name: PhaseDef["name"];
  elapsed: number;
  remaining: number;
}

export class World {
  readonly nodes: GraphNode[] = [];
  readonly edges: Edge[] = [];
  private readonly connectorCache = new Map<string, Polyline>();

  constructor(phaseSeed = 0.37) {
    for (let j = 0; j < GRID; j++) {
      for (let i = 0; i < GRID; i++) {
        const id = j * GRID + i;
        // Golden-ratio offsets give neighbouring signals different phases.
        const frac = (id * 0.618 + phaseSeed) % 1;
        this.nodes.push({ id, i, j, pos: vec(i * BLOCK, j * BLOCK), out: [-1, -1, -1, -1], phaseOffset: frac * CYCLE });
      }
    }
    for (const n of this.nodes) {
      for (let d = 0; d < 4; d++) {
        const dv = DIRS[d] as Vec2;
        const ni = n.i + dv.x;
        const nj = n.j + dv.y;
        if (ni < 0 || nj < 0 || ni >= GRID || nj >= GRID) continue;
        const to = this.nodes[nj * GRID + ni] as GraphNode;
        const start = add(n.pos, scale(dv, STOP_OFF));
        const end = sub(to.pos, scale(dv, STOP_OFF));
        const lane = (k: number): Polyline => {
          const off = scale(right(dv), laneOffset(k));
          return new Polyline([add(start, off), add(end, off)]);
        };
        const edge: Edge = {
          id: this.edges.length,
          from: n.id,
          to: to.id,
          dir: d,
          start,
          length: BLOCK - 2 * STOP_OFF,
          lanes: [lane(0), lane(1)],
        };
        n.out[d] = edge.id;
        this.edges.push(edge);
      }
    }
  }

  node(id: number): GraphNode {
    const n = this.nodes[id];
    if (!n) throw new Error(`Unknown node ${id}`);
    return n;
  }

  edge(id: number): Edge {
    const e = this.edges[id];
    if (!e) throw new Error(`Unknown edge ${id}`);
    return e;
  }

  /** Outgoing edges from the end of `e`, excluding U-turns. */
  successors(e: Edge): Edge[] {
    const n = this.node(e.to);
    const res: Edge[] = [];
    for (let d = 0; d < 4; d++) {
      const id = n.out[d] as number;
      if (id < 0 || turnOf(e.dir, d) === "uturn") continue;
      res.push(this.edge(id));
    }
    return res;
  }

  /** Point on an edge lane at edge coordinate u with an extra lateral offset (lane units already applied). */
  lanePoint(e: Edge, lateral: number, u: number): Vec2 {
    const d = DIRS[e.dir] as Vec2;
    return add(add(e.start, scale(d, u)), scale(right(d), lateral));
  }

  /** Edge coordinate (distance from edge start along the travel direction). */
  edgeCoord(e: Edge, p: Vec2): number {
    return dot(sub(p, e.start), DIRS[e.dir] as Vec2);
  }

  lateralCoord(e: Edge, p: Vec2): number {
    return dot(sub(p, e.start), right(DIRS[e.dir] as Vec2));
  }

  /** Lane-change path on `e` from lane `from` at u0 to lane `to`, continuing to the end of the edge. */
  laneChangePoly(e: Edge, from: number, to: number, u0: number, L: number): Polyline {
    const o0 = laneOffset(from);
    const o1 = laneOffset(to);
    const u1 = Math.min(u0 + L, e.length - 0.5);
    const pts: Vec2[] = [];
    for (let u = u0; u <= u1; u += 0.5) {
      const t = (u - u0) / Math.max(0.5, u1 - u0);
      const k = t * t * (3 - 2 * t);
      pts.push(this.lanePoint(e, o0 + (o1 - o0) * k, u));
    }
    pts.push(this.lanePoint(e, o1, e.length));
    return new Polyline(pts);
  }

  /** Path through an intersection, from the end of `a` (lane la) to the start of `b` (lane lb). */
  connector(a: Edge, la: number, b: Edge, lb: number): Polyline {
    const key = `${a.id}:${la}>${b.id}:${lb}`;
    const cached = this.connectorCache.get(key);
    if (cached) return cached;
    const dIn = DIRS[a.dir] as Vec2;
    const dOut = DIRS[b.dir] as Vec2;
    const p0 = (a.lanes[la as 0 | 1] as Polyline).pts[1] as Vec2;
    const p3 = (b.lanes[lb as 0 | 1] as Polyline).pts[0] as Vec2;
    let poly: Polyline;
    if (turnOf(a.dir, b.dir) === "straight") {
      const pts: Vec2[] = [];
      for (let t = 0; t <= 1.0001; t += 1 / 23) pts.push(add(p0, scale(sub(p3, p0), t)));
      poly = new Polyline(pts);
    } else {
      // Tangent lengths from the intersection of the two lane lines give a near-circular arc.
      const denom = dIn.x * dOut.y - dIn.y * dOut.x;
      const w = sub(p3, p0);
      const ta = (w.x * dOut.y - w.y * dOut.x) / denom;
      const tb = (w.x * dIn.y - w.y * dIn.x) / denom;
      const k = 0.5523;
      const p1 = add(p0, scale(dIn, ta * k));
      const p2 = sub(p3, scale(dOut, -tb * k));
      const pts: Vec2[] = [];
      const steps = 40;
      for (let s = 0; s <= steps; s++) pts.push(cubicBezier(p0, p1, p2, p3, s / steps));
      poly = new Polyline(pts);
    }
    this.connectorCache.set(key, poly);
    return poly;
  }

  phaseAt(nodeId: number, t: number): PhaseState {
    const n = this.node(nodeId);
    let tt = (((t + n.phaseOffset) % CYCLE) + CYCLE) % CYCLE;
    for (const p of PHASES) {
      if (tt < p.dur) return { name: p.name, elapsed: tt, remaining: p.dur - tt };
      tt -= p.dur;
    }
    const last = PHASES[PHASES.length - 1] as PhaseDef;
    return { name: last.name, elapsed: last.dur, remaining: 0 };
  }

  signal(nodeId: number, dirIn: number, turn: Turn, t: number): SignalColor {
    const ph = this.phaseAt(nodeId, t).name;
    const axis = dirIn % 2 === 0 ? "EW" : "NS";
    if (turn === "left") {
      if (ph === `${axis}_LEFT`) return "G";
      if (ph === `${axis}_LEFT_Y`) return "Y";
      return "R";
    }
    if (ph === `${axis}_THRU`) return "G";
    if (ph === `${axis}_THRU_Y`) return "Y";
    if (turn === "right") {
      if (ph === `${axis}_LEFT`) return "G";
      if (ph === `${axis}_LEFT_Y`) return "Y";
    }
    return "R";
  }

  /** Bounds of the drivable area, used by the minimap. */
  get extent(): { min: number; max: number } {
    return { min: -BLOCK * 0.5, max: (GRID - 1) * BLOCK + BLOCK * 0.5 };
  }
}
