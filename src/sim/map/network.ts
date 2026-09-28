import {
  Polyline,
  type Vec2,
  add,
  angleOf,
  convexHull,
  cross,
  cubicBezier,
  dist,
  dist2,
  dot,
  norm,
  offsetPolyline,
  pointInPolygon,
  resample,
  scale,
  slicePolyline,
  smooth,
  sub,
  wrapAngle,
} from "../geometry";
import type { MapData } from "./mapData";

export const LANE_W = 3.3;

export type Turn = "straight" | "left" | "right" | "uturn";

export const CLASS_RANK: Record<string, number> = {
  trunk: 6,
  primary: 5,
  secondary: 4,
  tertiary: 3,
  unclassified: 2,
  residential: 1,
  living_street: 0,
};

export interface Junction {
  id: number;
  pos: Vec2;
  incoming: DirRoad[];
  outgoing: DirRoad[];
  /** Connectors inside this junction. */
  connectors: Connector[];
  controller: SignalController | null;
  circular: boolean;
  boundary: boolean;
  /** Outline used for rendering and the drivable-area raster. */
  outline: Vec2[];
}

export interface Segment {
  id: number;
  a: number;
  b: number;
  center: Polyline;
  /** Extent to the right/left of the centreline in the a→b direction. */
  rightW: number;
  leftW: number;
  oneway: boolean;
  cls: string;
  name: string;
  forward: DirRoad | null;
  backward: DirRoad | null;
  trimA: number;
  trimB: number;
  /** OSM node id → arc length on the centreline, for placing crossings. */
  nodeS: Map<number, number>;
}

export interface DirRoad {
  id: number;
  seg: Segment;
  from: number;
  to: number;
  /** Untrimmed centreline of the way, in travel direction. */
  center: Polyline;
  lanes: Lane[];
  cls: string;
  rank: number;
  speed: number;
  circular: boolean;
  name: string;
  /** Signal group index at the downstream controller, -1 if the approach is not signalised. */
  signalGroup: number;
}

export interface Lane {
  id: number;
  road: DirRoad;
  /** 0 is the rightmost lane. */
  k: number;
  offset: number;
  poly: Polyline;
  out: Connector[];
  in: Connector[];
  left: Lane | null;
  right: Lane | null;
  /** Vehicles leave the map at the end of this lane. */
  sink: boolean;
  /** Vehicles may enter the map at the start of this lane. */
  source: boolean;
}

export interface Conflict {
  other: Connector;
  s: number;
  sOther: number;
}

export interface Connector {
  id: number;
  from: Lane;
  to: Lane;
  poly: Polyline;
  turn: Turn;
  junction: Junction;
  conflicts: Conflict[];
  priority: number;
}

export interface SignalController {
  id: number;
  junctions: Junction[];
  /** Approach bearings (mod π) of each phase group. */
  groups: number[];
  offset: number;
  green: number;
  yellow: number;
  allRed: number;
}

export type SignalColor = "G" | "Y" | "R";

export interface Crossing {
  id: number;
  pos: Vec2;
  /** Road direction at the crossing. */
  dir: Vec2;
  halfWidth: number;
  seg: Segment;
  /** Controller and group whose red phase gives pedestrians the right of way, if signalised. */
  controller: SignalController | null;
  group: number;
}

export interface Building {
  pts: Vec2[];
  h: number;
  name: string;
}

const DEFAULT_SPEED: Record<string, number> = {
  primary: 50,
  secondary: 50,
  tertiary: 40,
  unclassified: 30,
  residential: 30,
  living_street: 20,
};

function laneCounts(r: MapData["roads"][number]): { f: number; b: number } {
  const cap = (n: number): number => Math.max(1, Math.min(r.circular ? 4 : 5, n));
  const defaults: Record<string, [number, number]> = {
    trunk: [3, 2],
    primary: [3, 2],
    secondary: [2, 2],
    tertiary: [2, 1],
    unclassified: [1, 1],
    residential: [1, 1],
    living_street: [1, 1],
  };
  const [oneDef, twoDef] = defaults[r.cls] ?? [1, 1];
  if (r.oneway) return { f: cap(r.lanes || oneDef), b: 0 };
  if (r.lanesF || r.lanesB) return { f: cap(r.lanesF || 1), b: cap(r.lanesB || 1) };
  if (r.lanes) {
    const f = Math.max(1, Math.floor(r.lanes / 2));
    return { f: cap(f), b: cap(Math.max(1, r.lanes - f)) };
  }
  return { f: twoDef, b: twoDef };
}

interface RawSeg {
  road: MapData["roads"][number];
  a: number;
  b: number;
  pts: Vec2[];
  ids: number[];
}

export class RoadNetwork {
  readonly junctions = new Map<number, Junction>();
  readonly segments: Segment[] = [];
  readonly roads: DirRoad[] = [];
  readonly lanes: Lane[] = [];
  readonly connectors: Connector[] = [];
  readonly controllers: SignalController[] = [];
  readonly crossings: Crossing[] = [];
  readonly buildings: Building[] = [];
  readonly parks: Vec2[][] = [];
  readonly bounds: { minX: number; maxX: number; minY: number; maxY: number };
  readonly grid: MapGrid;
  private readonly laneIndex: SpatialIndex<{ lane: Lane | null; conn: Connector | null; s: number }>;

  constructor(data: MapData) {
    this.bounds = data.bounds;
    const raw = this.splitSegments(data);
    this.buildSegments(raw);
    this.computeTrims();
    this.buildLanes();
    this.buildConnectors();
    this.buildConflicts();
    this.buildSignals(data.signals);
    this.buildCrossings(data.crossings);
    for (const b of data.buildings) {
      const pts: Vec2[] = [];
      for (let i = 0; i < b.pts.length; i += 2) pts.push({ x: b.pts[i], y: b.pts[i + 1] });
      this.buildings.push({ pts, h: b.h, name: b.name });
    }
    for (const p of data.parks) {
      const pts: Vec2[] = [];
      for (let i = 0; i < p.pts.length; i += 2) pts.push({ x: p.pts[i], y: p.pts[i + 1] });
      this.parks.push(pts);
    }
    this.buildJunctionOutlines();
    this.dropBuildingsOnRoads();
    this.laneIndex = new SpatialIndex(12);
    for (const l of this.lanes) {
      for (let s = 0; s <= l.poly.length; s += 1.5) this.laneIndex.add(l.poly.sampleAt(s).p, { lane: l, conn: null, s });
    }
    for (const c of this.connectors) {
      for (let s = 0; s <= c.poly.length; s += 1.5) this.laneIndex.add(c.poly.sampleAt(s).p, { lane: null, conn: c, s });
    }
    this.grid = new MapGrid(this);
    this.core = this.computeCore();
  }

  /** Lanes in the main strongly connected part of the lane graph (every one reaches every other). */
  readonly core: Set<Lane>;

  private computeCore(): Set<Lane> {
    const fwd = (l: Lane): Lane[] => [...l.out.map((c) => c.to), ...(l.left ? [l.left] : []), ...(l.right ? [l.right] : [])];
    const bwd = (l: Lane): Lane[] => [...l.in.map((c) => c.from), ...(l.left ? [l.left] : []), ...(l.right ? [l.right] : [])];
    const bfs = (start: Lane, next: (l: Lane) => Lane[]): Set<Lane> => {
      const seen = new Set([start]);
      const q = [start];
      while (q.length) for (const n of next(q.pop() as Lane)) if (!seen.has(n)) (seen.add(n), q.push(n));
      return seen;
    };
    const seed = this.lanes.find((l) => l.road.circular) ?? this.lanes[0];
    const a = bfs(seed, fwd);
    const b = bfs(seed, bwd);
    return new Set(this.lanes.filter((l) => a.has(l) && b.has(l)));
  }

  // ---------------------------------------------------------------------------
  // Construction

  private splitSegments(data: MapData): RawSeg[] {
    const usage = new Map<number, number>();
    for (const r of data.roads) {
      r.nodes.forEach((id, i) => {
        const endpoint = i === 0 || i === r.nodes.length - 1;
        usage.set(id, (usage.get(id) ?? 0) + (endpoint ? 2 : 1));
      });
    }
    const out: RawSeg[] = [];
    for (const r of data.roads) {
      const pts: Vec2[] = [];
      for (let i = 0; i < r.pts.length; i += 2) pts.push({ x: r.pts[i], y: r.pts[i + 1] });
      let start = 0;
      for (let i = 1; i < r.nodes.length; i++) {
        const isNode = i === r.nodes.length - 1 || (usage.get(r.nodes[i]) ?? 0) >= 2;
        if (!isNode) continue;
        const segPts = pts.slice(start, i + 1);
        const ids = r.nodes.slice(start, i + 1);
        if (new Polyline(segPts).length > 0.5) out.push({ road: r, a: r.nodes[start], b: r.nodes[i], pts: segPts, ids });
        start = i;
      }
    }
    return out;
  }

  private junctionAt(id: number, pos: Vec2): Junction {
    let j = this.junctions.get(id);
    if (!j) {
      j = { id, pos, incoming: [], outgoing: [], connectors: [], controller: null, circular: false, boundary: id < 0, outline: [] };
      this.junctions.set(id, j);
    }
    return j;
  }

  private buildSegments(raw: RawSeg[]): void {
    for (const r of raw) {
      const { f, b } = laneCounts(r.road);
      const pts = r.pts.length > 2 || new Polyline(r.pts).length > 6 ? smooth(resample(r.pts, 1.5), 2, 2) : r.pts;
      const center = new Polyline(pts);
      const nodeS = new Map<number, number>();
      for (const id of r.ids) nodeS.set(id, 0);
      // Arc lengths of the original OSM nodes, used to place crossings.
      const orig = new Polyline(r.pts);
      r.ids.forEach((id, i) => nodeS.set(id, orig.project(r.pts[i]).s * (center.length / Math.max(orig.length, 1e-3))));
      const seg: Segment = {
        id: this.segments.length,
        a: r.a,
        b: r.b,
        center,
        rightW: r.road.oneway ? (f * LANE_W) / 2 : f * LANE_W,
        leftW: r.road.oneway ? (f * LANE_W) / 2 : b * LANE_W,
        oneway: !!r.road.oneway,
        cls: r.road.cls,
        name: r.road.name,
        forward: null,
        backward: null,
        trimA: 0,
        trimB: 0,
        nodeS,
      };
      this.segments.push(seg);
      const ja = this.junctionAt(r.a, pts[0]);
      const jb = this.junctionAt(r.b, pts[pts.length - 1]);
      const speed = (r.road.speed || DEFAULT_SPEED[r.road.cls] || 30) / 3.6;
      const mk = (from: Junction, to: Junction, poly: Polyline): DirRoad => {
        const d: DirRoad = {
          id: this.roads.length,
          seg,
          from: from.id,
          to: to.id,
          center: poly,
          lanes: [],
          cls: r.road.cls,
          rank: CLASS_RANK[r.road.cls] ?? 1,
          speed,
          circular: !!r.road.circular,
          name: r.road.name,
          signalGroup: -1,
        };
        this.roads.push(d);
        from.outgoing.push(d);
        to.incoming.push(d);
        if (d.circular) {
          from.circular = true;
          to.circular = true;
        }
        return d;
      };
      seg.forward = mk(ja, jb, center);
      if (!r.road.oneway) seg.backward = mk(jb, ja, new Polyline([...pts].reverse()));
      (seg as { _lanes?: [number, number] })._lanes = [f, b];
    }
  }

  /** Direction leaving junction `j` along segment `seg`. */
  private leaveDir(seg: Segment, j: number): Vec2 {
    const c = seg.center;
    if (seg.a === j) return norm(sub(c.sampleAt(Math.min(4, c.length)).p, c.pts[0]));
    return norm(sub(c.sampleAt(Math.max(0, c.length - 4)).p, c.pts[c.pts.length - 1]));
  }

  private computeTrims(): void {
    for (const j of this.junctions.values()) {
      const segs = this.segments.filter((s) => s.a === j.id || s.b === j.id);
      const degree = segs.length;
      for (const s of segs) {
        const ext = Math.max(s.rightW, s.leftW);
        const len = s.center.length;
        let trim: number;
        if (degree <= 1) trim = 0;
        else {
          const dS = this.leaveDir(s, j.id);
          let need = 0;
          for (const o of segs) {
            if (o === s) continue;
            const dO = this.leaveDir(o, j.id);
            const cosA = dot(dS, dO);
            const sinA = Math.sqrt(Math.max(0, 1 - cosA * cosA));
            if (cosA < -0.85) {
              need = Math.max(need, degree === 2 ? 0.8 : 1.5);
              continue;
            }
            const oExt = Math.max(o.rightW, o.leftW);
            need = Math.max(need, Math.min(45, (oExt + ext * Math.abs(cosA)) / Math.max(sinA, 0.3)) + 1.2);
          }
          trim = Math.min(need, len * 0.42);
          if (degree === 2 && trim < 0.8) trim = Math.min(0.8, len * 0.3);
        }
        if (s.a === j.id) s.trimA = trim;
        if (s.b === j.id) s.trimB = trim;
      }
    }
  }

  private buildLanes(): void {
    for (const seg of this.segments) {
      const [f, b] = (seg as { _lanes?: [number, number] })._lanes ?? [1, 1];
      const mkLanes = (road: DirRoad | null, n: number, forward: boolean): void => {
        if (!road) return;
        const pts = road.center.pts;
        const s0 = forward ? seg.trimA : seg.trimB;
        const s1 = road.center.length - (forward ? seg.trimB : seg.trimA);
        for (let k = 0; k < n; k++) {
          const offset = seg.oneway ? ((n - 1) / 2 - k) * LANE_W : (n - k - 0.5) * LANE_W;
          const off = new Polyline(offsetPolyline(pts, offset));
          // Cut by centreline arc length so all lanes of a road start and end together.
          const scaleF = off.length / Math.max(road.center.length, 1e-3);
          const lanePts = slicePolyline(off, s0 * scaleF, s1 * scaleF, 1.0);
          const lane: Lane = {
            id: this.lanes.length,
            road,
            k,
            offset,
            poly: new Polyline(lanePts),
            out: [],
            in: [],
            left: null,
            right: null,
            sink: false,
            source: false,
          };
          road.lanes.push(lane);
          this.lanes.push(lane);
        }
        for (const l of road.lanes) {
          l.left = road.lanes[l.k + 1] ?? null;
          l.right = road.lanes[l.k - 1] ?? null;
        }
        const toJ = this.junctions.get(road.to);
        const fromJ = this.junctions.get(road.from);
        if (toJ?.boundary) for (const l of road.lanes) l.sink = true;
        if (fromJ?.boundary) for (const l of road.lanes) l.source = true;
      };
      mkLanes(seg.forward, f, true);
      mkLanes(seg.backward, b, false);
    }
  }

  private makeConnector(from: Lane, to: Lane, turn: Turn, j: Junction): Connector {
    const p0 = from.poly.pts[from.poly.pts.length - 1];
    const p3 = to.poly.pts[0];
    const dIn = from.poly.sampleAt(from.poly.length).dir;
    const dOut = to.poly.sampleAt(0).dir;
    const d = dist(p0, p3);
    let pts: Vec2[];
    if (d < 0.2) pts = [p0, add(p0, scale(dIn, 0.2))];
    else {
      const h = Math.max(0.3, d / (turn === "uturn" ? 1.4 : 2.6));
      const p1 = add(p0, scale(dIn, h));
      const p2 = sub(p3, scale(dOut, h));
      const n = Math.max(3, Math.ceil((d * 1.3) / 0.5));
      pts = [];
      for (let i = 0; i <= n; i++) pts.push(cubicBezier(p0, p1, p2, p3, i / n));
    }
    const c: Connector = {
      id: this.connectors.length,
      from,
      to,
      poly: new Polyline(pts),
      turn,
      junction: j,
      conflicts: [],
      priority: 0,
    };
    from.out.push(c);
    to.in.push(c);
    j.connectors.push(c);
    this.connectors.push(c);
    return c;
  }

  private buildConnectors(): void {
    for (const j of this.junctions.values()) {
      if (j.boundary) continue;
      const deadEnd = j.incoming.length + j.outgoing.length > 0 && new Set([...j.incoming, ...j.outgoing].map((r) => r.seg)).size === 1;
      for (const inRoad of j.incoming) {
        const dIn = inRoad.lanes[0].poly.sampleAt(inRoad.lanes[0].poly.length).dir;
        const options = j.outgoing
          .filter((o) => deadEnd || o.seg !== inRoad.seg)
          .map((o) => {
            const dOut = o.lanes[0].poly.sampleAt(0).dir;
            const theta = Math.atan2(cross(dIn, dOut), dot(dIn, dOut));
            return { o, theta };
          })
          .filter((x) => deadEnd || Math.abs(x.theta) < 2.7);
        if (options.length === 0) {
          if (!deadEnd) for (const l of inRoad.lanes) l.sink = true;
          continue;
        }
        const nI = inRoad.lanes.length;
        const made = new Set<Lane>();
        const proportional = (o: DirRoad, turn: Turn): void => {
          const nO = o.lanes.length;
          for (const l of inRoad.lanes) {
            const kO = nI === 1 ? 0 : Math.round((l.k * (nO - 1)) / (nI - 1));
            this.makeConnector(l, o.lanes[Math.min(nO - 1, kO)], turn, j);
            made.add(l);
          }
        };
        for (const { o, theta } of options) {
          const turn: Turn = deadEnd ? "uturn" : Math.abs(theta) < 0.6 ? "straight" : theta > 0 ? "right" : "left";
          const exitRing = inRoad.circular && !o.circular;
          if (options.length === 1 || turn === "straight" || (inRoad.circular && o.circular)) {
            proportional(o, turn);
          } else if (turn === "right" || exitRing) {
            this.makeConnector(inRoad.lanes[0], o.lanes[0], turn, j);
            made.add(inRoad.lanes[0]);
            if (exitRing && nI >= 2 && o.lanes.length >= 2) {
              this.makeConnector(inRoad.lanes[1], o.lanes[1], turn, j);
              made.add(inRoad.lanes[1]);
            }
          } else if (turn === "left") {
            this.makeConnector(inRoad.lanes[nI - 1], o.lanes[o.lanes.length - 1], turn, j);
            made.add(inRoad.lanes[nI - 1]);
          } else {
            this.makeConnector(inRoad.lanes[nI - 1], o.lanes[o.lanes.length - 1], turn, j);
            made.add(inRoad.lanes[nI - 1]);
          }
        }
        // Every lane needs a way out; give orphans the gentlest option.
        const gentle = [...options].sort((a, b) => Math.abs(a.theta) - Math.abs(b.theta))[0];
        for (const l of inRoad.lanes) {
          if (made.has(l)) continue;
          const nO = gentle.o.lanes.length;
          const kO = nI === 1 ? 0 : Math.round((l.k * (nO - 1)) / (nI - 1));
          const turn: Turn = Math.abs(gentle.theta) < 0.6 ? "straight" : gentle.theta > 0 ? "right" : "left";
          this.makeConnector(l, gentle.o.lanes[Math.min(nO - 1, kO)], turn, j);
        }
      }
    }
    for (const c of this.connectors) {
      const turnScore = c.turn === "straight" ? 3 : c.turn === "right" ? 2 : c.turn === "left" ? 1 : 0;
      c.priority = (c.from.road.circular ? 1000 : 0) + c.from.road.rank * 10 + turnScore;
    }
  }

  private buildConflicts(): void {
    for (const j of this.junctions.values()) {
      const cs = j.connectors;
      const samples = cs.map((c) => {
        const out: { p: Vec2; s: number }[] = [];
        for (let s = 0; s <= c.poly.length + 1e-6; s += 0.8) out.push({ p: c.poly.sampleAt(s).p, s });
        return out;
      });
      for (let a = 0; a < cs.length; a++) {
        for (let b = a + 1; b < cs.length; b++) {
          const ca = cs[a];
          const cb = cs[b];
          if (ca.from === cb.from) continue;
          const sameRoad = ca.from.road === cb.from.road;
          let best = Infinity;
          let sa = 0;
          let sb = 0;
          for (const pa of samples[a]) {
            for (const pb of samples[b]) {
              const d = dist2(pa.p, pb.p);
              if (d < best) {
                best = d;
                sa = pa.s;
                sb = pb.s;
              }
            }
          }
          const threshold = sameRoad ? 1.2 : 2.2;
          if (best > threshold * threshold) continue;
          if (sameRoad && (sa < 1 || sb < 1)) continue;
          ca.conflicts.push({ other: cb, s: sa, sOther: sb });
          cb.conflicts.push({ other: ca, s: sb, sOther: sa });
        }
      }
    }
  }

  private buildSignals(signals: MapData["signals"]): void {
    const candidates = [...this.junctions.values()].filter((j) => {
      if (j.boundary || j.circular) return false;
      if (j.incoming.length + j.outgoing.length < 3) return false;
      return signals.some(([, x, y]) => dist2({ x, y }, j.pos) < 28 * 28);
    });
    // Cluster junction nodes that belong to one physical intersection.
    const used = new Set<Junction>();
    for (const j of candidates) {
      if (used.has(j)) continue;
      const cluster: Junction[] = [j];
      used.add(j);
      for (let i = 0; i < cluster.length; i++) {
        for (const o of candidates) {
          if (!used.has(o) && dist(o.pos, cluster[i].pos) < 36) {
            used.add(o);
            cluster.push(o);
          }
        }
      }
      const ids = new Set(cluster.map((c) => c.id));
      const bearings: number[] = [];
      const approaches: DirRoad[] = [];
      for (const cj of cluster) {
        for (const r of cj.incoming) {
          if (ids.has(r.from)) continue;
          approaches.push(r);
          bearings.push(this.bearingMod(r));
        }
      }
      if (approaches.length === 0) continue;
      const groups: number[] = [];
      for (const b of bearings) {
        if (!groups.some((g) => this.bearingDiff(g, b) < 0.6)) groups.push(b);
      }
      if (groups.length < 2) groups.push((groups[0] + Math.PI / 2) % Math.PI);
      const main = Math.max(...approaches.map((r) => r.rank));
      const ctrl: SignalController = {
        id: this.controllers.length,
        junctions: cluster,
        groups,
        offset: ((j.id * 7919) % 97) / 97,
        green: main >= 4 ? 17 : 13,
        yellow: 3,
        allRed: 2,
      };
      this.controllers.push(ctrl);
      for (const cj of cluster) cj.controller = ctrl;
      approaches.forEach((r, i) => {
        let bestG = 0;
        let bestD = Infinity;
        ctrl.groups.forEach((g, gi) => {
          const d = this.bearingDiff(g, bearings[i]);
          if (d < bestD) {
            bestD = d;
            bestG = gi;
          }
        });
        r.signalGroup = bestG;
      });
    }
  }

  private bearingMod(r: DirRoad): number {
    const l = r.lanes[0].poly;
    const a = angleOf(norm(sub(l.pts[l.pts.length - 1], l.sampleAt(Math.max(0, l.length - 8)).p)));
    return ((a % Math.PI) + Math.PI) % Math.PI;
  }

  private bearingDiff(a: number, b: number): number {
    const d = Math.abs(a - b) % Math.PI;
    return Math.min(d, Math.PI - d);
  }

  private buildCrossings(crossings: MapData["crossings"]): void {
    for (const [id, x, y] of crossings) {
      // Crossing nodes are usually on separately mapped footways, so match them to the nearest segment.
      const p = { x, y };
      let seg: Segment | null = null;
      let s = 0;
      let bestD = Infinity;
      for (const cand of this.segments) {
        const known = cand.nodeS.get(id);
        const pr = known !== undefined ? { s: known, dist: 0 } : cand.center.project(p);
        const limit = Math.max(cand.rightW, cand.leftW) + 4;
        if (pr.dist < limit && pr.dist < bestD) {
          bestD = pr.dist;
          seg = cand;
          s = pr.s;
        }
      }
      if (!seg) continue;
      const len = seg.center.length;
      const lo = Math.min(seg.trimA + 1.5, len / 2);
      const hi = Math.max(len - seg.trimB - 1.5, len / 2);
      s = Math.min(hi, Math.max(lo, s));
      const segRef = seg;
      if (this.crossings.some((c) => c.seg === segRef && dist(c.pos, segRef.center.sampleAt(s).p) < 7)) continue;
      const smp = seg.center.sampleAt(s);
      let controller: SignalController | null = null;
      let group = -1;
      for (const ctrl of this.controllers) {
        if (ctrl.junctions.some((j) => dist(j.pos, smp.p) < 40)) {
          controller = ctrl;
          const b = ((angleOf(smp.dir) % Math.PI) + Math.PI) % Math.PI;
          let bestD = Infinity;
          ctrl.groups.forEach((g, gi) => {
            const d = this.bearingDiff(g, b);
            if (d < bestD) {
              bestD = d;
              group = gi;
            }
          });
          break;
        }
      }
      this.crossings.push({
        id: this.crossings.length,
        pos: smp.p,
        dir: smp.dir,
        halfWidth: Math.max(seg.rightW, seg.leftW),
        seg,
        controller,
        group,
      });
    }
  }

  private buildJunctionOutlines(): void {
    for (const j of this.junctions.values()) {
      if (j.boundary) continue;
      const pts: Vec2[] = [];
      const segs = this.segments.filter((s) => s.a === j.id || s.b === j.id);
      for (const s of segs) {
        const atA = s.a === j.id;
        const sArc = atA ? s.trimA : s.center.length - s.trimB;
        const smp = s.center.sampleAt(sArc);
        const r = { x: -smp.dir.y, y: smp.dir.x };
        pts.push(add(smp.p, scale(r, s.rightW + 0.3)), add(smp.p, scale(r, -(s.leftW + 0.3))));
      }
      if (pts.length >= 3) j.outline = convexHull(pts);
    }
  }

  /** Some footprints in the data (roofs, underground structures) cover the carriageway; drop them. */
  private dropBuildingsOnRoads(): void {
    const near = new SpatialIndex<Lane>(10);
    for (const l of this.lanes) for (let s = 0; s <= l.poly.length; s += 2) near.add(l.poly.sampleAt(s).p, l);
    const keep = this.buildings.filter((b) => {
      let minX = Infinity;
      let minY = Infinity;
      let maxX = -Infinity;
      let maxY = -Infinity;
      for (const p of b.pts) {
        minX = Math.min(minX, p.x);
        minY = Math.min(minY, p.y);
        maxX = Math.max(maxX, p.x);
        maxY = Math.max(maxY, p.y);
      }
      let inside = 0;
      let onRoad = 0;
      const step = Math.max(1.5, Math.sqrt(((maxX - minX) * (maxY - minY)) / 400));
      for (let x = minX; x <= maxX; x += step) {
        for (let y = minY; y <= maxY; y += step) {
          const p = { x, y };
          if (!pointInPolygon(p, b.pts)) continue;
          inside++;
          if (near.query(p, LANE_W * 0.6).length > 0) onRoad++;
        }
      }
      return inside === 0 || onRoad / inside < 0.12;
    });
    this.buildings.length = 0;
    this.buildings.push(...keep);
  }

  // ---------------------------------------------------------------------------
  // Queries

  signalColor(ctrl: SignalController, group: number, t: number): SignalColor {
    const per = ctrl.green + ctrl.yellow + ctrl.allRed;
    const cycle = per * ctrl.groups.length;
    const tt = (((t + ctrl.offset * cycle) % cycle) + cycle) % cycle;
    const active = Math.floor(tt / per);
    if (active !== group) return "R";
    const within = tt - active * per;
    if (within < ctrl.green) return "G";
    if (within < ctrl.green + ctrl.yellow) return "Y";
    return "R";
  }

  /** Seconds until the given group turns green (0 if green now). */
  timeToGreen(ctrl: SignalController, group: number, t: number): number {
    const per = ctrl.green + ctrl.yellow + ctrl.allRed;
    const cycle = per * ctrl.groups.length;
    const tt = (((t + ctrl.offset * cycle) % cycle) + cycle) % cycle;
    const start = group * per;
    const d = (start - tt + cycle) % cycle;
    return this.signalColor(ctrl, group, t) === "G" ? 0 : d;
  }

  /** Signal at the end of lane `l`, or null when unsignalised. */
  laneSignal(l: Lane, t: number): SignalColor | null {
    const j = this.junctions.get(l.road.to);
    if (!j?.controller || l.road.signalGroup < 0) return null;
    return this.signalColor(j.controller, l.road.signalGroup, t);
  }

  /** Pedestrian signal state for a crossing: true when pedestrians may cross. */
  crossingOpen(c: Crossing, t: number): boolean {
    if (!c.controller) return true;
    return this.signalColor(c.controller, c.group, t) === "R";
  }

  nearest(p: Vec2, radius: number): { lane: Lane | null; conn: Connector | null; s: number; d: number }[] {
    return this.laneIndex.query(p, radius).map((e) => ({ ...e.v, d: dist(e.p, p) }));
  }

  /** Closest lane (not connector) point whose direction agrees with `heading`. */
  matchLane(p: Vec2, heading: number, maxDist: number, maxAngle: number): { lane: Lane; s: number; lateral: number } | null {
    let best: { lane: Lane; s: number; lateral: number } | null = null;
    let bestScore = Infinity;
    const seen = new Set<Lane>();
    for (const e of this.laneIndex.query(p, maxDist + 2)) {
      const l = e.v.lane;
      if (!l || seen.has(l)) continue;
      seen.add(l);
      const pr = l.poly.project(p);
      if (pr.dist > maxDist) continue;
      const dir = l.poly.sampleAt(pr.s).dir;
      const dAng = Math.abs(wrapAngle(angleOf(dir) - heading));
      if (dAng > maxAngle) continue;
      if (pr.s <= 0.01 && dot(sub(p, l.poly.pts[0]), dir) < -2) continue;
      if (pr.s >= l.poly.length - 0.01 && dot(sub(p, l.poly.pts[l.poly.pts.length - 1]), dir) > 2) continue;
      const score = pr.dist + dAng * 3;
      if (score < bestScore) {
        bestScore = score;
        best = { lane: l, s: pr.s, lateral: pr.lateral };
      }
    }
    return best;
  }

  /** Closest connector point whose direction agrees with `heading`. */
  matchConnector(p: Vec2, heading: number, maxDist: number, maxAngle: number): { conn: Connector; s: number } | null {
    let best: { conn: Connector; s: number } | null = null;
    let bestScore = Infinity;
    const seen = new Set<Connector>();
    for (const e of this.laneIndex.query(p, maxDist + 2)) {
      const c = e.v.conn;
      if (!c || seen.has(c)) continue;
      seen.add(c);
      const pr = c.poly.project(p);
      if (pr.dist > maxDist) continue;
      const dAng = Math.abs(wrapAngle(angleOf(c.poly.sampleAt(pr.s).dir) - heading));
      if (dAng > maxAngle) continue;
      const score = pr.dist + dAng * 3;
      if (score < bestScore) {
        bestScore = score;
        best = { conn: c, s: pr.s };
      }
    }
    return best;
  }

  /** Lanes that participate in routing (reachable interior lanes). */
  get drivableLanes(): Lane[] {
    return this.lanes.filter((l) => l.poly.length > 3);
  }
}

// -----------------------------------------------------------------------------

export class SpatialIndex<T> {
  private readonly cells = new Map<number, { p: Vec2; v: T }[]>();
  constructor(private readonly size: number) {}
  private key(ix: number, iy: number): number {
    return (ix + 4096) * 8192 + (iy + 4096);
  }
  add(p: Vec2, v: T): void {
    const k = this.key(Math.floor(p.x / this.size), Math.floor(p.y / this.size));
    let c = this.cells.get(k);
    if (!c) {
      c = [];
      this.cells.set(k, c);
    }
    c.push({ p, v });
  }
  query(p: Vec2, r: number): { p: Vec2; v: T }[] {
    const out: { p: Vec2; v: T }[] = [];
    const r2 = r * r;
    const x0 = Math.floor((p.x - r) / this.size);
    const x1 = Math.floor((p.x + r) / this.size);
    const y0 = Math.floor((p.y - r) / this.size);
    const y1 = Math.floor((p.y + r) / this.size);
    for (let ix = x0; ix <= x1; ix++) {
      for (let iy = y0; iy <= y1; iy++) {
        const c = this.cells.get(this.key(ix, iy));
        if (!c) continue;
        for (const e of c) if (dist2(e.p, p) <= r2) out.push(e);
      }
    }
    return out;
  }
}

/**
 * 1 m rasters of the map: building occupancy with a distance transform, and the road surface.
 * Used by the free-space planner and for vehicle-vs-building contact.
 */
export class MapGrid {
  readonly res = 1;
  readonly w: number;
  readonly h: number;
  readonly x0: number;
  readonly y0: number;
  /** 1 where a building occupies the cell. */
  readonly building: Uint8Array;
  /** 1 on the carriageway (lanes, connectors, junction areas). */
  readonly road: Uint8Array;
  /** Distance in metres to the nearest building cell (capped). */
  readonly clearance: Float32Array;

  constructor(net: RoadNetwork) {
    const b = net.bounds;
    this.x0 = b.minX - 20;
    this.y0 = b.minY - 20;
    this.w = Math.ceil(b.maxX - b.minX + 40);
    this.h = Math.ceil(b.maxY - b.minY + 40);
    this.building = new Uint8Array(this.w * this.h);
    this.road = new Uint8Array(this.w * this.h);
    for (const bd of net.buildings) this.fillPolygon(bd.pts, this.building);
    for (const j of net.junctions.values()) if (j.outline.length >= 3) this.fillPolygon(j.outline, this.road);
    const stamp = (poly: Polyline, r: number): void => {
      for (let s = 0; s <= poly.length; s += 0.5) {
        const p = poly.sampleAt(s).p;
        const ci = Math.floor((p.x - this.x0) / this.res);
        const cj = Math.floor((p.y - this.y0) / this.res);
        const rr = Math.ceil(r);
        for (let dj = -rr; dj <= rr; dj++) {
          for (let di = -rr; di <= rr; di++) {
            if (di * di + dj * dj > r * r) continue;
            const i = ci + di;
            const jj = cj + dj;
            if (i >= 0 && jj >= 0 && i < this.w && jj < this.h) this.road[jj * this.w + i] = 1;
          }
        }
      }
    };
    for (const l of net.lanes) stamp(l.poly, LANE_W / 2 + 0.2);
    for (const c of net.connectors) stamp(c.poly, LANE_W / 2);
    // Roads take precedence over building footprints that overlap them in the data.
    for (let i = 0; i < this.building.length; i++) if (this.road[i]) this.building[i] = 0;
    this.clearance = this.distanceTransform();
  }

  private fillPolygon(pts: Vec2[], target: Uint8Array): void {
    let minY = Infinity;
    let maxY = -Infinity;
    for (const p of pts) {
      minY = Math.min(minY, p.y);
      maxY = Math.max(maxY, p.y);
    }
    const j0 = Math.max(0, Math.floor((minY - this.y0) / this.res));
    const j1 = Math.min(this.h - 1, Math.ceil((maxY - this.y0) / this.res));
    for (let j = j0; j <= j1; j++) {
      const y = this.y0 + (j + 0.5) * this.res;
      const xs: number[] = [];
      for (let a = 0, bI = pts.length - 1; a < pts.length; bI = a++) {
        const p = pts[a];
        const q = pts[bI];
        if (p.y > y !== q.y > y) xs.push(p.x + ((y - p.y) * (q.x - p.x)) / (q.y - p.y));
      }
      xs.sort((u, v) => u - v);
      for (let k = 0; k + 1 < xs.length; k += 2) {
        const i0 = Math.max(0, Math.ceil((xs[k] - this.x0) / this.res - 0.5));
        const i1 = Math.min(this.w - 1, Math.floor((xs[k + 1] - this.x0) / this.res - 0.5));
        for (let i = i0; i <= i1; i++) target[j * this.w + i] = 1;
      }
    }
  }

  /** Two-pass chamfer distance transform (3-4 weights) in metres, capped at 30 m. */
  private distanceTransform(): Float32Array {
    const { w, h } = this;
    const INF = 1e6;
    const d = new Float32Array(w * h);
    for (let i = 0; i < w * h; i++) d[i] = this.building[i] ? 0 : INF;
    const a = 1;
    const bb = Math.SQRT2;
    for (let j = 0; j < h; j++) {
      for (let i = 0; i < w; i++) {
        const k = j * w + i;
        let v = d[k];
        if (i > 0) v = Math.min(v, d[k - 1] + a);
        if (j > 0) {
          v = Math.min(v, d[k - w] + a);
          if (i > 0) v = Math.min(v, d[k - w - 1] + bb);
          if (i < w - 1) v = Math.min(v, d[k - w + 1] + bb);
        }
        d[k] = v;
      }
    }
    for (let j = h - 1; j >= 0; j--) {
      for (let i = w - 1; i >= 0; i--) {
        const k = j * w + i;
        let v = d[k];
        if (i < w - 1) v = Math.min(v, d[k + 1] + a);
        if (j < h - 1) {
          v = Math.min(v, d[k + w] + a);
          if (i < w - 1) v = Math.min(v, d[k + w + 1] + bb);
          if (i > 0) v = Math.min(v, d[k + w - 1] + bb);
        }
        d[k] = Math.min(v, 30);
      }
    }
    return d;
  }

  private idx(p: Vec2): number {
    const i = Math.floor((p.x - this.x0) / this.res);
    const j = Math.floor((p.y - this.y0) / this.res);
    if (i < 0 || j < 0 || i >= this.w || j >= this.h) return -1;
    return j * this.w + i;
  }

  /** Distance to the nearest building (0 outside the map). */
  clearanceAt(p: Vec2): number {
    const k = this.idx(p);
    return k < 0 ? 0 : this.clearance[k];
  }

  onRoad(p: Vec2): boolean {
    const k = this.idx(p);
    return k >= 0 && this.road[k] === 1;
  }

  inside(p: Vec2, margin = 0): boolean {
    return p.x > this.x0 + 20 + margin && p.y > this.y0 + 20 + margin && p.x < this.x0 + this.w - 20 - margin && p.y < this.y0 + this.h - 20 - margin;
  }
}
