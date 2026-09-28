import {
  type Agent,
  CAR_HALF_LEN,
  CAR_LEN,
  IDM_DEFAULT,
  type IdmParams,
  type Obstacle,
  curvatureSpeed,
  idmAccel,
  idmFree,
  scanPath,
} from "./agents";
import { type PathSample, type Polyline, Rng, type Vec2, angleOf, clamp, dist, dist2 } from "./geometry";
import { type Edge, SPEED_LIMIT, type Turn, World, laneOffset, turnOf } from "./world";

export interface Seg {
  poly: Polyline;
  kind: "edge" | "conn";
  /** For edge segments the edge itself; for connectors the incoming edge. */
  edge: Edge;
  lane: number;
  /** Manoeuvre at the end of an edge segment, or the manoeuvre a connector performs. */
  turn: Turn;
  /** Edge coordinate where this segment starts (non-zero for lane-change segments). */
  u0: number;
  /** Length of the lane-change transition at the start of the segment, 0 if none. */
  lcLength: number;
  lcFrom: number;
}

export type WaitReason = "free" | "light" | "vehicle" | "ped" | "stalled";

/** A vehicle's position on a lane, used for lane-change gap checks. */
export interface Occupant {
  id: number;
  edgeId: number;
  lane: number;
  /** Edge coordinate of the vehicle centre (negative while still in the preceding intersection). */
  u: number;
  v: number;
  obstruction: boolean;
}

/** Samples a chain of segments, returning s values relative to position `s` on the first segment. */
export function sampleSegs(segs: readonly Seg[], s: number, distance: number, step: number): PathSample[] {
  const out: PathSample[] = [];
  let base = 0;
  let rel = 0.5;
  for (const seg of segs) {
    const L = seg.poly.length;
    while (rel <= distance) {
      const local = s + rel - base;
      if (local > L) break;
      const smp = seg.poly.sampleAt(local);
      out.push({ p: smp.p, dir: smp.dir, kappa: smp.kappa, s: rel });
      rel += step;
    }
    base += L;
    if (rel > distance) break;
  }
  return out;
}

let nextCarId = 1;

export class NpcCar implements Agent {
  readonly id: number;
  readonly kind = "car" as const;
  pos: Vec2 = { x: 0, y: 0 };
  heading = 0;
  v = 0;
  vel: Vec2 = { x: 0, y: 0 };
  accel = 0;
  segs: Seg[];
  s: number;
  v0: number;
  stalled: boolean;
  reason: WaitReason = "free";
  leadId = -1;
  stuckTime = 0;
  committedEdge = -1;
  /** -1 left, 1 right, 0 off. */
  blinker = 0;
  bodyTint: number;

  constructor(segs: Seg[], s: number, v0: number, stalled: boolean, tint: number) {
    this.id = nextCarId++;
    this.segs = segs;
    this.s = s;
    this.v0 = v0;
    this.stalled = stalled;
    this.bodyTint = tint;
    this.syncPose();
  }

  get seg(): Seg {
    const s = this.segs[0];
    if (!s) throw new Error("NPC car without segments");
    return s;
  }

  syncPose(): void {
    const smp = this.seg.poly.sampleAt(this.s);
    this.pos = smp.p;
    this.heading = angleOf(smp.dir);
    this.vel = { x: smp.dir.x * this.v, y: smp.dir.y * this.v };
  }
}

const NPC_IDM: IdmParams = { ...IDM_DEFAULT, s0: 2.8 };

export class Traffic {
  readonly cars: NpcCar[] = [];
  private readonly byId = new Map<number, NpcCar>();
  /** Lane occupants that are not NPC cars (the ego vehicle). */
  private external: Occupant[] = [];

  constructor(
    private readonly world: World,
    private readonly rng: Rng,
  ) {}

  get(id: number): NpcCar | undefined {
    return this.byId.get(id);
  }

  private edgeSeg(edge: Edge, lane: number, poly: Polyline, u0: number, lcLength: number, lcFrom: number): Seg {
    return { poly, kind: "edge", edge, lane, turn: "straight", u0, lcLength, lcFrom };
  }

  private planNext(edgeSeg: Seg): Seg[] {
    const succ = this.world.successors(edgeSeg.edge);
    const lane = edgeSeg.lane;
    const allowed = succ.filter((e) => {
      const t = turnOf(edgeSeg.edge.dir, e.dir);
      return t === "straight" || (lane === 0 ? t === "right" : t === "left");
    });
    const pool = allowed.length > 0 ? allowed : succ;
    const weighted: Edge[] = [];
    for (const e of pool) {
      const w = turnOf(edgeSeg.edge.dir, e.dir) === "straight" ? 3 : 2;
      for (let k = 0; k < w; k++) weighted.push(e);
    }
    const next = this.rng.pick(weighted);
    const turn = turnOf(edgeSeg.edge.dir, next.dir) as Turn;
    edgeSeg.turn = turn;
    const nextLane = turn === "left" ? 1 : turn === "right" ? 0 : lane;
    const conn: Seg = {
      poly: this.world.connector(edgeSeg.edge, lane, next, nextLane),
      kind: "conn",
      edge: edgeSeg.edge,
      lane: nextLane,
      turn,
      u0: 0,
      lcLength: 0,
      lcFrom: nextLane,
    };
    return [conn, this.edgeSeg(next, nextLane, next.lanes[nextLane as 0 | 1], 0, 0, nextLane)];
  }

  private tint(): number {
    const palette = [0x9aa0a8, 0xa7adb5, 0x8c9299, 0xb3b8bf, 0x959ba3];
    return this.rng.pick(palette);
  }

  spawnOnEdge(edge: Edge, lane: number, u: number, stalled = false): NpcCar {
    const first = this.edgeSeg(edge, lane, edge.lanes[lane as 0 | 1], 0, 0, lane);
    const segs = [first, ...this.planNext(first)];
    const car = new NpcCar(segs, u, this.rng.range(0.78, 0.98) * SPEED_LIMIT, stalled, this.tint());
    car.v = stalled ? 0 : Math.min(car.v0, 8);
    car.reason = stalled ? "stalled" : "free";
    car.syncPose();
    this.cars.push(car);
    this.byId.set(car.id, car);
    return car;
  }

  /** Spawns on a random lane position that is clear of the given points. */
  spawnRandom(avoid: readonly Vec2[], minDist: number): NpcCar | null {
    for (let attempt = 0; attempt < 40; attempt++) {
      const edge = this.rng.pick(this.world.edges);
      const lane = this.rng.int(0, 2);
      const u = this.rng.range(4, edge.length - 6);
      const p = this.world.lanePoint(edge, laneOffset(lane), u);
      if (avoid.some((a) => dist(a, p) < minDist)) continue;
      if (this.cars.some((c) => dist2(c.pos, p) < 12 * 12)) continue;
      return this.spawnOnEdge(edge, lane, u);
    }
    return null;
  }

  remove(car: NpcCar): void {
    const idx = this.cars.indexOf(car);
    if (idx >= 0) this.cars.splice(idx, 1);
    this.byId.delete(car.id);
  }

  occupant(car: NpcCar): Occupant | null {
    const s0 = car.segs[0];
    if (!s0) return null;
    const obstruction = this.isObstruction(car);
    if (s0.kind === "edge") return { id: car.id, edgeId: s0.edge.id, lane: s0.lane, u: s0.u0 + car.s, v: car.v, obstruction };
    const s1 = car.segs[1];
    if (s1 && s1.kind === "edge") {
      return { id: car.id, edgeId: s1.edge.id, lane: s1.lane, u: car.s - s0.poly.length, v: car.v, obstruction };
    }
    return null;
  }

  /** True when the car is stopped for a reason other than a signal queue or a pedestrian. */
  isObstruction(car: NpcCar): boolean {
    if (car.stalled) return true;
    if (car.stuckTime < 3) return false;
    let cur: NpcCar | undefined = car;
    for (let depth = 0; depth < 12 && cur; depth++) {
      if (cur.reason === "light" || cur.reason === "ped") return false;
      if (cur.stalled) return true;
      cur = this.byId.get(cur.leadId);
    }
    return true;
  }

  /** Gap acceptance for merging into `lane` at edge coordinate `u` with speed `v`. */
  laneFree(edge: Edge, lane: number, u: number, v: number, selfId: number): boolean {
    const check = (o: Occupant): boolean => {
      if (o.id === selfId || o.edgeId !== edge.id || o.lane !== lane) return true;
      const du = o.u - u;
      if (du >= 0) {
        if (o.obstruction && du < 70) return false;
        return du >= Math.max(7, v) + Math.max(0, v - o.v) * 2.2 + CAR_LEN;
      }
      const rearGap = o.v < 0.5 ? 1.5 : Math.max(6, o.v * 0.9);
      return -du >= rearGap + Math.max(0, o.v - v) * 2.6 + CAR_LEN;
    };
    for (const car of this.cars) {
      const o = this.occupant(car);
      if (o && !check(o)) return false;
    }
    return this.external.every(check);
  }

  update(dt: number, t: number, agents: readonly Agent[], egoPos: Vec2, ensureCount: number, external: Occupant[]): void {
    this.external = external;
    for (const car of this.cars) this.step(car, dt, t, agents);

    // Recycle vehicles that have been blocked for a long time, preferably out of the driver's view.
    for (const car of [...this.cars]) {
      const d = dist(car.pos, egoPos);
      if (!car.stalled && ((car.stuckTime > 30 && d > 90) || car.stuckTime > 90)) this.remove(car);
      else if (car.stalled && d > 260) this.remove(car);
    }
    let guard = 0;
    while (this.cars.filter((c) => !c.stalled).length < ensureCount && guard++ < 4) {
      if (!this.spawnRandom([egoPos], 70)) break;
    }
    while (this.cars.filter((c) => !c.stalled).length > ensureCount) {
      const far = this.cars.filter((c) => !c.stalled).sort((a, b) => dist2(b.pos, egoPos) - dist2(a.pos, egoPos))[0];
      if (!far) break;
      this.remove(far);
    }
  }

  private tryLaneChange(car: NpcCar): boolean {
    const seg = car.seg;
    if (seg.kind !== "edge" || seg.lcLength > 0) return false;
    const u = seg.u0 + car.s;
    const L = clamp(car.v * 2.6, 9, 25);
    if (seg.edge.length - u < L + 6) return false;
    const target = 1 - seg.lane;
    if (!this.laneFree(seg.edge, target, u, car.v, car.id)) return false;
    const poly = this.world.laneChangePoly(seg.edge, seg.lane, target, u, L);
    const next = this.edgeSeg(seg.edge, target, poly, u, L, seg.lane);
    car.segs = [next, ...this.planNext(next)];
    car.s = 0;
    return true;
  }

  private step(car: NpcCar, dt: number, t: number, agents: readonly Agent[]): void {
    if (car.stalled) {
      car.v = 0;
      car.accel = 0;
      car.vel = { x: 0, y: 0 };
      car.reason = "stalled";
      car.blinker = 2;
      return;
    }
    const p = NPC_IDM;
    const lookahead = Math.min(70, Math.max(28, car.v * 3.5 + 16));
    let samples = sampleSegs(car.segs, car.s, lookahead, 1.0);
    let obs: Obstacle | null = scanPath(samples, agents, { selfId: car.id, tube: 1.0, pedMargin: 0.5, pedHorizon: 2.5, selfSpeed: car.v });

    if (obs && obs.agent.kind === "car" && obs.gap < 25) {
      const lead = this.byId.get(obs.agent.id);
      if (lead && this.isObstruction(lead) && this.tryLaneChange(car)) {
        samples = sampleSegs(car.segs, car.s, lookahead, 1.0);
        obs = scanPath(samples, agents, { selfId: car.id, tube: 1.0, pedMargin: 0.5, pedHorizon: 2.5, selfSpeed: car.v });
      }
    }

    const vCurve = curvatureSpeed(samples, 0, 2.4, 2.5, car.v0);
    let a = idmFree(car.v, Math.min(car.v0, vCurve), p);
    let reason: WaitReason = "free";
    let leadId = -1;
    if (obs) {
      const ao = idmAccel(car.v, car.v0, obs.gap, car.v - Math.max(0, obs.speed), p);
      if (ao < a) {
        a = ao;
        reason = obs.agent.kind === "ped" ? "ped" : "vehicle";
        leadId = obs.agent.id;
      }
    }

    const seg = car.seg;
    if (seg.kind === "edge") {
      const stopGap = seg.poly.length - car.s - CAR_HALF_LEN;
      if (car.committedEdge !== seg.edge.id && stopGap > -1.0 && stopGap < 60) {
        const color = this.world.signal(seg.edge.to, seg.edge.dir, seg.turn, t);
        let mustStop = color === "R";
        if (color === "Y") {
          const brakeDist = (car.v * car.v) / (2 * 3.5);
          mustStop = brakeDist < stopGap - 0.5;
          if (!mustStop) car.committedEdge = seg.edge.id;
        }
        if (color === "G" && stopGap < 3) car.committedEdge = seg.edge.id;
        if (mustStop) {
          const al = idmAccel(car.v, car.v0, stopGap, car.v, { ...p, s0: 0.8 });
          if (al < a) {
            a = al;
            reason = "light";
            leadId = -1;
          }
        }
      }
      if (seg.lcLength > 0 && car.s < seg.lcLength) car.blinker = seg.lane > seg.lcFrom ? -1 : 1;
      else car.blinker = seg.turn === "left" && stopGap < 40 ? -1 : seg.turn === "right" && stopGap < 40 ? 1 : 0;
    } else {
      car.blinker = seg.turn === "left" ? -1 : seg.turn === "right" ? 1 : 0;
    }

    a = Math.max(-9, Math.min(p.aMax, a));
    car.accel = a;
    car.v = Math.max(0, car.v + a * dt);
    car.s += car.v * dt;
    car.reason = car.v < 0.3 ? reason : "free";
    car.leadId = leadId;
    car.stuckTime = car.v < 0.3 && reason !== "light" && reason !== "ped" ? car.stuckTime + dt : 0;

    while (car.s > car.seg.poly.length) {
      const done = car.segs.shift();
      if (!done) break;
      car.s -= done.poly.length;
      car.committedEdge = -1;
      const cur = car.segs[0];
      if (!cur) break;
      if (cur.kind === "edge" && car.segs.length < 3) car.segs.push(...this.planNext(cur));
    }
    car.syncPose();
  }
}
