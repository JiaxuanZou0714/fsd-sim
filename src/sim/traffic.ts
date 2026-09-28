import { type Agent, type IdmParams, type Obstacle, curvatureSpeed, idmAccel, idmFree, scanPath } from "./agents";
import { type PathSample, Polyline, Rng, type Vec2, angleOf, clamp, dist, dist2, lerp, smoothstep } from "./geometry";
import { type Connector, type Lane, RoadNetwork } from "./map/network";
import { type DriverProfile, type VehicleKind, type VehicleSpec, VEHICLES, pickKind, randomProfile } from "./vehicles";

export interface PathSeg {
  poly: Polyline;
  lane: Lane | null;
  conn: Connector | null;
  /** Lane coordinate where this segment starts (non-zero for lane-change segments). */
  u0: number;
  lcFrom: Lane | null;
  lcLen: number;
}

export type WaitReason = "free" | "light" | "vehicle" | "ped" | "yield" | "stop" | "stalled";

interface StopTask {
  lane: Lane;
  s: number;
  dwell: number;
  kind: "bus" | "pickup" | "delivery";
  phase: "approach" | "dwell";
}

let nextVehicleId = 1;

export class NpcVehicle implements Agent {
  readonly id = nextVehicleId++;
  readonly kind = "vehicle" as const;
  readonly vkind: VehicleKind;
  readonly spec: VehicleSpec;
  readonly profile: DriverProfile;
  readonly length: number;
  readonly width: number;
  readonly color: number;
  pos: Vec2 = { x: 0, y: 0 };
  heading = 0;
  v = 0;
  vel: Vec2 = { x: 0, y: 0 };
  accel = 0;
  blinker = 0;
  segs: PathSeg[];
  s: number;
  v0: number;
  stalled = false;
  reason: WaitReason = "free";
  leadId = -1;
  stuckTime = 0;
  yieldTime = 0;
  ignoreYieldUntil = 0;
  committedLane = -1;
  task: StopTask | null = null;
  lateral = 0;

  constructor(kind: VehicleKind, profile: DriverProfile, segs: PathSeg[], s: number, color: number) {
    this.vkind = kind;
    this.spec = VEHICLES[kind];
    this.profile = profile;
    this.length = this.spec.length;
    this.width = this.spec.width;
    this.color = color;
    this.segs = segs;
    this.s = s;
    this.v0 = 10;
  }

  get seg(): PathSeg {
    return this.segs[0];
  }

  /** Lane the vehicle is travelling in, or will enter after its current connector. */
  get lane(): Lane | null {
    return this.seg.lane ?? this.segs[1]?.lane ?? null;
  }

  syncPose(): void {
    const smp = this.seg.poly.sampleAt(this.s);
    const lat = this.vkind === "bike" ? (this.seg.conn ? 0.5 : 0.95) : 0;
    this.lateral += (lat - this.lateral) * 0.2;
    this.pos = { x: smp.p.x - smp.dir.y * this.lateral, y: smp.p.y + smp.dir.x * this.lateral };
    this.heading = angleOf(smp.dir);
    this.vel = { x: smp.dir.x * this.v, y: smp.dir.y * this.v };
  }
}

/** A vehicle occupying (or about to occupy) a lane, for gap acceptance. */
export interface Occupant {
  id: number;
  lane: number;
  u: number;
  v: number;
  length: number;
  obstruction: boolean;
}

/** A vehicle heading into a connector, for right-of-way decisions at conflict points. */
export interface Approach {
  agent: Agent;
  /** Distance from the vehicle centre to the connector start (negative once on it). */
  dStart: number;
  /** The vehicle is held at a red signal before this connector. */
  held: boolean;
}

export interface EgoExposure {
  agent: Agent;
  occupant: Occupant | null;
  approaches: { conn: Connector; dStart: number }[];
}

export class Traffic {
  readonly vehicles: NpcVehicle[] = [];
  private readonly byId = new Map<number, NpcVehicle>();
  private occupants = new Map<number, Occupant[]>();
  private approaches = new Map<number, Approach[]>();
  private readonly busStops = new Map<number, number>();
  targetCount: number;
  t = 0;

  constructor(
    private readonly net: RoadNetwork,
    private readonly rng: Rng,
    count: number,
  ) {
    this.targetCount = count;
    for (const l of net.lanes) {
      if (l.k === 0 && l.road.rank >= 4 && !l.road.circular && l.poly.length > 45 && rng.chance(0.5)) {
        this.busStops.set(l.id, l.poly.length * 0.55);
      }
    }
  }

  get(id: number): NpcVehicle | undefined {
    return this.byId.get(id);
  }

  busStopsList(): { lane: Lane; s: number }[] {
    return [...this.busStops.entries()].map(([id, s]) => ({ lane: this.net.lanes[id], s }));
  }

  // ---------------------------------------------------------------------------
  // Path planning

  private laneSeg(l: Lane): PathSeg {
    return { poly: l.poly, lane: l, conn: null, u0: 0, lcFrom: null, lcLen: 0 };
  }

  private chooseConnector(l: Lane): Connector | null {
    if (l.out.length === 0) return null;
    const weights = l.out.map((c) => {
      if (c.to.poly.length < 1) return 0.05;
      const w = c.turn === "straight" ? 3 : c.turn === "uturn" ? 0.15 : 1.4;
      // Avoid steering the whole population off the map.
      return c.to.sink && c.to.out.length === 0 ? w * 0.35 : w;
    });
    const total = weights.reduce((a, b) => a + b, 0);
    let r = this.rng.next() * total;
    for (let i = 0; i < l.out.length; i++) {
      r -= weights[i];
      if (r <= 0) return l.out[i];
    }
    return l.out[l.out.length - 1];
  }

  private extend(car: NpcVehicle): void {
    while (car.segs.length < 3) {
      const last = car.segs[car.segs.length - 1];
      if (last.conn) {
        car.segs.push(this.laneSeg(last.conn.to));
        continue;
      }
      const lane = last.lane;
      if (!lane) break;
      const c = this.chooseConnector(lane);
      if (!c) break;
      car.segs.push({ poly: c.poly, lane: null, conn: c, u0: 0, lcFrom: null, lcLen: 0 });
    }
  }

  private replanAfter(car: NpcVehicle, idx: number): void {
    car.segs.length = idx + 1;
    this.extend(car);
  }

  // ---------------------------------------------------------------------------
  // Spawning

  spawn(lane: Lane, s: number, kind: VehicleKind = pickKind(this.rng), stalled = false): NpcVehicle {
    const spec = VEHICLES[kind];
    const profile = randomProfile(this.rng, kind);
    const car = new NpcVehicle(kind, profile, [this.laneSeg(lane)], s, this.rng.pick(spec.colors));
    car.v0 = Math.min(spec.vMax, lane.road.speed * spec.speedFactor * profile.speedMul);
    car.stalled = stalled;
    car.v = stalled ? 0 : Math.min(car.v0, 7);
    this.extend(car);
    car.syncPose();
    this.vehicles.push(car);
    this.byId.set(car.id, car);
    return car;
  }

  private isFree(lane: Lane, s: number, clearance: number, avoid: readonly Vec2[], avoidDist: number): boolean {
    const p = lane.poly.sampleAt(s).p;
    if (avoid.some((a) => dist(a, p) < avoidDist)) return false;
    for (const c of this.vehicles) if (dist2(c.pos, p) < clearance * clearance) return false;
    return true;
  }

  spawnRandom(avoid: readonly Vec2[], avoidDist: number): NpcVehicle | null {
    const lanes = this.net.lanes;
    for (let attempt = 0; attempt < 30; attempt++) {
      const l = this.rng.pick(lanes);
      if (l.poly.length < 12 || l.out.length === 0) continue;
      const s = this.rng.range(3, l.poly.length - 3);
      if (!this.isFree(l, s, 14, avoid, avoidDist)) continue;
      const kind = pickKind(this.rng);
      if (kind === "bike" && l.k !== 0) continue;
      return this.spawn(l, s, kind);
    }
    return null;
  }

  private spawnAtSource(avoid: readonly Vec2[]): void {
    const sources = this.net.lanes.filter((l) => l.source && l.poly.length > 6);
    if (sources.length === 0) return;
    const l = this.rng.pick(sources);
    if (!this.isFree(l, 2, 12, avoid, 40)) return;
    const kind = pickKind(this.rng);
    if (kind === "bike" && l.k !== 0) return;
    this.spawn(l, 2, kind);
  }

  remove(car: NpcVehicle): void {
    const i = this.vehicles.indexOf(car);
    if (i >= 0) this.vehicles.splice(i, 1);
    this.byId.delete(car.id);
  }

  // ---------------------------------------------------------------------------
  // Shared state per step

  isObstruction(car: NpcVehicle): boolean {
    if (car.stalled) return true;
    if (car.task?.phase === "dwell") return true;
    if (car.vkind === "bike") return true;
    if (car.stuckTime < 3) return false;
    let cur: NpcVehicle | undefined = car;
    for (let depth = 0; depth < 10 && cur; depth++) {
      if (cur.reason === "light" || cur.reason === "ped" || cur.reason === "yield") return false;
      if (cur.stalled || cur.task?.phase === "dwell") return true;
      cur = this.byId.get(cur.leadId);
    }
    return true;
  }

  occupant(car: NpcVehicle): Occupant | null {
    const seg = car.seg;
    const obstruction = this.isObstruction(car);
    if (seg.lane) return { id: car.id, lane: seg.lane.id, u: seg.u0 + car.s, v: car.v, length: car.length, obstruction };
    const next = car.segs[1];
    if (next?.lane) return { id: car.id, lane: next.lane.id, u: car.s - seg.poly.length, v: car.v, length: car.length, obstruction };
    return null;
  }

  private rebuildIndices(ego: EgoExposure | null): void {
    this.occupants = new Map();
    this.approaches = new Map();
    const addOcc = (o: Occupant): void => {
      let list = this.occupants.get(o.lane);
      if (!list) {
        list = [];
        this.occupants.set(o.lane, list);
      }
      list.push(o);
    };
    const addApp = (connId: number, a: Approach): void => {
      let list = this.approaches.get(connId);
      if (!list) {
        list = [];
        this.approaches.set(connId, list);
      }
      list.push(a);
    };
    for (const car of this.vehicles) {
      const o = this.occupant(car);
      if (o) addOcc(o);
      let d = -car.s;
      for (let i = 0; i < car.segs.length && d < 70; i++) {
        const seg = car.segs[i];
        if (seg.conn) {
          const prev = car.segs[i - 1]?.lane ?? null;
          const sig = prev ? this.net.laneSignal(prev, this.t) : null;
          const held = i > 0 && sig === "R" && car.committedLane !== prev?.id;
          addApp(seg.conn.id, { agent: car, dStart: d, held });
        }
        d += seg.poly.length;
      }
    }
    if (ego) {
      if (ego.occupant) addOcc(ego.occupant);
      for (const a of ego.approaches) addApp(a.conn.id, { agent: ego.agent, dStart: a.dStart, held: false });
    }
  }

  /** Gap acceptance for merging into `lane` at lane coordinate `u`. */
  laneFree(lane: Lane, u: number, v: number, selfId: number, length: number, gapMul = 1): boolean {
    for (const o of this.occupants.get(lane.id) ?? []) {
      if (o.id === selfId) continue;
      const du = o.u - u;
      const bumpers = (o.length + length) / 2;
      if (du >= 0) {
        if (o.obstruction && du < 60) return false;
        if (du - bumpers < (Math.max(5, v * 0.9) + Math.max(0, v - o.v) * 2.0) * gapMul) return false;
      } else {
        const rear = o.v < 0.5 ? 1.5 : Math.max(5, o.v * 0.9) * gapMul;
        if (-du - bumpers < rear + Math.max(0, o.v - v) * 2.4 * gapMul) return false;
      }
    }
    return true;
  }

  approachesTo(conn: Connector): Approach[] {
    return this.approaches.get(conn.id) ?? [];
  }

  // ---------------------------------------------------------------------------
  // Update

  update(dt: number, t: number, agents: readonly Agent[], egoPos: Vec2, ego: EgoExposure | null): void {
    this.t = t;
    this.rebuildIndices(ego);
    for (const car of this.vehicles) this.step(car, dt, t, agents);

    for (const car of [...this.vehicles]) {
      const d = dist(car.pos, egoPos);
      const atSink = !car.seg.conn && car.seg.lane?.sink && car.s > car.seg.poly.length - 0.5 && car.segs.length === 1;
      if (atSink) this.remove(car);
      else if (!car.stalled && ((car.stuckTime > 40 && d > 90) || car.stuckTime > 120)) this.remove(car);
      else if (car.stalled && d > 260) this.remove(car);
    }
    const moving = this.vehicles.filter((c) => !c.stalled).length;
    if (moving < this.targetCount) {
      if (this.rng.chance(0.5)) this.spawnAtSource([egoPos]);
      else this.spawnRandom([egoPos], 90);
    } else if (moving > this.targetCount + 3) {
      const far = this.vehicles.filter((c) => !c.stalled).sort((a, b) => dist2(b.pos, egoPos) - dist2(a.pos, egoPos))[0];
      if (far && dist(far.pos, egoPos) > 90) this.remove(far);
    }
  }

  private sampleAhead(car: NpcVehicle, distance: number): PathSample[] {
    const out: PathSample[] = [];
    let base = 0;
    let rel = 0.5;
    for (const seg of car.segs) {
      const L = seg.poly.length;
      while (rel <= distance) {
        const local = car.s + rel - base;
        if (local > L) break;
        const smp = seg.poly.sampleAt(local);
        out.push({ p: smp.p, dir: smp.dir, kappa: smp.kappa, s: rel });
        rel += 1;
      }
      base += L;
      if (rel > distance) break;
    }
    return out;
  }

  private tryLaneChange(car: NpcVehicle, prefer: "left" | "right" | "any"): boolean {
    const seg = car.seg;
    const lane = seg.lane;
    if (!lane || seg.lcLen > 0) return false;
    const u = seg.u0 + car.s;
    const L = clamp(car.v * 2.4, 8, 26);
    if (lane.poly.length - u < L + 10) return false;
    const options = prefer === "right" ? [lane.right, lane.left] : [lane.left, lane.right];
    for (const target of options) {
      if (!target || target.poly.length < u + L + 4) continue;
      if (!this.laneFree(target, u, car.v, car.id, car.length, car.profile.gapMul)) continue;
      const pts: Vec2[] = [];
      for (let du = 0; du <= L; du += 0.5) {
        const k = smoothstep(du / L);
        pts.push(lerp(lane.poly.sampleAt(u + du).p, target.poly.sampleAt(u + du).p, k));
      }
      for (let uu = u + L + 1; uu < target.poly.length; uu += 1) pts.push(target.poly.sampleAt(uu).p);
      pts.push(target.poly.pts[target.poly.pts.length - 1]);
      car.segs = [{ poly: new Polyline(pts), lane: target, conn: null, u0: u, lcFrom: lane, lcLen: L }];
      car.s = 0;
      car.task = null;
      this.extend(car);
      return true;
    }
    return false;
  }

  private maybeAssignTask(car: NpcVehicle, lane: Lane): void {
    if (car.task || lane.k !== 0 || lane.poly.length < 35) return;
    if (car.vkind === "bus") {
      const s = this.busStops.get(lane.id);
      if (s !== undefined) car.task = { lane, s, dwell: this.rng.range(8, 14), kind: "bus", phase: "approach" };
    } else if (car.vkind === "taxi" && this.rng.chance(0.18)) {
      car.task = { lane, s: this.rng.range(15, lane.poly.length - 12), dwell: this.rng.range(6, 12), kind: "pickup", phase: "approach" };
    } else if (car.vkind === "van" && this.rng.chance(0.22)) {
      car.task = { lane, s: this.rng.range(15, lane.poly.length - 12), dwell: this.rng.range(20, 40), kind: "delivery", phase: "approach" };
    }
  }

  private step(car: NpcVehicle, dt: number, t: number, agents: readonly Agent[]): void {
    if (car.stalled) {
      car.v = 0;
      car.accel = 0;
      car.vel = { x: 0, y: 0 };
      car.reason = "stalled";
      car.blinker = 2;
      return;
    }
    const spec = car.spec;
    const p: IdmParams = { aMax: spec.aMax, bComf: spec.bComf, T: spec.headway * car.profile.headwayMul, s0: spec.s0 };
    const lookahead = Math.min(80, Math.max(30, car.v * 3.8 + 18));
    let samples = this.sampleAhead(car, lookahead);
    const scanOpts = { self: car, margin: 0.45, pedMargin: 0.6, horizon: 2.5 };
    let obs: Obstacle | null = scanPath(samples, agents, scanOpts);

    // Lane changes: around obstructions, or to overtake when the driver is inclined to.
    if (obs && obs.agent.kind === "vehicle" && obs.gap < 30 && car.seg.lane) {
      const lead = this.byId.get(obs.agent.id);
      const obstructed = lead ? this.isObstruction(lead) : false;
      const slow = car.profile.overtakes && car.vkind !== "bus" && car.vkind !== "truck" && obs.agent.v < car.v0 - 4 && obs.gap < 22;
      if ((obstructed || slow) && this.tryLaneChange(car, "left")) {
        samples = this.sampleAhead(car, lookahead);
        obs = scanPath(samples, agents, scanOpts);
      }
    }

    const seg = car.seg;
    const vLimit = seg.lane ? Math.min(car.v0, spec.vMax) : car.v0;
    const vCurve = curvatureSpeed(samples, car.vkind === "moto" ? 3.0 : 2.3, 2.5, vLimit);
    let a = idmFree(car.v, Math.min(vLimit, vCurve), p);
    let reason = "free" as WaitReason;
    let leadId = -1;
    if (obs) {
      const ao = idmAccel(car.v, vLimit, obs.gap, car.v - Math.max(0, obs.speed), p);
      if (ao < a) {
        a = ao;
        reason = obs.agent.kind === "ped" ? "ped" : "vehicle";
        leadId = obs.agent.id;
      }
    }

    const stopAt = (gap: number, why: WaitReason, s0: number): void => {
      const al = idmAccel(car.v, vLimit, gap, car.v, { ...p, s0 });
      if (al < a) {
        a = al;
        reason = why;
        leadId = -1;
      }
    };

    // Signal at the end of the current lane.
    let distToLaneEnd = Infinity;
    if (seg.lane) {
      distToLaneEnd = seg.poly.length - car.s;
      const gap = distToLaneEnd - car.length / 2;
      const color = this.net.laneSignal(seg.lane, t);
      if (color && car.committedLane !== seg.lane.id && gap > -1 && gap < 70) {
        let mustStop = color === "R";
        if (color === "Y") {
          mustStop = (car.v * car.v) / (2 * 3.5) < gap - 0.5;
          if (!mustStop) car.committedLane = seg.lane.id;
        }
        if (color === "G" && gap < 2) car.committedLane = seg.lane.id;
        if (mustStop) stopAt(gap, "light", 0.8);
      }
    }

    // Right of way at conflict points of the next connector.
    const connIdx = seg.conn ? 0 : car.segs[1]?.conn ? 1 : -1;
    if (connIdx >= 0 && t > car.ignoreYieldUntil) {
      const conn = car.segs[connIdx].conn as Connector;
      const dStart = connIdx === 0 ? -car.s : distToLaneEnd;
      const y = this.yieldGap(car, conn, dStart);
      if (y !== null) stopAt(y, "yield", 0.5);
    }

    // Scheduled stops (bus stops, pick-ups, deliveries).
    if (car.task) {
      const task = car.task;
      if (seg.lane !== task.lane) {
        if (!car.segs.some((s) => s.lane === task.lane)) car.task = null;
      } else if (task.phase === "approach") {
        const gap = task.s - car.s - car.length / 2;
        if (gap < -2) car.task = null;
        else {
          stopAt(Math.max(0.05, gap + 1), "stop", 0.2);
          if (gap < 1.5 && car.v < 0.3) task.phase = "dwell";
        }
      } else {
        task.dwell -= dt;
        a = Math.min(a, -3);
        reason = "stop";
        if (task.dwell <= 0) car.task = null;
      }
    }

    a = clamp(a, -8, p.aMax);
    car.accel = a;
    car.v = Math.max(0, car.v + a * dt);
    car.s += car.v * dt;
    car.reason = car.v < 0.3 ? reason : "free";
    car.leadId = leadId;
    const waiting = car.v < 0.3 && reason !== "light" && reason !== "ped" && reason !== "stop";
    car.stuckTime = waiting ? car.stuckTime + dt : 0;
    car.yieldTime = car.v < 0.3 && reason === "yield" ? car.yieldTime + dt : 0;
    if (car.yieldTime > 9) {
      car.ignoreYieldUntil = t + 3;
      car.yieldTime = 0;
    }

    // Turn signals: lane change, upcoming turn, hazards while stopped for a task.
    const cur = car.seg;
    const next = car.segs[1];
    if (car.task?.phase === "dwell" && car.task.kind !== "bus") car.blinker = 2;
    else if (cur.lcLen > 0 && car.s < cur.lcLen && cur.lcFrom && cur.lane) car.blinker = cur.lane.k > cur.lcFrom.k ? -1 : 1;
    else if (cur.conn) car.blinker = cur.conn.turn === "left" ? -1 : cur.conn.turn === "right" ? 1 : 0;
    else if (next?.conn && distToLaneEnd < 45) car.blinker = next.conn.turn === "left" ? -1 : next.conn.turn === "right" ? 1 : 0;
    else car.blinker = car.task?.phase === "approach" ? 1 : 0;

    while (car.s > car.seg.poly.length && car.segs.length > 1) {
      const done = car.segs.shift() as PathSeg;
      car.s -= done.poly.length;
      car.committedLane = -1;
      const now = car.seg;
      if (now.lane) {
        this.replanAfter(car, 0);
        this.maybeAssignTask(car, now.lane);
      } else this.extend(car);
    }
    if (car.s > car.seg.poly.length) car.s = car.seg.poly.length;
    car.syncPose();
  }

  /** Front gap to the point where `car` must wait for a vehicle with right of way, or null. */
  private yieldGap(car: NpcVehicle, conn: Connector, dStart: number): number | null {
    if (conn.conflicts.length === 0) return null;
    const signalised = !!conn.junction.controller;
    const prio = (c: Connector): number => (signalised ? c.priority % 10 : c.priority);
    const myPrio = prio(conn);
    const myEta = (x: number): number => Math.max(0, x) / Math.max(car.v, 2);
    let best: number | null = null;
    for (const cf of conn.conflicts) {
      const dSelf = dStart + cf.s;
      if (dSelf < -car.length / 2) continue;
      const brake = (car.v * car.v) / (2 * 4) + 0.5;
      if (dSelf - car.length / 2 < brake && car.v > 1) continue;
      for (const f of this.approachesTo(cf.other)) {
        if (f.agent.id === car.id || f.held) continue;
        const dFoe = f.dStart + cf.sOther;
        if (dFoe < -f.agent.length / 2 - 1) continue;
        const foeEta = Math.max(0, dFoe) / Math.max(f.agent.v, 2);
        const occupying = dFoe < f.agent.length / 2 + 2.5 && dFoe > -f.agent.length / 2 - 1;
        const foePrio = prio(cf.other);
        const foeFirst = foePrio > myPrio || (foePrio === myPrio && (foeEta < myEta(dSelf) - 0.4 || (Math.abs(foeEta - myEta(dSelf)) <= 0.4 && f.agent.id < car.id)));
        const relevant = occupying || (foeFirst && foeEta < myEta(dSelf) + 3 && foeEta < 7);
        if (!relevant) continue;
        // Wait at the lane end if not yet in the junction, otherwise just before the conflict point.
        const stop = dStart > 0 ? dStart - car.length / 2 - 0.3 : dSelf - car.length / 2 - 2.2;
        best = best === null ? stop : Math.min(best, stop);
      }
    }
    return best === null ? null : Math.max(0.05, best);
  }
}
