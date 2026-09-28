import {
  type Agent,
  CAR_HALF_LEN,
  IDM_DEFAULT,
  type IdmParams,
  type Obstacle,
  curvatureSpeed,
  idmAccel,
  idmFree,
  scanPath,
} from "./agents";
import {
  Polyline,
  type PathSample,
  type Vec2,
  add,
  angleOf,
  clamp,
  cross,
  dist,
  dot,
  fromAngle,

  scale,
  smoothstep,
  wrapAngle,
} from "./geometry";
import { planRoute } from "./routing";
import type { NpcCar, Traffic } from "./traffic";
import {
  DIRS,
  type Edge,
  SPEED_LIMIT,
  type SignalColor,
  type Turn,
  World,
  laneOffset,
  requiredLane,
  turnOf,
} from "./world";

export const WHEELBASE = 2.9;
const MAX_STEER = 0.6;

export type DriveMode = "manual" | "fsd";

export type IntentKind =
  | "manual"
  | "cruise"
  | "curve"
  | "follow"
  | "light"
  | "yellow"
  | "ped"
  | "obstacle"
  | "lanechange"
  | "waitlane"
  | "arriving"
  | "arrived"
  | "emergency";

export interface ManualInput {
  throttle: number;
  brake: number;
  steer: number;
}

export interface StopMarker {
  s: number;
  node: number;
  dirIn: number;
  turn: Turn;
  ri: number;
  pos: Vec2;
  dir: Vec2;
}

export interface DriveContext {
  world: World;
  traffic: Traffic;
  agents: readonly Agent[];
  t: number;
}

export interface Destination {
  edge: Edge;
  u: number;
  pos: Vec2;
}

export interface Localization {
  edge: Edge;
  lane: number;
  u: number;
}

const PATH_KIND_EDGE = 0;
const PATH_KIND_CONN = 1;
const PATH_KIND_LC = 2;

export class Ego implements Agent {
  readonly id = 0;
  readonly kind = "car" as const;
  pos: Vec2;
  heading: number;
  v = 0;
  vel: Vec2 = { x: 0, y: 0 };
  steer = 0;
  accel = 0;
  mode: DriveMode = "manual";

  route: Edge[] = [];
  routeIdx = 0;
  lane = 0;
  dest: Destination | null = null;

  path: Polyline | null = null;
  private pathRi: number[] = [];
  private pathLane: number[] = [];
  private pathKind: number[] = [];
  stops: StopMarker[] = [];
  destS = 0;
  pathS = 0;
  private pathIdx = 0;
  lcEndS = -1;
  lcTarget = -1;
  private lcFrom = 0;
  private lcBlockedTime = 0;
  private lcCooldown = 0;
  private committedStopRi = -1;

  intent: IntentKind = "manual";
  leadId = -1;
  activeStop: { marker: StopMarker; color: SignalColor } | null = null;
  blinker = 0;
  arrived = false;
  offRoute = false;

  constructor(pos: Vec2, heading: number) {
    this.pos = pos;
    this.heading = heading;
  }

  get fwd(): Vec2 {
    return fromAngle(this.heading);
  }

  /** Matches the vehicle pose to a lane whose direction agrees with the heading. */
  localize(world: World): Localization | null {
    let best: Localization | null = null;
    let bestScore = Infinity;
    for (const e of world.edges) {
      const d = DIRS[e.dir] as Vec2;
      if (dot(d, this.fwd) < 0.6) continue;
      const u = world.edgeCoord(e, this.pos);
      if (u < -16 || u > e.length + 2) continue;
      const lat = world.lateralCoord(e, this.pos);
      if (lat < -1.5 || lat > 8.5) continue;
      const lane = lat < 3.5 ? 1 : 0;
      const outside = u < 0 ? -u : 0;
      const score = Math.abs(lat - laneOffset(lane)) + outside * 0.6;
      if (score < bestScore) {
        bestScore = score;
        best = { edge: e, lane, u };
      }
    }
    return best;
  }

  /** Engages FSD towards `dest`. Returns an error message on failure. */
  engage(world: World, dest: Destination): string | null {
    const loc = this.localize(world);
    if (!loc) return "无法匹配车道，请先将车辆驶入车道并保持行驶方向";
    const route = planRoute(world, loc.edge, Math.max(0, loc.u), dest.edge, dest.u);
    if (!route) return "无法规划到目的地的路线";
    this.route = route;
    this.routeIdx = 0;
    this.lane = loc.lane;
    this.dest = dest;
    this.mode = "fsd";
    this.arrived = false;
    this.offRoute = false;
    this.committedStopRi = -1;
    this.rebuildPath(world, null);
    return null;
  }

  disengage(): void {
    this.mode = "manual";
    this.intent = "manual";
    this.activeStop = null;
    this.leadId = -1;
    this.lcEndS = -1;
    this.lcTarget = -1;
  }

  /** Rebuilds the reference path from the current position; optionally starts a lane change now. */
  rebuildPath(world: World, lcTarget: number | null): void {
    const pts: Vec2[] = [];
    const ri: number[] = [];
    const lanes: number[] = [];
    const kinds: number[] = [];
    const push = (p: Vec2, r: number, l: number, k: number): void => {
      const last = pts[pts.length - 1];
      if (last && dist(last, p) < 0.05) return;
      pts.push(p);
      ri.push(r);
      lanes.push(l);
      kinds.push(k);
    };
    const pushLane = (e: Edge, r: number, lane: number, u0: number, u1Raw: number, step: number): void => {
      const u1 = Math.max(u0, u1Raw);
      for (let u = u0; u < u1; u += step) push(world.lanePoint(e, laneOffset(lane), u), r, lane, PATH_KIND_EDGE);
      push(world.lanePoint(e, laneOffset(lane), u1), r, lane, PATH_KIND_EDGE);
    };
    const pushChange = (e: Edge, r: number, from: number, to: number, u0: number, u1: number): void => {
      const o0 = laneOffset(from);
      const o1 = laneOffset(to);
      for (let u = u0; u <= u1; u += 0.5) {
        const k = smoothstep((u - u0) / (u1 - u0));
        push(world.lanePoint(e, o0 + (o1 - o0) * k, u), r, to, PATH_KIND_LC);
      }
    };

    const lastIdx = this.route.length - 1;
    const cur = this.route[this.routeIdx];
    if (!cur) return;
    const u0 = clamp(world.edgeCoord(cur, this.pos), 0, cur.length);
    let lane = this.lane;
    const endU = (i: number, e: Edge): number => (i === lastIdx && this.dest ? Math.min(e.length, this.dest.u + 14) : e.length);

    let lcEndIndex = -1;
    this.lcFrom = lane;
    if (lcTarget !== null && lcTarget !== lane) {
      const L = clamp(this.v * 2.6, 9, 30);
      const u1 = Math.min(u0 + L, cur.length - 1);
      pushChange(cur, this.routeIdx, lane, lcTarget, u0, u1);
      lcEndIndex = pts.length - 1;
      lane = lcTarget;
      pushLane(cur, this.routeIdx, lane, u1 + 1, endU(this.routeIdx, cur), 1);
    } else {
      pushLane(cur, this.routeIdx, lane, u0, endU(this.routeIdx, cur), 1);
    }

    const stopIdx: { idx: number; node: number; dirIn: number; turn: Turn; ri: number }[] = [];
    for (let i = this.routeIdx; i < lastIdx; i++) {
      const a = this.route[i] as Edge;
      const b = this.route[i + 1] as Edge;
      const turn = turnOf(a.dir, b.dir) as Turn;
      stopIdx.push({ idx: pts.length - 1, node: a.to, dirIn: a.dir, turn, ri: i });
      const nextLane = turn === "left" ? 1 : turn === "right" ? 0 : lane;
      const conn = world.connector(a, lane, b, nextLane);
      for (const p of conn.pts) push(p, i, nextLane, PATH_KIND_CONN);
      lane = nextLane;
      const c = this.route[i + 2];
      const need = c && i + 1 < lastIdx ? requiredLane(turnOf(b.dir, c.dir) as Turn) : null;
      if (need !== null && need !== lane) {
        pushLane(b, i + 1, lane, 0, 4, 1);
        pushChange(b, i + 1, lane, need, 4.5, 26);
        lane = need;
        pushLane(b, i + 1, lane, 27, endU(i + 1, b), 1);
      } else {
        pushLane(b, i + 1, lane, 0, endU(i + 1, b), 1);
      }
    }

    const poly = new Polyline(pts);
    this.path = poly;
    this.pathRi = ri;
    this.pathLane = lanes;
    this.pathKind = kinds;
    this.stops = stopIdx.map((m) => {
      const s = poly.cum[m.idx] as number;
      const smp = poly.sampleAt(s);
      return { s, node: m.node, dirIn: m.dirIn, turn: m.turn, ri: m.ri, pos: smp.p, dir: smp.dir };
    });
    this.lcEndS = lcEndIndex >= 0 ? (poly.cum[lcEndIndex] as number) : -1;
    this.lcTarget = lcEndIndex >= 0 ? lane : -1;
    if (this.dest) {
      const destPoint = world.lanePoint(this.route[lastIdx] as Edge, laneOffset(lane), this.dest.u);
      this.destS = poly.project(destPoint).s;
    }
    const proj = poly.project(this.pos, 0, 12);
    this.pathS = proj.s;
    this.pathIdx = proj.idx;
  }

  private laneFree(ctx: DriveContext, e: Edge, lane: number, u: number): boolean {
    if (!ctx.traffic.laneFree(e, lane, u, this.v, this.id)) return false;
    for (const a of ctx.agents) {
      if (a.kind !== "ped") continue;
      const lat = ctx.world.lateralCoord(e, a.pos);
      const du = ctx.world.edgeCoord(e, a.pos) - u;
      if (du > -3 && du < 30 && Math.abs(lat - laneOffset(lane)) < 2.5) return false;
    }
    return true;
  }

  update(dt: number, ctx: DriveContext, manual: ManualInput): void {
    if (this.mode === "manual") this.updateManual(dt, manual);
    else this.updateFsd(dt, ctx);
  }

  /** Kinematic bicycle model referenced at the vehicle centre. */
  private integrate(dt: number): void {
    const beta = Math.atan(0.5 * Math.tan(this.steer));
    const course = fromAngle(this.heading + beta);
    this.pos = add(this.pos, scale(course, this.v * dt));
    this.heading = wrapAngle(this.heading + (this.v / WHEELBASE) * Math.cos(beta) * Math.tan(this.steer) * dt);
    this.vel = scale(course, this.v);
  }

  private updateManual(dt: number, input: ManualInput): void {
    this.intent = "manual";
    this.blinker = 0;
    let a = -0.25 - 0.004 * this.v * Math.abs(this.v);
    if (input.throttle > 0) a = this.v < -0.2 ? 6 : 3.2 * input.throttle;
    if (input.brake > 0) a = this.v > 0.3 ? -7.5 * input.brake : -2.5;
    if (input.throttle === 0 && input.brake === 0 && Math.abs(this.v) < 0.3) {
      this.v = 0;
      a = 0;
    }
    if (this.v < 0 && input.brake === 0 && input.throttle === 0) a = 1.5;
    this.accel = a;
    this.v = clamp(this.v + a * dt, -5, 32);
    const steerLimit = MAX_STEER / (1 + (Math.abs(this.v) / 11) ** 2 * 0.8);
    const target = input.steer * steerLimit;
    const rate = input.steer === 0 ? 2.4 : 1.6;
    this.steer += clamp(target - this.steer, -rate * dt, rate * dt);
    this.integrate(dt);
  }

  private nextStop(): StopMarker | null {
    for (const m of this.stops) {
      if (m.s - this.pathS - CAR_HALF_LEN > -1.0) return m;
    }
    return null;
  }

  private updateFsd(dt: number, ctx: DriveContext): void {
    const { world } = ctx;
    if (!this.path || !this.dest) {
      this.disengage();
      return;
    }
    let proj = this.path.project(this.pos, this.pathIdx, 30);
    if (proj.dist > 7) {
      const err = this.engage(world, this.dest);
      if (err) {
        this.offRoute = true;
        this.disengage();
        return;
      }
      proj = (this.path as Polyline).project(this.pos, this.pathIdx, 30);
    }
    this.pathS = proj.s;
    this.pathIdx = proj.idx;

    const kind = this.pathKind[proj.idx] ?? PATH_KIND_EDGE;
    const ri = this.pathRi[proj.idx] ?? this.routeIdx;
    if (kind !== PATH_KIND_CONN && ri > this.routeIdx) {
      this.routeIdx = ri;
      this.lane = this.pathLane[proj.idx] ?? this.lane;
      this.rebuildPath(world, null);
    } else if (kind !== PATH_KIND_CONN) {
      this.lane = this.pathLane[proj.idx] ?? this.lane;
    }
    if (this.lcEndS >= 0 && this.pathS > this.lcEndS) {
      this.lcEndS = -1;
      this.lcTarget = -1;
    }
    const path = this.path as Polyline;

    // ---- Longitudinal planning -------------------------------------------------
    const p: IdmParams = { ...IDM_DEFAULT, aMax: 2.4, T: 1.4, s0: 3.2 };
    const lookahead = clamp(this.v * 4 + 22, 35, 90);
    const samples: PathSample[] = [];
    for (let rel = 0.5; rel <= lookahead; rel += 1) {
      const smp = path.sampleAt(this.pathS + rel);
      samples.push({ p: smp.p, dir: smp.dir, kappa: smp.kappa, s: rel });
    }
    const vLimit = SPEED_LIMIT;
    const vCurve = curvatureSpeed(samples, 0, 2.2, 2.0, vLimit);
    let a = idmFree(this.v, Math.min(vLimit, vCurve), p);
    let intent: IntentKind = vCurve < vLimit - 1.5 ? "curve" : "cruise";
    this.leadId = -1;
    this.activeStop = null;

    const obs: Obstacle | null = scanPath(samples, ctx.agents, {
      selfId: this.id,
      tube: 1.1,
      pedMargin: 0.9,
      pedHorizon: 4,
      selfSpeed: this.v,
    });
    let obstructionCar: NpcCar | null = null;
    if (obs) {
      const ao = idmAccel(this.v, vLimit, obs.gap, this.v - Math.max(0, obs.speed), p);
      if (ao < a) {
        a = ao;
        this.leadId = obs.agent.id;
        if (obs.agent.kind === "ped") intent = "ped";
        else {
          const car = ctx.traffic.get(obs.agent.id);
          intent = car && ctx.traffic.isObstruction(car) ? "obstacle" : "follow";
        }
      }
      if (obs.agent.kind === "car") {
        const car = ctx.traffic.get(obs.agent.id);
        const beforeDest = obs.gap + CAR_HALF_LEN < this.destS - this.pathS;
        if (car && ctx.traffic.isObstruction(car) && obs.gap < 45 && beforeDest) obstructionCar = car;
      }
    }

    const stop = this.nextStop();
    let stopAhead: "R" | "Y" | null = null;
    if (stop) {
      const gap = stop.s - this.pathS - CAR_HALF_LEN;
      if (gap < 80 && this.committedStopRi !== stop.ri) {
        const color = world.signal(stop.node, stop.dirIn, stop.turn, ctx.t);
        let mustStop = color === "R";
        if (color === "Y") {
          const brakeDist = (this.v * this.v) / (2 * 3.2);
          mustStop = brakeDist < gap - 0.5;
          if (!mustStop) this.committedStopRi = stop.ri;
        }
        if (color === "G" && gap < 2.5) this.committedStopRi = stop.ri;
        if (gap < 60) this.activeStop = { marker: stop, color };
        if (mustStop) {
          if (gap < 60) stopAhead = color === "Y" ? "Y" : "R";
          const al = idmAccel(this.v, vLimit, gap, this.v, { ...p, s0: 0.9 });
          if (al < a) {
            a = al;
            intent = color === "Y" ? "yellow" : "light";
            this.leadId = -1;
          }
        }
      }
    }

    const destGap = this.destS - this.pathS;
    if (destGap < 70) {
      const ad = idmAccel(this.v, vLimit, Math.max(0.05, destGap), this.v, { ...p, s0: 0.2, T: 0.8 });
      if (ad < a) {
        a = ad;
        intent = "arriving";
      }
    }
    if (a < -4.5 && (intent === "ped" || intent === "follow" || intent === "obstacle")) intent = "emergency";
    if (stopAhead && (intent === "cruise" || intent === "curve" || intent === "follow" || intent === "arriving")) {
      intent = stopAhead === "Y" ? "yellow" : "light";
    }

    a = clamp(a, -9, p.aMax);
    // Rate-limited acceleration for comfort; hard braking bypasses the filter.
    this.accel = a < -4 ? a : this.accel + clamp(a - this.accel, -8 * dt, 3.5 * dt);
    this.v = Math.max(0, this.v + this.accel * dt);
    if (this.v < 0.05 && this.accel < 0) this.v = 0;

    if (Math.abs(destGap) < 2.5 && this.v < 0.3) {
      this.arrived = true;
      intent = "arrived";
    }

    // ---- Lane-change behaviour ------------------------------------------------
    const cur = this.route[this.routeIdx];
    this.lcCooldown = Math.max(0, this.lcCooldown - dt);
    if (this.lcEndS >= 0) {
      this.lcBlockedTime = this.v < 0.2 ? this.lcBlockedTime + dt : 0;
      if (this.lcBlockedTime > 4) {
        // The merge is blocked; return to the original lane and retry later.
        this.lane = this.lcFrom;
        this.rebuildPath(world, null);
        this.lcBlockedTime = 0;
        this.lcCooldown = 5;
      }
    }
    if (cur && kind === PATH_KIND_EDGE && this.lcEndS < 0 && !this.arrived && this.lcCooldown <= 0) {
      const u = world.edgeCoord(cur, this.pos);
      const remaining = cur.length - u;
      const L = clamp(this.v * 2.6, 9, 30);
      const next = this.route[this.routeIdx + 1];
      const need = next ? requiredLane(turnOf(cur.dir, next.dir) as Turn) : null;
      let want: number | null = null;
      let waiting = false;
      if (need !== null && need !== this.lane && remaining > L + 3) {
        want = need;
        waiting = true;
      } else if (obstructionCar && remaining > L + 6) {
        want = 1 - this.lane;
      }
      if (want !== null) {
        if (this.laneFree(ctx, cur, want, u)) {
          this.rebuildPath(world, want);
          intent = "lanechange";
        } else if (waiting && (intent === "cruise" || intent === "follow")) {
          intent = "waitlane";
        }
      }
    }
    if (this.lcEndS >= 0 && intent === "cruise") intent = "lanechange";

    // ---- Lateral control: Stanley at the vehicle centre with curvature feedforward ----
    const activePath = this.path as Polyline;
    const cp = activePath.project(this.pos, this.pathIdx, 30);
    const sRef = cp.s + this.v * 0.25;
    const d1 = activePath.sampleAt(sRef - 1).dir;
    const d2 = activePath.sampleAt(sRef + 1).dir;
    const kappa = Math.atan2(cross(d1, d2), dot(d1, d2)) / 2;
    const beta = Math.asin(clamp((kappa * WHEELBASE) / 2, -0.9, 0.9));
    const steerFF = Math.atan(2 * Math.tan(beta));
    const pathHeading = angleOf(activePath.sampleAt(cp.s).dir);
    const slip = Math.atan(0.5 * Math.tan(this.steer));
    const headingErr = wrapAngle(pathHeading - (this.heading + slip));
    const steerCmd = clamp(steerFF + headingErr + Math.atan2(-1.6 * cp.lateral, this.v + 1.5), -MAX_STEER, MAX_STEER);
    this.steer += clamp(steerCmd - this.steer, -2.5 * dt, 2.5 * dt);
    this.integrate(dt);

    // ---- Signals ------------------------------------------------------------
    let blink = 0;
    if (this.lcEndS >= 0) blink = this.lcTarget === 1 ? -1 : 1;
    else if (kind === PATH_KIND_CONN) {
      const turn = this.stops.find((m) => m.ri === ri)?.turn;
      blink = turn === "left" ? -1 : turn === "right" ? 1 : 0;
    } else if (stop && stop.s - this.pathS < 45) blink = stop.turn === "left" ? -1 : stop.turn === "right" ? 1 : 0;
    this.blinker = blink;
    this.intent = intent;
  }

  /** Remaining route distance and the next manoeuvre, for the navigation card. */
  navInfo(): { turn: Turn | "arrive"; distance: number; remaining: number } | null {
    if (this.mode !== "fsd" || !this.path) return null;
    const stop = this.nextStop();
    const remaining = Math.max(0, this.destS - this.pathS);
    if (!stop) return { turn: "arrive", distance: remaining, remaining };
    return { turn: stop.turn, distance: Math.max(0, stop.s - this.pathS), remaining };
  }

  /** Current heading of the reference path, used to orient the path ribbon. */
  pathHeadingAt(s: number): number {
    if (!this.path) return this.heading;
    return angleOf(this.path.sampleAt(s).dir);
  }
}
