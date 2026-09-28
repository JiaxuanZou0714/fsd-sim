import { type Agent, footprint } from "./agents";
import { Polyline, type Vec2, add, angleOf, clamp, cross, dist, dot, fromAngle, scale, sub, wrapAngle } from "./geometry";
import type { Connector, Lane, RoadNetwork } from "./map/network";
import { type FreePathPoint, hybridAStar, splitByDirection } from "./planner/hybridAstar";
import { LatticePlanner, type Plan } from "./planner/lattice";
import { PRED_DT, type Predictor } from "./planner/prediction";
import { type Reference, buildReference } from "./planner/reference";
import { type RouteStep, goalForPoint, planRoute } from "./planner/route";
import type { EgoExposure, Occupant, Traffic } from "./traffic";

export const WHEELBASE = 2.85;
const MAX_STEER = 0.6;
const MAX_CURVATURE = Math.tan(MAX_STEER) / WHEELBASE;

export type DriveMode = "manual" | "fsd";
export type FsdState = "lane" | "free" | "blocked" | "arrived";

export type Intent =
  | "manual"
  | "cruise"
  | "follow"
  | "ped"
  | "bike"
  | "yield"
  | "obstacle"
  | "red"
  | "yellow"
  | "curve"
  | "arriving"
  | "arrived"
  | "emergency"
  | "freespace"
  | "reverse"
  | "searching"
  | "noroute";

export type LateralIntent = "keep" | "lanechange" | "overtake" | "nudge";

export interface ManualInput {
  throttle: number;
  brake: number;
  steer: number;
}

export interface Destination {
  lane: Lane;
  s: number;
  pos: Vec2;
}

export interface DriveContext {
  net: RoadNetwork;
  traffic: Traffic;
  predictor: Predictor;
  agents: readonly Agent[];
  t: number;
}

interface FreeManeuver {
  path: FreePathPoint[];
  runs: { poly: Polyline; dir: number }[];
  run: number;
  goalLane: Lane | null;
}

export class Ego implements Agent {
  readonly id = 0;
  readonly kind = "vehicle" as const;
  readonly vkind = "car" as const;
  readonly length = 4.7;
  readonly width = 1.9;
  pos: Vec2;
  heading: number;
  v = 0;
  vel: Vec2 = { x: 0, y: 0 };
  steer = 0;
  accel = 0;
  blinker = 0;
  mode: DriveMode = "manual";
  fsd: FsdState = "lane";

  dest: Destination | null = null;
  route: RouteStep[] | null = null;
  ref: Reference | null = null;
  plan: Plan | null = null;
  planPath: Polyline | null = null;
  private planAge = 0;
  private planTimer = 0;
  private routeTimer = 0;
  private refHint = 0;
  private routeKey = "";
  private blockedTimer = 0;
  private prevDT: number | null = null;
  private staticBlockTime = 0;
  private freeHazardTime = 0;
  private noRouteTime = 0;
  private freeExclude: Lane | null = null;
  free: FreeManeuver | null = null;
  private lastLoc: { lane: Lane; s: number } | null = null;
  private lastConn: { conn: Connector; s: number } | null = null;
  private lastAgents: readonly Agent[] = [];
  private freeAheadOnly = false;
  /** Why free-space driving was last entered (diagnostics). */
  freeReason = "";

  intent: Intent = "manual";
  lateral: LateralIntent = "keep";
  leadId = -1;
  arrived = false;
  bumped = false;
  message = "";

  private readonly lattice = new LatticePlanner();

  constructor(pos: Vec2, heading: number) {
    this.pos = pos;
    this.heading = heading;
  }

  get fwd(): Vec2 {
    return fromAngle(this.heading);
  }

  // ---------------------------------------------------------------------------
  // Mode changes

  engage(net: RoadNetwork, dest: Destination): void {
    this.dest = dest;
    this.mode = "fsd";
    this.arrived = false;
    this.route = null;
    this.ref = null;
    this.plan = null;
    this.free = null;
    this.fsd = "lane";
    this.routeKey = "";
    this.planTimer = 0;
    this.routeTimer = 0;
    this.message = "";
    if (!this.localize(net)) this.enterFree(net, false);
  }

  disengage(): void {
    this.mode = "manual";
    this.intent = "manual";
    this.lateral = "keep";
    this.plan = null;
    this.planPath = null;
    this.free = null;
    this.leadId = -1;
  }

  /** Lane (or junction connector) matching the current pose and direction of travel. */
  localize(net: RoadNetwork): { lane: Lane; s: number } | null {
    const m = net.matchLane(this.pos, this.heading, 2.9, 0.85);
    if (m) {
      this.lastLoc = { lane: m.lane, s: m.s };
      this.lastConn = null;
      return this.lastLoc;
    }
    const c = net.matchConnector(this.pos, this.heading, 2.6, 0.85);
    if (c) {
      this.lastConn = c;
      this.lastLoc = { lane: c.conn.to, s: 0 };
      return this.lastLoc;
    }
    this.lastLoc = null;
    this.lastConn = null;
    return null;
  }

  exposure(): EgoExposure {
    let occupant: Occupant | null = null;
    if (this.lastLoc) {
      const u = this.lastConn ? this.lastConn.s - this.lastConn.conn.poly.length : this.lastLoc.s;
      occupant = { id: this.id, lane: this.lastLoc.lane.id, u, v: this.v, length: this.length, obstruction: false };
    }
    const approaches: { conn: Connector; dStart: number }[] = [];
    if (this.mode === "fsd" && this.ref && this.plan) {
      const s0 = this.plan.s0;
      for (const sp of this.ref.spans) {
        const d = sp.s0 - s0;
        if (d < -(sp.s1 - sp.s0) - 3 || d > 70) continue;
        approaches.push({ conn: sp.conn, dStart: d });
      }
    }
    return { agent: this, occupant, approaches };
  }

  // ---------------------------------------------------------------------------
  // Update

  update(dt: number, ctx: DriveContext, input: ManualInput): void {
    this.bumped = false;
    this.lastAgents = ctx.agents;
    if (this.mode === "manual") {
      this.localize(ctx.net);
      this.updateManual(dt, input, ctx.net);
      return;
    }
    if (this.fsd === "free" || this.fsd === "blocked") this.updateFree(dt, ctx);
    else this.updateLane(dt, ctx);
  }

  private integrate(dt: number, net: RoadNetwork): void {
    const beta = Math.atan(0.5 * Math.tan(this.steer));
    const prevPos = this.pos;
    const prevHeading = this.heading;
    const course = fromAngle(this.heading + beta);
    this.pos = add(this.pos, scale(course, this.v * dt));
    this.heading = wrapAngle(this.heading + (this.v / WHEELBASE) * Math.cos(beta) * Math.tan(this.steer) * dt);
    this.vel = scale(course, this.v);
    // Buildings are solid.
    for (const c of footprint(this.pos, this.heading, this.length, this.width)) {
      if (net.grid.clearanceAt(c) < this.width / 2 - 0.1) {
        this.pos = prevPos;
        this.heading = prevHeading;
        this.v = 0;
        this.vel = { x: 0, y: 0 };
        this.bumped = true;
        break;
      }
    }
  }

  private updateManual(dt: number, input: ManualInput, net: RoadNetwork): void {
    this.intent = "manual";
    this.lateral = "keep";
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
    this.integrate(dt, net);
  }

  // ---- Structured driving ------------------------------------------------------------------------

  private replanRoute(net: RoadNetwork, loc: { lane: Lane; s: number }): void {
    const dest = this.dest as Destination;
    const route = planRoute(loc.lane, loc.s, goalForPoint(dest.lane, dest.s));
    if (!route) {
      this.route = null;
      return;
    }
    const key = route.map((r) => `${r.lane.id}${r.via[0]}`).join(",");
    const refStale = !this.ref || !this.ref.lanesUsed.has(loc.lane) || (this.plan && Math.abs(this.plan.d0) > 4.5);
    this.route = route;
    if (key !== this.routeKey || refStale) {
      this.routeKey = key;
      const prefix = this.lastConn ? this.lastConn : null;
      this.ref = buildReference(net, route, loc.s, this.v, dest);
      if (prefix) this.ref = this.withConnectorPrefix(net, route, dest, prefix);
      this.refHint = 0;
      this.prevDT = null;
    }
  }

  /** Reference that starts on the connector the vehicle is currently in. */
  private withConnectorPrefix(net: RoadNetwork, route: RouteStep[], dest: Destination, c: { conn: Connector; s: number }): Reference {
    const conn = c.conn;
    const fromLane = conn.from;
    const steps: RouteStep[] = [{ lane: fromLane, sIn: fromLane.poly.length, via: "start", conn: null }, { ...route[0], via: "conn", conn }, ...route.slice(1)];
    return buildReference(net, steps, fromLane.poly.length - 0.01, this.v, dest);
  }

  private updateLane(dt: number, ctx: DriveContext): void {
    const { net } = ctx;
    const dest = this.dest;
    if (!dest) {
      this.disengage();
      return;
    }
    this.planTimer -= dt;
    this.routeTimer -= dt;
    if (this.planTimer <= 0) {
      this.planTimer = 0.1;
      let loc = this.localize(net);
      let onGuide = true;
      if (!loc) {
        // Stay in lane mode while near the guide line (wide nudges, tight junction geometry).
        const wide = net.matchLane(this.pos, this.heading, 4.2, 1.1);
        const pr = this.ref?.poly.project(this.pos, this.refHint, 60);
        const guideOk = !!pr && pr.dist < 5 && Math.abs(wrapAngle(angleOf(this.ref!.poly.sampleAt(pr.s).dir) - this.heading)) < 0.9;
        if (wide) loc = this.lastLoc = { lane: wide.lane, s: wide.s };
        else if (guideOk && this.lastLoc) {
          loc = this.lastLoc;
          onGuide = false;
        }
      }
      if (!loc) {
        this.enterFree(net, false);
        return;
      }
      if (onGuide && (this.routeTimer <= 0 || !this.ref || !this.ref.lanesUsed.has(loc.lane))) {
        this.routeTimer = 1;
        this.replanRoute(net, loc);
      }
      if (!this.route || !this.ref) {
        this.intent = "noroute";
        this.message = "当前车道无法到达目的地，正在规划掉头路径";
        this.noRouteTime += 0.1;
        this.routeTimer = 0;
        if (this.noRouteTime > 2 && this.v < 0.5) {
          this.noRouteTime = 0;
          this.enterFree(net, false, ctx.agents, loc.lane);
          return;
        }
        this.brake(dt, net, 4);
        return;
      }
      this.noRouteTime = 0;
      const ref = this.ref;
      const preds = ctx.predictor.predict(ctx.agents, this.id, this.pos, 85);
      const plan = this.lattice.plan({
        ref,
        ego: { pos: this.pos, heading: this.heading, v: this.v, a: this.accel, length: this.length, width: this.width },
        preds,
        net,
        signal: (g) => net.laneSignal(g.lane, ctx.t),
        prevDT: this.prevDT,
        hintIdx: this.refHint,
      });
      this.refHint = ref.index(plan.s0);
      this.plan = plan;
      this.prevDT = plan.dT;
      this.planPath = new Polyline(plan.path);
      this.planAge = 0;
      this.explain(plan, ref);
      // Stuck behind a stationary obstacle with no lateral option: manoeuvre in free space.
      const blocker = plan.limitAgent;
      const staticLimit = plan.limit === "static" || plan.limit === "building" || plan.limit === "curve" || plan.limit === "bike";
      const parked = blocker ? this.looksParked(blocker, ctx) : true;
      const blockedStatic = staticLimit && parked && this.v < 0.3;
      this.staticBlockTime = blockedStatic ? this.staticBlockTime + 0.1 : 0;
      const patience = blocker && blocker.blinker === 2 ? 5 : 10;
      if (this.staticBlockTime > patience) {
        this.staticBlockTime = 0;
        this.enterFree(net, true);
        this.message = "前方道路被占用，正在规划绕行路径";
        return;
      }
      if ((ref.destS - plan.s0 < 3 || dist(this.pos, dest.pos) < 4) && this.v < 0.4) {
        this.arrived = true;
        this.fsd = "arrived";
        this.intent = "arrived";
      }
    }
    this.track(dt, net);
  }

  private explain(plan: Plan, ref: Reference): void {
    this.leadId = plan.limitAgent?.id ?? -1;
    const map: Record<Plan["limit"], Intent> = {
      none: "cruise",
      ped: "ped",
      bike: "bike",
      vehicle: "yield",
      static: "obstacle",
      follow: "follow",
      red: "red",
      yellow: "yellow",
      curve: "curve",
      dest: "arriving",
      end: "cruise",
      building: "obstacle",
    };
    let intent = map[plan.limit];
    const slowing = plan.v[4] < this.v - 0.5 || plan.v[8] < 0.5;
    if (intent === "follow" && !slowing && this.v > 3) intent = "follow";
    if (!plan.feasible) intent = "emergency";
    else if (plan.a[1] < -4.5 && (intent === "ped" || intent === "yield" || intent === "bike" || intent === "obstacle")) intent = "emergency";
    if (ref.destS - plan.s0 < 40 && intent === "cruise") intent = "arriving";
    this.intent = intent;
    this.lateral = plan.lateral === "lanechange" ? (plan.avoiding ? "overtake" : "lanechange") : plan.lateral === "nudge" ? "nudge" : "keep";
    this.message = "";
    // Turn signal: planned lateral move, else the next manoeuvre on the route.
    if (Math.abs(plan.dT - plan.d0) > 1.2 && this.lateral !== "nudge") this.blinker = plan.dT > plan.d0 ? 1 : -1;
    else {
      const next = ref.spans.find((sp) => sp.s1 > plan.s0 && sp.s0 - plan.s0 < 45);
      this.blinker = next ? (next.conn.turn === "left" ? -1 : next.conn.turn === "right" ? 1 : 0) : 0;
    }
  }

  private track(dt: number, net: RoadNetwork): void {
    const plan = this.plan;
    const path = this.planPath;
    if (!plan || !path) {
      this.brake(dt, net, 4);
      return;
    }
    this.planAge += dt;
    const tau = this.planAge;
    const k = Math.min(plan.v.length - 2, Math.floor(tau / PRED_DT));
    const f = clamp((tau - k * PRED_DT) / PRED_DT, 0, 1);
    const vRef = plan.v[k] + (plan.v[k + 1] - plan.v[k]) * f;
    const aRef = plan.a[k] + (plan.a[k + 1] - plan.a[k]) * f;
    const aCmd = clamp(aRef + 1.4 * (vRef - this.v), -8.5, 3);
    this.accel = aCmd;
    this.v = Math.max(0, this.v + aCmd * dt);
    if (vRef < 0.05 && this.v < 0.3 && aCmd < 0.1) this.v = 0;
    this.steerTo(path, dt);
    this.integrate(dt, net);
  }

  /** Stanley steering at the vehicle centre with curvature feedforward. */
  private steerTo(path: Polyline, dt: number): void {
    const pr = path.project(this.pos);
    const sRef = pr.s + Math.max(this.v, 0) * 0.25;
    const d1 = path.sampleAt(sRef - 0.8).dir;
    const d2 = path.sampleAt(sRef + 0.8).dir;
    const kappa = Math.atan2(cross(d1, d2), dot(d1, d2)) / 1.6;
    const beta = Math.asin(clamp((kappa * WHEELBASE) / 2, -0.9, 0.9));
    const ff = Math.atan(2 * Math.tan(beta));
    const slip = Math.atan(0.5 * Math.tan(this.steer));
    const headingErr = wrapAngle(angleOf(path.sampleAt(pr.s).dir) - (this.heading + slip));
    const cmd = clamp(ff + headingErr + Math.atan2(-1.5 * pr.lateral, this.v + 1.5), -MAX_STEER, MAX_STEER);
    this.steer += clamp(cmd - this.steer, -2.5 * dt, 2.5 * dt);
  }

  private brake(dt: number, net: RoadNetwork, decel: number): void {
    this.accel = -decel;
    this.v = Math.max(0, this.v - decel * dt);
    this.integrate(dt, net);
  }

  // ---- Free-space driving ------------------------------------------------------------------------

  private enterFree(net: RoadNetwork, aheadOnly: boolean, agents: readonly Agent[] = this.lastAgents, exclude: Lane | null = null): void {
    this.freeExclude = exclude;
    this.fsd = "free";
    this.plan = null;
    this.planPath = null;
    this.ref = null;
    this.route = null;
    this.free = null;
    this.blockedTimer = 0;
    this.freeAheadOnly = aheadOnly;
    this.freeReason = aheadOnly ? "blocked" : `unlocalized@${this.pos.x.toFixed(0)},${this.pos.y.toFixed(0)} h=${this.heading.toFixed(2)} v=${this.v.toFixed(1)}`;
    this.searchFree(net, this.staticObstacles(agents));
  }

  private searchFree(net: RoadNetwork, obstacles: { p: Vec2; r: number }[]): void {
    const fwd = this.fwd;
    const goals: { x: number; y: number; h: number; lane: Lane }[] = [];
    for (const radius of [45, 110]) {
      goals.length = 0;
      const seen = new Set<Lane>();
      for (const e of net.nearest(this.pos, radius)) {
        const l = e.lane;
        if (!l || seen.has(l) || l.poly.length < 8 || !net.core.has(l) || l.road === this.freeExclude?.road) continue;
        seen.add(l);
        for (let s = 3; s < l.poly.length - 4; s += 4) {
          const smp = l.poly.sampleAt(s);
          if (dist(smp.p, this.pos) > radius) continue;
          // When going around an obstacle, only poses clearly ahead count as done.
          if (this.freeAheadOnly && (dot(sub(smp.p, this.pos), fwd) < 12 || obstacles.some((o) => dist(o.p, smp.p) < 5))) continue;
          goals.push({ x: smp.p.x, y: smp.p.y, h: angleOf(smp.dir), lane: l });
        }
      }
      goals.sort((a, b) => dist(a, this.pos) - dist(b, this.pos));
      goals.length = Math.min(goals.length, 140);
      if (goals.length === 0) continue;
      const res = hybridAStar({
        start: { x: this.pos.x, y: this.pos.y, h: this.heading },
        goals,
        grid: net.grid,
        obstacles,
        length: this.length,
        width: this.width,
        maxCurvature: MAX_CURVATURE * 0.92,
        maxExpansions: radius < 50 ? 25000 : 45000,
      });
      if (res) {
        const runs = splitByDirection(res.path).map((r) => ({ poly: new Polyline(r.map((p) => ({ x: p.x, y: p.y }))), dir: r[0].dir }));
        const goal = goals.find((g) => g.x === res.goal.x && g.y === res.goal.y) ?? null;
        this.free = { path: res.path, runs, run: 0, goalLane: goal?.lane ?? null };
        this.fsd = "free";
        this.message = "";
        return;
      }
    }
    this.free = null;
    this.fsd = "blocked";
    this.message = "暂未找到返回道路的可行路径，稍后重试";
  }

  private updateFree(dt: number, ctx: DriveContext): void {
    const { net } = ctx;
    this.lateral = "keep";
    this.leadId = -1;
    if (this.fsd === "blocked" || !this.free) {
      this.intent = "searching";
      this.brake(dt, net, 3);
      this.blockedTimer += dt;
      if (this.blockedTimer > 2.5) {
        this.blockedTimer = 0;
        this.searchFree(net, this.staticObstacles(ctx.agents));
      }
      return;
    }
    const m = this.free;
    const run = m.runs[m.run];
    if (!run) {
      this.fsd = "lane";
      return;
    }
    const pr = run.poly.project(this.pos);
    const remaining = run.poly.length - pr.s;
    // Hand back to lane driving once aligned with a lane in the direction of travel.
    const lastRun = m.run === m.runs.length - 1;
    const progressed = !this.freeAheadOnly || pr.s > run.poly.length * 0.65;
    const loc = net.matchLane(this.pos, this.heading, 1.4, 0.3);
    if (loc && run.dir > 0 && lastRun && progressed) {
      this.fsd = "lane";
      this.free = null;
      this.routeTimer = 0;
      this.planTimer = 0;
      return;
    }
    if (remaining < 0.5 && Math.abs(this.v) < 0.25) {
      m.run++;
      if (m.run >= m.runs.length) {
        this.fsd = "lane";
        this.free = null;
        this.routeTimer = 0;
        this.planTimer = 0;
      }
      this.v = 0;
      return;
    }
    // Road users on or moving towards the next metres of the manoeuvre make the vehicle wait.
    let hazard = false;
    for (const a of ctx.agents) {
      if (a.id === this.id || dist(a.pos, this.pos) > 25) continue;
      const still = a.v < 0.3;
      const margin = still ? 0.15 : 1.2;
      for (let t = 0; t <= (still ? 0 : 3.5) && !hazard; t += 0.5) {
        const q = { x: a.pos.x + a.vel.x * t, y: a.pos.y + a.vel.y * t };
        const circles = still && a.kind === "vehicle" ? footprint(q, a.heading, a.length, a.width) : [q];
        const ar = a.kind === "ped" ? 0.35 : a.width / 2;
        for (let s = pr.s; s < Math.min(run.poly.length, pr.s + (still ? 7 : 10)) && !hazard; s += 1) {
          const own = run.poly.sampleAt(s);
          for (const c of footprint(own.p, angleOf(own.dir), this.length, this.width)) {
            if (circles.some((q2) => dist(c, q2) < this.width / 2 + ar + margin)) {
              hazard = true;
              break;
            }
          }
        }
      }
      if (hazard) {
        this.leadId = a.id;
        break;
      }
    }
    const vMax = run.dir > 0 ? 2.8 : 1.6;
    const vTarget = hazard ? 0 : run.dir * Math.min(vMax, Math.sqrt(2 * 0.8 * Math.max(0, remaining - 0.2)) + 0.15);
    const aCmd = clamp(1.8 * (vTarget - this.v), -4, 1.5);
    this.accel = aCmd;
    this.v = clamp(this.v + aCmd * dt, -vMax, vMax);
    // Pure pursuit in the direction of motion: tan(delta) = 2 L y / Ld^2 holds for both directions.
    const Ld = 3.2;
    const target = run.poly.sampleAt(Math.min(run.poly.length, pr.s + Ld)).p;
    const rel = sub(target, this.pos);
    const ly = -rel.x * Math.sin(this.heading) + rel.y * Math.cos(this.heading);
    const dLen = Math.max(1.5, Math.hypot(rel.x, rel.y));
    const cmd = clamp(Math.atan((2 * WHEELBASE * ly) / (dLen * dLen)), -MAX_STEER, MAX_STEER);
    this.steer += clamp(cmd - this.steer, -2 * dt, 2 * dt);
    this.freeHazardTime = hazard && Math.abs(this.v) < 0.2 ? this.freeHazardTime + dt : 0;
    if (this.freeHazardTime > 4) {
      this.freeHazardTime = 0;
      this.searchFree(net, this.staticObstacles(ctx.agents));
      return;
    }
    this.intent = hazard ? "yield" : run.dir < 0 ? "reverse" : "freespace";
    this.blinker = hazard ? 0 : 2;
    this.integrate(dt, net);
    if (this.bumped) {
      this.fsd = "blocked";
      this.blockedTimer = 2;
    }
  }

  /**
   * Whether a stationary road user appears parked rather than queued, from observable cues only:
   * hazard lights, a vehicle stopped directly in front of it, or a red signal just ahead of it.
   */
  private looksParked(a: Agent, ctx: DriveContext): boolean {
    if (a.kind !== "vehicle") return false;
    if (a.blinker === 2 || a.vkind === "bike") return true;
    const f = fromAngle(a.heading);
    for (const b of ctx.agents) {
      if (b === a || b.id === this.id || b.kind !== "vehicle" || b.v > 1) continue;
      const rel = sub(b.pos, a.pos);
      const along = dot(rel, f);
      const lat = Math.abs(cross(f, rel));
      if (along > 0 && along < (a.length + b.length) / 2 + 7 && lat < 2) return false;
    }
    const m = ctx.net.matchLane(a.pos, a.heading, 2.6, 0.6);
    if (m && m.lane.poly.length - m.s < 25) {
      const sig = ctx.net.laneSignal(m.lane, ctx.t);
      if (sig === "R" || sig === "Y") return false;
      // Waiting to enter a junction (giving way) also counts as queued.
      if (m.lane.out.some((c) => c.conflicts.length > 0) && m.lane.poly.length - m.s < 8) return false;
    }
    return true;
  }

  private staticObstacles(agents: readonly Agent[]): { p: Vec2; r: number }[] {
    const out: { p: Vec2; r: number }[] = [];
    for (const a of agents) {
      if (a.id === this.id || a.v > 0.3 || dist(a.pos, this.pos) > 70) continue;
      for (const c of a.kind === "ped" ? [a.pos] : footprint(a.pos, a.heading, a.length, a.width)) out.push({ p: c, r: a.kind === "ped" ? 0.4 : a.width / 2 });
    }
    return out;
  }

  /** Next manoeuvre on the route and remaining distance, for the navigation card. */
  navInfo(): { turn: "straight" | "left" | "right" | "uturn" | "arrive"; distance: number; remaining: number } | null {
    if (this.mode !== "fsd" || !this.ref || !this.plan) return null;
    const s0 = this.plan.s0;
    const remaining = Math.max(0, this.ref.destS - s0);
    const next = this.ref.spans.find((sp) => sp.s1 > s0 && sp.conn.turn !== "straight");
    if (!next || next.s0 - s0 > remaining) return { turn: "arrive", distance: remaining, remaining };
    return { turn: next.conn.turn, distance: Math.max(0, next.s0 - s0), remaining };
  }
}
