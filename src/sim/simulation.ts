import { type Agent, CAR_CIRCLE_R, PED_R, carCircles } from "./agents";
import { type Destination, Ego, type ManualInput } from "./ego";
import { Rng, type Vec2, clamp, dist, dist2 } from "./geometry";
import { Pedestrians } from "./pedestrians";
import { type Occupant, Traffic } from "./traffic";
import { type Edge, World, angleOfDir, laneOffset, turnOf } from "./world";

export interface SimSettings {
  trafficCount: number;
  pedCount: number;
  /** Automatically pick a new destination after arriving. */
  autoplay: boolean;
  /** Periodically script jaywalkers and stalled vehicles ahead of the ego car. */
  randomEvents: boolean;
}

export type EventLevel = "info" | "success" | "warn" | "danger";

export interface SimEvent {
  text: string;
  level: EventLevel;
}

export interface SimStats {
  fsdDistance: number;
  manualDistance: number;
  interventions: number;
  collisions: number;
  arrivals: number;
  emergencyStops: number;
}

export const DEFAULT_SETTINGS: SimSettings = {
  trafficCount: 34,
  pedCount: 40,
  autoplay: true,
  randomEvents: true,
};

const NO_INPUT: ManualInput = { throttle: 0, brake: 0, steer: 0 };

export class Simulation {
  readonly world: World;
  readonly traffic: Traffic;
  readonly peds: Pedestrians;
  readonly ego: Ego;
  readonly rng: Rng;
  readonly settings: SimSettings;
  readonly stats: SimStats = {
    fsdDistance: 0,
    manualDistance: 0,
    interventions: 0,
    collisions: 0,
    arrivals: 0,
    emergencyStops: 0,
  };
  t = 0;
  private pending: SimEvent[] = [];
  private contacts = new Set<number>();
  private nextScriptedEvent = 18;
  private arrivedAt = -1;
  private lastIntent = "";

  constructor(seed = 7, settings: Partial<SimSettings> = {}) {
    this.rng = new Rng(seed);
    this.settings = { ...DEFAULT_SETTINGS, ...settings };
    this.world = new World();
    this.traffic = new Traffic(this.world, this.rng);
    this.peds = new Pedestrians(this.world, this.rng);

    const startEdge = this.world.edge(this.world.node(2 * 5 + 1).out[0] as number);
    const startPos = this.world.lanePoint(startEdge, laneOffset(0), 18);
    this.ego = new Ego(startPos, angleOfDir(startEdge.dir));

    for (let i = 0; i < this.settings.trafficCount; i++) this.traffic.spawnRandom([startPos], 25);
    for (let i = 0; i < this.settings.pedCount; i++) this.peds.spawnWalker();
  }

  get agents(): Agent[] {
    return [this.ego, ...this.traffic.cars, ...this.peds.peds];
  }

  drainEvents(): SimEvent[] {
    const e = this.pending;
    this.pending = [];
    return e;
  }

  private emit(text: string, level: EventLevel): void {
    this.pending.push({ text, level });
  }

  /** Destination on the lane closest to a world point. */
  destinationNear(p: Vec2): Destination {
    let best: Destination | null = null;
    let bestD = Infinity;
    for (const e of this.world.edges) {
      const u = clamp(this.world.edgeCoord(e, p), 6, e.length - 6);
      const q = this.world.lanePoint(e, 3.5, u);
      const d = dist2(p, q);
      if (d < bestD) {
        bestD = d;
        best = { edge: e, u, pos: this.world.lanePoint(e, laneOffset(0), u) };
      }
    }
    if (!best) throw new Error("World has no edges");
    return best;
  }

  randomDestination(): Destination {
    for (let i = 0; i < 30; i++) {
      const e = this.rng.pick(this.world.edges);
      const u = this.rng.range(10, e.length - 10);
      const pos = this.world.lanePoint(e, laneOffset(0), u);
      const d = dist(pos, this.ego.pos);
      if (d > 140 && d < 380) return { edge: e, u, pos };
    }
    const e = this.rng.pick(this.world.edges);
    return { edge: e, u: e.length / 2, pos: this.world.lanePoint(e, laneOffset(0), e.length / 2) };
  }

  /** Returns an error message if FSD could not engage. */
  engageFsd(dest?: Destination): string | null {
    const target = dest ?? this.ego.dest ?? this.randomDestination();
    const err = this.ego.engage(this.world, target);
    if (err) {
      this.emit(err, "warn");
      return err;
    }
    this.arrivedAt = -1;
    this.emit("FSD 已启用，正在前往目的地", "info");
    return null;
  }

  setDestination(dest: Destination): void {
    if (this.ego.mode === "fsd") this.engageFsd(dest);
    else {
      this.ego.dest = dest;
      this.emit("目的地已设置，按 F 启用 FSD", "info");
    }
  }

  takeover(): void {
    if (this.ego.mode !== "fsd") return;
    this.ego.disengage();
    this.stats.interventions++;
    this.emit("驾驶员接管，FSD 已退出", "warn");
  }

  /** Where the ego car is along the road network, preferring the active route. */
  private egoAnchor(): { edge: Edge; u: number; lane: number; next: Edge | null } | null {
    const ego = this.ego;
    if (ego.mode === "fsd" && ego.route.length > 0) {
      const edge = ego.route[ego.routeIdx] as Edge;
      return { edge, u: this.world.edgeCoord(edge, ego.pos), lane: ego.lane, next: ego.route[ego.routeIdx + 1] ?? null };
    }
    const loc = ego.localize(this.world);
    if (!loc) return null;
    const succ = this.world.successors(loc.edge).filter((e) => turnOf(loc.edge.dir, e.dir) === "straight");
    return { edge: loc.edge, u: loc.u, lane: loc.lane, next: succ[0] ?? null };
  }

  spawnJaywalkerAhead(): boolean {
    const anchor = this.egoAnchor();
    if (!anchor) return false;
    const ahead = Math.max(26, this.ego.v * 2.8 + 6);
    let edge = anchor.edge;
    let u = anchor.u + ahead;
    if (u > edge.length - 3) {
      if (!anchor.next) return false;
      u = u - edge.length + 2;
      edge = anchor.next;
      if (u > edge.length - 3) return false;
    }
    this.peds.spawnJaywalker(edge, u, this.rng.chance(0.6));
    this.emit("前方行人横穿马路", "warn");
    return true;
  }

  spawnStalledAhead(): boolean {
    const anchor = this.egoAnchor();
    if (!anchor) return false;
    const lane = anchor.lane;
    let edge = anchor.edge;
    let u = anchor.u + Math.max(40, this.ego.v * 4);
    if (u > edge.length - 10) {
      if (!anchor.next) return false;
      edge = anchor.next;
      u = 26;
    }
    const p = this.world.lanePoint(edge, laneOffset(lane), u);
    if (this.traffic.cars.some((c) => dist(c.pos, p) < 9)) return false;
    if (this.traffic.cars.some((c) => c.stalled && c.seg.edge.id === edge.id)) return false;
    const dest = this.ego.dest;
    if (dest && dest.edge.id === edge.id && Math.abs(dest.u - u) < 20) return false;
    this.traffic.spawnOnEdge(edge, lane, u, true);
    this.emit("前方车辆故障停驶", "warn");
    return true;
  }

  step(dt: number, manual: ManualInput = NO_INPUT): void {
    this.t += dt;
    const agents = this.agents;
    const loc = this.ego.localize(this.world);
    const egoOcc: Occupant[] = loc
      ? [{ id: this.ego.id, edgeId: loc.edge.id, lane: loc.lane, u: loc.u, v: this.ego.v, obstruction: false }]
      : [];
    this.traffic.update(dt, this.t, agents, this.ego.pos, this.settings.trafficCount, egoOcc);
    this.peds.update(dt, this.t, this.settings.pedCount, [this.ego, ...this.traffic.cars]);

    const before = this.ego.pos;
    const wasFsd = this.ego.mode === "fsd";
    this.ego.update(dt, { world: this.world, traffic: this.traffic, agents: this.agents, t: this.t }, manual);
    const moved = dist(before, this.ego.pos);
    if (wasFsd) this.stats.fsdDistance += moved;
    else this.stats.manualDistance += moved;

    if (wasFsd && this.ego.mode === "manual" && this.ego.offRoute) {
      this.ego.offRoute = false;
      this.stats.interventions++;
      this.emit("偏离路线过远，FSD 已退出", "danger");
    }

    if (this.ego.intent === "emergency" && this.lastIntent !== "emergency") this.stats.emergencyStops++;
    this.lastIntent = this.ego.intent;

    if (this.ego.mode === "fsd" && this.ego.arrived) {
      if (this.arrivedAt < 0) {
        this.arrivedAt = this.t;
        this.stats.arrivals++;
        this.emit("已到达目的地", "success");
      } else if (this.settings.autoplay && this.t - this.arrivedAt > 2.5) {
        this.engageFsd(this.randomDestination());
      }
    }

    if (this.settings.randomEvents && this.ego.mode === "fsd" && this.t > this.nextScriptedEvent && this.ego.v > 6) {
      const ok = this.rng.chance(0.6) ? this.spawnJaywalkerAhead() : this.spawnStalledAhead();
      this.nextScriptedEvent = this.t + (ok ? this.rng.range(30, 50) : 4);
    }

    this.detectCollisions();
  }

  private detectCollisions(): void {
    const egoCircles = carCircles(this.ego.pos, this.ego.heading);
    const touching = new Set<number>();
    for (const car of this.traffic.cars) {
      if (dist2(car.pos, this.ego.pos) > 36) continue;
      const cc = carCircles(car.pos, car.heading);
      if (egoCircles.some((a) => cc.some((b) => dist(a, b) < CAR_CIRCLE_R * 2 - 0.12))) touching.add(car.id);
    }
    for (const ped of this.peds.peds) {
      if (dist2(ped.pos, this.ego.pos) > 16) continue;
      if (egoCircles.some((a) => dist(a, ped.pos) < CAR_CIRCLE_R + PED_R - 0.05)) touching.add(ped.id);
    }
    for (const id of touching) {
      if (this.contacts.has(id)) continue;
      this.stats.collisions++;
      this.ego.v = 0;
      this.emit(id >= 10000 ? "碰撞：与行人发生接触" : "碰撞：与车辆发生接触", "danger");
    }
    this.contacts = touching;
  }
}
