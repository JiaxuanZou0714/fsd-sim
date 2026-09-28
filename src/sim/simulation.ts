import { type Agent, agentCircles, agentRadius, footprint } from "./agents";
import { type Destination, Ego, type ManualInput } from "./ego";
import { Rng, type Vec2, angleOf, dist, dist2, wrapAngle } from "./geometry";
import { PARIS_ETOILE } from "./map/mapData";
import { type Lane, RoadNetwork } from "./map/network";
import { Pedestrians } from "./pedestrians";
import { Predictor } from "./planner/prediction";
import { Traffic } from "./traffic";

export interface SimSettings {
  trafficCount: number;
  pedCount: number;
  autoplay: boolean;
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
  recoveries: number;
  planMs: number;
}

export const DEFAULT_SETTINGS: SimSettings = {
  trafficCount: 75,
  pedCount: 70,
  autoplay: true,
  randomEvents: true,
};

const NO_INPUT: ManualInput = { throttle: 0, brake: 0, steer: 0 };

let sharedNetwork: RoadNetwork | null = null;

/** The road network is immutable, so it is built once and shared between simulations. */
export function getNetwork(): RoadNetwork {
  if (!sharedNetwork) sharedNetwork = new RoadNetwork(PARIS_ETOILE);
  return sharedNetwork;
}

export class Simulation {
  readonly net: RoadNetwork;
  readonly traffic: Traffic;
  readonly peds: Pedestrians;
  readonly ego: Ego;
  readonly predictor: Predictor;
  readonly rng: Rng;
  readonly settings: SimSettings;
  readonly stats: SimStats = {
    fsdDistance: 0,
    manualDistance: 0,
    interventions: 0,
    collisions: 0,
    arrivals: 0,
    emergencyStops: 0,
    recoveries: 0,
    planMs: 0,
  };
  /** Lanes from which every other lane in the set is reachable (the main strongly connected part). */
  readonly core: Lane[];
  t = 0;
  private pending: SimEvent[] = [];
  private contacts = new Set<number>();
  private nextScripted = 20;
  private arrivedAt = -1;
  private lastIntent = "";
  private lastFsd = "lane";

  constructor(seed = 7, settings: Partial<SimSettings> = {}) {
    this.rng = new Rng(seed);
    this.settings = { ...DEFAULT_SETTINGS, ...settings };
    this.net = getNetwork();
    this.core = [...this.net.core].filter((l) => l.poly.length > 6);
    this.traffic = new Traffic(this.net, this.rng, this.settings.trafficCount);
    this.peds = new Pedestrians(this.net, this.rng, this.settings.pedCount);
    this.predictor = new Predictor(this.net);

    const start = this.startLane();
    const p = start.poly.sampleAt(12);
    this.ego = new Ego(p.p, angleOf(p.dir));

    for (let i = 0; i < this.settings.trafficCount; i++) this.traffic.spawnRandom([this.ego.pos], 18);
    for (let i = 0; i < this.settings.pedCount; i++) {
      if (this.rng.chance(0.5)) this.peds.spawnAtCrossing(null);
      else this.peds.spawnStroller();
    }
  }

  private startLane(): Lane {
    const candidates = this.core.filter((l) => l.road.rank >= 5 && !l.road.circular && l.poly.length > 60 && l.k === 0);
    candidates.sort((a, b) => dist(a.poly.pts[0], { x: 0, y: 0 }) - dist(b.poly.pts[0], { x: 0, y: 0 }));
    return candidates[Math.min(candidates.length - 1, 3)] ?? this.core[0];
  }

  get agents(): Agent[] {
    return [this.ego, ...this.traffic.vehicles, ...this.peds.peds];
  }

  drainEvents(): SimEvent[] {
    const e = this.pending;
    this.pending = [];
    return e;
  }

  private emit(text: string, level: EventLevel): void {
    this.pending.push({ text, level });
  }

  // ---------------------------------------------------------------------------
  // Destinations and modes

  destinationNear(p: Vec2): Destination | null {
    const core = new Set(this.core);
    let best: Destination | null = null;
    let bestD = Infinity;
    for (const e of this.net.nearest(p, 80)) {
      const l = e.lane;
      if (!l || !core.has(l)) continue;
      if (e.d < bestD) {
        bestD = e.d;
        const s = Math.min(l.poly.length - 3, Math.max(3, e.s));
        best = { lane: l, s, pos: l.poly.sampleAt(s).p };
      }
    }
    return best;
  }

  randomDestination(): Destination {
    for (let i = 0; i < 60; i++) {
      const l = this.rng.pick(this.core);
      if (l.poly.length < 25) continue;
      const s = this.rng.range(8, l.poly.length - 8);
      const pos = l.poly.sampleAt(s).p;
      const d = dist(pos, this.ego.pos);
      if (d > 220 && d < 650) return { lane: l, s, pos };
    }
    const l = this.core[0];
    return { lane: l, s: l.poly.length / 2, pos: l.poly.sampleAt(l.poly.length / 2).p };
  }

  engageFsd(dest?: Destination): void {
    const target = dest ?? this.ego.dest ?? this.randomDestination();
    this.ego.engage(this.net, target);
    this.arrivedAt = -1;
    this.emit(this.ego.fsd === "lane" ? "FSD 已启用，正在前往目的地" : "FSD 已启用：车辆不在车道上，正在规划返回道路的路径", "info");
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

  // ---------------------------------------------------------------------------
  // Scripted situations

  /** Lane and arc length on the ego's reference roughly `ahead` metres in front. */
  private pointAhead(ahead: number): { lane: Lane; s: number } | null {
    const ego = this.ego;
    if (ego.mode === "fsd" && ego.ref && ego.plan) {
      const ref = ego.ref;
      for (let s = ego.plan.s0 + ahead; s < Math.min(ref.poly.length, ego.plan.s0 + ahead + 40); s += 2) {
        const laneId = ref.lane[ref.index(s)];
        if (laneId < 0) continue;
        const lane = this.net.lanes[laneId];
        const ls = lane.poly.project(ref.poly.sampleAt(s).p).s;
        if (ls > 15 && ls < lane.poly.length - 8) return { lane, s: ls };
      }
      return null;
    }
    const loc = ego.localize(this.net);
    if (!loc) return null;
    const s = loc.s + ahead;
    return s < loc.lane.poly.length - 8 ? { lane: loc.lane, s } : null;
  }

  spawnJaywalkerAhead(): boolean {
    const at = this.pointAhead(Math.max(24, this.ego.v * 2.8 + 6));
    if (!at) return false;
    this.peds.spawnJaywalker(at.lane, at.s, this.rng.chance(0.6));
    this.emit("前方行人横穿马路", "warn");
    return true;
  }

  spawnStalledAhead(): boolean {
    const at = this.pointAhead(Math.max(38, this.ego.v * 4));
    if (!at) return false;
    const p = at.lane.poly.sampleAt(at.s).p;
    if (this.traffic.vehicles.some((c) => dist(c.pos, p) < 10)) return false;
    const car = this.traffic.spawn(at.lane, at.s, this.rng.chance(0.5) ? "van" : "car", true);
    car.syncPose();
    this.emit(car.vkind === "van" ? "前方货车违停" : "前方车辆故障停驶", "warn");
    return true;
  }

  spawnCyclistAhead(): boolean {
    const at = this.pointAhead(Math.max(30, this.ego.v * 3));
    if (!at) return false;
    const lane = at.lane.road.lanes[0];
    const s = Math.min(lane.poly.length - 8, at.s);
    const p = lane.poly.sampleAt(s).p;
    if (this.traffic.vehicles.some((c) => dist(c.pos, p) < 8)) return false;
    this.traffic.spawn(lane, s, "bike");
    this.emit("前方有自行车", "info");
    return true;
  }

  /** Teleports the ego to a random spot off the carriageway (courtyard, sidewalk, plaza). */
  placeOffRoad(): boolean {
    const g = this.net.grid;
    for (let i = 0; i < 400; i++) {
      const lane = this.rng.pick(this.core);
      const smp = lane.poly.sampleAt(this.rng.range(0, lane.poly.length));
      const ang = this.rng.range(0, Math.PI * 2);
      const r = this.rng.range(9, 28);
      const p = { x: smp.p.x + Math.cos(ang) * r, y: smp.p.y + Math.sin(ang) * r };
      if (g.onRoad(p) || !g.inside(p, 30)) continue;
      const h = this.rng.range(-Math.PI, Math.PI);
      if (footprint(p, h, this.ego.length + 1, this.ego.width + 0.8).some((c) => g.clearanceAt(c) < 1.6)) continue;
      if (this.agents.some((a) => a.id !== 0 && dist(a.pos, p) < 6)) continue;
      this.teleport(p, h);
      this.emit("车辆已放置在非道路区域", "info");
      return true;
    }
    return false;
  }

  /** Places the ego on a one-way street facing against traffic. */
  placeWrongWay(): boolean {
    const lanes = this.core.filter((l) => l.road.seg.oneway && l.poly.length > 40 && !l.road.circular);
    for (let i = 0; i < 60; i++) {
      const l = this.rng.pick(lanes);
      const s = this.rng.range(12, l.poly.length - 12);
      const smp = l.poly.sampleAt(s);
      if (this.agents.some((a) => a.id !== 0 && dist(a.pos, smp.p) < 18)) continue;
      this.teleport(smp.p, wrapAngle(angleOf(smp.dir) + Math.PI));
      this.emit("车辆已放置在单行道上且方向与车流相反", "info");
      return true;
    }
    return false;
  }

  private teleport(p: Vec2, h: number): void {
    const ego = this.ego;
    ego.pos = p;
    ego.heading = h;
    ego.v = 0;
    ego.steer = 0;
    ego.accel = 0;
    this.contacts.clear();
    if (ego.mode === "fsd") this.engageFsd(ego.dest ?? undefined);
  }

  // ---------------------------------------------------------------------------

  step(dt: number, manual: ManualInput = NO_INPUT): void {
    this.t += dt;
    const ego = this.ego;
    this.traffic.targetCount = this.settings.trafficCount;
    this.peds.targetCount = this.settings.pedCount;
    this.traffic.update(dt, this.t, this.agents, ego.pos, ego.exposure());
    this.peds.update(dt, this.t, [ego, ...this.traffic.vehicles], ego.pos);
    const agents = this.agents;
    this.predictor.observe(agents, this.t);

    const before = ego.pos;
    const wasFsd = ego.mode === "fsd";
    ego.update(dt, { net: this.net, traffic: this.traffic, predictor: this.predictor, agents, t: this.t }, manual);
    if (ego.plan) this.stats.planMs = this.stats.planMs * 0.95 + ego.plan.ms * 0.05;
    const moved = dist(before, ego.pos);
    if (wasFsd) this.stats.fsdDistance += moved;
    else this.stats.manualDistance += moved;

    if (ego.intent === "emergency" && this.lastIntent !== "emergency") this.stats.emergencyStops++;
    this.lastIntent = ego.intent;
    if (ego.mode === "fsd" && (this.lastFsd === "free" || this.lastFsd === "blocked") && ego.fsd === "lane") {
      this.stats.recoveries++;
      this.emit("已回到道路，切换为车道行驶", "success");
    }
    this.lastFsd = ego.fsd;

    if (ego.mode === "fsd" && ego.arrived) {
      if (this.arrivedAt < 0) {
        this.arrivedAt = this.t;
        this.stats.arrivals++;
        this.emit("已到达目的地", "success");
      } else if (this.settings.autoplay && this.t - this.arrivedAt > 2.5) {
        this.engageFsd(this.randomDestination());
      }
    }

    if (this.settings.randomEvents && ego.mode === "fsd" && ego.fsd === "lane" && this.t > this.nextScripted && ego.v > 5) {
      const r = this.rng.next();
      const ok = r < 0.4 ? this.spawnJaywalkerAhead() : r < 0.7 ? this.spawnStalledAhead() : this.spawnCyclistAhead();
      this.nextScripted = this.t + (ok ? this.rng.range(25, 45) : 4);
    }

    this.detectCollisions();
  }

  private detectCollisions(): void {
    const ego = this.ego;
    const mine = footprint(ego.pos, ego.heading, ego.length, ego.width);
    const r = ego.width / 2;
    const touching = new Set<number>();
    for (const a of [...this.traffic.vehicles, ...this.peds.peds]) {
      if (dist2(a.pos, ego.pos) > 100) continue;
      const ar = agentRadius(a);
      if (mine.some((m) => agentCircles(a).some((c) => dist(m, c) < r + ar - 0.12))) touching.add(a.id);
    }
    if (ego.bumped) touching.add(-1);
    for (const id of touching) {
      if (this.contacts.has(id)) continue;
      this.stats.collisions++;
      ego.v = 0;
      this.emit(id === -1 ? "碰撞：与建筑物发生接触" : id >= 100000 ? "碰撞：与行人发生接触" : "碰撞：与车辆发生接触", "danger");
    }
    this.contacts = touching;
  }
}
