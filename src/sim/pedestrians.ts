import { type Agent, PED_R, agentCircles } from "./agents";
import { Rng, type Vec2, add, dist, dist2, dot, norm, scale, sub, vec } from "./geometry";
import type { Crossing, Lane, RoadNetwork } from "./map/network";

type PedState = "wait" | "cross" | "walk" | "done";

let nextPedId = 100000;

export class Pedestrian implements Agent {
  readonly id = nextPedId++;
  readonly kind = "ped" as const;
  readonly vkind = "ped" as const;
  readonly length = PED_R * 2;
  readonly width = PED_R * 2;
  blinker = 0;
  pos: Vec2;
  heading = 0;
  v = 0;
  vel: Vec2 = { x: 0, y: 0 };
  state: PedState;
  /** Remaining waypoints. */
  route: Vec2[];
  speed: number;
  gait = 0;
  blockedTime = 0;
  waitTime = 0;
  crossed = false;
  crossing: Crossing | null;
  /** Crosses even when vehicles are approaching (drivers must yield). */
  readonly assertive: boolean;
  readonly jaywalker: boolean;
  readonly tone: number;
  origin: Vec2;

  constructor(pos: Vec2, route: Vec2[], state: PedState, speed: number, crossing: Crossing | null, assertive: boolean, jaywalker: boolean, tone: number) {
    this.pos = pos;
    this.origin = pos;
    this.route = route;
    this.state = state;
    this.speed = speed;
    this.crossing = crossing;
    this.assertive = assertive;
    this.jaywalker = jaywalker;
    this.tone = tone;
  }
}

export class Pedestrians {
  readonly peds: Pedestrian[] = [];
  targetCount: number;

  constructor(
    private readonly net: RoadNetwork,
    private readonly rng: Rng,
    count: number,
  ) {
    this.targetCount = count;
  }

  private kerbPoints(c: Crossing): [Vec2, Vec2] {
    const r = { x: -c.dir.y, y: c.dir.x };
    return [add(c.pos, scale(r, c.seg.rightW + 1.6)), add(c.pos, scale(r, -(c.seg.leftW + 1.6)))];
  }

  spawnAtCrossing(avoid: Vec2 | null): Pedestrian | null {
    if (this.net.crossings.length === 0) return null;
    const c = this.rng.pick(this.net.crossings);
    if (avoid && dist(c.pos, avoid) > 260) return null;
    const [a, b] = this.kerbPoints(c);
    const [from, to] = this.rng.chance(0.5) ? [a, b] : [b, a];
    const along = scale(c.dir, this.rng.range(-1.2, 1.2));
    const start = add(from, along);
    const end = add(to, along);
    const onward = add(end, scale(c.dir, this.rng.range(-25, 25)));
    const approach = add(start, scale(c.dir, this.rng.range(-12, 12)));
    const offRoad = (a: Vec2, b: Vec2): boolean => {
      for (let t = 0; t <= 1; t += 0.1) if (this.net.grid.onRoad({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t })) return false;
      return true;
    };
    if (!offRoad(approach, start)) return null;
    const ped = new Pedestrian(
      approach,
      offRoad(end, onward) ? [start, end, onward] : [start, end],
      "walk",
      this.rng.range(1.15, 1.55),
      c,
      this.rng.chance(0.3),
      false,
      this.rng.int(0, 4),
    );
    this.peds.push(ped);
    return ped;
  }

  /** A pedestrian on the sidewalk of a random segment, walking its length. */
  spawnStroller(): Pedestrian | null {
    const seg = this.rng.pick(this.net.segments);
    if (seg.center.length < 25) return null;
    const side = this.rng.chance(0.5) ? 1 : -1;
    const off = side > 0 ? seg.rightW + 2.2 : -(seg.leftW + 2.2);
    const pts: Vec2[] = [];
    const s0 = seg.trimA + 2;
    const s1 = seg.center.length - seg.trimB - 2;
    if (s1 - s0 < 15) return null;
    const n = Math.ceil((s1 - s0) / 3);
    for (let i = 0; i <= n; i++) {
      const smp = seg.center.sampleAt(s0 + ((s1 - s0) * i) / n);
      pts.push(add(smp.p, scale({ x: -smp.dir.y, y: smp.dir.x }, off)));
    }
    if (this.rng.chance(0.5)) pts.reverse();
    if (pts.some((p) => this.net.grid.clearanceAt(p) < 0.6 || this.net.grid.onRoad(p))) return null;
    const ped = new Pedestrian(pts[0], pts.slice(1), "walk", this.rng.range(1.0, 1.45), null, false, false, this.rng.int(0, 4));
    this.peds.push(ped);
    return ped;
  }

  /** A pedestrian stepping off the kerb mid-block across `lane` at arc length s. */
  spawnJaywalker(lane: Lane, s: number, fromRight: boolean): Pedestrian {
    const seg = lane.road.seg;
    const forward = seg.forward === lane.road;
    const center = seg.center;
    const cs = forward ? s + (lane.road.seg.trimA ?? 0) : center.length - s - (seg.trimB ?? 0);
    const smp = center.sampleAt(Math.max(0, Math.min(center.length, cs)));
    const r = { x: -smp.dir.y, y: smp.dir.x };
    const right = add(smp.p, scale(r, seg.rightW + 1.4));
    const left = add(smp.p, scale(r, -(seg.leftW + 1.4)));
    // "fromRight" is relative to the lane's travel direction.
    const laneRightIsSegRight = forward;
    const startRight = fromRight === laneRightIsSegRight;
    const [start, end] = startRight ? [right, left] : [left, right];
    const ped = new Pedestrian(start, [end], "cross", this.rng.range(1.5, 1.9), null, true, true, this.rng.int(0, 4));
    this.peds.push(ped);
    return ped;
  }

  update(dt: number, t: number, vehicles: readonly Agent[], egoPos: Vec2): void {
    for (const p of this.peds) this.step(p, dt, t, vehicles);
    for (let i = this.peds.length - 1; i >= 0; i--) {
      const p = this.peds[i];
      if (p.state === "done" || dist(p.pos, egoPos) > 420) this.peds.splice(i, 1);
    }
    const regular = this.peds.filter((p) => !p.jaywalker).length;
    if (regular < this.targetCount) {
      if (this.rng.chance(0.55)) this.spawnAtCrossing(egoPos);
      else this.spawnStroller();
    }
  }

  private blockedByVehicle(ped: Pedestrian, dir: Vec2, vehicles: readonly Agent[]): boolean {
    const probe = add(ped.pos, dir);
    for (const v of vehicles) {
      if (dist2(v.pos, ped.pos) > 100) continue;
      const clearance = v.width / 2 + PED_R + 0.35;
      for (const c of agentCircles(v)) {
        const d = dist(c, probe);
        if (d < clearance && d < dist(c, ped.pos)) return true;
      }
    }
    return false;
  }

  /** Time until the nearest vehicle reaches the crossing line, looking only at approaching vehicles. */
  private gapAt(c: Crossing, vehicles: readonly Agent[]): number {
    let best = Infinity;
    for (const v of vehicles) {
      const rel = sub(c.pos, v.pos);
      const d = Math.abs(dot(rel, c.dir));
      const lat = Math.abs(rel.x * c.dir.y - rel.y * c.dir.x);
      if (d > 60 || lat > c.halfWidth + 3) continue;
      const vdir = { x: Math.cos(v.heading), y: Math.sin(v.heading) };
      if (dot(vdir, rel) <= 0) continue;
      best = Math.min(best, (d - v.length / 2) / Math.max(v.v, 0.5));
    }
    return best;
  }

  private step(ped: Pedestrian, dt: number, t: number, vehicles: readonly Agent[]): void {
    const target = ped.route[0];
    if (!target) {
      ped.state = "done";
      return;
    }
    // Arriving at the kerb before a crossing: wait for the signal or a gap.
    if (ped.state === "walk" && ped.crossing && !ped.crossed && dist(ped.pos, target) < 0.3) {
      ped.state = "wait";
    }
    if (ped.state === "wait") {
      ped.v = 0;
      ped.vel = vec(0, 0);
      ped.waitTime += dt;
      const c = ped.crossing as Crossing;
      const open = c.controller
        ? this.net.crossingOpen(c, t)
        : this.gapAt(c, vehicles) > (ped.assertive ? 1.6 : 4.5) || ped.waitTime > 25;
      if (open) {
        ped.route.shift();
        ped.state = "cross";
        ped.crossed = true;
      }
      return;
    }
    const delta = sub(target, ped.pos);
    const d = dist(target, ped.pos);
    if (d < 0.2) {
      ped.route.shift();
      if (ped.state === "cross") ped.state = "walk";
      if (ped.route.length === 0) ped.state = "done";
      return;
    }
    const dir = norm(delta);
    if (this.blockedByVehicle(ped, dir, vehicles)) {
      ped.v = 0;
      ped.vel = vec(0, 0);
      ped.blockedTime += dt;
      if (ped.blockedTime > 4 && ped.state === "cross") {
        // A vehicle is waiting on the crossing line; step back to the kerb.
        ped.blockedTime = 0;
        ped.route = [ped.origin];
        ped.state = "walk";
      } else if (ped.blockedTime > 6) ped.state = "done";
      return;
    }
    ped.blockedTime = 0;
    const speed = ped.state === "cross" ? ped.speed * 1.12 : ped.speed;
    ped.pos = add(ped.pos, scale(dir, Math.min(d, speed * dt)));
    ped.v = speed;
    ped.vel = scale(dir, speed);
    ped.heading = Math.atan2(dir.y, dir.x);
    ped.gait += dt * speed * 4.2;
  }
}
