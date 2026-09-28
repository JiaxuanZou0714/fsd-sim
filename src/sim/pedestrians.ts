import type { Agent } from "./agents";
import { Rng, type Vec2, add, dist, norm, scale, sub, vec } from "./geometry";
import { CORNER, type Edge, GRID, SIDEWALK, World } from "./world";

type PedState = "wait" | "walk" | "cross" | "jaywalk" | "done";

interface CornerRef {
  node: number;
  sx: number;
  sy: number;
}

interface Plan {
  kind: "walk" | "cross";
  to: CornerRef;
  needsSignal: boolean;
}

let nextPedId = 10000;

export class Pedestrian implements Agent {
  readonly id = nextPedId++;
  readonly kind = "ped" as const;
  pos: Vec2;
  heading = 0;
  v = 0;
  vel: Vec2 = { x: 0, y: 0 };
  state: PedState;
  corner: CornerRef;
  plan: Plan | null = null;
  target: Vec2;
  speed: number;
  jitter: Vec2;
  /** Walk-cycle phase for animation. */
  gait = 0;
  readonly jaywalker: boolean;
  readonly tone: number;

  constructor(pos: Vec2, corner: CornerRef, state: PedState, speed: number, jitter: Vec2, tone: number, jaywalker: boolean) {
    this.pos = pos;
    this.corner = corner;
    this.state = state;
    this.target = pos;
    this.speed = speed;
    this.jitter = jitter;
    this.tone = tone;
    this.jaywalker = jaywalker;
  }
}

export class Pedestrians {
  readonly peds: Pedestrian[] = [];

  constructor(
    private readonly world: World,
    private readonly rng: Rng,
  ) {}

  private cornerPos(c: CornerRef, jitter: Vec2): Vec2 {
    const n = this.world.node(c.node);
    return add(n.pos, vec(c.sx * CORNER + jitter.x, c.sy * CORNER + jitter.y));
  }

  spawnWalker(): Pedestrian {
    const node = this.rng.pick(this.world.nodes);
    const corner: CornerRef = { node: node.id, sx: this.rng.chance(0.5) ? 1 : -1, sy: this.rng.chance(0.5) ? 1 : -1 };
    const jitter = vec(this.rng.range(-0.5, 0.5), this.rng.range(-0.5, 0.5));
    const ped = new Pedestrian(
      this.cornerPos(corner, jitter),
      corner,
      "wait",
      this.rng.range(1.15, 1.5),
      jitter,
      this.rng.int(0, 4),
      false,
    );
    this.peds.push(ped);
    return ped;
  }

  /** A pedestrian who steps off the kerb mid-block at edge coordinate u and crosses the road. */
  spawnJaywalker(edge: Edge, u: number, fromRight: boolean): Pedestrian {
    const side = fromRight ? 1 : -1;
    const start = this.world.lanePoint(edge, side * SIDEWALK, u);
    const end = this.world.lanePoint(edge, -side * SIDEWALK, u + this.rng.range(-2, 2));
    const ped = new Pedestrian(start, { node: edge.from, sx: 1, sy: 1 }, "jaywalk", this.rng.range(1.5, 1.8), vec(0, 0), this.rng.int(0, 4), true);
    ped.target = end;
    this.peds.push(ped);
    return ped;
  }

  private choosePlan(c: CornerRef): Plan {
    const n = this.world.node(c.node);
    const options: Plan[] = [];
    // Crossing the arm on the sy side flips sx; crossing the arm on the sx side flips sy.
    const armY = n.out[c.sy > 0 ? 1 : 3] as number;
    const armX = n.out[c.sx > 0 ? 0 : 2] as number;
    options.push({ kind: "cross", to: { node: c.node, sx: -c.sx, sy: c.sy }, needsSignal: armY >= 0 });
    options.push({ kind: "cross", to: { node: c.node, sx: c.sx, sy: -c.sy }, needsSignal: armX >= 0 });
    const ni = n.i + c.sx;
    const nj = n.j + c.sy;
    if (ni >= 0 && ni < GRID) {
      const walk: Plan = { kind: "walk", to: { node: n.j * GRID + ni, sx: -c.sx, sy: c.sy }, needsSignal: false };
      options.push(walk, walk);
    }
    if (nj >= 0 && nj < GRID) {
      const walk: Plan = { kind: "walk", to: { node: nj * GRID + n.i, sx: c.sx, sy: -c.sy }, needsSignal: false };
      options.push(walk, walk);
    }
    return this.rng.pick(options);
  }

  update(dt: number, t: number, targetCount: number): void {
    for (const ped of this.peds) this.step(ped, dt, t);
    for (let i = this.peds.length - 1; i >= 0; i--) {
      if ((this.peds[i] as Pedestrian).state === "done") this.peds.splice(i, 1);
    }
    const walkers = this.peds.filter((p) => !p.jaywalker);
    if (walkers.length < targetCount) this.spawnWalker();
    else if (walkers.length > targetCount) {
      const idle = walkers.find((p) => p.state === "wait");
      if (idle) idle.state = "done";
    }
  }

  private step(ped: Pedestrian, dt: number, t: number): void {
    if (ped.state === "wait") {
      ped.v = 0;
      ped.vel = vec(0, 0);
      if (!ped.plan) ped.plan = this.choosePlan(ped.corner);
      const plan = ped.plan;
      let go = !plan.needsSignal;
      if (plan.needsSignal) {
        const ph = this.world.phaseAt(ped.corner.node, t);
        go = ph.name === "PED" && ph.elapsed < 3.5;
      }
      if (go) {
        ped.state = plan.kind === "cross" && plan.needsSignal ? "cross" : "walk";
        ped.target = this.cornerPos(plan.to, ped.jitter);
      }
      return;
    }
    const speed = ped.state === "cross" ? ped.speed * 1.2 : ped.speed;
    const delta = sub(ped.target, ped.pos);
    const d = dist(ped.target, ped.pos);
    if (d < 0.15) {
      if (ped.state === "jaywalk") {
        ped.state = "done";
        return;
      }
      if (ped.plan) ped.corner = ped.plan.to;
      ped.plan = null;
      ped.state = "wait";
      ped.v = 0;
      ped.vel = vec(0, 0);
      return;
    }
    const dir = norm(delta);
    const stepLen = Math.min(d, speed * dt);
    ped.pos = add(ped.pos, scale(dir, stepLen));
    ped.v = speed;
    ped.vel = scale(dir, speed);
    ped.heading = Math.atan2(dir.y, dir.x);
    ped.gait += dt * speed * 4.2;
  }
}
