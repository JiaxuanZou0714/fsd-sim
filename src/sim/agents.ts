import type { PathSample, Vec2 } from "./geometry";
import { dist2 } from "./geometry";
import type { VehicleKind } from "./vehicles";

export const PED_R = 0.35;

export interface Agent {
  readonly id: number;
  readonly kind: "vehicle" | "ped";
  readonly vkind: VehicleKind | "ped";
  pos: Vec2;
  heading: number;
  v: number;
  vel: Vec2;
  length: number;
  width: number;
  /** -1 left, 1 right, 2 hazard, 0 off; observable by other road users. */
  blinker: number;
}

/** Circle centres covering a vehicle footprint; each circle has radius width / 2. */
export function footprint(pos: Vec2, heading: number, length: number, width: number): Vec2[] {
  if (length <= width * 1.05) return [pos];
  const n = Math.max(2, Math.ceil((length - width) / (width * 0.85)) + 1);
  const half = (length - width) / 2;
  const c = Math.cos(heading);
  const s = Math.sin(heading);
  const out: Vec2[] = [];
  for (let i = 0; i < n; i++) {
    const k = -half + (2 * half * i) / (n - 1);
    out.push({ x: pos.x + c * k, y: pos.y + s * k });
  }
  return out;
}

export function agentRadius(a: Agent): number {
  return a.kind === "ped" ? PED_R : a.width / 2;
}

export function agentCircles(a: Agent): Vec2[] {
  return a.kind === "ped" ? [a.pos] : footprint(a.pos, a.heading, a.length, a.width);
}

export interface Obstacle {
  /** Distance from the scanning vehicle's front to the obstacle along the path. */
  gap: number;
  /** Obstacle speed projected on the path direction. */
  speed: number;
  agent: Agent;
}

export interface ScanOptions {
  self: Agent;
  /** Lateral margin added to the scanning vehicle's half-width. */
  margin: number;
  pedMargin: number;
  /** Prediction horizon for pedestrians and cyclists crossing the path. */
  horizon: number;
}

/**
 * First agent intersecting the corridor swept along the sampled path (sample s is relative to the
 * scanning vehicle's centre). Moving pedestrians are checked at the position they will occupy when
 * the vehicle reaches each sample.
 */
export function scanPath(samples: PathSample[], agents: readonly Agent[], opts: ScanOptions): Obstacle | null {
  if (samples.length === 0) return null;
  const self = opts.self;
  const first = samples[0];
  const last = samples[samples.length - 1];
  const reach = last.s - first.s + 14;
  const reach2 = reach * reach;
  const halfW = self.width / 2 + opts.margin;
  const tSpeed = Math.max(self.v, 1.5);
  let best: Obstacle | null = null;
  let bestS = Infinity;
  for (const a of agents) {
    if (a.id === self.id) continue;
    if (dist2(a.pos, first.p) > reach2) continue;
    // Agents behind the scanning vehicle are its followers' concern; ignoring them breaks mutual waits.
    if ((a.pos.x - first.p.x) * first.dir.x + (a.pos.y - first.p.y) * first.dir.y < -0.5) continue;
    const isPed = a.kind === "ped";
    const probes = agentCircles(a);
    const radius = agentRadius(a) + halfW + (isPed ? opts.pedMargin : 0);
    const r2 = radius * radius;
    const predict = (isPed || a.vkind === "bike") && a.v > 0.2;
    for (const smp of samples) {
      if (smp.s >= bestS) break;
      let hit = false;
      for (const q of probes) {
        if (dist2(q, smp.p) < r2) {
          hit = true;
          break;
        }
      }
      if (!hit && predict) {
        const t = Math.min(opts.horizon, smp.s / tSpeed);
        const q = { x: a.pos.x + a.vel.x * t, y: a.pos.y + a.vel.y * t };
        const rr = radius + 0.25 * t;
        hit = dist2(q, smp.p) < rr * rr;
      }
      if (hit) {
        bestS = smp.s;
        best = { gap: smp.s - self.length / 2, speed: a.vel.x * smp.dir.x + a.vel.y * smp.dir.y, agent: a };
        break;
      }
    }
  }
  return best;
}

export interface IdmParams {
  aMax: number;
  bComf: number;
  T: number;
  s0: number;
}

export function idmFree(v: number, v0: number, p: IdmParams): number {
  return p.aMax * (1 - Math.pow(v / Math.max(v0, 0.3), 4));
}

export function idmAccel(v: number, v0: number, gap: number, dv: number, p: IdmParams): number {
  const sStar = p.s0 + Math.max(0, v * p.T + (v * dv) / (2 * Math.sqrt(p.aMax * p.bComf)));
  return p.aMax * (1 - Math.pow(v / Math.max(v0, 0.3), 4) - (sStar / Math.max(gap, 0.05)) ** 2);
}

/** Speed cap from path curvature, propagated backwards with a comfortable deceleration. */
export function curvatureSpeed(samples: PathSample[], aLat: number, decel: number, vMax: number): number {
  let v = vMax;
  for (const smp of samples) {
    if (smp.kappa < 1e-3) continue;
    const vc = Math.sqrt(aLat / smp.kappa);
    const allowed = Math.sqrt(vc * vc + 2 * decel * Math.max(0, smp.s));
    if (allowed < v) v = allowed;
  }
  return v;
}
