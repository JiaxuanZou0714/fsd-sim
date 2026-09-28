import type { PathSample, Vec2 } from "./geometry";
import { dist2 } from "./geometry";

export const CAR_LEN = 4.7;
export const CAR_HALF_LEN = CAR_LEN / 2;
export const CAR_WIDTH = 1.95;
export const CAR_CIRCLE_R = 1.02;
export const PED_R = 0.35;

export interface Agent {
  readonly id: number;
  readonly kind: "car" | "ped";
  pos: Vec2;
  heading: number;
  v: number;
  vel: Vec2;
}

export function carCircles(pos: Vec2, heading: number): Vec2[] {
  const c = Math.cos(heading);
  const s = Math.sin(heading);
  return [-1.5, 0, 1.5].map((k) => ({ x: pos.x + c * k, y: pos.y + s * k }));
}

export interface Obstacle {
  /** Bumper-to-obstacle distance along the path. */
  gap: number;
  /** Obstacle speed projected on the path direction. */
  speed: number;
  agent: Agent;
}

export interface ScanOptions {
  selfId: number;
  /** Path half-width used for vehicle obstacles. */
  tube: number;
  /** Additional margin applied to pedestrians. */
  pedMargin: number;
  /**
   * Prediction horizon for pedestrians in seconds. Each path sample is checked against
   * where the pedestrian will be when the scanning vehicle reaches that sample.
   */
  pedHorizon: number;
  /** Speed of the scanning vehicle, used to time-align pedestrian predictions. */
  selfSpeed: number;
}

/**
 * Finds the first agent that intersects the swept corridor along the sampled path.
 * Sample s values are relative to the scanning vehicle's centre.
 */
export function scanPath(samples: PathSample[], agents: readonly Agent[], opts: ScanOptions): Obstacle | null {
  if (samples.length === 0) return null;
  const first = samples[0] as PathSample;
  const last = samples[samples.length - 1] as PathSample;
  const reach = last.s - first.s + 8;
  const reach2 = reach * reach;
  let best: Obstacle | null = null;
  let bestS = Infinity;

  for (const a of agents) {
    if (a.id === opts.selfId) continue;
    if (dist2(a.pos, first.p) > reach2) continue;
    // Agents behind the scanning vehicle are its followers' concern; ignoring them breaks mutual waits.
    if ((a.pos.x - first.p.x) * first.dir.x + (a.pos.y - first.p.y) * first.dir.y < -0.5) continue;
    const isPed = a.kind === "ped";
    const probes: Vec2[] = isPed ? [a.pos] : carCircles(a.pos, a.heading);
    const radius = isPed ? PED_R + opts.tube + opts.pedMargin : CAR_CIRCLE_R + opts.tube;
    const r2 = radius * radius;
    const predictPed = isPed && a.v > 0.2 && opts.pedHorizon > 0;
    const tSpeed = Math.max(opts.selfSpeed, 1.5);
    for (const smp of samples) {
      if (smp.s >= bestS) break;
      let hit = false;
      for (const q of probes) {
        if (dist2(q, smp.p) < r2) {
          hit = true;
          break;
        }
      }
      if (!hit && predictPed) {
        const t = Math.min(opts.pedHorizon, smp.s / tSpeed);
        const q = { x: a.pos.x + a.vel.x * t, y: a.pos.y + a.vel.y * t };
        const rr = radius + 0.25 * t;
        hit = dist2(q, smp.p) < rr * rr;
      }
      if (hit) {
        bestS = smp.s;
        best = {
          gap: smp.s - CAR_HALF_LEN,
          speed: a.vel.x * smp.dir.x + a.vel.y * smp.dir.y,
          agent: a,
        };
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

export const IDM_DEFAULT: IdmParams = { aMax: 2.2, bComf: 3.0, T: 1.3, s0: 2.2 };

export function idmFree(v: number, v0: number, p: IdmParams): number {
  const target = Math.max(v0, 0.3);
  return p.aMax * (1 - Math.pow(v / target, 4));
}

export function idmAccel(v: number, v0: number, gap: number, dv: number, p: IdmParams): number {
  const target = Math.max(v0, 0.3);
  const sStar = p.s0 + Math.max(0, v * p.T + (v * dv) / (2 * Math.sqrt(p.aMax * p.bComf)));
  const g = Math.max(gap, 0.05);
  return p.aMax * (1 - Math.pow(v / target, 4) - (sStar / g) ** 2);
}

/**
 * Speed cap from path curvature: v(s) <= sqrt(aLat / kappa), propagated backwards
 * with a comfortable deceleration so the vehicle slows before the curve.
 */
export function curvatureSpeed(samples: PathSample[], selfS: number, aLat: number, decel: number, vMax: number): number {
  let v = vMax;
  for (const smp of samples) {
    if (smp.kappa < 1e-3) continue;
    const vc = Math.sqrt(aLat / smp.kappa);
    const d = Math.max(0, smp.s - selfS);
    const allowed = Math.sqrt(vc * vc + 2 * decel * d);
    if (allowed < v) v = allowed;
  }
  return v;
}
