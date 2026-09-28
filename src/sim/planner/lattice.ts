import { type Agent, footprint } from "../agents";
import { type Vec2, clamp } from "../geometry";
import { LANE_W, type RoadNetwork, type SignalColor } from "../map/network";
import { PRED_DT, PRED_STEPS, type Prediction } from "./prediction";
import type { Gate, Reference } from "./reference";

export interface EgoState {
  pos: Vec2;
  heading: number;
  v: number;
  a: number;
  length: number;
  width: number;
}

export interface PlanContext {
  ref: Reference;
  ego: EgoState;
  preds: Prediction[];
  net: RoadNetwork;
  signal: (g: Gate) => SignalColor | null;
  prevDT: number | null;
  hintIdx: number;
}

/** The cost term that stopped the planner from choosing a faster trajectory. */
export type Limit =
  | "none"
  | "ped"
  | "vehicle"
  | "static"
  | "bike"
  | "follow"
  | "red"
  | "yellow"
  | "curve"
  | "dest"
  | "end"
  | "building";

export interface CandidateView {
  pts: Vec2[];
  feasible: boolean;
  chosen: boolean;
}

export interface Plan {
  s0: number;
  d0: number;
  dT: number;
  /** Chosen path in world coordinates, sampled every PATH_STEP metres from the vehicle. */
  path: Vec2[];
  /** Speed profile: arc length, speed and acceleration at t = k * PRED_DT. */
  s: Float32Array;
  v: Float32Array;
  a: Float32Array;
  feasible: boolean;
  limit: Limit;
  limitAgent: Agent | null;
  lateral: "keep" | "lanechange" | "nudge";
  /** The guide lane itself was blocked, so the lateral choice was an avoidance manoeuvre. */
  avoiding: boolean;
  candidates: CandidateView[];
  /** Best result per lateral path, for diagnostics. */
  pathSummary: { dT: number; Ls: number; feasible: boolean; cost: number; limit: Limit; pathCost: number; blockedAt: number }[];
  evaluated: number;
  feasibleCount: number;
  ms: number;
}

const PATH_STEP = 0.5;
const N = PRED_STEPS;

interface PathCand {
  dT: number;
  Ls: number;
  x: Float32Array;
  y: Float32Array;
  h: Float32Array;
  k: Float32Array;
  n: number;
  cost: number;
  /** Arc length at which the path hits a building, or Infinity. */
  blockedAt: number;
}

interface Profile {
  s: Float32Array;
  v: Float32Array;
  a: Float32Array;
  cost: number;
  hard: boolean;
}

function quinticCoeffs(x0: number, v0: number, a0: number, x1: number, T: number): [number, number, number, number, number, number] {
  const D = x1 - x0;
  const T2 = T * T;
  const T3 = T2 * T;
  return [
    x0,
    v0,
    a0 / 2,
    (20 * D - 12 * v0 * T - 3 * a0 * T2) / (2 * T3),
    (-30 * D + 16 * v0 * T + 3 * a0 * T2) / (2 * T3 * T),
    (12 * D - 6 * v0 * T - a0 * T2) / (2 * T3 * T2),
  ];
}

function evalPoly(c: number[], t: number): [number, number, number] {
  const [c0, c1, c2, c3, c4, c5] = c;
  const t2 = t * t;
  const t3 = t2 * t;
  const t4 = t3 * t;
  return [
    c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t4 * t,
    c1 + 2 * c2 * t + 3 * c3 * t2 + 4 * c4 * t3 + 5 * c5 * t4,
    2 * c2 + 6 * c3 * t + 12 * c4 * t2 + 20 * c5 * t3,
  ];
}

function finishProfile(s: Float32Array, v: Float32Array, a: Float32Array, hard: boolean): Profile {
  // Vehicles do not reverse in lane driving: freeze once the profile reaches standstill.
  let stopped = false;
  for (let k = 0; k <= N; k++) {
    if (stopped || v[k] < 0) {
      stopped = true;
      v[k] = 0;
      a[k] = 0;
      s[k] = k > 0 ? s[k - 1] : 0;
    }
    if (k > 0 && s[k] < s[k - 1]) s[k] = s[k - 1];
  }
  let cost = 0;
  for (let k = 1; k <= N; k++) {
    const jerk = (a[k] - a[k - 1]) / PRED_DT;
    cost += (0.08 * jerk * jerk + 0.25 * a[k] * a[k]) * PRED_DT;
    if (a[k] < -3.5) cost += 12 * (-3.5 - a[k]) ** 2 * PRED_DT;
    if (a[k] > 2.5) cost += 10 * (a[k] - 2.5) ** 2 * PRED_DT;
  }
  return { s, v, a, cost, hard };
}

function speedProfiles(v0: number, a0: number, vMax: number): Profile[] {
  const out: Profile[] = [];
  const a0c = clamp(a0, -6, 2.5);
  const targets = new Set<number>([0, 0.3, 0.55, 0.8, 1.0, 1.12].map((f) => Math.round(f * vMax * 10) / 10));
  targets.add(Math.round(v0 * 10) / 10);
  for (const vT of targets) {
    for (const T of [2.5, 5]) {
      const c3 = (vT - v0 - (2 / 3) * a0c * T) / (T * T);
      const c4 = (-a0c - 6 * c3 * T) / (12 * T * T);
      const s = new Float32Array(N + 1);
      const v = new Float32Array(N + 1);
      const a = new Float32Array(N + 1);
      const sT = v0 * T + (a0c / 2) * T * T + c3 * T ** 3 + c4 * T ** 4;
      for (let k = 0; k <= N; k++) {
        const t = k * PRED_DT;
        if (t <= T) {
          s[k] = v0 * t + (a0c / 2) * t * t + c3 * t ** 3 + c4 * t ** 4;
          v[k] = v0 + a0c * t + 3 * c3 * t * t + 4 * c4 * t ** 3;
          a[k] = a0c + 6 * c3 * t + 12 * c4 * t * t;
        } else {
          s[k] = sT + vT * (t - T);
          v[k] = vT;
          a[k] = 0;
        }
      }
      out.push(finishProfile(s, v, a, false));
    }
  }
  for (const ds of [1.5, 3, 5, 7.5, 10.5, 14, 18.5, 24, 31, 40, 51, 64]) {
    const minimumJerkT = Math.sqrt((5.77 * ds) / 1.8);
    const T = clamp(Math.max(minimumJerkT, (2 * ds) / Math.max(v0 + 0.3, 0.3)), 0.8, 11);
    const c = quinticCoeffs(0, v0, a0c, ds, T);
    const s = new Float32Array(N + 1);
    const v = new Float32Array(N + 1);
    const a = new Float32Array(N + 1);
    for (let k = 0; k <= N; k++) {
      const t = Math.min(k * PRED_DT, T);
      const [ss, vv, aa] = evalPoly(c, t);
      s[k] = k * PRED_DT > T ? ds : ss;
      v[k] = k * PRED_DT > T ? 0 : vv;
      a[k] = k * PRED_DT > T ? 0 : aa;
    }
    out.push(finishProfile(s, v, a, false));
  }
  {
    const s = new Float32Array(N + 1);
    const v = new Float32Array(N + 1);
    const a = new Float32Array(N + 1);
    let ss = 0;
    let vv = v0;
    for (let k = 0; k <= N; k++) {
      s[k] = ss;
      v[k] = vv;
      a[k] = vv > 0 ? -7 : 0;
      const vn = Math.max(0, vv - 7 * PRED_DT);
      ss += ((vv + vn) / 2) * PRED_DT;
      vv = vn;
    }
    out.push(finishProfile(s, v, a, true));
  }
  return out;
}

export class LatticePlanner {
  plan(ctx: PlanContext): Plan {
    const t0 = performance.now();
    const { ref, ego } = ctx;
    const proj = ref.poly.project(ego.pos, ctx.hintIdx, 60);
    const s0 = proj.s;
    const d0 = proj.lateral;
    const refDir = ref.poly.sampleAt(s0).dir;
    const dPsi = Math.atan2(Math.sin(ego.heading) * refDir.x - Math.cos(ego.heading) * refDir.y, Math.cos(ego.heading) * refDir.x + Math.sin(ego.heading) * refDir.y);
    const dp0 = clamp(Math.tan(clamp(dPsi, -1.2, 1.2)), -3, 3);
    const idx0 = ref.index(s0);
    const vLimitHere = ref.speed[idx0] || 13.9;

    const profiles = speedProfiles(ego.v, ego.a, Math.max(vLimitHere, 5));
    let maxS = 0;
    for (const p of profiles) maxS = Math.max(maxS, p.s[N]);
    const pathLen = Math.min(maxS + ego.length + 6, ref.poly.length - s0 + 20);

    // ---- Lateral path candidates --------------------------------------------------------------
    const idxAhead = ref.index(Math.min(ref.poly.length, s0 + 12));
    const leftLanes = Math.max(0, Math.round((ref.leftEdge[idxAhead] - LANE_W / 2) / LANE_W));
    const rightLanes = Math.max(0, Math.round((ref.rightEdge[idxAhead] - LANE_W / 2) / LANE_W));
    const targets = new Set<number>([0, -0.6, 0.6, -1.2, 1.2]);
    for (let k = 1; k <= Math.min(2, leftLanes); k++) targets.add(-k * LANE_W);
    for (let k = 1; k <= Math.min(2, rightLanes); k++) targets.add(k * LANE_W);
    if (ref.oppWidth[idxAhead] > 0 && leftLanes === 0) targets.add(-LANE_W);
    if (Math.abs(d0) > 0.4) targets.add(Math.round(d0 * 10) / 10);
    const lengths = [Math.max(7, 1.6 * ego.v + 6), Math.max(15, 3.0 * ego.v + 10)];
    const paths: PathCand[] = [];
    for (const dT of targets) {
      for (const Ls of lengths) paths.push(this.makePath(ctx, s0, d0, dp0, dT, Ls, pathLen));
    }

    // ---- Static terms of the environment -------------------------------------------------------
    const gates = ref.gates
      .map((g) => ({ rel: g.s - s0, color: ctx.signal(g) }))
      .filter((g) => g.color && g.color !== "G" && g.rel > ego.length / 2 - 0.5);
    const destRel = ref.destS - s0;
    const endRel = ref.poly.length - s0 - 2;
    const preds = ctx.preds;
    const egoR = ego.width / 2;
    const probeOffsets = (() => {
      const fp = footprint({ x: 0, y: 0 }, 0, ego.length, ego.width);
      return fp.map((p) => p.x);
    })();

    let best: { path: PathCand; prof: Profile; cost: number; feasible: boolean } | null = null;
    let fallback: { path: PathCand; prof: Profile; cost: number; hitT: number } | null = null;
    const perPathBest = new Map<PathCand, { cost: number; feasible: boolean }>();
    const results: { path: PathCand; prof: Profile; cost: number; feasible: boolean; limit: Limit; agent: Agent | null; sN: number }[] = [];
    let feasibleCount = 0;

    for (const path of paths) {
      for (const prof of profiles) {
        let cost = path.cost + prof.cost;
        let feasible = true;
        let limit: Limit = "none";
        let limitAgent: Agent | null = null;
        let worstTerm = 0;
        const note = (c: number, l: Limit, ag: Agent | null): void => {
          if (c > worstTerm) {
            worstTerm = c;
            limit = l;
            limitAgent = ag;
          }
        };
        let hitT = Infinity;
        for (let k = 0; k <= N && feasible; k++) {
          const t = k * PRED_DT;
          const sk = prof.s[k];
          const vk = prof.v[k];
          const i = Math.min(path.n - 1, Math.floor(sk / PATH_STEP));
          const x = path.x[i];
          const y = path.y[i];
          const h = path.h[i];
          const kap = path.k[i];
          const refIdx = ref.index(Math.min(ref.poly.length, s0 + sk));
          const vLim = ref.speed[refIdx] || vLimitHere;
          // Speed tracking, progress and dynamic limits.
          const vDes = Math.min(vLim, Math.sqrt(2.2 / Math.max(Math.abs(kap), 1e-3)));
          if (k > 0) cost += 0.55 * (vDes - vk) ** 2 * PRED_DT;
          if (vk > vLim + 0.4) cost += 25 * (vk - vLim) ** 2 * PRED_DT;
          const ay = vk * vk * Math.abs(kap);
          if (ay > 2.4) {
            const c = 40 * (ay - 2.4) ** 2 * PRED_DT;
            cost += c;
            note(c, "curve", null);
            if (ay > 4.5 && vk > 2) {
              feasible = false;
              limit = "curve";
            }
          }
          if (sk + ego.length / 2 > path.blockedAt) {
            feasible = false;
            limit = "building";
            break;
          }
          // Signals and destination.
          for (const g of gates) {
            if (sk + ego.length / 2 > g.rel + 0.2) {
              if (g.color === "R") {
                feasible = false;
                limit = "red";
              } else if (t > 2.0) {
                cost += 800;
                note(800, "yellow", null);
              }
            }
          }
          if (!feasible) break;
          if (sk > destRel + 0.5) {
            const c = 400 * (sk - destRel);
            cost += c;
            note(c, "dest", null);
          }
          if (sk > endRel) {
            const c = 400 * (sk - endRel);
            cost += c;
            note(c, "end", null);
          }
          // Other road users.
          const ch = Math.cos(h);
          const sh = Math.sin(h);
          for (const pr of preds) {
            const a = pr.agent;
            const ap = pr.pts[k];
            const dx = ap.x - x;
            const dy = ap.y - y;
            const reach = (ego.length + a.length) / 2 + 3.5;
            if (dx * dx + dy * dy > reach * reach) continue;
            const aR = a.kind === "ped" ? 0.35 : a.width / 2;
            const aC = a.kind === "ped" ? [ap] : footprint(ap, pr.headings[k], a.length, a.width);
            let clear = Infinity;
            for (const off of probeOffsets) {
              const ex = x + ch * off;
              const ey = y + sh * off;
              for (const q of aC) {
                const c = Math.hypot(q.x - ex, q.y - ey) - egoR - aR;
                if (c < clear) clear = c;
              }
            }
            const margin = (a.kind === "ped" ? 0.55 : 0.2) + 0.05 * t;
            const cls: Limit = a.kind === "ped" ? "ped" : a.vkind === "bike" ? "bike" : pr.still ? "static" : "vehicle";
            if (clear < margin) {
              hitT = Math.min(hitT, t);
              if (t <= 5) {
                feasible = false;
                limit = cls;
                limitAgent = a;
                break;
              }
              cost += 2000;
              note(2000, cls, a);
              continue;
            }
            if (clear < 1.6) {
              const c = 6 * Math.exp(-clear / 0.45) * (1 + vk * 0.12) * PRED_DT * 4;
              cost += c;
              note(c, cls, a);
            }
            // Following distance to road users ahead in the direction of travel.
            const along = dx * ch + dy * sh;
            const lat = Math.abs(-dx * sh + dy * ch);
            if (along > 0 && lat < (ego.width + a.width) / 2 + 0.5) {
              const gap = along - (ego.length + a.length) / 2;
              // Stationary road users get extra room so there is space to pull out around them.
              const want = (pr.still && a.kind === "vehicle" ? 4.5 : 2.4) + vk * 1.15;
              if (gap < want) {
                const c = 7 * (want - gap) ** 2 * PRED_DT;
                cost += c;
                note(c, a.kind === "ped" ? "ped" : pr.still ? "static" : a.vkind === "bike" ? "bike" : "follow", a);
              }
            }
          }
        }
        // Progress, capped at the destination.
        const progress = Math.min(prof.s[N], Math.max(0, destRel));
        cost -= 1.1 * progress;
        if (prof.hard) cost += 60;
        if (!feasible) {
          if (!fallback || hitT > fallback.hitT || (hitT === fallback.hitT && cost < fallback.cost)) fallback = { path, prof, cost, hitT };
        } else {
          feasibleCount++;
          if (!best || cost < best.cost) best = { path, prof, cost, feasible: true };
        }
        const pb = perPathBest.get(path);
        if (!pb || (feasible && (!pb.feasible || cost < pb.cost)) || (!pb.feasible && !feasible && cost < pb.cost)) perPathBest.set(path, { cost, feasible });
        results.push({ path, prof, cost, feasible, limit, agent: limitAgent, sN: prof.s[N] });
      }
    }

    let chosen: { path: PathCand; prof: Profile; feasible: boolean };
    if (best) chosen = best;
    else if (fallback) {
      // No safe option: brake as hard as possible along the path that delays contact the longest.
      const brake = profiles.find((p) => p.hard) as Profile;
      chosen = { path: fallback.path, prof: brake, feasible: false };
    } else chosen = { path: paths[0], prof: profiles[profiles.length - 1], feasible: false };

    // ---- Explanation: what prevented a faster trajectory? --------------------------------------
    let limit: Limit = "none";
    let limitAgent: Agent | null = null;
    const chosenS = chosen.prof.s[N];
    // Only explain when the chosen trajectory ends below the desired speed.
    const belowDesired = chosen.prof.v[N] < 0.85 * vLimitHere || chosen.prof.v[8] < ego.v - 0.8;
    const faster = belowDesired ? results.filter((r) => r.path === chosen.path && r.sN > chosenS + 4) : [];
    if (faster.length > 0) {
      // The smallest speed-up that is blocked (or penalised) names the binding constraint.
      const ranked = [...faster].sort((a, b) => a.sN - b.sN);
      const pick = ranked.find((r) => !r.feasible && r.limit !== "none") ?? ranked.find((r) => r.limit !== "none");
      if (pick) {
        limit = pick.limit;
        limitAgent = pick.agent;
      }
    }
    const zeroPath = paths.filter((p) => p.dT === 0);
    const guideBlocked = zeroPath.every((p) => !(perPathBest.get(p)?.feasible ?? false)) ||
      results.some((r) => r.path.dT === 0 && r.feasible && (r.limit === "static" || r.limit === "bike") && r.cost > chosen.path.cost + 30);
    const lateralOffset = Math.abs(chosen.path.dT);
    const lateral = lateralOffset > LANE_W * 0.6 ? "lanechange" : lateralOffset > 0.5 && Math.abs(d0) > 0.3 ? "nudge" : "keep";
    if (limit === "curve" && chosen.prof.v[N] > ego.v + 1) limit = "none";

    const pathPts: Vec2[] = [];
    for (let i = 0; i < chosen.path.n; i++) pathPts.push({ x: chosen.path.x[i], y: chosen.path.y[i] });

    const candidates: CandidateView[] = paths.map((p) => {
      const pts: Vec2[] = [];
      const stop = Math.min(p.n, Math.floor(Math.max(12, Math.min(pathLen, 45)) / PATH_STEP));
      for (let i = 0; i < stop; i += 4) pts.push({ x: p.x[i], y: p.y[i] });
      return { pts, feasible: perPathBest.get(p)?.feasible ?? false, chosen: p === chosen.path };
    });

    const pathSummary = paths.map((p) => {
      const rs = results.filter((r) => r.path === p);
      const bestR = rs.filter((r) => r.feasible).sort((a, b) => a.cost - b.cost)[0] ?? rs.sort((a, b) => b.sN - a.sN)[0];
      return { dT: p.dT, Ls: p.Ls, feasible: !!bestR?.feasible, cost: bestR?.cost ?? Infinity, limit: bestR?.limit ?? "none", pathCost: p.cost, blockedAt: p.blockedAt };
    });
    return {
      pathSummary,
      s0,
      d0,
      dT: chosen.path.dT,
      path: pathPts,
      s: chosen.prof.s,
      v: chosen.prof.v,
      a: chosen.prof.a,
      feasible: chosen.feasible,
      limit,
      limitAgent,
      lateral,
      avoiding: lateral !== "keep" && guideBlocked,
      candidates,
      evaluated: results.length,
      feasibleCount,
      ms: performance.now() - t0,
    };
  }

  private makePath(ctx: PlanContext, s0: number, d0: number, dp0: number, dT: number, Ls: number, len: number): PathCand {
    const { ref, net, ego } = ctx;
    const n = Math.max(2, Math.ceil(len / PATH_STEP) + 1);
    const x = new Float32Array(n);
    const y = new Float32Array(n);
    const h = new Float32Array(n);
    const k = new Float32Array(n);
    const c = quinticCoeffs(d0, dp0, 0, dT, Ls);
    let cost = 0;
    let blockedAt = Infinity;
    const hw = ego.width / 2;
    for (let i = 0; i < n; i++) {
      const sigma = i * PATH_STEP;
      const smp = ref.poly.sampleAt(s0 + sigma);
      const [d, dd] = sigma < Ls ? evalPoly(c, sigma) : [dT, 0, 0];
      const nx = -smp.dir.y;
      const ny = smp.dir.x;
      x[i] = smp.p.x + nx * d;
      y[i] = smp.p.y + ny * d;
      h[i] = Math.atan2(smp.dir.y, smp.dir.x) + Math.atan(dd);
      if (i % 2 === 0 && sigma < 60) {
        const idx = ref.index(Math.min(ref.poly.length, s0 + sigma));
        const junction = ref.inJunction[idx] === 1;
        const leftLimit = -(ref.leftEdge[idx] - hw - 0.15);
        const rightLimit = ref.rightEdge[idx] - hw - 0.15;
        const w = PATH_STEP * 2;
        if (d < leftLimit) {
          const excess = leftLimit - d;
          const opp = ref.oppWidth[idx];
          // Oncoming lanes are usable at a price; beyond them lies the kerb.
          cost += junction ? 22 * excess * w : excess <= opp ? 9 * excess * w : (9 * opp + 150 * (excess - opp)) * w;
        }
        if (d > rightLimit) cost += (junction ? 22 : 150) * (d - rightLimit) * w;
        if (sigma < 30) cost += 0.9 * d * d * w * 0.1;
        if (blockedAt === Infinity && net.grid.clearanceAt({ x: x[i], y: y[i] }) < hw + 0.15) blockedAt = sigma;
      }
    }
    for (let i = 1; i < n - 1; i++) {
      let dh = h[i + 1] - h[i - 1];
      while (dh > Math.PI) dh -= Math.PI * 2;
      while (dh < -Math.PI) dh += Math.PI * 2;
      k[i] = dh / (2 * PATH_STEP);
    }
    k[0] = k[1];
    k[n - 1] = k[n - 2];
    let maxK = 0;
    for (let i = 0; i < Math.min(n, 60); i++) maxK = Math.max(maxK, Math.abs(k[i]));
    if (maxK > 0.26) cost += 5000;
    cost += 1.0 * dT * dT + (Math.abs(dT) > 0.1 && Math.abs(dT) < LANE_W * 0.6 ? 1.5 : 0);
    if (Math.abs(dT) > LANE_W * 0.6) cost += 7;
    if (ctx.prevDT !== null) cost += 4 * (dT - ctx.prevDT) ** 2 + (Math.abs(dT - ctx.prevDT) > 0.05 ? 1.5 : 0);
    return { dT, Ls, x, y, h, k, n, cost, blockedAt };
  }
}
