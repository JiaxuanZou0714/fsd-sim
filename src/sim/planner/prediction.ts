import type { Agent } from "../agents";
import { type Vec2, angleOf, dist2 } from "../geometry";
import type { Connector, Lane, RoadNetwork } from "../map/network";

export const PRED_DT = 0.25;
export const PRED_STEPS = 24;

export interface Prediction {
  agent: Agent;
  /** Predicted centre positions at t = k * PRED_DT, k = 0..PRED_STEPS. */
  pts: Vec2[];
  headings: number[];
  /** Stationary during the whole horizon. */
  still: boolean;
}

/**
 * Motion prediction from observable state only (pose, speed, estimated acceleration, turn signal).
 * Vehicles are projected along the lane graph; at a junction the turn signal selects the connector.
 */
export class Predictor {
  private readonly lastV = new Map<number, { v: number; t: number; a: number }>();

  constructor(private readonly net: RoadNetwork) {}

  /** Estimated longitudinal acceleration of each agent, from successive speed observations. */
  observe(agents: readonly Agent[], t: number): void {
    for (const a of agents) {
      const prev = this.lastV.get(a.id);
      if (prev && t > prev.t) {
        const raw = (a.v - prev.v) / (t - prev.t);
        this.lastV.set(a.id, { v: a.v, t, a: prev.a * 0.6 + raw * 0.4 });
      } else if (!prev) this.lastV.set(a.id, { v: a.v, t, a: 0 });
    }
    if (this.lastV.size > 2000) {
      const live = new Set(agents.map((a) => a.id));
      for (const id of this.lastV.keys()) if (!live.has(id)) this.lastV.delete(id);
    }
  }

  predict(agents: readonly Agent[], selfId: number, center: Vec2, range: number): Prediction[] {
    const out: Prediction[] = [];
    const r2 = range * range;
    for (const a of agents) {
      if (a.id === selfId || dist2(a.pos, center) > r2) continue;
      out.push(this.predictOne(a));
    }
    return out;
  }

  private distances(a: Agent): number[] {
    const acc = Math.max(-6, Math.min(2, this.lastV.get(a.id)?.a ?? 0));
    const d: number[] = [];
    let s = 0;
    let v = a.v;
    for (let k = 0; k <= PRED_STEPS; k++) {
      d.push(s);
      // Acceleration trends are only trusted for the first two seconds.
      const ak = k * PRED_DT < 2 ? acc : 0;
      const vNext = Math.max(0, v + ak * PRED_DT);
      s += ((v + vNext) / 2) * PRED_DT;
      v = vNext;
    }
    return d;
  }

  private predictOne(a: Agent): Prediction {
    const still = a.v < 0.3;
    if (still || a.kind === "ped") return this.constantVelocity(a, still);
    const route = this.mapRoute(a);
    if (!route) return this.constantVelocity(a, false);
    const d = this.distances(a);
    const pts: Vec2[] = [];
    const headings: number[] = [];
    let segIdx = 0;
    let offset = 0;
    for (const s of d) {
      let local = route.s0 + s - offset;
      while (segIdx < route.polys.length - 1 && local > route.polys[segIdx].length) {
        offset += route.polys[segIdx].length;
        segIdx++;
        local = route.s0 + s - offset;
      }
      const poly = route.polys[segIdx];
      const smp = poly.sampleAt(Math.min(local, poly.length + 30));
      pts.push({ x: smp.p.x + route.lat.x, y: smp.p.y + route.lat.y });
      headings.push(angleOf(smp.dir));
    }
    // Offset from the lane centre is kept for the first second, then decays.
    return { agent: a, pts, headings, still: false };
  }

  private constantVelocity(a: Agent, still: boolean): Prediction {
    const pts: Vec2[] = [];
    const headings: number[] = [];
    for (let k = 0; k <= PRED_STEPS; k++) {
      const t = still ? 0 : k * PRED_DT;
      pts.push({ x: a.pos.x + a.vel.x * t, y: a.pos.y + a.vel.y * t });
      headings.push(a.heading);
    }
    return { agent: a, pts, headings, still };
  }

  private chooseByBlinker(lane: Lane, blinker: number): Connector | null {
    if (lane.out.length === 0) return null;
    const want = blinker === -1 ? "left" : blinker === 1 ? "right" : "straight";
    return lane.out.find((c) => c.turn === want) ?? lane.out.find((c) => c.turn === "straight") ?? lane.out[0];
  }

  private mapRoute(a: Agent): { polys: { length: number; sampleAt: (s: number) => { p: Vec2; dir: Vec2 } }[]; s0: number; lat: Vec2 } | null {
    const m = this.net.matchLane(a.pos, a.heading, 2.6, 0.7);
    if (m) {
      const polys = [m.lane.poly];
      const c = this.chooseByBlinker(m.lane, a.blinker);
      if (c) {
        polys.push(c.poly, c.to.poly);
        const c2 = this.chooseByBlinker(c.to, 0);
        if (c2) polys.push(c2.poly);
      }
      const smp = m.lane.poly.sampleAt(m.s);
      const lat = { x: a.pos.x - smp.p.x, y: a.pos.y - smp.p.y };
      return { polys, s0: m.s, lat: a.vkind === "bike" ? lat : { x: lat.x * 0.5, y: lat.y * 0.5 } };
    }
    const mc = this.net.matchConnector(a.pos, a.heading, 2.2, 0.7);
    if (mc) {
      const polys = [mc.conn.poly, mc.conn.to.poly];
      const c2 = this.chooseByBlinker(mc.conn.to, 0);
      if (c2) polys.push(c2.poly);
      return { polys, s0: mc.s, lat: { x: 0, y: 0 } };
    }
    return null;
  }
}
