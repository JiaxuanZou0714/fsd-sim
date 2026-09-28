import { describe, expect, it } from "vitest";
import { Rng, dist } from "../src/sim/geometry";
import { goalForPoint, planRoute } from "../src/sim/planner/route";
import { Simulation, getNetwork } from "../src/sim/simulation";

const DT = 1 / 30;

function run(sim: Simulation, seconds: number, until?: () => boolean): void {
  const steps = Math.round(seconds / DT);
  for (let i = 0; i < steps; i++) {
    sim.step(DT);
    if (until?.()) return;
  }
}

describe("road network from OpenStreetMap", () => {
  const net = getNetwork();

  it("builds lanes, junctions, conflicts, signals and crossings", () => {
    expect(net.lanes.length).toBeGreaterThan(700);
    expect(net.connectors.length).toBeGreaterThan(900);
    expect(net.connectors.reduce((a, c) => a + c.conflicts.length, 0)).toBeGreaterThan(1000);
    expect(net.controllers.length).toBeGreaterThan(8);
    expect(net.crossings.length).toBeGreaterThan(40);
    expect(net.buildings.length).toBeGreaterThan(1000);
    expect([...net.junctions.values()].some((j) => j.circular)).toBe(true);
  });

  it("has a large strongly connected core where every lane has a way out", () => {
    expect(net.core.size).toBeGreaterThan(600);
    for (const l of net.core) expect(l.out.length + (l.left ? 1 : 0) + (l.right ? 1 : 0)).toBeGreaterThan(0);
  });

  it("routes between random lanes of the core, including destinations behind the vehicle", () => {
    const rng = new Rng(3);
    const core = [...net.core].filter((l) => l.poly.length > 20);
    for (let i = 0; i < 60; i++) {
      const a = rng.pick(core);
      const b = rng.pick(core);
      const route = planRoute(a, 0, goalForPoint(b, b.poly.length * 0.3));
      expect(route, `route ${a.id} -> ${b.id}`).not.toBeNull();
      const r = route!;
      expect(r[0].lane).toBe(a);
      for (let k = 1; k < r.length; k++) {
        if (r[k].via === "conn") expect(r[k].conn?.from).toBe(r[k - 1].lane);
        if (r[k].via === "lc") expect([r[k - 1].lane.left, r[k - 1].lane.right]).toContain(r[k].lane);
      }
    }
    const l = core[0];
    const loop = planRoute(l, l.poly.length - 2, goalForPoint(l, 2));
    expect(loop).not.toBeNull();
    expect(loop!.length).toBeGreaterThan(2);
  });
});

describe("FSD behaviour", () => {
  it("yields to a pedestrian crossing mid-block", () => {
    const sim = new Simulation(5, { randomEvents: false, trafficCount: 0, pedCount: 0 });
    sim.engageFsd();
    run(sim, 60, () => sim.ego.v > 6 && sim.ego.intent === "cruise");
    expect(sim.spawnJaywalkerAhead()).toBe(true);
    let yielded = false;
    run(sim, 20, () => {
      if (sim.ego.intent === "ped" || sim.ego.intent === "emergency") yielded = true;
      return false;
    });
    expect(yielded).toBe(true);
    expect(sim.stats.collisions).toBe(0);
  });

  it("gets past a stalled vehicle in its lane", () => {
    const sim = new Simulation(6, { randomEvents: false, trafficCount: 0, pedCount: 0 });
    sim.engageFsd();
    run(sim, 60, () => sim.ego.v > 6 && sim.ego.intent === "cruise");
    expect(sim.spawnStalledAhead()).toBe(true);
    const stalled = sim.traffic.vehicles.find((v) => v.stalled)!;
    let passed = false;
    run(sim, 90, () => {
      const rel = { x: sim.ego.pos.x - stalled.pos.x, y: sim.ego.pos.y - stalled.pos.y };
      passed = rel.x * Math.cos(stalled.heading) + rel.y * Math.sin(stalled.heading) > 8;
      return passed;
    });
    expect(passed).toBe(true);
    expect(sim.stats.collisions).toBe(0);
  });

  it("recovers from an off-road start and reaches lane driving", () => {
    for (const seed of [11, 12, 13]) {
      const sim = new Simulation(seed, { randomEvents: false, trafficCount: 20, pedCount: 10 });
      sim.engageFsd();
      run(sim, 2);
      expect(sim.placeOffRoad()).toBe(true);
      expect(sim.ego.fsd === "free" || sim.ego.fsd === "blocked").toBe(true);
      run(sim, 120, () => sim.ego.fsd === "lane" && sim.stats.recoveries > 0);
      expect(sim.ego.fsd, `seed ${seed}`).toBe("lane");
      expect(sim.stats.collisions, `seed ${seed}`).toBe(0);
    }
  }, 120_000);

  it("turns around when placed against traffic on a one-way street", () => {
    for (const seed of [21, 22]) {
      const sim = new Simulation(seed, { randomEvents: false, trafficCount: 15, pedCount: 10 });
      sim.engageFsd();
      run(sim, 2);
      expect(sim.placeWrongWay()).toBe(true);
      const start = { ...sim.ego.pos };
      run(sim, 120, () => sim.ego.fsd === "lane" && sim.stats.recoveries > 0);
      expect(sim.ego.fsd, `seed ${seed}`).toBe("lane");
      const loc = sim.ego.localize(sim.net);
      expect(loc).not.toBeNull();
      expect(dist(start, sim.ego.pos)).toBeGreaterThan(1);
      expect(sim.stats.collisions).toBe(0);
    }
  }, 120_000);

  it("drives in mixed traffic without collisions and reaches destinations", () => {
    let arrivals = 0;
    for (const seed of [23, 25, 28]) {
      const sim = new Simulation(seed, { randomEvents: true });
      sim.engageFsd();
      run(sim, 240);
      expect(sim.stats.collisions, `seed ${seed}`).toBe(0);
      expect(sim.stats.fsdDistance, `seed ${seed}`).toBeGreaterThan(600);
      arrivals += sim.stats.arrivals;
    }
    expect(arrivals).toBeGreaterThanOrEqual(3);
  }, 300_000);

  it("counts a manual takeover as an intervention", () => {
    const sim = new Simulation(8, { trafficCount: 10, pedCount: 0 });
    sim.engageFsd();
    run(sim, 2);
    sim.takeover();
    expect(sim.ego.mode).toBe("manual");
    expect(sim.stats.interventions).toBe(1);
  });
});
