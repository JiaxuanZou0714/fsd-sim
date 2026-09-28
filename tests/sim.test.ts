import { describe, expect, it } from "vitest";
import { planRoute } from "../src/sim/routing";
import { Simulation } from "../src/sim/simulation";
import { World, turnOf } from "../src/sim/world";

describe("routing", () => {
  it("finds a U-turn-free route between any two edges", () => {
    const world = new World();
    for (let k = 0; k < 60; k++) {
      const a = world.edges[(k * 7) % world.edges.length]!;
      const b = world.edges[(k * 13 + 5) % world.edges.length]!;
      const route = planRoute(world, a, 10, b, 20);
      expect(route).not.toBeNull();
      const r = route!;
      expect(r[0]!.id).toBe(a.id);
      expect(r[r.length - 1]!.id).toBe(b.id);
      for (let i = 0; i + 1 < r.length; i++) {
        expect(r[i]!.to).toBe(r[i + 1]!.from);
        expect(turnOf(r[i]!.dir, r[i + 1]!.dir)).not.toBe("uturn");
      }
    }
  });

  it("loops around the block when the goal is behind on the same edge", () => {
    const world = new World();
    const e = world.edges[12]!;
    const route = planRoute(world, e, 40, e, 10)!;
    expect(route.length).toBeGreaterThan(3);
    expect(route[route.length - 1]!.id).toBe(e.id);
  });
});

function run(sim: Simulation, seconds: number, dt = 1 / 30): void {
  const steps = Math.round(seconds / dt);
  for (let i = 0; i < steps; i++) sim.step(dt);
}

describe("FSD driving", () => {
  it("reaches destinations repeatedly without collisions", () => {
    for (const seed of [1, 2, 3]) {
      const sim = new Simulation(seed, { randomEvents: true });
      expect(sim.engageFsd()).toBeNull();
      run(sim, 600);
      expect(sim.stats.collisions, `seed ${seed} collisions`).toBe(0);
      expect(sim.stats.arrivals, `seed ${seed} arrivals`).toBeGreaterThanOrEqual(3);
      expect(sim.ego.mode).toBe("fsd");
    }
  }, 120_000);

  it("stops for a red light", () => {
    const sim = new Simulation(4, { randomEvents: false, trafficCount: 0, pedCount: 0 });
    sim.engageFsd();
    let stoppedForLight = false;
    let moving = false;
    for (let i = 0; i < 30 * 180 && !stoppedForLight; i++) {
      sim.step(1 / 30);
      if (sim.ego.v > 6) moving = true;
      if (moving && sim.ego.intent === "light" && sim.ego.v < 0.05) stoppedForLight = true;
    }
    expect(stoppedForLight).toBe(true);
    const stop = sim.ego.activeStop!;
    expect(stop.color).toBe("R");
    const gap = stop.marker.s - sim.ego.pathS - 2.35;
    expect(gap).toBeGreaterThan(-0.5);
    expect(gap).toBeLessThan(3);
  });

  it("brakes for a jaywalker ahead", () => {
    const sim = new Simulation(5, { randomEvents: false, trafficCount: 0, pedCount: 0 });
    sim.engageFsd();
    for (let i = 0; i < 30 * 180 && !(sim.ego.v > 8 && sim.ego.intent === "cruise"); i++) sim.step(1 / 30);
    expect(sim.ego.v).toBeGreaterThan(8);
    expect(sim.spawnJaywalkerAhead()).toBe(true);
    let yielded = false;
    for (let i = 0; i < 30 * 20; i++) {
      sim.step(1 / 30);
      if (sim.ego.intent === "ped" || sim.ego.intent === "emergency") yielded = true;
    }
    expect(yielded).toBe(true);
    expect(sim.stats.collisions).toBe(0);
  });

  it("changes lanes around a stalled vehicle", () => {
    const sim = new Simulation(6, { randomEvents: false, trafficCount: 0, pedCount: 0 });
    sim.engageFsd();
    run(sim, 3);
    expect(sim.spawnStalledAhead()).toBe(true);
    let changed = false;
    for (let i = 0; i < 30 * 70; i++) {
      sim.step(1 / 30);
      if (sim.ego.lcEndS >= 0) changed = true;
    }
    expect(changed).toBe(true);
    expect(sim.stats.collisions).toBe(0);
  });

  it("counts a manual takeover as an intervention", () => {
    const sim = new Simulation(8);
    sim.engageFsd();
    run(sim, 2);
    sim.takeover();
    expect(sim.ego.mode).toBe("manual");
    expect(sim.stats.interventions).toBe(1);
  });
});
