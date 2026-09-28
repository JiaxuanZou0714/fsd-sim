import { it } from "vitest";
import { Simulation } from "../src/sim/simulation";

declare const process: { env: Record<string, string | undefined> };

it("trace", () => {
  const t0 = performance.now();
  const sim = new Simulation(Number(process.env.SEED ?? 1), { randomEvents: process.env.EV !== "0" });
  console.log("init ms", (performance.now() - t0).toFixed(0), "core lanes", sim.core.length);
  sim.engageFsd();
  const dur = Number(process.env.DUR ?? 120);
  let last = "";
  const w0 = performance.now();
  for (let i = 0; i < 30 * dur; i++) {
    sim.step(1 / 30);
    for (const e of sim.drainEvents()) console.log(sim.t.toFixed(1), "EVENT", e.text);
    const ego = sim.ego;
    const line = `${ego.fsd} ${ego.intent}/${ego.lateral}`;
    if (line !== last || i % 300 === 0) {
      console.log(
        sim.t.toFixed(1),
        line,
        `v=${ego.v.toFixed(1)} pos=${ego.pos.x.toFixed(0)},${ego.pos.y.toFixed(0)} d0=${ego.plan?.d0.toFixed(2)} dT=${ego.plan?.dT.toFixed(1)} feas=${ego.plan?.feasibleCount}/${ego.plan?.evaluated} ms=${ego.plan?.ms.toFixed(1)} lead=${ego.leadId} dest=${ego.ref ? (ego.ref.destS - (ego.plan?.s0 ?? 0)).toFixed(0) : "-"} ${ego.message}`,
      );
      last = line;
    }
  }
  const s = sim.stats;
  console.log(`wall ${(performance.now() - w0).toFixed(0)}ms arrivals=${s.arrivals} collisions=${s.collisions} dist=${s.fsdDistance.toFixed(0)} emergency=${s.emergencyStops} planMs=${s.planMs.toFixed(1)} npc=${sim.traffic.vehicles.length} peds=${sim.peds.peds.length}`);
}, 600_000);
