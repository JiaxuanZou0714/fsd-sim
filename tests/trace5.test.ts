import { it } from "vitest";
import { Simulation } from "../src/sim/simulation";
declare const process: { env: Record<string, string | undefined> };
it("dest", () => {
  const sim = new Simulation(Number(process.env.SEED ?? 10), { randomEvents: true });
  sim.engageFsd();
  const d = sim.ego.dest!;
  console.log("dest lane", d.lane.id, "road", d.lane.road.name, "s", d.s.toFixed(0), "pos", d.pos, "core", sim.net.core.has(d.lane));
  for (let i = 0; i < 30 * 300; i++) {
    sim.step(1 / 30);
    for (const e of sim.drainEvents()) if (e.text.includes("到达")) console.log(sim.t.toFixed(0), e.text);
    if (i % 450 === 0) { const e = sim.ego; console.log(sim.t.toFixed(0), e.fsd, e.intent, "destRem", e.ref ? (e.ref.destS - (e.plan?.s0 ?? 0)).toFixed(0) : "-", "refLen", e.ref?.poly.length.toFixed(0), "route", e.route?.length, "pos", e.pos.x.toFixed(0), e.pos.y.toFixed(0), "straight", Math.hypot(e.pos.x - d.pos.x, e.pos.y - d.pos.y).toFixed(0)); }
  }
}, 600000);
