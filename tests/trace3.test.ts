import { it } from "vitest";
import { Simulation } from "../src/sim/simulation";
declare const process: { env: Record<string, string | undefined> };
it("free", () => {
  const sim = new Simulation(Number(process.env.SEED ?? 1), { randomEvents: true });
  sim.engageFsd();
  let last = "lane";
  for (let i = 0; i < 30 * Number(process.env.DUR ?? 300); i++) {
    sim.step(1 / 30);
    for (const e of sim.drainEvents()) if (e.level === "danger") {
      const ego = sim.ego;
      console.log(sim.t.toFixed(1), e.text, `ego=${ego.pos.x.toFixed(1)},${ego.pos.y.toFixed(1)} h=${ego.heading.toFixed(2)} v=${ego.v.toFixed(1)} ${ego.fsd}/${ego.intent}/${ego.lateral}`);
      for (const a of sim.agents) if (a.id !== 0 && Math.hypot(a.pos.x - ego.pos.x, a.pos.y - ego.pos.y) < 7) console.log("   near", a.id, a.vkind, `${a.pos.x.toFixed(1)},${a.pos.y.toFixed(1)} h=${a.heading.toFixed(2)} v=${a.v.toFixed(1)} blink=${a.blinker}`);
    }
    if (sim.ego.fsd !== last) { if (sim.ego.fsd === "free") console.log(sim.t.toFixed(1), "FREE", sim.ego.freeReason); last = sim.ego.fsd; }
  }
}, 600_000);
