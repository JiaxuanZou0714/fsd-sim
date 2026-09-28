import { it } from "vitest";
import { Simulation } from "../src/sim/simulation";
declare const process: { env: Record<string, string | undefined> };
// Stress sweep; runs only when SEEDS is set, e.g. SEEDS=1,2,3 DUR=300 npx vitest run tests/sweep.test.ts
it.runIf(!!process.env.SEEDS)("sweep", () => {
  const seeds = (process.env.SEEDS ?? "1,2,3,4,5,6").split(",").map(Number);
  const dur = Number(process.env.DUR ?? 300);
  for (const seed of seeds) {
    const sim = new Simulation(seed, { randomEvents: true });
    sim.engageFsd();
    const bad: string[] = [];
    let stuck = 0; let maxStuck = 0; let where = "";
    const w0 = performance.now();
    for (let i = 0; i < 30 * dur; i++) {
      sim.step(1 / 30);
      for (const e of sim.drainEvents()) if (e.level === "danger") bad.push(`${sim.t.toFixed(0)}:${e.text}`);
      const ego = sim.ego;
      stuck = ego.v < 0.2 && ego.intent !== "red" && ego.intent !== "arrived" ? stuck + 1 / 30 : 0;
      if (stuck > maxStuck) { maxStuck = stuck; where = `t=${sim.t.toFixed(0)} ${ego.fsd}/${ego.intent} pos=${ego.pos.x.toFixed(0)},${ego.pos.y.toFixed(0)} lead=${ego.leadId}`; }
    }
    const s = sim.stats;
    console.log(`seed ${seed} arrivals=${s.arrivals} collisions=${s.collisions} dist=${s.fsdDistance.toFixed(0)} recov=${s.recoveries} emerg=${s.emergencyStops} planMs=${s.planMs.toFixed(1)} wall=${((performance.now() - w0) / 1000).toFixed(0)}s maxStuck=${maxStuck.toFixed(0)}s ${where}`, bad.join(" | "));
  }
}, 3_600_000);
