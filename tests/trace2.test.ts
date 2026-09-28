import { it } from "vitest";
import { Simulation } from "../src/sim/simulation";
declare const process: { env: Record<string, string | undefined> };
it("probe", () => {
  const sim = new Simulation(Number(process.env.SEED ?? 1), { randomEvents: true });
  sim.engageFsd();
  const at = Number(process.env.AT ?? 40);
  while (sim.t < at) sim.step(1 / 30);
  const ego = sim.ego;
  const p = ego.plan!;
  console.log("ego", ego.pos, "v", ego.v, "s0", p.s0, "d0", p.d0, "limit", p.limit, "lead", ego.leadId);
  for (const x of p.pathSummary) console.log("  path dT", x.dT.toFixed(1), "Ls", x.Ls.toFixed(0), "feas", x.feasible, "cost", x.cost.toFixed(0), "pathCost", x.pathCost.toFixed(0), "limit", x.limit, "blocked", x.blockedAt);
  const lead = sim.traffic.get(85) ?? sim.traffic.vehicles.find((v) => v.stalled);
  if (lead) console.log("stalled", lead.id, lead.pos, "lane", lead.seg.lane?.id, "k", lead.seg.lane?.k, "s", lead.s.toFixed(1), "/", lead.seg.poly.length.toFixed(1));
  const ref = ego.ref!;
  const i = ref.index(p.s0);
  for (let s = p.s0; s < p.s0 + 40; s += 4) { const k = ref.index(s); console.log("  ref s", s.toFixed(0), "lane", ref.lane[k], "left", ref.leftEdge[k].toFixed(1), "right", ref.rightEdge[k].toFixed(1), "opp", ref.oppWidth[k].toFixed(1), "junc", ref.inJunction[k]); }
  void i;
});
