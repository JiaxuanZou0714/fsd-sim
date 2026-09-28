import { it } from "vitest";
import { Simulation } from "../src/sim/simulation";
declare const process: { env: Record<string, string | undefined> };
it("chain", () => {
  const sim = new Simulation(Number(process.env.SEED ?? 7), { randomEvents: true });
  sim.engageFsd();
  while (sim.t < Number(process.env.AT ?? 250)) sim.step(1 / 30);
  const ego = sim.ego;
  console.log("ego", ego.pos, ego.intent, "lead", ego.leadId);
  let id = ego.leadId;
  for (let k = 0; k < 8; k++) {
    const c = sim.traffic.get(id);
    if (!c) { const o = sim.agents.find((a) => a.id === id); if (o) console.log("  ->", o.kind, o.id, o.pos); break; }
    const nextConn = c.segs.find((s) => s.conn)?.conn;
    console.log("  -> car", c.id, c.vkind, `pos=${c.pos.x.toFixed(1)},${c.pos.y.toFixed(1)} v=${c.v.toFixed(1)} reason=${c.reason} stuck=${c.stuckTime.toFixed(0)} lead=${c.leadId} lane=${c.seg.lane?.id ?? "conn"} s=${c.s.toFixed(1)}/${c.seg.poly.length.toFixed(1)} task=${c.task?.kind ?? "-"}:${c.task?.phase ?? ""} stalled=${c.stalled} blink=${c.blinker} next=${nextConn?.turn} conflicts=${nextConn?.conflicts.length} sig=${c.seg.lane ? sim.net.laneSignal(c.seg.lane, sim.t) : "-"}`);
    id = c.leadId;
  }
});
