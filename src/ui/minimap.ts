import type { Vec2 } from "../sim/geometry";
import type { Simulation } from "../sim/simulation";
import { BLOCK, GRID, ROAD_HALF } from "../sim/world";

export class Minimap {
  private readonly ctx: CanvasRenderingContext2D;
  private readonly min = -BLOCK * 0.45;
  private readonly span = (GRID - 1) * BLOCK + BLOCK * 0.9;
  private size = 240;

  constructor(
    private readonly canvas: HTMLCanvasElement,
    onPick: (p: Vec2) => void,
  ) {
    const ctx = canvas.getContext("2d");
    if (!ctx) throw new Error("2D canvas unavailable");
    this.ctx = ctx;
    this.resize();
    canvas.addEventListener("click", (ev) => {
      const rect = canvas.getBoundingClientRect();
      const sx = ((ev.clientX - rect.left) / rect.width) * this.size;
      const sy = ((ev.clientY - rect.top) / rect.height) * this.size;
      onPick({ x: this.min + (sx / this.size) * this.span, y: this.min + (sy / this.size) * this.span });
    });
  }

  resize(): void {
    const dpr = Math.min(window.devicePixelRatio, 2);
    this.size = this.canvas.clientWidth || 240;
    this.canvas.width = this.size * dpr;
    this.canvas.height = this.size * dpr;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  private px(v: number): number {
    return ((v - this.min) / this.span) * this.size;
  }

  draw(sim: Simulation, wallTime: number): void {
    const c = this.ctx;
    const s = this.size;
    c.clearRect(0, 0, s, s);
    c.fillStyle = "#101216";
    c.fillRect(0, 0, s, s);

    const roadW = ((ROAD_HALF * 2) / this.span) * s;
    c.strokeStyle = "#2e323a";
    c.lineWidth = roadW;
    c.lineCap = "square";
    const end = (GRID - 1) * BLOCK;
    for (let k = 0; k < GRID; k++) {
      const p = this.px(k * BLOCK);
      c.beginPath();
      c.moveTo(this.px(0), p);
      c.lineTo(this.px(end), p);
      c.moveTo(p, this.px(0));
      c.lineTo(p, this.px(end));
      c.stroke();
    }

    for (const n of sim.world.nodes) {
      const ph = sim.world.phaseAt(n.id, sim.t).name;
      const ew = ph.startsWith("EW") ? (ph.endsWith("_Y") ? "#ffc21a" : "#2cff7a") : "#ff3b3b";
      const ns = ph.startsWith("NS") ? (ph.endsWith("_Y") ? "#ffc21a" : "#2cff7a") : "#ff3b3b";
      const x = this.px(n.pos.x);
      const y = this.px(n.pos.y);
      c.fillStyle = ew;
      c.fillRect(x - 3.5, y - 0.8, 7, 1.6);
      c.fillStyle = ns;
      c.fillRect(x - 0.8, y - 3.5, 1.6, 7);
    }

    const ego = sim.ego;
    if (ego.mode === "fsd" && ego.path) {
      c.strokeStyle = "#2f7bff";
      c.lineWidth = 3;
      c.lineCap = "round";
      c.lineJoin = "round";
      c.beginPath();
      const stepS = 4;
      for (let sArc = ego.pathS; sArc <= ego.destS; sArc += stepS) {
        const p = ego.path.sampleAt(sArc).p;
        if (sArc === ego.pathS) c.moveTo(this.px(p.x), this.px(p.y));
        else c.lineTo(this.px(p.x), this.px(p.y));
      }
      c.stroke();
    }

    for (const car of sim.traffic.cars) {
      c.fillStyle = car.stalled ? "#e08a2c" : car.id === ego.leadId ? "#6f9bff" : "#8a909a";
      c.beginPath();
      c.arc(this.px(car.pos.x), this.px(car.pos.y), 1.9, 0, Math.PI * 2);
      c.fill();
    }
    c.fillStyle = "#c3c7cf";
    for (const p of sim.peds.peds) c.fillRect(this.px(p.pos.x) - 0.6, this.px(p.pos.y) - 0.6, 1.2, 1.2);

    if (ego.dest) {
      const x = this.px(ego.dest.pos.x);
      const y = this.px(ego.dest.pos.y);
      const r = 5 + Math.sin(wallTime * 3) * 1.2;
      c.strokeStyle = "rgba(63,140,255,0.7)";
      c.lineWidth = 2;
      c.beginPath();
      c.arc(x, y, r, 0, Math.PI * 2);
      c.stroke();
      c.fillStyle = "#3f8cff";
      c.beginPath();
      c.arc(x, y, 2.6, 0, Math.PI * 2);
      c.fill();
    }

    const ex = this.px(ego.pos.x);
    const ey = this.px(ego.pos.y);
    c.save();
    c.translate(ex, ey);
    c.rotate(ego.heading);
    c.fillStyle = "#ffffff";
    c.strokeStyle = "#2f7bff";
    c.lineWidth = 2;
    c.beginPath();
    c.moveTo(7, 0);
    c.lineTo(-5, 4.5);
    c.lineTo(-2.5, 0);
    c.lineTo(-5, -4.5);
    c.closePath();
    c.fill();
    c.stroke();
    c.restore();
  }
}
