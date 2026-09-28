import type { Vec2 } from "../sim/geometry";
import type { RoadNetwork } from "../sim/map/network";
import type { Simulation } from "../sim/simulation";

export class Minimap {
  private readonly ctx: CanvasRenderingContext2D;
  private size = 244;
  private base: HTMLCanvasElement | null = null;
  private readonly minX: number;
  private readonly minY: number;
  private readonly span: number;

  constructor(
    private readonly canvas: HTMLCanvasElement,
    private readonly net: RoadNetwork,
    onPick: (p: Vec2) => void,
  ) {
    const ctx = canvas.getContext("2d");
    if (!ctx) throw new Error("2D canvas unavailable");
    this.ctx = ctx;
    const b = net.bounds;
    this.span = Math.max(b.maxX - b.minX, b.maxY - b.minY) + 20;
    this.minX = (b.minX + b.maxX) / 2 - this.span / 2;
    this.minY = (b.minY + b.maxY) / 2 - this.span / 2;
    this.resize();
    canvas.addEventListener("click", (ev) => {
      const rect = canvas.getBoundingClientRect();
      const sx = ((ev.clientX - rect.left) / rect.width) * this.size;
      const sy = ((ev.clientY - rect.top) / rect.height) * this.size;
      onPick({ x: this.minX + (sx / this.size) * this.span, y: this.minY + (sy / this.size) * this.span });
    });
  }

  resize(): void {
    const dpr = Math.min(window.devicePixelRatio, 2);
    this.size = this.canvas.clientWidth || 244;
    this.canvas.width = this.size * dpr;
    this.canvas.height = this.size * dpr;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.base = null;
  }

  private px(x: number): number {
    return ((x - this.minX) / this.span) * this.size;
  }
  private py(y: number): number {
    return ((y - this.minY) / this.span) * this.size;
  }

  private drawBase(): HTMLCanvasElement {
    const dpr = Math.min(window.devicePixelRatio, 2);
    const c = document.createElement("canvas");
    c.width = this.size * dpr;
    c.height = this.size * dpr;
    const g = c.getContext("2d") as CanvasRenderingContext2D;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.fillStyle = "#101216";
    g.fillRect(0, 0, this.size, this.size);
    g.fillStyle = "#1d2027";
    for (const b of this.net.buildings) {
      g.beginPath();
      b.pts.forEach((p, i) => (i === 0 ? g.moveTo(this.px(p.x), this.py(p.y)) : g.lineTo(this.px(p.x), this.py(p.y))));
      g.closePath();
      g.fill();
    }
    g.fillStyle = "#1c2a20";
    for (const p of this.net.parks) {
      g.beginPath();
      p.forEach((q, i) => (i === 0 ? g.moveTo(this.px(q.x), this.py(q.y)) : g.lineTo(this.px(q.x), this.py(q.y))));
      g.closePath();
      g.fill();
    }
    g.strokeStyle = "#3a3f49";
    g.lineCap = "round";
    for (const seg of this.net.segments) {
      g.lineWidth = Math.max(1, ((seg.rightW + seg.leftW) / this.span) * this.size);
      g.beginPath();
      seg.center.pts.forEach((p, i) => (i === 0 ? g.moveTo(this.px(p.x), this.py(p.y)) : g.lineTo(this.px(p.x), this.py(p.y))));
      g.stroke();
    }
    return c;
  }

  draw(sim: Simulation, wall: number): void {
    const c = this.ctx;
    if (!this.base) this.base = this.drawBase();
    c.clearRect(0, 0, this.size, this.size);
    c.drawImage(this.base, 0, 0, this.size, this.size);

    for (const ctrl of this.net.controllers) {
      const j = ctrl.junctions[0];
      const col = [0, 1].map((g) => this.net.signalColor(ctrl, g, sim.t));
      c.fillStyle = col[0] === "G" ? "#2cff7a" : col[0] === "Y" ? "#ffc21a" : "#ff3b3b";
      c.fillRect(this.px(j.pos.x) - 1.5, this.py(j.pos.y) - 1.5, 3, 3);
    }

    const ego = sim.ego;
    if (ego.mode === "fsd" && ego.ref && ego.plan) {
      c.strokeStyle = "#2f7bff";
      c.lineWidth = 2.5;
      c.lineJoin = "round";
      c.beginPath();
      const ref = ego.ref;
      for (let s = ego.plan.s0, first = true; s <= ref.destS; s += 5, first = false) {
        const p = ref.poly.sampleAt(s).p;
        if (first) c.moveTo(this.px(p.x), this.py(p.y));
        else c.lineTo(this.px(p.x), this.py(p.y));
      }
      c.stroke();
    }
    if (ego.mode === "fsd" && ego.free) {
      c.strokeStyle = "#35d6e8";
      c.lineWidth = 2;
      c.beginPath();
      ego.free.path.forEach((p, i) => (i === 0 ? c.moveTo(this.px(p.x), this.py(p.y)) : c.lineTo(this.px(p.x), this.py(p.y))));
      c.stroke();
    }

    for (const v of sim.traffic.vehicles) {
      c.fillStyle = v.stalled ? "#e08a2c" : v.id === ego.leadId ? "#6f9bff" : v.vkind === "bus" || v.vkind === "truck" ? "#b7bcc6" : "#7d838d";
      const r = v.vkind === "bike" || v.vkind === "moto" ? 0.9 : v.vkind === "bus" || v.vkind === "truck" ? 1.8 : 1.3;
      c.beginPath();
      c.arc(this.px(v.pos.x), this.py(v.pos.y), r, 0, Math.PI * 2);
      c.fill();
    }
    c.fillStyle = "#c3c7cf";
    for (const p of sim.peds.peds) c.fillRect(this.px(p.pos.x) - 0.5, this.py(p.pos.y) - 0.5, 1, 1);

    if (ego.dest) {
      const x = this.px(ego.dest.pos.x);
      const y = this.py(ego.dest.pos.y);
      c.strokeStyle = "rgba(63,140,255,0.7)";
      c.lineWidth = 2;
      c.beginPath();
      c.arc(x, y, 5 + Math.sin(wall * 3) * 1.2, 0, Math.PI * 2);
      c.stroke();
      c.fillStyle = "#3f8cff";
      c.beginPath();
      c.arc(x, y, 2.4, 0, Math.PI * 2);
      c.fill();
    }
    c.save();
    c.translate(this.px(ego.pos.x), this.py(ego.pos.y));
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
