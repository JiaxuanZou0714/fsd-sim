import type { IntentKind } from "../sim/ego";
import type { EventLevel, Simulation } from "../sim/simulation";
import { SPEED_LIMIT } from "../sim/world";

const INTENT_TEXT: Record<IntentKind, string> = {
  manual: "手动驾驶",
  cruise: "巡航行驶",
  curve: "减速通过弯道",
  follow: "跟随前车",
  light: "红灯，停车等待",
  yellow: "黄灯，准备停车",
  ped: "礼让行人",
  obstacle: "前方障碍，准备绕行",
  lanechange: "正在变道",
  waitlane: "等待变道时机",
  arriving: "即将到达目的地",
  arrived: "已到达目的地",
  emergency: "紧急制动",
};

const INTENT_TONE: Partial<Record<IntentKind, string>> = {
  light: "danger",
  emergency: "danger",
  yellow: "warn",
  ped: "warn",
  obstacle: "warn",
  waitlane: "warn",
  lanechange: "accent",
  arrived: "success",
};

const ARROWS: Record<string, string> = {
  straight: '<path d="M12 3l6 6h-4v12h-4V9H6z"/>',
  left: '<path d="M4 10l6-6v4h6a4 4 0 014 4v9h-4v-9h-6v4z"/>',
  right: '<path d="M20 10l-6-6v4H8a4 4 0 00-4 4v9h4v-9h6v4z"/>',
  arrive: '<path d="M12 2a7 7 0 00-7 7c0 5.2 7 13 7 13s7-7.8 7-13a7 7 0 00-7-7zm0 9.5A2.5 2.5 0 1112 6.5a2.5 2.5 0 010 5z"/>',
};

const TURN_TEXT: Record<string, string> = {
  straight: "直行通过路口",
  left: "左转",
  right: "右转",
  arrive: "到达目的地",
};

function el<T extends HTMLElement>(id: string): T {
  const e = document.getElementById(id);
  if (!e) throw new Error(`Missing element #${id}`);
  return e as T;
}

function formatDistance(m: number): string {
  if (m >= 1000) return `${(m / 1000).toFixed(1)} 公里`;
  if (m >= 100) return `${Math.round(m / 10) * 10} 米`;
  return `${Math.max(0, Math.round(m))} 米`;
}

export class Hud {
  private readonly speed = el("speed");
  private readonly gear = el("gear");
  private readonly limit = el("limit");
  private readonly fsdPill = el("fsd-pill");
  private readonly fsdLabel = el("fsd-label");
  private readonly intent = el("intent");
  private readonly intentText = el("intent-text");
  private readonly nav = el("nav");
  private readonly navIcon = el("nav-icon");
  private readonly navDist = el("nav-dist");
  private readonly navText = el("nav-text");
  private readonly navRemain = el("nav-remain");
  private readonly blinkL = el("blink-l");
  private readonly blinkR = el("blink-r");
  private readonly toasts = el("toasts");
  private readonly stat = {
    fsd: el("stat-fsd"),
    manual: el("stat-manual"),
    interventions: el("stat-interventions"),
    collisions: el("stat-collisions"),
    arrivals: el("stat-arrivals"),
    emergency: el("stat-emergency"),
    time: el("stat-time"),
  };
  private readonly fsdButton = el<HTMLButtonElement>("btn-fsd");
  private lastNavTurn = "";

  constructor() {
    this.limit.textContent = String(Math.round(SPEED_LIMIT * 3.6));
  }

  update(sim: Simulation, wallTime: number): void {
    const ego = sim.ego;
    const kmh = Math.abs(ego.v) * 3.6;
    this.speed.textContent = String(Math.round(kmh));
    const gear = ego.v < -0.1 ? "R" : ego.mode === "fsd" || Math.abs(ego.v) > 0.1 ? "D" : "P";
    for (const child of Array.from(this.gear.children)) {
      child.classList.toggle("active", child.textContent === gear);
    }

    const fsd = ego.mode === "fsd";
    this.fsdPill.classList.toggle("on", fsd);
    this.fsdLabel.textContent = fsd ? "FSD 已启用" : "FSD 未启用";
    this.fsdButton.classList.toggle("on", fsd);
    this.fsdButton.querySelector(".label")!.textContent = fsd ? "退出 FSD" : "启用 FSD";

    const tone = INTENT_TONE[ego.intent] ?? (fsd ? "accent-soft" : "muted");
    this.intent.dataset.tone = tone;
    this.intentText.textContent = fsd ? INTENT_TEXT[ego.intent] : ego.dest ? "手动驾驶 · 按 F 启用 FSD" : "手动驾驶 · 点击地图设置目的地";

    const blinkOn = Math.floor(wallTime * 2.6) % 2 === 0;
    this.blinkL.classList.toggle("on", blinkOn && ego.blinker === -1);
    this.blinkR.classList.toggle("on", blinkOn && ego.blinker === 1);

    const nav = ego.navInfo();
    this.nav.classList.toggle("hidden", !nav);
    if (nav) {
      if (nav.turn !== this.lastNavTurn) {
        this.navIcon.innerHTML = `<svg viewBox="0 0 24 24" fill="currentColor">${ARROWS[nav.turn] ?? ""}</svg>`;
        this.lastNavTurn = nav.turn;
      }
      this.navDist.textContent = formatDistance(nav.distance);
      this.navText.textContent = TURN_TEXT[nav.turn] ?? "";
      const etaSec = nav.remaining / Math.max(6, SPEED_LIMIT * 0.6);
      this.navRemain.textContent = `剩余 ${formatDistance(nav.remaining)} · 约 ${Math.max(1, Math.round(etaSec / 60))} 分钟`;
    }

    const st = sim.stats;
    this.stat.fsd.textContent = (st.fsdDistance / 1000).toFixed(2);
    this.stat.manual.textContent = (st.manualDistance / 1000).toFixed(2);
    this.stat.interventions.textContent = String(st.interventions);
    this.stat.collisions.textContent = String(st.collisions);
    this.stat.collisions.parentElement?.classList.toggle("bad", st.collisions > 0);
    this.stat.arrivals.textContent = String(st.arrivals);
    this.stat.emergency.textContent = String(st.emergencyStops);
    const mins = Math.floor(sim.t / 60);
    const secs = Math.floor(sim.t % 60);
    this.stat.time.textContent = `${mins}:${secs.toString().padStart(2, "0")}`;
  }

  toast(text: string, level: EventLevel): void {
    const t = document.createElement("div");
    t.className = `toast ${level}`;
    t.textContent = text;
    this.toasts.prepend(t);
    while (this.toasts.children.length > 4) this.toasts.lastElementChild?.remove();
    window.setTimeout(() => t.classList.add("leaving"), 3200);
    window.setTimeout(() => t.remove(), 3800);
  }
}
