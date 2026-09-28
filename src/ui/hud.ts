import type { Intent, LateralIntent } from "../sim/ego";
import type { EventLevel, Simulation } from "../sim/simulation";

const INTENT_TEXT: Record<Intent, string> = {
  manual: "手动驾驶",
  cruise: "巡航行驶",
  follow: "跟随前车",
  ped: "礼让行人",
  bike: "避让自行车",
  yield: "让行来车",
  obstacle: "前方停驶车辆",
  red: "红灯，停车等待",
  yellow: "黄灯，准备停车",
  curve: "减速通过弯道",
  arriving: "即将到达目的地",
  arrived: "已到达目的地",
  emergency: "紧急制动",
  freespace: "非结构化道路低速行驶",
  reverse: "倒车调整",
  searching: "正在搜索可行路径",
  noroute: "重新规划路线",
};

const LATERAL_TEXT: Record<LateralIntent, string> = {
  keep: "",
  lanechange: "变道",
  overtake: "绕行",
  nudge: "横向避让",
};

const INTENT_TONE: Partial<Record<Intent, string>> = {
  red: "danger",
  emergency: "danger",
  yellow: "warn",
  ped: "warn",
  bike: "warn",
  yield: "warn",
  obstacle: "warn",
  searching: "warn",
  noroute: "warn",
  freespace: "accent",
  reverse: "accent",
  arrived: "success",
};

const LIMIT_TEXT: Record<string, string> = {
  none: "无",
  ped: "行人",
  vehicle: "来车",
  static: "静止车辆",
  bike: "自行车",
  follow: "前车",
  red: "红灯",
  yellow: "黄灯",
  curve: "横向加速度",
  dest: "目的地",
  end: "参考线末端",
  building: "建筑物",
};

const ARROWS: Record<string, string> = {
  straight: '<path d="M12 3l6 6h-4v12h-4V9H6z"/>',
  left: '<path d="M4 10l6-6v4h6a4 4 0 014 4v9h-4v-9h-6v4z"/>',
  right: '<path d="M20 10l-6-6v4H8a4 4 0 00-4 4v9h4v-9h6v4z"/>',
  uturn: '<path d="M8 21V9a5 5 0 0110 0v4h3l-5 5-5-5h3V9a1 1 0 00-2 0v12z"/>',
  arrive: '<path d="M12 2a7 7 0 00-7 7c0 5.2 7 13 7 13s7-7.8 7-13a7 7 0 00-7-7zm0 9.5A2.5 2.5 0 1112 6.5a2.5 2.5 0 010 5z"/>',
};

const TURN_TEXT: Record<string, string> = { straight: "直行", left: "左转", right: "右转", uturn: "掉头", arrive: "到达目的地" };

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
  private readonly street = el("street");
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
  private readonly planMode = el("plan-mode");
  private readonly planCount = el("plan-count");
  private readonly planMs = el("plan-ms");
  private readonly planLimit = el("plan-limit");
  private readonly stat = {
    fsd: el("stat-fsd"),
    manual: el("stat-manual"),
    interventions: el("stat-interventions"),
    collisions: el("stat-collisions"),
    arrivals: el("stat-arrivals"),
    emergency: el("stat-emergency"),
    recoveries: el("stat-recoveries"),
    time: el("stat-time"),
  };
  private readonly fsdButton = el<HTMLButtonElement>("btn-fsd");
  private lastTurn = "";

  update(sim: Simulation, wall: number): void {
    const ego = sim.ego;
    this.speed.textContent = String(Math.round(Math.abs(ego.v) * 3.6));
    const gear = ego.v < -0.1 ? "R" : ego.mode === "fsd" || Math.abs(ego.v) > 0.1 ? "D" : "P";
    for (const child of Array.from(this.gear.children)) child.classList.toggle("active", child.textContent === gear);

    const loc = ego.localize(sim.net);
    const road = loc?.lane.road;
    this.limit.textContent = road ? String(Math.round(road.speed * 3.6)) : "--";
    this.street.textContent = road?.name || (loc ? "未命名道路" : "非道路区域");

    const fsd = ego.mode === "fsd";
    this.fsdPill.classList.toggle("on", fsd);
    this.fsdLabel.textContent = fsd ? "FSD 已启用" : "FSD 未启用";
    this.fsdButton.classList.toggle("on", fsd);
    (this.fsdButton.querySelector(".label") as HTMLElement).textContent = fsd ? "退出 FSD" : "启用 FSD";

    this.intent.dataset.tone = INTENT_TONE[ego.intent] ?? (fsd ? "accent-soft" : "muted");
    if (fsd) {
      const lat = LATERAL_TEXT[ego.lateral];
      this.intentText.textContent = ego.message || (lat ? `${INTENT_TEXT[ego.intent]} · ${lat}` : INTENT_TEXT[ego.intent]);
    } else this.intentText.textContent = ego.dest ? "手动驾驶 · 按 F 启用 FSD" : "手动驾驶 · 点击地图设置目的地";

    const on = Math.floor(wall * 2.6) % 2 === 0;
    this.blinkL.classList.toggle("on", on && (ego.blinker === -1 || ego.blinker === 2));
    this.blinkR.classList.toggle("on", on && (ego.blinker === 1 || ego.blinker === 2));

    const nav = ego.navInfo();
    this.nav.classList.toggle("hidden", !nav);
    if (nav) {
      if (nav.turn !== this.lastTurn) {
        this.navIcon.innerHTML = `<svg viewBox="0 0 24 24" fill="currentColor">${ARROWS[nav.turn] ?? ""}</svg>`;
        this.lastTurn = nav.turn;
      }
      this.navDist.textContent = formatDistance(nav.distance);
      this.navText.textContent = TURN_TEXT[nav.turn] ?? "";
      this.navRemain.textContent = `剩余 ${formatDistance(nav.remaining)} · 约 ${Math.max(1, Math.round(nav.remaining / 7 / 60))} 分钟`;
    }

    if (!fsd) {
      this.planMode.textContent = "—";
      this.planCount.textContent = "—";
      this.planMs.textContent = "—";
      this.planLimit.textContent = "—";
    } else if (ego.fsd === "lane" && ego.plan) {
      this.planMode.textContent = "车道行驶 · 采样轨迹优化";
      this.planCount.textContent = `${ego.plan.feasibleCount} / ${ego.plan.evaluated}`;
      this.planMs.textContent = `${ego.plan.ms.toFixed(1)} ms`;
      this.planLimit.textContent = LIMIT_TEXT[ego.plan.limit] ?? ego.plan.limit;
    } else {
      this.planMode.textContent = ego.fsd === "arrived" ? "已到达" : "自由空间 · Hybrid A*";
      this.planCount.textContent = ego.free ? `${ego.free.path.length} 个路径点，${ego.free.runs.length} 段` : "—";
      this.planMs.textContent = "—";
      this.planLimit.textContent = ego.free ? `第 ${ego.free.run + 1} 段 · ${ego.free.runs[ego.free.run]?.dir === -1 ? "倒车" : "前进"}` : "—";
    }

    const st = sim.stats;
    this.stat.fsd.textContent = (st.fsdDistance / 1000).toFixed(2);
    this.stat.manual.textContent = (st.manualDistance / 1000).toFixed(2);
    this.stat.interventions.textContent = String(st.interventions);
    this.stat.collisions.textContent = String(st.collisions);
    this.stat.collisions.parentElement?.classList.toggle("bad", st.collisions > 0);
    this.stat.arrivals.textContent = String(st.arrivals);
    this.stat.emergency.textContent = String(st.emergencyStops);
    this.stat.recoveries.textContent = String(st.recoveries);
    this.stat.time.textContent = `${Math.floor(sim.t / 60)}:${Math.floor(sim.t % 60).toString().padStart(2, "0")}`;
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
