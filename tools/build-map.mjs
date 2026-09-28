#!/usr/bin/env node
// Converts an Overpass JSON extract into the compact local-coordinate map used by the simulator.
// Usage: node tools/build-map.mjs tools/osm-etoile.raw.json src/data/paris-etoile.json
import { readFileSync, writeFileSync } from "node:fs";

const [input, output] = process.argv.slice(2);
if (!input || !output) {
  console.error("usage: build-map.mjs <overpass.json> <out.json>");
  process.exit(1);
}

const CENTER = { lat: 48.8738, lon: 2.295 };
const HALF_X = 620;
const HALF_Y = 590;
const M_PER_DEG_LAT = 111_320;
const M_PER_DEG_LON = 111_320 * Math.cos((CENTER.lat * Math.PI) / 180);

const raw = JSON.parse(readFileSync(input, "utf8"));
const nodes = new Map();
for (const e of raw.elements) {
  if (e.type === "node") {
    // Simulation frame: x east, y south.
    nodes.set(e.id, {
      x: (e.lon - CENTER.lon) * M_PER_DEG_LON,
      y: -(e.lat - CENTER.lat) * M_PER_DEG_LAT,
      tags: e.tags,
    });
  }
}

const inside = (p) => Math.abs(p.x) <= HALF_X && Math.abs(p.y) <= HALF_Y;
const r1 = (v) => Math.round(v * 10) / 10;

/** Point where the segment a→b leaves the map rectangle. */
function clipPoint(a, b) {
  let t = 1;
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  if (dx > 0 && b.x > HALF_X) t = Math.min(t, (HALF_X - a.x) / dx);
  if (dx < 0 && b.x < -HALF_X) t = Math.min(t, (-HALF_X - a.x) / dx);
  if (dy > 0 && b.y > HALF_Y) t = Math.min(t, (HALF_Y - a.y) / dy);
  if (dy < 0 && b.y < -HALF_Y) t = Math.min(t, (-HALF_Y - a.y) / dy);
  return { x: a.x + dx * t, y: a.y + dy * t };
}

const CLASSES = new Set([
  "trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "living_street",
  "trunk_link", "primary_link", "secondary_link", "tertiary_link",
]);

let syntheticId = -1;
const roads = [];
for (const e of raw.elements) {
  if (e.type !== "way" || !e.tags || !CLASSES.has(e.tags.highway)) continue;
  if (e.tags.area === "yes" || e.tags.tunnel === "yes") continue;
  const t = e.tags;
  let ids = [...e.nodes];
  let oneway = t.oneway === "yes" || t.oneway === "1" || t.junction === "roundabout" || t.junction === "circular";
  if (t.oneway === "-1") {
    ids.reverse();
    oneway = true;
  }
  const base = {
    osm: e.id,
    cls: t.highway.replace("_link", ""),
    link: t.highway.endsWith("_link") ? 1 : 0,
    name: t.name ?? "",
    lanes: t.lanes ? Number.parseInt(t.lanes, 10) : 0,
    lanesF: t["lanes:forward"] ? Number.parseInt(t["lanes:forward"], 10) : 0,
    lanesB: t["lanes:backward"] ? Number.parseInt(t["lanes:backward"], 10) : 0,
    oneway: oneway ? 1 : 0,
    circular: t.junction === "roundabout" || t.junction === "circular" ? 1 : 0,
    speed: t.maxspeed ? Number.parseInt(t.maxspeed, 10) : 0,
  };
  // Split into runs that lie inside the map, adding synthetic boundary nodes at the cuts.
  let run = { ids: [], pts: [] };
  const flush = () => {
    if (run.ids.length >= 2) roads.push({ ...base, nodes: run.ids, pts: run.pts.flatMap((p) => [r1(p.x), r1(p.y)]) });
    run = { ids: [], pts: [] };
  };
  for (let i = 0; i < ids.length; i++) {
    const n = nodes.get(ids[i]);
    if (!n) continue;
    const prev = i > 0 ? nodes.get(ids[i - 1]) : null;
    if (inside(n)) {
      if (prev && !inside(prev)) {
        run.ids.push(syntheticId--);
        run.pts.push(clipPoint(n, prev));
      }
      run.ids.push(ids[i]);
      run.pts.push(n);
    } else if (prev && inside(prev)) {
      run.ids.push(syntheticId--);
      run.pts.push(clipPoint(prev, n));
      flush();
    }
  }
  flush();
}

const pointNodes = (kind) => {
  const out = [];
  for (const [id, n] of nodes) {
    if (n.tags?.highway === kind && inside(n)) out.push([id, r1(n.x), r1(n.y)]);
  }
  return out;
};

const polygons = (pred) => {
  const out = [];
  for (const e of raw.elements) {
    if (e.type !== "way" || !e.tags || !pred(e.tags)) continue;
    const pts = e.nodes.map((id) => nodes.get(id)).filter(Boolean);
    if (pts.length < 4) continue;
    const cx = pts.reduce((a, p) => a + p.x, 0) / pts.length;
    const cy = pts.reduce((a, p) => a + p.y, 0) / pts.length;
    if (!inside({ x: cx, y: cy })) continue;
    out.push({ e, pts });
  }
  return out;
};

const buildings = polygons((t) => !!t.building).map(({ e, pts }) => {
  const t = e.tags;
  let h = 0;
  if (t.height) h = Number.parseFloat(t.height);
  else if (t["building:levels"]) h = Number.parseFloat(t["building:levels"]) * 3.2 + 2;
  if (!Number.isFinite(h) || h <= 0) h = 18;
  return { h: r1(h), name: t.name ?? "", pts: pts.slice(0, -1).flatMap((p) => [r1(p.x), r1(p.y)]) };
});

const parks = polygons((t) => t.leisure === "park" || t.leisure === "garden").map(({ pts }) => ({
  pts: pts.slice(0, -1).flatMap((p) => [r1(p.x), r1(p.y)]),
}));
const parking = polygons((t) => t.amenity === "parking").map(({ pts }) => ({
  pts: pts.slice(0, -1).flatMap((p) => [r1(p.x), r1(p.y)]),
}));

const out = {
  source: "© OpenStreetMap contributors, ODbL",
  area: "Paris, Place Charles de Gaulle (Étoile)",
  center: CENTER,
  bounds: { minX: -HALF_X, maxX: HALF_X, minY: -HALF_Y, maxY: HALF_Y },
  roads,
  signals: pointNodes("traffic_signals"),
  crossings: pointNodes("crossing"),
  buildings,
  parks,
  parking,
};
writeFileSync(output, JSON.stringify(out));
console.log(
  `roads=${roads.length} signals=${out.signals.length} crossings=${out.crossings.length} buildings=${buildings.length} parks=${parks.length} parking=${parking.length}`,
);
