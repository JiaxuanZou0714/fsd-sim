import raw from "../../data/paris-etoile.json";

export interface MapRoad {
  osm: number;
  cls: string;
  link: number;
  name: string;
  lanes: number;
  lanesF: number;
  lanesB: number;
  oneway: number;
  circular: number;
  speed: number;
  nodes: number[];
  /** Flat [x0, y0, x1, y1, ...] in metres; x east, y south. */
  pts: number[];
}

export interface MapData {
  source: string;
  area: string;
  center: { lat: number; lon: number };
  bounds: { minX: number; maxX: number; minY: number; maxY: number };
  roads: MapRoad[];
  /** [osm id, x, y] */
  signals: [number, number, number][];
  crossings: [number, number, number][];
  buildings: { h: number; name: string; pts: number[] }[];
  parks: { pts: number[] }[];
  parking: { pts: number[] }[];
}

export const PARIS_ETOILE = raw as unknown as MapData;
