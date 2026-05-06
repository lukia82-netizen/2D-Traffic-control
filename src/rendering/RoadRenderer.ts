import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, EdgeData, NodeData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';
import { projectPoint } from '../map/MapLibreSetup';

// ─── Road colour palettes ─────────────────────────────────────────────────────

export interface RoadStyle {
  color: number;
  /** Half-width of ONE lane in pixels at zoom 16. Full width = halfPx * lanes * 2 * zoomScale. */
  halfPx: number;
  zIndex: number;
}

/** Game / sketch – dark palette designed for the dark #1a1a2e background. */
const GAME_STYLES: Record<string, RoadStyle> = {
  motorway:       { color: 0xe8d080, halfPx: 30, zIndex: 5 },
  motorway_link:  { color: 0xe0c870, halfPx: 22, zIndex: 4 },
  trunk:          { color: 0xe8d080, halfPx: 30, zIndex: 5 },
  trunk_link:     { color: 0xe0c870, halfPx: 22, zIndex: 4 },
  primary:        { color: 0xcccccc, halfPx: 24, zIndex: 4 },
  primary_link:   { color: 0xbbbbbb, halfPx: 18, zIndex: 3 },
  secondary:      { color: 0xaaaaaa, halfPx: 18, zIndex: 3 },
  secondary_link: { color: 0x999999, halfPx: 15, zIndex: 3 },
  tertiary:       { color: 0x909090, halfPx: 15, zIndex: 2 },
  tertiary_link:  { color: 0x808080, halfPx: 12, zIndex: 2 },
  residential:    { color: 0x787878, halfPx: 12, zIndex: 1 },
  living_street:  { color: 0x646464, halfPx: 10, zIndex: 1 },
  service:        { color: 0x585858, halfPx:  9, zIndex: 1 },
  unclassified:   { color: 0x6c6c6c, halfPx: 12, zIndex: 1 },
};
const GAME_DEFAULT: RoadStyle = { color: 0x707070, halfPx: 12, zIndex: 0 };

/** OSM Carto palette – close to the standard openstreetmap.org style. */
const OSM_STYLES: Record<string, RoadStyle> = {
  motorway:       { color: 0xe892a2, halfPx: 30, zIndex: 5 },
  motorway_link:  { color: 0xe892a2, halfPx: 22, zIndex: 4 },
  trunk:          { color: 0xf9b29c, halfPx: 30, zIndex: 5 },
  trunk_link:     { color: 0xf9b29c, halfPx: 22, zIndex: 4 },
  primary:        { color: 0xfcd6a4, halfPx: 24, zIndex: 4 },
  primary_link:   { color: 0xfcd6a4, halfPx: 18, zIndex: 3 },
  secondary:      { color: 0xf7fabf, halfPx: 18, zIndex: 3 },
  secondary_link: { color: 0xf7fabf, halfPx: 15, zIndex: 3 },
  tertiary:       { color: 0xffffff, halfPx: 15, zIndex: 2 },
  tertiary_link:  { color: 0xe8e8e8, halfPx: 12, zIndex: 2 },
  residential:    { color: 0xffffff, halfPx: 12, zIndex: 1 },
  living_street:  { color: 0xe8e8e8, halfPx: 10, zIndex: 1 },
  service:        { color: 0xc8c8c8, halfPx:  9, zIndex: 1 },
  unclassified:   { color: 0xf0f0f0, halfPx: 12, zIndex: 1 },
};
const OSM_DEFAULT: RoadStyle = { color: 0xd8d8d8, halfPx: 12, zIndex: 0 };

// ─── Road type → sandbox layer group ─────────────────────────────────────────

/** Maps each OSM highway type to a sandbox layer group for visibility toggles. */
export const ROAD_TYPE_GROUP: Record<string, string> = {
  motorway: 'motorway',       motorway_link: 'motorway',
  trunk: 'motorway',          trunk_link: 'motorway',
  primary: 'primary',         primary_link: 'primary',
  secondary: 'secondary',     secondary_link: 'secondary',
  tertiary: 'secondary',      tertiary_link: 'secondary',
  residential: 'residential', unclassified: 'residential',
  living_street: 'residential',
  service: 'service',
};

/** Representative display info for each group (used in the sandbox legend). */
export const GROUP_LEGEND: Record<string, { label: string; gameColor: string; osmColor: string }> = {
  motorway:    { label: 'Autostrady / Ekspresówki', gameColor: '#e8d080', osmColor: '#e892a2' },
  primary:     { label: 'Drogi krajowe (DK)',        gameColor: '#cccccc', osmColor: '#fcd6a4' },
  secondary:   { label: 'Drogi woj. / lokalne',      gameColor: '#aaaaaa', osmColor: '#f7fabf' },
  residential: { label: 'Ulice i osiedlowe',          gameColor: '#787878', osmColor: '#ffffff' },
  service:     { label: 'Serwisowe / wewnętrzne',    gameColor: '#585858', osmColor: '#c8c8c8' },
};

// ─── Internal helpers ─────────────────────────────────────────────────────────

interface EdgeRenderable {
  from:  { x: number; y: number };
  to:    { x: number; y: number };
  w:     number;
  style: RoadStyle;
  alpha: number;
  dx: number; dy: number; len: number;
  edge: EdgeData;
}

interface JunctionCircle {
  x: number; y: number;
  w: number; color: number; alpha: number;
}

// ─── RoadRenderer ─────────────────────────────────────────────────────────────

export class RoadRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;
  private readonly camera: CameraManager;

  private gfx: PIXI.Graphics | null = null;

  /** O(1) node lookup — built once per mapData load. */
  private nodeMap: Map<number, NodeData> = new Map();

  /** Edges pre-sorted by zIndex (minor roads first, major roads on top). */
  private sortedEdges: EdgeData[] = [];

  // ── Sandbox state ─────────────────────────────────────────────────────────
  /** Groups whose roads are currently hidden. */
  private hiddenGroups: Set<string> = new Set(['service']);

  /** When true: use OSM Carto colours instead of the dark game palette. */
  private osmMode = false;

  constructor(overlay: PixiOverlay, map: maplibregl.Map, camera: CameraManager) {
    this.overlay = overlay;
    this.map = map;
    this.camera = camera;
  }

  // ─── Sandbox layer control ─────────────────────────────────────────────────

  setGroupVisible(group: string, visible: boolean): void {
    if (visible) this.hiddenGroups.delete(group);
    else          this.hiddenGroups.add(group);
    this.drawRoads();
  }

  setOsmMode(enabled: boolean): void {
    this.osmMode = enabled;
    this.drawRoads();
  }

  isGroupVisible(group: string): boolean {
    return !this.hiddenGroups.has(group);
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  /** Call once when mapData first arrives — builds lookup caches. */
  build(mapData: MapData): void {
    this.nodeMap.clear();
    for (const n of mapData.nodes) {
      this.nodeMap.set(n.id, n);
    }
    this.sortedEdges = [...mapData.edges].sort(
      (a, b) => this.styleFor(a).zIndex - this.styleFor(b).zIndex,
    );
    this.drawRoads();
  }

  /** Called on every map 'render' event (pan / zoom). */
  rebuildOnCameraChange(_mapData: MapData): void {
    this.drawRoads();
  }

  // ─── Rendering ─────────────────────────────────────────────────────────────

  private drawRoads(): void {
    if (this.sortedEdges.length === 0) return;

    if (!this.gfx) {
      this.gfx = new PIXI.Graphics();
      this.overlay.roads.addChild(this.gfx);
    }
    const gfx = this.gfx;
    gfx.clear();

    const zoomScale = Math.pow(2, this.camera.zoom - 16);
    const casingColor = this.osmMode ? 0x888888 : 0x111111;

    // ── Step 1: project visible edges ─────────────────────────────────────
    const renderables: EdgeRenderable[] = [];
    for (const edge of this.sortedEdges) {
      const group = ROAD_TYPE_GROUP[edge.roadType] ?? 'residential';
      if (this.hiddenGroups.has(group)) continue;

      const fromNode = this.nodeMap.get(edge.from);
      const toNode   = this.nodeMap.get(edge.to);
      if (!fromNode || !toNode) continue;

      const from = projectPoint(this.map, fromNode.lng, fromNode.lat);
      const to   = projectPoint(this.map, toNode.lng,   toNode.lat);
      const dx = to.x - from.x;
      const dy = to.y - from.y;
      const len = Math.hypot(dx, dy);
      if (len < 0.5) continue;

      const style = this.styleFor(edge);
      const lanes = Math.max(1, edge.lanes);
      const w     = Math.max(5, lanes * style.halfPx * zoomScale);
      const alpha = edge.infraType === 'tunnel' ? 0.5 : 1.0;

      renderables.push({ from, to, w, style, alpha, dx, dy, len, edge });
    }

    // ── Step 2: junction circles (max road width at each node) ────────────
    const junctionMap = new Map<number, JunctionCircle>();
    for (const r of renderables) {
      for (const nodeId of [r.edge.from, r.edge.to]) {
        const node = this.nodeMap.get(nodeId);
        if (!node) continue;
        const prev = junctionMap.get(nodeId);
        if (!prev || r.w > prev.w) {
          const px = projectPoint(this.map, node.lng, node.lat);
          junctionMap.set(nodeId, { x: px.x, y: px.y, w: r.w, color: r.style.color, alpha: r.alpha });
        }
      }
    }

    // ── Pass 1: road casings ───────────────────────────────────────────────
    for (const r of renderables) {
      gfx.moveTo(r.from.x, r.from.y).lineTo(r.to.x, r.to.y)
         .stroke({ width: r.w * 2 + 3, color: casingColor, alpha: r.alpha, cap: 'butt' });
    }

    // ── Pass 2: junction casing circles ───────────────────────────────────
    for (const j of junctionMap.values()) {
      gfx.circle(j.x, j.y, j.w + 2).fill({ color: casingColor, alpha: j.alpha });
    }

    // ── Pass 3: road fills ─────────────────────────────────────────────────
    for (const r of renderables) {
      gfx.moveTo(r.from.x, r.from.y).lineTo(r.to.x, r.to.y)
         .stroke({ width: r.w * 2, color: r.style.color, alpha: r.alpha, cap: 'butt' });
    }

    // ── Pass 4: junction fill circles ──────────────────────────────────────
    for (const j of junctionMap.values()) {
      gfx.circle(j.x, j.y, j.w + 0.5).fill({ color: j.color, alpha: j.alpha });
    }

    // ── Pass 5: centre dashes on two-way roads ─────────────────────────────
    for (const r of renderables) {
      if (r.w > 8 && !r.edge.oneway) {
        this.drawDashes(gfx, r.from, r.dx / r.len, r.dy / r.len, r.len);
      }
    }
  }

  private drawDashes(
    gfx: PIXI.Graphics,
    from: { x: number; y: number },
    ux: number, uy: number, len: number,
  ): void {
    const DASH = 12, GAP = 9, STEP = DASH + GAP;
    const count = Math.floor(len / STEP);
    for (let i = 0; i < count; i++) {
      const d0 = i * STEP + GAP * 0.5;
      gfx
        .moveTo(from.x + ux * d0,        from.y + uy * d0)
        .lineTo(from.x + ux * (d0 + DASH), from.y + uy * (d0 + DASH))
        .stroke({ width: 1, color: 0xffffff, alpha: 0.3 });
    }
  }

  private styleFor(edge: EdgeData): RoadStyle {
    const palette = this.osmMode ? OSM_STYLES : GAME_STYLES;
    const def     = this.osmMode ? OSM_DEFAULT  : GAME_DEFAULT;
    return palette[edge.roadType] ?? def;
  }

  // ─── Cleanup ───────────────────────────────────────────────────────────────

  destroy(): void {
    this.gfx?.destroy();
    this.nodeMap.clear();
  }
}
