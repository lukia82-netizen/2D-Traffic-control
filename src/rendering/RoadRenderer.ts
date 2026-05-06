import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, EdgeData, NodeData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';
import { projectPoint } from '../map/MapLibreSetup';

// ─── Road colour palette (sketch / game aesthetic) ────────────────────────────

interface RoadStyle {
  color: number;
  /**
   * Half-width in pixels of ONE LANE at zoom 16.
   * Full road width = halfPx * lanes * 2 * zoomScale
   */
  halfPx: number;
  zIndex: number;
}

// Sized so a standard 2-lane residential road is ~36 px wide at zoom 16,
// giving enough room for the 12 px-wide car sprites (75% lane fill).
const ROAD_STYLES: Record<string, RoadStyle> = {
  motorway:       { color: 0xe8d080, halfPx: 22, zIndex: 5 },
  motorway_link:  { color: 0xe0c870, halfPx: 16, zIndex: 4 },
  trunk:          { color: 0xe8d080, halfPx: 22, zIndex: 5 },
  trunk_link:     { color: 0xe0c870, halfPx: 16, zIndex: 4 },
  primary:        { color: 0xcccccc, halfPx: 18, zIndex: 4 },
  primary_link:   { color: 0xbbbbbb, halfPx: 14, zIndex: 3 },
  secondary:      { color: 0xaaaaaa, halfPx: 14, zIndex: 3 },
  secondary_link: { color: 0x999999, halfPx: 12, zIndex: 3 },
  tertiary:       { color: 0x909090, halfPx: 12, zIndex: 2 },
  tertiary_link:  { color: 0x808080, halfPx: 10, zIndex: 2 },
  residential:    { color: 0x787878, halfPx:  9, zIndex: 1 },
  living_street:  { color: 0x646464, halfPx:  8, zIndex: 1 },
  service:        { color: 0x585858, halfPx:  7, zIndex: 1 },
  unclassified:   { color: 0x6c6c6c, halfPx:  9, zIndex: 1 },
};
const DEFAULT_STYLE: RoadStyle = { color: 0x707070, halfPx: 9, zIndex: 0 };

// ─── Internal helpers ─────────────────────────────────────────────────────────

interface EdgeRenderable {
  from:  { x: number; y: number };
  to:    { x: number; y: number };
  /** Half of the full road stroke width (equals what we pass as stroke.width / 2). */
  w:     number;
  style: RoadStyle;
  alpha: number;
  dx: number; dy: number; len: number;
  edge: EdgeData;
}

interface JunctionCircle {
  x: number; y: number;
  /** Radius = w of the widest road at this node. */
  w:     number;
  color: number;
  alpha: number;
}

// ─── RoadRenderer ─────────────────────────────────────────────────────────────

export class RoadRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;
  private readonly camera: CameraManager;

  private gfx: PIXI.Graphics | null = null;

  /** O(1) node lookup built once per mapData load. */
  private nodeMap: Map<number, NodeData> = new Map();

  /** Edges pre-sorted by zIndex (minor roads first, major roads on top). */
  private sortedEdges: EdgeData[] = [];

  constructor(overlay: PixiOverlay, map: maplibregl.Map, camera: CameraManager) {
    this.overlay = overlay;
    this.map = map;
    this.camera = camera;
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

  /** Half-width of one lane (px) at current zoom — exported for CameraManager. */
  get laneHalfPxAtZoom(): number {
    return DEFAULT_STYLE.halfPx * Math.pow(2, this.camera.zoom - 16);
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

    // ── Step 1: project all edges and compute widths ────────────────────────
    const renderables: EdgeRenderable[] = [];
    for (const edge of this.sortedEdges) {
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
      const w     = Math.max(3, lanes * style.halfPx * zoomScale);
      const alpha = edge.infraType === 'tunnel' ? 0.5 : 1.0;

      renderables.push({ from, to, w, style, alpha, dx, dy, len, edge });
    }

    // ── Step 2: compute junction circles (max width + color per node) ───────
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

    // ── Pass 1: road casings (dark outline, butt cap) ────────────────────────
    for (const r of renderables) {
      gfx.moveTo(r.from.x, r.from.y).lineTo(r.to.x, r.to.y)
         .stroke({ width: r.w * 2 + 3, color: 0x111111, alpha: r.alpha, cap: 'butt' });
    }

    // ── Pass 2: junction casing circles ─────────────────────────────────────
    for (const j of junctionMap.values()) {
      gfx.circle(j.x, j.y, j.w + 1.5).fill({ color: 0x111111, alpha: j.alpha });
    }

    // ── Pass 3: road fills (colored, butt cap) ───────────────────────────────
    for (const r of renderables) {
      gfx.moveTo(r.from.x, r.from.y).lineTo(r.to.x, r.to.y)
         .stroke({ width: r.w * 2, color: r.style.color, alpha: r.alpha, cap: 'butt' });
    }

    // ── Pass 4: junction fill circles ────────────────────────────────────────
    for (const j of junctionMap.values()) {
      gfx.circle(j.x, j.y, j.w).fill({ color: j.color, alpha: j.alpha });
    }

    // ── Pass 5: centre dashes on two-way roads (higher zoom only) ───────────
    for (const r of renderables) {
      if (r.w > 8 && !r.edge.oneway) {
        const ux = r.dx / r.len;
        const uy = r.dy / r.len;
        this.drawDashes(gfx, r.from, r.to, ux, uy, r.len);
      }
    }
  }

  private drawDashes(
    gfx: PIXI.Graphics,
    from: { x: number; y: number },
    to:   { x: number; y: number },
    ux: number, uy: number, len: number,
  ): void {
    const DASH = 12;
    const GAP  =  9;
    const STEP = DASH + GAP;
    const count = Math.floor(len / STEP);
    for (let i = 0; i < count; i++) {
      const d0 = i * STEP + GAP * 0.5;
      const d1 = d0 + DASH;
      gfx
        .moveTo(from.x + ux * d0, from.y + uy * d0)
        .lineTo(from.x + ux * d1, from.y + uy * d1)
        .stroke({ width: 1, color: 0xffffff, alpha: 0.3 });
    }
  }

  private styleFor(edge: EdgeData): RoadStyle {
    return ROAD_STYLES[edge.roadType] ?? DEFAULT_STYLE;
  }

  // ─── Cleanup ───────────────────────────────────────────────────────────────

  destroy(): void {
    this.gfx?.destroy();
    this.nodeMap.clear();
  }
}
