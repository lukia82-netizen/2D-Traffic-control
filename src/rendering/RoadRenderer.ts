import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, EdgeData, NodeData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';
import { projectPoint } from '../map/MapLibreSetup';

// ─── Road colour palette (sketch / game aesthetic) ────────────────────────────

interface RoadStyle {
  color: number;
  /** Base stroke half-width in pixels at zoom 16 */
  halfPx: number;
  zIndex: number;
}

const ROAD_STYLES: Record<string, RoadStyle> = {
  motorway:       { color: 0xe8d080, halfPx: 10, zIndex: 5 },
  motorway_link:  { color: 0xe0c870, halfPx:  7, zIndex: 4 },
  trunk:          { color: 0xe8d080, halfPx: 10, zIndex: 5 },
  trunk_link:     { color: 0xe0c870, halfPx:  7, zIndex: 4 },
  primary:        { color: 0xcccccc, halfPx:  8, zIndex: 4 },
  primary_link:   { color: 0xbbbbbb, halfPx:  6, zIndex: 3 },
  secondary:      { color: 0xaaaaaa, halfPx:  6, zIndex: 3 },
  secondary_link: { color: 0x999999, halfPx:  5, zIndex: 3 },
  tertiary:       { color: 0x909090, halfPx:  5, zIndex: 2 },
  tertiary_link:  { color: 0x808080, halfPx:  4, zIndex: 2 },
  residential:    { color: 0x787878, halfPx:  4, zIndex: 1 },
  living_street:  { color: 0x646464, halfPx:  3, zIndex: 1 },
  service:        { color: 0x585858, halfPx:  3, zIndex: 1 },
  unclassified:   { color: 0x6c6c6c, halfPx:  4, zIndex: 1 },
};
const DEFAULT_STYLE: RoadStyle = { color: 0x707070, halfPx: 4, zIndex: 0 };

// ─── RoadRenderer ─────────────────────────────────────────────────────────────

export class RoadRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;
  private readonly camera: CameraManager;

  private gfx: PIXI.Graphics | null = null;

  /**
   * O(1) node lookup built once per mapData load — avoids O(n) Array.find()
   * on every render frame which would block the main thread for thousands of nodes.
   */
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
    // Build O(1) node lookup
    this.nodeMap.clear();
    for (const n of mapData.nodes) {
      this.nodeMap.set(n.id, n);
    }

    // Pre-sort: minor roads (low zIndex) first so major roads render on top
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

    const zoom = this.camera.zoom;
    const zoomScale = Math.pow(2, zoom - 16);

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

      const style  = this.styleFor(edge);
      const lanes  = Math.max(1, edge.lanes);
      // Width = lanes × base half-width × zoom factor, minimum 2px
      const w = Math.max(2, lanes * style.halfPx * zoomScale);
      const alpha = edge.infraType === 'tunnel' ? 0.5 : 1.0;

      // ── Casing / kerb (dark outline drawn first, slightly wider) ──────────
      gfx.moveTo(from.x, from.y).lineTo(to.x, to.y)
         .stroke({ width: w * 2 + 2, color: 0x1a1a1a, alpha, cap: 'round' });

      // ── Road fill ─────────────────────────────────────────────────────────
      gfx.moveTo(from.x, from.y).lineTo(to.x, to.y)
         .stroke({ width: w * 2, color: style.color, alpha, cap: 'round' });

      // ── Centre line (only at higher zoom and wider roads) ─────────────────
      if (w > 5 && !edge.oneway) {
        const ux = dx / len;
        const uy = dy / len;
        this.drawDashes(gfx, from, to, ux, uy, len);
      }
    }
  }

  private drawDashes(
    gfx: PIXI.Graphics,
    from: { x: number; y: number },
    to: { x: number; y: number },
    ux: number,
    uy: number,
    len: number,
  ): void {
    const DASH = 10;
    const GAP  =  8;
    const STEP = DASH + GAP;
    const count = Math.floor(len / STEP);

    for (let i = 0; i < count; i++) {
      const d0 = i * STEP;
      const d1 = d0 + DASH;
      gfx
        .moveTo(from.x + ux * d0, from.y + uy * d0)
        .lineTo(from.x + ux * d1, from.y + uy * d1)
        .stroke({ width: 0.8, color: 0xffffff, alpha: 0.35 });
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
