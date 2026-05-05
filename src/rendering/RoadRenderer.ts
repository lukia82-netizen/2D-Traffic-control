import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, EdgeData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';
import { projectPoint } from '../map/MapLibreSetup';

// ─── Road colour palette (sketch / game aesthetic) ────────────────────────────
//  Dark background: #1a1a2e  →  roads are lighter

interface RoadStyle {
  fill: number;
  /** Lane width in metres (used for pixel width calculation) */
  lanePx: number;
  zIndex: number;
}

const ROAD_STYLES: Record<string, RoadStyle> = {
  motorway:       { fill: 0xd0c090, lanePx: 5, zIndex: 5 },
  motorway_link:  { fill: 0xc8b880, lanePx: 4, zIndex: 4 },
  trunk:          { fill: 0xd0c090, lanePx: 5, zIndex: 5 },
  trunk_link:     { fill: 0xc8b880, lanePx: 4, zIndex: 4 },
  primary:        { fill: 0xb8b8b8, lanePx: 4, zIndex: 4 },
  primary_link:   { fill: 0xa8a8a8, lanePx: 3, zIndex: 3 },
  secondary:      { fill: 0x999999, lanePx: 3, zIndex: 3 },
  secondary_link: { fill: 0x888888, lanePx: 3, zIndex: 3 },
  tertiary:       { fill: 0x808080, lanePx: 3, zIndex: 2 },
  tertiary_link:  { fill: 0x707070, lanePx: 2, zIndex: 2 },
  residential:    { fill: 0x686868, lanePx: 2, zIndex: 1 },
  living_street:  { fill: 0x505050, lanePx: 2, zIndex: 1 },
  service:        { fill: 0x484848, lanePx: 2, zIndex: 1 },
  unclassified:   { fill: 0x606060, lanePx: 2, zIndex: 1 },
};
const DEFAULT_ROAD_STYLE: RoadStyle = { fill: 0x606060, lanePx: 2, zIndex: 0 };

// ─── RoadRenderer ─────────────────────────────────────────────────────────────

/**
 * Draws road geometry as PixiJS polygons (sketch mode).
 * Re-renders every time the camera moves so road widths stay proportional.
 */
export class RoadRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;
  private readonly camera: CameraManager;

  private roadGraphics: PIXI.Graphics | null = null;

  constructor(overlay: PixiOverlay, map: maplibregl.Map, camera: CameraManager) {
    this.overlay = overlay;
    this.map = map;
    this.camera = camera;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  build(mapData: MapData): void {
    this.rebuild(mapData);
  }

  /** Call on every map 'render' event to keep roads aligned with camera. */
  rebuildOnCameraChange(mapData: MapData): void {
    this.rebuild(mapData);
  }

  // ─── Rendering ─────────────────────────────────────────────────────────────

  private rebuild(mapData: MapData): void {
    if (!this.roadGraphics) {
      this.roadGraphics = new PIXI.Graphics();
      this.overlay.roads.addChild(this.roadGraphics);
    }
    const gfx = this.roadGraphics;
    gfx.clear();

    const zoom = this.camera.zoom;

    // Sort edges by zIndex so major roads draw on top of minor ones
    const sorted = [...mapData.edges].sort(
      (a, b) => this.styleFor(a).zIndex - this.styleFor(b).zIndex,
    );

    for (const edge of sorted) {
      const fromNode = mapData.nodes.find((n) => n.id === edge.from);
      const toNode   = mapData.nodes.find((n) => n.id === edge.to);
      if (!fromNode || !toNode) continue;

      const from = projectPoint(this.map, fromNode.lng, fromNode.lat);
      const to   = projectPoint(this.map, toNode.lng,   toNode.lat);

      const style = this.styleFor(edge);
      // Total road width in pixels = lanes × per-lane px, scaled by zoom
      const laneZoom = Math.pow(2, zoom - 16);
      const halfWidth = Math.max(1.5, edge.lanes * style.lanePx * laneZoom);

      this.drawRoadSegment(gfx, from, to, halfWidth, style.fill, edge.infraType);
    }
  }

  private drawRoadSegment(
    gfx: PIXI.Graphics,
    from: { x: number; y: number },
    to:   { x: number; y: number },
    halfW: number,
    color: number,
    infraType: string,
  ): void {
    const dx = to.x - from.x;
    const dy = to.y - from.y;
    const len = Math.hypot(dx, dy);
    if (len < 0.5) return;

    // Perpendicular unit vector
    const px = -dy / len;
    const py =  dx / len;

    // Road polygon (rectangle)
    const pts = [
      from.x + px * halfW, from.y + py * halfW,
      to.x   + px * halfW, to.y   + py * halfW,
      to.x   - px * halfW, to.y   - py * halfW,
      from.x - px * halfW, from.y - py * halfW,
    ];

    const alpha = infraType === 'tunnel' ? 0.5 : 1.0;
    gfx.poly(pts).fill({ color, alpha });

    // Road edge lines (kerb)
    const kerbColor = 0x333333;
    gfx.poly(pts).stroke({ color: kerbColor, width: 0.8, alpha: 0.6 });

    // Center line for two-way roads (dashed white)
    if (halfW > 4) {
      this.drawCenterLine(gfx, from, to, dx, dy, len);
    }
  }

  private drawCenterLine(
    gfx: PIXI.Graphics,
    from: { x: number; y: number },
    to:   { x: number; y: number },
    dx: number,
    dy: number,
    len: number,
  ): void {
    const dashLen = 10;
    const gapLen  = 8;
    const stepLen = dashLen + gapLen;
    const steps   = Math.floor(len / stepLen);

    const ux = dx / len;
    const uy = dy / len;

    for (let i = 0; i < steps; i++) {
      const t0 = i * stepLen / len;
      const t1 = (i * stepLen + dashLen) / len;

      gfx.moveTo(from.x + ux * t0 * len, from.y + uy * t0 * len)
         .lineTo(from.x + ux * t1 * len, from.y + uy * t1 * len);
    }
    gfx.stroke({ color: 0xffffff, width: 0.8, alpha: 0.3 });
  }

  private styleFor(edge: EdgeData): RoadStyle {
    return ROAD_STYLES[edge.roadType] ?? DEFAULT_ROAD_STYLE;
  }

  // ─── Cleanup ───────────────────────────────────────────────────────────────

  destroy(): void {
    this.roadGraphics?.destroy();
  }
}
