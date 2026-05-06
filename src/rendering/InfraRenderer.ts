import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, EdgeData, NodeData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';
import { projectPoint } from '../map/MapLibreSetup';

// ─── Constants ────────────────────────────────────────────────────────────────

const TUNNEL_DASH_COLOR = 0x334455;
const TUNNEL_ALPHA = 0.6;
const BRIDGE_SHADOW_OFFSET = 4;
const BRIDGE_SHADOW_ALPHA = 0.35;

/** Minimum zoom to draw per-lane direction arrows. */
const LANE_ARROW_MIN_ZOOM = 15;
/** Fraction of edge length at which lane arrows are placed (near intersection end). */
const LANE_ARROW_T = 0.75;

// ─── InfraRenderer ───────────────────────────────────────────────────────────

/**
 * Renders infrastructure markings:
 *
 * 1. **Static layer** (`staticMarkings` / `tunnelOverlay`): bridge shadows and
 *    tunnel dashes baked into `RenderTexture`s — rebuilt on camera change.
 *
 * 2. **Static arrow layer** (`arrowLayer`): fixed triangle markers on one-way
 *    roads showing the direction of travel.  Rebuilt on camera change.
 *    (Arrows are intentionally NOT animated — they are road markings, not flow.)
 */
export class InfraRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;
  private readonly camera: CameraManager;

  private staticSprite: PIXI.Sprite | null = null;
  private staticTexture: PIXI.RenderTexture | null = null;
  private tunnelSprite: PIXI.Sprite | null = null;
  private tunnelTexture: PIXI.RenderTexture | null = null;

  /** O(1) node lookup, populated in buildStaticLayer(). */
  private nodeMap: Map<number, NodeData> = new Map();
  /** Node intersection-type lookup for quick non-plain detection. */
  private intersectionNodeIds: Set<number> = new Set();

  constructor(overlay: PixiOverlay, map: maplibregl.Map, camera: CameraManager) {
    this.overlay = overlay;
    this.map = map;
    this.camera = camera;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  buildStaticLayer(mapData: MapData): void {
    // Build O(1) node lookup so we never use Array.find() per edge
    this.nodeMap.clear();
    this.intersectionNodeIds.clear();
    for (const n of mapData.nodes) {
      this.nodeMap.set(n.id, n);
      if (n.intersectionType !== 'plain') {
        this.intersectionNodeIds.add(n.id);
      }
    }
    this.rebuildMarkings(mapData);
    this.rebuildTunnelOverlay(mapData);
    this.rebuildArrows(mapData);
    this.rebuildLaneArrows(mapData);
  }

  /** Must be called on map `render` so all markings follow camera pan / zoom. */
  rebuildOnCameraChange(mapData: MapData): void {
    this.rebuildMarkings(mapData);
    this.rebuildTunnelOverlay(mapData);
    this.rebuildArrows(mapData);
    this.rebuildLaneArrows(mapData);
  }

  /**
   * No-op — arrows are now static road markings.
   * Kept for API compatibility (game.ts calls this each tick).
   */
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  update(_deltaMS: number): void {
    // intentionally empty
  }

  // ─── Static markings (bridge shadows + stop lines) ────────────────────────

  private rebuildMarkings(mapData: MapData): void {
    const w = this.overlay.width;
    const h = this.overlay.height;
    const gfx = new PIXI.Graphics();

    for (const edge of mapData.edges) {
      const fromNode = this.nodeMap.get(edge.from);
      const toNode   = this.nodeMap.get(edge.to);
      if (!fromNode || !toNode) continue;

      const fromPx = projectPoint(this.map, fromNode.lng, fromNode.lat);
      const toPx   = projectPoint(this.map, toNode.lng,   toNode.lat);

      if (edge.infraType === 'bridge') {
        this.drawBridgeShadow(gfx, fromPx, toPx, edge);
      }

      // Stop line for STOP-sign approaches.
      // Traffic-light stop lines are rendered in TrafficLightRenderer so we
      // don't draw duplicate bars in different positions.
      const targetNode = toNode;
      if (targetNode.intersectionType === 'stop') {
        this.drawStopLine(gfx, fromPx, toPx, edge);
      }
    }

    const rt = PIXI.RenderTexture.create({ width: w, height: h });
    this.overlay.app.renderer.render({ container: gfx, target: rt });
    gfx.destroy();

    if (this.staticTexture) this.staticTexture.destroy(true);
    this.staticTexture = rt;

    if (!this.staticSprite) {
      this.staticSprite = new PIXI.Sprite(rt);
      this.overlay.staticMarkings.addChild(this.staticSprite);
    } else {
      this.staticSprite.texture = rt;
    }
  }

  private drawBridgeShadow(
    gfx: PIXI.Graphics,
    from: { x: number; y: number },
    to:   { x: number; y: number },
    edge: EdgeData,
  ): void {
    const laneW = this.camera.getRoadOverlayWidth(edge.lanes);
    gfx.setStrokeStyle({ width: laneW + 6, color: 0x111122, alpha: BRIDGE_SHADOW_ALPHA });
    gfx.moveTo(from.x + BRIDGE_SHADOW_OFFSET, from.y + BRIDGE_SHADOW_OFFSET);
    gfx.lineTo(to.x   + BRIDGE_SHADOW_OFFSET, to.y   + BRIDGE_SHADOW_OFFSET);
    gfx.stroke();
  }

  /**
   * Draw a white stop line perpendicular to the road, set back 12 px from the
   * intersection node.  Width = road overlay width so it spans the full lane.
   */
  private drawStopLine(
    gfx: PIXI.Graphics,
    from: { x: number; y: number },
    to:   { x: number; y: number },
    edge: EdgeData,
  ): void {
    const dx  = to.x - from.x;
    const dy  = to.y - from.y;
    const len = Math.hypot(dx, dy);
    if (len < 20) return; // road segment too short to place a stop line

    const SETBACK = Math.min(14, len * 0.15); // px back from the node
    const t  = (len - SETBACK) / len;
    const sx = from.x + dx * t;
    const sy = from.y + dy * t;

    // Perpendicular unit vector (90° CW)
    const px = -dy / len;
    const py =  dx / len;

    const hw = this.camera.getRoadOverlayWidth(edge.lanes) * 0.85; // half-width

    gfx.moveTo(sx + px * hw, sy + py * hw)
       .lineTo(sx - px * hw, sy - py * hw)
       .stroke({ width: 2.5, color: 0xffffff, alpha: 0.85, cap: 'round' });
  }

  // ─── Static one-way arrows ─────────────────────────────────────────────────

  private rebuildArrows(mapData: MapData): void {
    this.overlay.arrowLayer.removeChildren();

    const spacing = this.camera.getArrowSpacing();
    const sz      = this.camera.getArrowSize();

    for (const edge of mapData.edges) {
      if (!edge.oneway) continue;

      const fromNode = this.nodeMap.get(edge.from);
      const toNode   = this.nodeMap.get(edge.to);
      if (!fromNode || !toNode) continue;

      const from = projectPoint(this.map, fromNode.lng, fromNode.lat);
      const to   = projectPoint(this.map, toNode.lng,   toNode.lat);

      const dx     = to.x - from.x;
      const dy     = to.y - from.y;
      const segLen = Math.hypot(dx, dy);
      if (segLen < spacing) continue;

      const dirX = dx / segLen;
      const dirY = dy / segLen;
      const numArrows = Math.floor(segLen / spacing);

      for (let i = 0; i < numArrows; i++) {
        // Place arrow at centre of its slot along the edge
        const t = (i + 0.5) * spacing;
        const gfx = this.makeArrowShape(dirX, dirY, sz);
        gfx.x = from.x + dirX * t;
        gfx.y = from.y + dirY * t;
        this.overlay.arrowLayer.addChild(gfx);
      }
    }
  }

  private makeArrowShape(dirX: number, dirY: number, sz: number): PIXI.Graphics {
    const px = -dirY;
    const py =  dirX;
    const gfx = new PIXI.Graphics();
    gfx
      .moveTo(dirX * sz,                  dirY * sz)
      .lineTo(-dirX * sz + px * sz * 0.6, -dirY * sz + py * sz * 0.6)
      .lineTo(-dirX * sz - px * sz * 0.6, -dirY * sz - py * sz * 0.6)
      .closePath()
      .fill({ color: 0xffffff, alpha: 0.45 });
    return gfx;
  }

  // ─── Tunnel overlay ────────────────────────────────────────────────────────

  private rebuildTunnelOverlay(mapData: MapData): void {
    const w = this.overlay.width;
    const h = this.overlay.height;
    const gfx = new PIXI.Graphics();

    for (const edge of mapData.edges) {
      if (edge.infraType !== 'tunnel') continue;

      const fromNode = this.nodeMap.get(edge.from);
      const toNode   = this.nodeMap.get(edge.to);
      if (!fromNode || !toNode) continue;

      const fromPx = projectPoint(this.map, fromNode.lng, fromNode.lat);
      const toPx   = projectPoint(this.map, toNode.lng,   toNode.lat);
      const laneW  = Math.max(4, this.camera.getRoadOverlayWidth(edge.lanes));

      gfx.setStrokeStyle({ width: laneW, color: TUNNEL_DASH_COLOR, alpha: TUNNEL_ALPHA });
      gfx.moveTo(fromPx.x, fromPx.y);
      gfx.lineTo(toPx.x,   toPx.y);
      gfx.stroke();

      this.drawTunnelPortal(gfx, fromPx.x, fromPx.y);
      this.drawTunnelPortal(gfx, toPx.x,   toPx.y);
    }

    const rt = PIXI.RenderTexture.create({ width: w, height: h });
    this.overlay.app.renderer.render({ container: gfx, target: rt });
    gfx.destroy();

    if (this.tunnelTexture) this.tunnelTexture.destroy(true);
    this.tunnelTexture = rt;

    if (!this.tunnelSprite) {
      this.tunnelSprite = new PIXI.Sprite(rt);
      this.overlay.tunnelOverlay.addChild(this.tunnelSprite);
    } else {
      this.tunnelSprite.texture = rt;
    }
  }

  private drawTunnelPortal(gfx: PIXI.Graphics, x: number, y: number): void {
    gfx.rect(x - 6, y - 4, 12, 8).fill({ color: 0x1a1a2e, alpha: 0.8 });
  }

  // ─── Per-lane direction arrows near intersections ──────────────────────────

  /**
   * Draw small directional arrows (↑ straight, ↖ left, ↗ right) at ~75% along
   * each edge that leads to a non-plain intersection node.
   *
   * Arrows are placed as live `Graphics` children of `overlay.arrowLayer`.
   * They are regenerated on each camera change together with the one-way arrows.
   *
   * Only drawn at zoom ≥ LANE_ARROW_MIN_ZOOM.
   */
  private rebuildLaneArrows(mapData: MapData): void {
    const zoom = this.map.getZoom();
    if (zoom < LANE_ARROW_MIN_ZOOM) return;

    const sz = Math.max(5, this.camera.getArrowSize() * 0.8);

    for (const edge of mapData.edges) {
      // Only draw arrows on edges that lead into a meaningful intersection
      if (!this.intersectionNodeIds.has(edge.to)) continue;
      if (!edge.laneDirections || edge.laneDirections.length === 0) continue;

      const fromNode = this.nodeMap.get(edge.from);
      const toNode   = this.nodeMap.get(edge.to);
      if (!fromNode || !toNode) continue;

      const from = projectPoint(this.map, fromNode.lng, fromNode.lat);
      const to   = projectPoint(this.map, toNode.lng,   toNode.lat);

      const dx     = to.x - from.x;
      const dy     = to.y - from.y;
      const segLen = Math.hypot(dx, dy);
      if (segLen < 20) continue;   // too short to draw

      const dirX = dx / segLen;
      const dirY = dy / segLen;
      // Perpendicular (right side)
      const perpX = -dirY;
      const perpY =  dirX;

      // Position the arrow group at LANE_ARROW_T along the edge
      const anchorX = from.x + dirX * segLen * LANE_ARROW_T;
      const anchorY = from.y + dirY * segLen * LANE_ARROW_T;

      const lanes = edge.laneDirections.length;
      // Lane width in pixels – approximate from road overlay width
      const laneW = Math.max(5, this.camera.getRoadOverlayWidth(lanes) / lanes);

      // Centre offset: shift the lane group so it's centred on the road axis
      const groupOffset = -(lanes - 1) * 0.5 * laneW;

      for (let i = 0; i < lanes; i++) {
        const lateralOffset = groupOffset + i * laneW;
        const cx = anchorX + perpX * lateralOffset;
        const cy = anchorY + perpY * lateralOffset;

        const dir = edge.laneDirections[i] ?? 'straight';
        const gfx = this.makeLaneArrowGlyph(dir, dirX, dirY, perpX, perpY, sz);
        gfx.x = cx;
        gfx.y = cy;
        this.overlay.arrowLayer.addChild(gfx);
      }
    }
  }

  /**
   * Create a small lane-direction glyph for a single lane.
   *
   * @param dir   "left" | "straight" | "right" | "uturn"
   * @param dirX  Unit vector along the road (forward).
   * @param dirY  Unit vector along the road (forward).
   * @param perpX Perpendicular unit vector (right side of road).
   * @param perpY Perpendicular unit vector (right side of road).
   * @param sz    Half-size of the arrowhead in pixels.
   */
  private makeLaneArrowGlyph(
    dir: string,
    dirX: number,
    dirY: number,
    perpX: number,
    perpY: number,
    sz: number,
  ): PIXI.Graphics {
    const gfx  = new PIXI.Graphics();
    const half = sz * 0.55;

    switch (dir) {
      case 'left': {
        // Arrow pointing left (90° CCW from forward)
        const lx = -perpX;
        const ly = -perpY;
        gfx
          .moveTo(lx * sz,                    ly * sz)
          .lineTo(-lx * sz + dirX * half,     -ly * sz + dirY * half)
          .lineTo(-lx * sz - dirX * half,     -ly * sz - dirY * half)
          .closePath()
          .fill({ color: 0xffffff, alpha: 0.55 });
        break;
      }
      case 'right': {
        // Arrow pointing right (90° CW from forward)
        const rx = perpX;
        const ry = perpY;
        gfx
          .moveTo(rx * sz,                    ry * sz)
          .lineTo(-rx * sz + dirX * half,     -ry * sz + dirY * half)
          .lineTo(-rx * sz - dirX * half,     -ry * sz - dirY * half)
          .closePath()
          .fill({ color: 0xffffff, alpha: 0.55 });
        break;
      }
      default: {
        // Straight arrow (forward direction)
        gfx
          .moveTo(dirX * sz,                  dirY * sz)
          .lineTo(-dirX * sz + perpX * half,  -dirY * sz + perpY * half)
          .lineTo(-dirX * sz - perpX * half,  -dirY * sz - perpY * half)
          .closePath()
          .fill({ color: 0xffffff, alpha: 0.55 });
        break;
      }
    }

    return gfx;
  }

  destroy(): void {
    this.staticSprite?.destroy();
    this.staticTexture?.destroy(true);
    this.tunnelSprite?.destroy();
    this.tunnelTexture?.destroy(true);
    this.nodeMap.clear();
    this.intersectionNodeIds.clear();
  }
}
