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

  constructor(overlay: PixiOverlay, map: maplibregl.Map, camera: CameraManager) {
    this.overlay = overlay;
    this.map = map;
    this.camera = camera;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  buildStaticLayer(mapData: MapData): void {
    // Build O(1) node lookup so we never use Array.find() per edge
    this.nodeMap.clear();
    for (const n of mapData.nodes) {
      this.nodeMap.set(n.id, n);
    }
    this.rebuildMarkings(mapData);
    this.rebuildTunnelOverlay(mapData);
    this.rebuildArrows(mapData);
  }

  /** Must be called on map `render` so all markings follow camera pan / zoom. */
  rebuildOnCameraChange(mapData: MapData): void {
    this.rebuildMarkings(mapData);
    this.rebuildTunnelOverlay(mapData);
    this.rebuildArrows(mapData);
  }

  /**
   * No-op — arrows are now static road markings.
   * Kept for API compatibility (game.ts calls this each tick).
   */
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  update(_deltaMS: number): void {
    // intentionally empty
  }

  // ─── Static markings (bridge shadows) ─────────────────────────────────────

  private rebuildMarkings(mapData: MapData): void {
    const w = this.overlay.width;
    const h = this.overlay.height;
    const gfx = new PIXI.Graphics();

    for (const edge of mapData.edges) {
      if (edge.infraType !== 'bridge') continue;

      const fromNode = this.nodeMap.get(edge.from);
      const toNode   = this.nodeMap.get(edge.to);
      if (!fromNode || !toNode) continue;

      const fromPx = projectPoint(this.map, fromNode.lng, fromNode.lat);
      const toPx   = projectPoint(this.map, toNode.lng,   toNode.lat);
      this.drawBridgeShadow(gfx, fromPx, toPx, edge);
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

  destroy(): void {
    this.staticSprite?.destroy();
    this.staticTexture?.destroy(true);
    this.tunnelSprite?.destroy();
    this.tunnelTexture?.destroy(true);
    this.nodeMap.clear();
  }
}
