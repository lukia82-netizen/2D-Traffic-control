import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, EdgeData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { projectPoint } from '../map/MapLibreSetup';

// ─── Constants ────────────────────────────────────────────────────────────────

const ARROW_SPACING_PX = 80;   // draw oneway arrow every N pixels along road
const ARROW_SIZE = 8;           // half-size of arrowhead in pixels
const TUNNEL_DASH_COLOR = 0x334455;
const TUNNEL_ALPHA = 0.6;
const BRIDGE_SHADOW_OFFSET = 4;
const BRIDGE_SHADOW_ALPHA = 0.35;

// ─── InfraRenderer ───────────────────────────────────────────────────────────

/**
 * Renders static infrastructure markings (oneway arrows, lane markings,
 * tunnel overlays, bridge shadows) into a RenderTexture that is then
 * displayed as a single sprite.
 *
 * Call buildStaticLayer() once on map load, then rebuildOnCameraChange()
 * on every map 'render' event so arrows follow map panning/zooming.
 */
export class InfraRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  private staticSprite: PIXI.Sprite | null = null;
  private staticTexture: PIXI.RenderTexture | null = null;
  private tunnelSprite: PIXI.Sprite | null = null;
  private tunnelTexture: PIXI.RenderTexture | null = null;

  constructor(overlay: PixiOverlay, map: maplibregl.Map) {
    this.overlay = overlay;
    this.map = map;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  buildStaticLayer(mapData: MapData): void {
    this.rebuildMarkings(mapData);
    this.rebuildTunnelOverlay(mapData);
  }

  /**
   * Must be called on map 'render' so arrows/overlays move with the camera.
   */
  rebuildOnCameraChange(mapData: MapData): void {
    this.rebuildMarkings(mapData);
    this.rebuildTunnelOverlay(mapData);
  }

  // ─── Markings (oneway arrows + bridge shadows) ─────────────────────────────

  private rebuildMarkings(mapData: MapData): void {
    const w = this.overlay.width;
    const h = this.overlay.height;

    const gfx = new PIXI.Graphics();

    for (const edge of mapData.edges) {
      const fromNode = mapData.nodes.find((n) => n.id === edge.from);
      const toNode = mapData.nodes.find((n) => n.id === edge.to);
      if (!fromNode || !toNode) continue;

      const fromPx = projectPoint(this.map, fromNode.lng, fromNode.lat);
      const toPx = projectPoint(this.map, toNode.lng, toNode.lat);

      // Bridge shadow
      if (edge.infraType === 'bridge') {
        this.drawBridgeShadow(gfx, fromPx, toPx, edge);
      }

      // Oneway arrows
      if (edge.oneway) {
        this.drawOnewayArrows(gfx, fromPx, toPx);
      }
    }

    // Render to texture
    const rt = PIXI.RenderTexture.create({ width: w, height: h });
    this.overlay.app.renderer.render({ container: gfx, target: rt });
    gfx.destroy();

    // Swap sprites
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
    to: { x: number; y: number },
    edge: EdgeData,
  ): void {
    const laneW = edge.lanes * 4;
    gfx.setStrokeStyle({
      width: laneW + 6,
      color: 0x111122,
      alpha: BRIDGE_SHADOW_ALPHA,
    });
    gfx.moveTo(from.x + BRIDGE_SHADOW_OFFSET, from.y + BRIDGE_SHADOW_OFFSET);
    gfx.lineTo(to.x + BRIDGE_SHADOW_OFFSET, to.y + BRIDGE_SHADOW_OFFSET);
    gfx.stroke();
  }

  private drawOnewayArrows(
    gfx: PIXI.Graphics,
    from: { x: number; y: number },
    to: { x: number; y: number },
  ): void {
    const dx = to.x - from.x;
    const dy = to.y - from.y;
    const len = Math.hypot(dx, dy);
    if (len < ARROW_SPACING_PX) return;

    const ux = dx / len;
    const uy = dy / len;
    const angle = Math.atan2(dy, dx);
    const numArrows = Math.floor(len / ARROW_SPACING_PX);

    for (let i = 1; i <= numArrows; i++) {
      const t = (i * ARROW_SPACING_PX) / len;
      const cx = from.x + dx * t;
      const cy = from.y + dy * t;
      this.drawArrow(gfx, cx, cy, angle, ux, uy);
    }
  }

  private drawArrow(
    gfx: PIXI.Graphics,
    x: number,
    y: number,
    _angle: number,
    ux: number,
    uy: number,
  ): void {
    // Perpendicular vector
    const px = -uy;
    const py = ux;

    const tip = { x: x + ux * ARROW_SIZE, y: y + uy * ARROW_SIZE };
    const left = { x: x - ux * ARROW_SIZE + px * ARROW_SIZE * 0.6, y: y - uy * ARROW_SIZE + py * ARROW_SIZE * 0.6 };
    const right = { x: x - ux * ARROW_SIZE - px * ARROW_SIZE * 0.6, y: y - uy * ARROW_SIZE - py * ARROW_SIZE * 0.6 };

    gfx
      .moveTo(tip.x, tip.y)
      .lineTo(left.x, left.y)
      .lineTo(right.x, right.y)
      .closePath()
      .fill({ color: 0xffffff, alpha: 0.45 });
  }

  // ─── Tunnel overlay ────────────────────────────────────────────────────────

  private rebuildTunnelOverlay(mapData: MapData): void {
    const w = this.overlay.width;
    const h = this.overlay.height;
    const gfx = new PIXI.Graphics();

    for (const edge of mapData.edges) {
      if (edge.infraType !== 'tunnel') continue;

      const fromNode = mapData.nodes.find((n) => n.id === edge.from);
      const toNode = mapData.nodes.find((n) => n.id === edge.to);
      if (!fromNode || !toNode) continue;

      const fromPx = projectPoint(this.map, fromNode.lng, fromNode.lat);
      const toPx = projectPoint(this.map, toNode.lng, toNode.lat);

      const laneW = Math.max(4, edge.lanes * 5);

      // Dashed tunnel line
      gfx.setStrokeStyle({
        width: laneW,
        color: TUNNEL_DASH_COLOR,
        alpha: TUNNEL_ALPHA,
      });
      gfx.moveTo(fromPx.x, fromPx.y);
      gfx.lineTo(toPx.x, toPx.y);
      gfx.stroke();

      // Tunnel portal markers
      this.drawTunnelPortal(gfx, fromPx.x, fromPx.y);
      this.drawTunnelPortal(gfx, toPx.x, toPx.y);
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
    gfx
      .rect(x - 6, y - 4, 12, 8)
      .fill({ color: 0x1a1a2e, alpha: 0.8 });
  }

  destroy(): void {
    this.staticSprite?.destroy();
    this.staticTexture?.destroy(true);
    this.tunnelSprite?.destroy();
    this.tunnelTexture?.destroy(true);
  }
}
