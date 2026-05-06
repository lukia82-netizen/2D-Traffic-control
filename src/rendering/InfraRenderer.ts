import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, EdgeData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';
import { projectPoint } from '../map/MapLibreSetup';

// ─── Constants ────────────────────────────────────────────────────────────────

const TUNNEL_DASH_COLOR = 0x334455;
const TUNNEL_ALPHA = 0.6;
const BRIDGE_SHADOW_OFFSET = 4;
const BRIDGE_SHADOW_ALPHA = 0.35;

/** How fast animated arrows scroll along one-way roads (px / second). */
const ARROW_SCROLL_SPEED = 28;

// ─── Internal types ───────────────────────────────────────────────────────────

interface ArrowEdgeInfo {
  /** Individual arrow triangles placed along this edge. */
  arrows: PIXI.Graphics[];
  startX: number;
  startY: number;
  /** Unit direction vector (from → to). */
  dirX: number;
  dirY: number;
  spacing: number;
  segLen: number;
}

// ─── InfraRenderer ───────────────────────────────────────────────────────────

/**
 * Renders infrastructure markings in two passes:
 *
 * 1. **Static layer** (`staticMarkings` / `tunnelOverlay`): bridge shadows and
 *    tunnel dashes baked into `RenderTexture`s — rebuilt on camera change.
 *
 * 2. **Animated arrow layer** (`arrowLayer`): live `PIXI.Graphics` triangles
 *    on one-way roads that scroll in the direction of travel.  Call
 *    `update(deltaMS)` every game-loop tick to advance the animation.
 *
 * Call `buildStaticLayer()` once on map load and `rebuildOnCameraChange()` on
 * every map `render` event.
 */
export class InfraRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;
  private readonly camera: CameraManager;

  private staticSprite: PIXI.Sprite | null = null;
  private staticTexture: PIXI.RenderTexture | null = null;
  private tunnelSprite: PIXI.Sprite | null = null;
  private tunnelTexture: PIXI.RenderTexture | null = null;

  /** Accumulated scroll offset in pixels, reset modulo max-segment-length. */
  private arrowPhase = 0;
  private arrowEdges: ArrowEdgeInfo[] = [];

  constructor(overlay: PixiOverlay, map: maplibregl.Map, camera: CameraManager) {
    this.overlay = overlay;
    this.map = map;
    this.camera = camera;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  buildStaticLayer(mapData: MapData): void {
    this.rebuildMarkings(mapData);
    this.rebuildTunnelOverlay(mapData);
    this.rebuildArrows(mapData);
  }

  /**
   * Must be called on map `render` so all markings follow camera pan / zoom.
   */
  rebuildOnCameraChange(mapData: MapData): void {
    this.rebuildMarkings(mapData);
    this.rebuildTunnelOverlay(mapData);
    this.rebuildArrows(mapData);
  }

  /**
   * Advance the arrow animation.  Call once per game-loop tick.
   * The animation runs even when the simulation is paused so that the map
   * always looks alive.
   */
  update(deltaMS: number): void {
    if (this.arrowEdges.length === 0) return;

    this.arrowPhase += (deltaMS / 1000) * ARROW_SCROLL_SPEED;

    for (const edge of this.arrowEdges) {
      const { arrows, startX, startY, dirX, dirY, spacing, segLen } = edge;
      const phaseOffset = this.arrowPhase % spacing;

      for (let i = 0; i < arrows.length; i++) {
        const t = (i * spacing + phaseOffset) % segLen;
        arrows[i].x = startX + dirX * t;
        arrows[i].y = startY + dirY * t;
      }
    }
  }

  // ─── Static markings (bridge shadows only) ─────────────────────────────────

  private rebuildMarkings(mapData: MapData): void {
    const w = this.overlay.width;
    const h = this.overlay.height;

    const gfx = new PIXI.Graphics();

    for (const edge of mapData.edges) {
      if (edge.infraType !== 'bridge') continue;

      const fromNode = mapData.nodes.find((n) => n.id === edge.from);
      const toNode = mapData.nodes.find((n) => n.id === edge.to);
      if (!fromNode || !toNode) continue;

      const fromPx = projectPoint(this.map, fromNode.lng, fromNode.lat);
      const toPx = projectPoint(this.map, toNode.lng, toNode.lat);
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
    to: { x: number; y: number },
    edge: EdgeData,
  ): void {
    const laneW = this.camera.getRoadOverlayWidth(edge.lanes);
    gfx.setStrokeStyle({
      width: laneW + 6,
      color: 0x111122,
      alpha: BRIDGE_SHADOW_ALPHA,
    });
    gfx.moveTo(from.x + BRIDGE_SHADOW_OFFSET, from.y + BRIDGE_SHADOW_OFFSET);
    gfx.lineTo(to.x + BRIDGE_SHADOW_OFFSET, to.y + BRIDGE_SHADOW_OFFSET);
    gfx.stroke();
  }

  // ─── Animated oneway arrows ────────────────────────────────────────────────

  private rebuildArrows(mapData: MapData): void {
    // Destroy all previous arrow graphics and clear the layer
    for (const edge of this.arrowEdges) {
      for (const a of edge.arrows) a.destroy();
    }
    this.arrowEdges = [];
    this.overlay.arrowLayer.removeChildren();

    const spacing = this.camera.getArrowSpacing();

    for (const edge of mapData.edges) {
      if (!edge.oneway) continue;

      const fromNode = mapData.nodes.find((n) => n.id === edge.from);
      const toNode = mapData.nodes.find((n) => n.id === edge.to);
      if (!fromNode || !toNode) continue;

      const from = projectPoint(this.map, fromNode.lng, fromNode.lat);
      const to = projectPoint(this.map, toNode.lng, toNode.lat);

      const dx = to.x - from.x;
      const dy = to.y - from.y;
      const segLen = Math.hypot(dx, dy);
      if (segLen < spacing) continue;

      const dirX = dx / segLen;
      const dirY = dy / segLen;
      const numArrows = Math.floor(segLen / spacing);

      const arrows: PIXI.Graphics[] = [];
      for (let i = 0; i < numArrows; i++) {
        const gfx = this.makeArrowShape(dirX, dirY);
        this.overlay.arrowLayer.addChild(gfx);
        arrows.push(gfx);
      }

      this.arrowEdges.push({
        arrows,
        startX: from.x,
        startY: from.y,
        dirX,
        dirY,
        spacing,
        segLen,
      });
    }

    // Immediately position all arrows at the current phase so there is no
    // one-frame pop after a camera change.
    this.update(0);
  }

  private makeArrowShape(dirX: number, dirY: number): PIXI.Graphics {
    const sz = this.camera.getArrowSize();
    const px = -dirY;
    const py = dirX;

    const gfx = new PIXI.Graphics();
    gfx
      .moveTo(dirX * sz, dirY * sz)
      .lineTo(-dirX * sz + px * sz * 0.6, -dirY * sz + py * sz * 0.6)
      .lineTo(-dirX * sz - px * sz * 0.6, -dirY * sz - py * sz * 0.6)
      .closePath()
      .fill({ color: 0xffffff, alpha: 0.5 });
    return gfx;
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

      const laneW = Math.max(4, this.camera.getRoadOverlayWidth(edge.lanes));

      gfx.setStrokeStyle({
        width: laneW,
        color: TUNNEL_DASH_COLOR,
        alpha: TUNNEL_ALPHA,
      });
      gfx.moveTo(fromPx.x, fromPx.y);
      gfx.lineTo(toPx.x, toPx.y);
      gfx.stroke();

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
    for (const edge of this.arrowEdges) {
      for (const a of edge.arrows) a.destroy();
    }
    this.arrowEdges = [];
  }
}
