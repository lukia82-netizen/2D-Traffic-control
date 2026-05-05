import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { CongestionData } from '../bridge/events';
import type { MapData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { projectPoint } from '../map/MapLibreSetup';

const WARNING_LEVEL = 0.7;
const CRITICAL_LEVEL = 0.9;
const LINE_BASE_WIDTH = 6;
const EDGE_MARGIN = 24; // pixels from screen edge for indicator positioning

// ─── CongestionRenderer ───────────────────────────────────────────────────────

/**
 * Draws coloured overlays on congested edges and creates pulsing
 * edge-indicator divs for off-screen congestion.
 *
 * Updated only on congestion_update events (every ~500ms real), not per frame.
 */
export class CongestionRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;
  private readonly hudContainer: HTMLElement;

  private congestionSprite: PIXI.Sprite | null = null;
  private congestionTexture: PIXI.RenderTexture | null = null;
  private readonly edgeIndicators: Map<string, HTMLDivElement> = new Map();

  constructor(overlay: PixiOverlay, map: maplibregl.Map, hudContainer: HTMLElement) {
    this.overlay = overlay;
    this.map = map;
    this.hudContainer = hudContainer;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  update(
    congestionData: CongestionData[],
    _mapData: MapData,
  ): void {
    const w = this.overlay.width;
    const h = this.overlay.height;
    const gfx = new PIXI.Graphics();

    // Collect ids of congested edges for indicator cleanup
    const activeEdgeIds = new Set<string>();

    for (const cong of congestionData) {
      if (cong.level < WARNING_LEVEL) continue;

      const color = this.getCongestionColor(cong.level);
      const alpha = cong.level > CRITICAL_LEVEL ? 0.85 : 0.6;
      const px = projectPoint(this.map, cong.lng, cong.lat);
      const onScreen = this.isOnScreen(px.x, px.y, w, h);

      if (onScreen) {
        // Draw a thickened coloured dot/dash on the congested position
        gfx
          .circle(px.x, px.y, LINE_BASE_WIDTH + cong.level * 6)
          .fill({ color, alpha });
      }

      // Edge indicator for off-screen congestion
      const edgeKey = String(cong.edgeId);
      activeEdgeIds.add(edgeKey);

      if (!onScreen) {
        let div = this.edgeIndicators.get(edgeKey);
        if (!div) {
          div = this.createEdgeIndicator(edgeKey, cong.lat, cong.lng);
          this.edgeIndicators.set(edgeKey, div);
        }
        this.positionEdgeIndicator(div, px.x, px.y, w, h, cong.level);
        div.className = `edge-indicator ${cong.level > CRITICAL_LEVEL ? 'critical' : 'warning'}`;
      } else {
        // Remove indicator if now on screen
        const existing = this.edgeIndicators.get(edgeKey);
        if (existing) {
          existing.remove();
          this.edgeIndicators.delete(edgeKey);
        }
      }
    }

    // Remove stale indicators for edges that are no longer congested
    for (const [key, div] of this.edgeIndicators) {
      if (!activeEdgeIds.has(key)) {
        div.remove();
        this.edgeIndicators.delete(key);
      }
    }

    // Render to texture
    const rt = PIXI.RenderTexture.create({ width: w, height: h });
    this.overlay.app.renderer.render({ container: gfx, target: rt });
    gfx.destroy();

    if (this.congestionTexture) this.congestionTexture.destroy(true);
    this.congestionTexture = rt;

    if (!this.congestionSprite) {
      this.congestionSprite = new PIXI.Sprite(rt);
      this.overlay.congestionLayer.addChild(this.congestionSprite);
    } else {
      this.congestionSprite.texture = rt;
    }
  }

  // ─── Helpers ───────────────────────────────────────────────────────────────

  private isOnScreen(x: number, y: number, w: number, h: number): boolean {
    return x >= 0 && x <= w && y >= 0 && y <= h;
  }

  private createEdgeIndicator(
    edgeKey: string,
    _lat: number,
    _lng: number,
  ): HTMLDivElement {
    const div = document.createElement('div');
    div.className = 'edge-indicator warning';
    div.dataset.edgeId = edgeKey;
    div.textContent = '!';
    this.hudContainer.appendChild(div);
    return div;
  }

  private positionEdgeIndicator(
    div: HTMLDivElement,
    px: number,
    py: number,
    w: number,
    h: number,
    _level: number,
  ): void {
    // Clamp to screen edge
    const clampedX = Math.max(EDGE_MARGIN, Math.min(w - EDGE_MARGIN, px));
    const clampedY = Math.max(EDGE_MARGIN, Math.min(h - EDGE_MARGIN, py));

    // If the point is off-screen, project to the nearest screen edge
    let finalX = clampedX;
    let finalY = clampedY;

    const cx = w / 2;
    const cy = h / 2;
    const dx = px - cx;
    const dy = py - cy;

    if (Math.abs(dx) > w / 2 || Math.abs(dy) > h / 2) {
      const scaleX = (w / 2 - EDGE_MARGIN) / Math.abs(dx || 1);
      const scaleY = (h / 2 - EDGE_MARGIN) / Math.abs(dy || 1);
      const scale = Math.min(scaleX, scaleY);
      finalX = Math.round(cx + dx * scale);
      finalY = Math.round(cy + dy * scale);
    }

    div.style.left = `${finalX - 14}px`;
    div.style.top = `${finalY - 14}px`;
  }

  /** Map congestion level 0–1 to a colour between yellow (0.7) and red (1.0). */
  private getCongestionColor(level: number): number {
    const t = Math.max(0, Math.min(1, (level - WARNING_LEVEL) / (1 - WARNING_LEVEL)));
    const r = Math.round(255);
    const g = Math.round(255 * (1 - t));
    const b = 0;
    return (r << 16) | (g << 8) | b;
  }

  destroy(): void {
    for (const div of this.edgeIndicators.values()) div.remove();
    this.edgeIndicators.clear();
    this.congestionSprite?.destroy();
    this.congestionTexture?.destroy(true);
  }
}
