import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { PixiOverlay } from './PixiOverlay';
import type { NodeData } from '../bridge/commands';
import type { LightStateUpdate } from '../bridge/events';
import { projectPoint } from '../map/MapLibreSetup';

// ─── Constants ────────────────────────────────────────────────────────────────

/** Radius of the outer housing circle in px at reference zoom 16. */
const HOUSING_RADIUS_REF = 8;
/** Radius of the light bulb circle in px at reference zoom 16. */
const BULB_RADIUS_REF = 5;
/** Minimum zoom at which traffic lights are drawn. */
const MIN_ZOOM = 13;

const COLOR_RED    = 0xff2222;
const COLOR_YELLOW = 0xffcc00;
const COLOR_GREEN  = 0x22dd55;
const COLOR_OFF    = 0x333333;

// ─── TrafficLightRenderer ─────────────────────────────────────────────────────

/**
 * Renders coloured circle indicators at every traffic-light intersection node.
 *
 * Design:
 *   • Dark housing circle (always visible) with a bright inner bulb.
 *   • Bulb colour = current phase: Red / Yellow / Green.
 *   • Rebuilt each time the MapLibre camera changes (same pattern as InfraRenderer).
 *   • Updated on `light_state_change` events without full rebuild — only the
 *     affected sprite's tint is changed.
 *
 * Layer: `overlay.trafficLights` (Layer 6 in PixiOverlay stack).
 */
export class TrafficLightRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  /** All traffic-light nodes from the road graph. */
  private lightNodes: NodeData[] = [];

  /** Latest known phase (0=Red, 1=Yellow, 2=Green) per intersection OSM id. */
  private lightPhases: Map<number, number> = new Map();

  /** Sprite lookup for incremental phase updates. */
  private sprites: Map<number, PIXI.Graphics> = new Map();

  constructor(overlay: PixiOverlay, map: maplibregl.Map) {
    this.overlay = overlay;
    this.map = map;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  /** Must be called after map data is loaded. */
  init(nodes: NodeData[]): void {
    this.lightNodes = nodes.filter((n) => n.intersectionType === 'traffic_light');
    this.rebuild();
  }

  /**
   * Sync light states from a `light_state_change` event.
   * Updates only the affected sprites for efficiency.
   */
  updateStates(updates: LightStateUpdate[]): void {
    for (const upd of updates) {
      this.lightPhases.set(upd.intersectionId, upd.phase);
      const sprite = this.sprites.get(upd.intersectionId);
      if (sprite) {
        this.applyPhaseColor(sprite, upd.phase);
      }
    }
  }

  /** Rebuild all sprites when the camera moves (pan / zoom). */
  rebuildOnCameraChange(): void {
    this.rebuild();
  }

  destroy(): void {
    this.sprites.clear();
    this.overlay.trafficLights.removeChildren();
  }

  // ─── Private helpers ───────────────────────────────────────────────────────

  private rebuild(): void {
    const zoom = this.map.getZoom();

    this.overlay.trafficLights.removeChildren();
    this.sprites.clear();

    if (zoom < MIN_ZOOM) return;

    const scale = this.scaleForZoom(zoom);
    const hr    = HOUSING_RADIUS_REF * scale;
    const br    = BULB_RADIUS_REF    * scale;

    for (const node of this.lightNodes) {
      const px    = projectPoint(this.map, node.lng, node.lat);
      const phase = this.lightPhases.get(node.id) ?? 0; // default Red

      const gfx = new PIXI.Graphics();
      this.drawLight(gfx, hr, br, phase);
      gfx.x = px.x;
      gfx.y = px.y;

      this.overlay.trafficLights.addChild(gfx);
      this.sprites.set(node.id, gfx);
    }
  }

  /** Draw one traffic-light indicator (housing + bulb) into `gfx`. */
  private drawLight(gfx: PIXI.Graphics, hr: number, br: number, phase: number): void {
    // Dark housing
    gfx.circle(0, 0, hr).fill({ color: 0x1a1a1a, alpha: 0.85 });

    // Coloured bulb
    const color = this.phaseColor(phase);
    gfx.circle(0, 0, br).fill({ color, alpha: 0.95 });
  }

  /** Update only the bulb colour of an existing sprite. */
  private applyPhaseColor(gfx: PIXI.Graphics, phase: number): void {
    const zoom  = this.map.getZoom();
    const scale = this.scaleForZoom(zoom);
    const hr    = HOUSING_RADIUS_REF * scale;
    const br    = BULB_RADIUS_REF    * scale;
    gfx.clear();
    this.drawLight(gfx, hr, br, phase);
  }

  private phaseColor(phase: number): number {
    switch (phase) {
      case 0:  return COLOR_RED;
      case 1:  return COLOR_YELLOW;
      case 2:  return COLOR_GREEN;
      default: return COLOR_OFF;
    }
  }

  /**
   * Scale factor: 1.0 at zoom 16, grows/shrinks by powers of 2.
   * Clamped so lights don't become invisible at low zoom or giant at high zoom.
   */
  private scaleForZoom(zoom: number): number {
    return Math.min(2.5, Math.max(0.4, Math.pow(2, zoom - 16)));
  }
}
