import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { PixiOverlay } from './PixiOverlay';
import type { NodeData, EdgeData } from '../bridge/commands';
import type { LightStateUpdate } from '../bridge/events';
import { projectPoint } from '../map/MapLibreSetup';

// ─── Constants ────────────────────────────────────────────────────────────────

/** Minimum zoom at which traffic lights are drawn. */
const MIN_ZOOM = 13;

/** How far along the edge (0–1) the stop line sits. */
const STOP_LINE_T = 0.88;
/** How far along the edge the signal head sits (just before stop line). */
const SIGNAL_HEAD_T = 0.82;

/** Radius of signal housing circle at zoom 16. */
const HOUSING_R_REF = 5;
/** Radius of signal bulb at zoom 16. */
const BULB_R_REF    = 3.5;

/** Half-lane-width approximation (px at zoom 16, matches RoadRenderer halfPx). */
const HALF_PX_REF = 13;

const COLOR_RED    = 0xff2222;
const COLOR_YELLOW = 0xffcc00;
const COLOR_GREEN  = 0x22dd55;

// ─── Internal types ───────────────────────────────────────────────────────────

/** One road approach leading into a traffic-light intersection. */
interface Approach {
  nodeId: number;   // intersection osm_id (for phase lookup)
  fromLat: number; fromLng: number;
  toLat: number;   toLng: number;
  lanes: number;
}

// ─── TrafficLightRenderer ─────────────────────────────────────────────────────

/**
 * Renders traffic-light signal heads and stop lines **per road approach**.
 *
 * For each incoming edge at a TrafficLight node:
 *   • A white stop line is drawn perpendicular to the road at ~88 % along the edge.
 *   • A coloured signal head (housing + bulb) is drawn at ~82 %.
 *   • All approaches at the same intersection share the same phase.
 *
 * Layer: `overlay.trafficLights`.
 */
export class TrafficLightRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  /** All traffic-light node ids. */
  private lightNodeIds: Set<number> = new Set();

  /** Precomputed approach list, rebuilt on init. */
  private approaches: Approach[] = [];

  /** Latest known phase (0=Red, 1=Yellow, 2=Green) per intersection id. */
  private lightPhases: Map<number, number> = new Map();

  /**
   * Graphics objects per intersection id.
   * A single intersection may have multiple approaches (= multiple Graphics).
   */
  private spritesByNode: Map<number, PIXI.Graphics[]> = new Map();

  /** Node IDs whose roads are all hidden. */
  private hiddenNodeIds: Set<number> = new Set();

  constructor(overlay: PixiOverlay, map: maplibregl.Map) {
    this.overlay = overlay;
    this.map = map;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  /** Call after map data loads.  Rebuilds approach list and re-draws. */
  init(nodes: NodeData[], edges: EdgeData[]): void {
    this.lightNodeIds = new Set(
      nodes
        .filter((n) => n.intersectionType === 'traffic_light')
        .map((n) => n.id),
    );

    // Build node lookup for quick position access
    const nodeMap = new Map(nodes.map((n) => [n.id, n]));

    // An "approach" is every edge whose target is a TL node.
    this.approaches = [];
    for (const edge of edges) {
      if (!this.lightNodeIds.has(edge.to)) continue;
      const fromNode = nodeMap.get(edge.from);
      const toNode   = nodeMap.get(edge.to);
      if (!fromNode || !toNode) continue;
      this.approaches.push({
        nodeId:  edge.to,
        fromLat: fromNode.lat, fromLng: fromNode.lng,
        toLat:   toNode.lat,   toLng:   toNode.lng,
        lanes:   edge.lanes,
      });
    }

    this.rebuild();
  }

  /** Sync phase states; only re-colours affected approach sprites. */
  updateStates(updates: LightStateUpdate[]): void {
    for (const upd of updates) {
      this.lightPhases.set(upd.intersectionId, upd.phase);
      const gfxList = this.spritesByNode.get(upd.intersectionId);
      if (gfxList) {
        const zoom  = this.map.getZoom();
        const scale = this.scaleForZoom(zoom);
        const hr = HOUSING_R_REF * scale;
        const br = BULB_R_REF * scale;
        for (const gfx of gfxList) {
          this.redrawSignalHead(gfx, hr, br, upd.phase);
        }
      }
    }
  }

  rebuildOnCameraChange(): void { this.rebuild(); }

  setHiddenNodeIds(ids: Set<number>): void {
    this.hiddenNodeIds = ids;
    this.rebuild();
  }

  destroy(): void {
    this.spritesByNode.clear();
    this.overlay.trafficLights.removeChildren();
  }

  // ─── Rendering ─────────────────────────────────────────────────────────────

  private rebuild(): void {
    this.overlay.trafficLights.removeChildren();
    this.spritesByNode.clear();

    const zoom = this.map.getZoom();
    if (zoom < MIN_ZOOM) return;

    const scale     = this.scaleForZoom(zoom);
    const hr        = HOUSING_R_REF * scale;
    const br        = BULB_R_REF * scale;
    const stopGfx   = new PIXI.Graphics(); // shared batch for all stop lines
    this.overlay.trafficLights.addChild(stopGfx);

    for (const ap of this.approaches) {
      if (this.hiddenNodeIds.has(ap.nodeId)) continue;

      const phase = this.lightPhases.get(ap.nodeId) ?? 0;

      // Project from/to
      const from = projectPoint(this.map, ap.fromLng, ap.fromLat);
      const to   = projectPoint(this.map, ap.toLng,   ap.toLat);

      const dx  = to.x - from.x;
      const dy  = to.y - from.y;
      const len = Math.hypot(dx, dy);
      if (len < 2) continue;

      const ux = dx / len; // unit vector along road (from → to)
      const uy = dy / len;
      // Right-perpendicular (for right-hand traffic offset)
      const rx = -uy;
      const ry =  ux;

      // ── Stop line ─────────────────────────────────────────────────────────
      // Position at STOP_LINE_T along the edge, full lane half-width extent
      const slx  = from.x + ux * len * STOP_LINE_T;
      const sly  = from.y + uy * len * STOP_LINE_T;
      const hw   = ap.lanes * HALF_PX_REF * scale; // half-width of stop line

      stopGfx
        .moveTo(slx - rx * hw, sly - ry * hw)
        .lineTo(slx + rx * hw, sly + ry * hw)
        .stroke({ width: Math.max(1, 1.5 * scale), color: 0xffffff, alpha: 0.85 });

      // ── Signal head ───────────────────────────────────────────────────────
      // Placed at SIGNAL_HEAD_T, offset right of road centre
      const shx = from.x + ux * len * SIGNAL_HEAD_T + rx * hw * 0.6;
      const shy = from.y + uy * len * SIGNAL_HEAD_T + ry * hw * 0.6;

      const headGfx = new PIXI.Graphics();
      this.drawSignalHead(headGfx, hr, br, phase);
      headGfx.x = shx;
      headGfx.y = shy;
      this.overlay.trafficLights.addChild(headGfx);

      // Register for incremental colour updates
      const list = this.spritesByNode.get(ap.nodeId) ?? [];
      list.push(headGfx);
      this.spritesByNode.set(ap.nodeId, list);
    }
  }

  /** Draw housing + coloured bulb at origin. */
  private drawSignalHead(gfx: PIXI.Graphics, hr: number, br: number, phase: number): void {
    gfx.circle(0, 0, hr).fill({ color: 0x111111, alpha: 0.9 });
    gfx.circle(0, 0, br).fill({ color: this.phaseColor(phase), alpha: 1.0 });
  }

  /** Re-draw only the bulb of an existing signal head. */
  private redrawSignalHead(gfx: PIXI.Graphics, hr: number, br: number, phase: number): void {
    gfx.clear();
    this.drawSignalHead(gfx, hr, br, phase);
  }

  private phaseColor(phase: number): number {
    switch (phase) {
      case 0:  return COLOR_RED;
      case 1:  return COLOR_YELLOW;
      case 2:  return COLOR_GREEN;
      default: return 0x333333;
    }
  }

  private scaleForZoom(zoom: number): number {
    return Math.min(2.5, Math.max(0.4, Math.pow(2, zoom - 16)));
  }
}
