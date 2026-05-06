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
const STOP_LINE_T = 0.985;
/** How far along the edge the signal head sits (just before stop line). */
const SIGNAL_HEAD_T = 0.965;

/** Radius of signal housing circle at zoom 16. */
const HOUSING_R_REF = 5;
/** Radius of signal bulb at zoom 16. */
const BULB_R_REF    = 3.5;

/** Half-lane-width approximation (px at zoom 16, matches RoadRenderer halfPx). */
const HALF_PX_REF = 13;

const COLOR_RED    = 0xff2222;
const COLOR_YELLOW = 0xffcc00;
const COLOR_GREEN  = 0x22dd55;

/** Width of each zebra stripe at zoom 16. */
const ZEBRA_STRIPE_W_REF = 4;
/** Number of zebra stripes. */
const ZEBRA_STRIPES = 5;
/** Extra lateral offset so heads sit beside the right road edge, not center. */
const SIGNAL_SIDE_MARGIN_REF = 9;

// ─── Internal types ───────────────────────────────────────────────────────────

/** One road approach leading into a traffic-light intersection. */
interface Approach {
  nodeId: number; // intersection osm_id (for phase lookup)
  fromLat: number; fromLng: number;
  toLat: number; toLng: number;
  lanes: number;
  isPedestrian: boolean;
  /** Matches Rust `JunctionLayout` clockwise inbound sort (bearing = atan2(Δlng, Δlat)). */
  armIndex: number;
}

// ─── TrafficLightRenderer ─────────────────────────────────────────────────────

/**
 * Renders traffic-light signal heads and stop lines **per road approach**.
 *
 * For each incoming edge at a TrafficLight node:
 *   • A white stop line is drawn perpendicular to the road at ~88 % along the edge.
 *   • A coloured signal head (housing + bulb) is drawn at ~82 %.
 *   • Vehicle junctions: per-arm bulbs when the backend sends `junctionArmPhases` (opposing greens).
 *
 * Layer: `overlay.trafficLights`.
 */
export class TrafficLightRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  /** All traffic-light node ids (includes pedestrian crossings). */
  private lightNodeIds: Set<number> = new Set();
  /** Node ids that are pedestrian crossings. */
  private pedestrianNodeIds: Set<number> = new Set();

  /** Precomputed approach list, rebuilt on init. */
  private approaches: Approach[] = [];

  /** Latest known coarse phase (0=Red, 1=Yellow, 2=Green) per intersection id. */
  private lightPhases: Map<number, number> = new Map();
  /** Per-arm phases when the backend sends [`junctionArmPhases`]; `null` = use `lightPhases` only. */
  private junctionArmSignals: Map<number, number[] | null> = new Map();

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
        .filter((n) => n.intersectionType === 'traffic_light' || n.intersectionType === 'pedestrian_crossing')
        .map((n) => n.id),
    );
    this.pedestrianNodeIds = new Set(
      nodes
        .filter((n) => n.intersectionType === 'pedestrian_crossing')
        .map((n) => n.id),
    );

    // Build node lookup for quick position access
    const nodeMap = new Map(nodes.map((n) => [n.id, n]));

    type RawAp = {
      fromN: number;
      toN: number;
      nodeId: number;
      fromLat: number;
      fromLng: number;
      toLat: number;
      toLng: number;
      lanes: number;
      isPedestrian: boolean;
    };

    /** Match Rust phased layout: atan2(deltaLng, deltaLat) clocksorted. */
    const bearingIn = (
      fl: number,
      flg: number,
      tl: number,
      tlg: number,
    ): number => Math.atan2(tlg - flg, tl - fl);

    const raw: RawAp[] = [];
    for (const edge of edges) {
      if (!this.lightNodeIds.has(edge.to)) continue;
      const fromNode = nodeMap.get(edge.from);
      const toNode = nodeMap.get(edge.to);
      if (!fromNode || !toNode) continue;
      raw.push({
        fromN: edge.from,
        toN: edge.to,
        nodeId: edge.to,
        fromLat: fromNode.lat,
        fromLng: fromNode.lng,
        toLat: toNode.lat,
        toLng: toNode.lng,
        lanes: edge.lanes,
        isPedestrian: this.pedestrianNodeIds.has(edge.to),
      });
    }

    const byDest = new Map<number, RawAp[]>();
    for (const r of raw) {
      const list = byDest.get(r.nodeId) ?? [];
      list.push(r);
      byDest.set(r.nodeId, list);
    }

    // Per intersection: sort inbound arms clockwise (same rule as Rust `JunctionLayout`).
    this.approaches = [];
    for (const [, group] of byDest) {
      group.sort((a, b) => {
        const ba = bearingIn(a.fromLat, a.fromLng, a.toLat, a.toLng);
        const bb = bearingIn(b.fromLat, b.fromLng, b.toLat, b.toLng);
        const dba = ba - bb;
        if (Math.abs(dba) > 1e-9) return dba;
        return a.fromN - b.fromN || a.toN - b.toN;
      });

      group.forEach((r, armIndex) => {
        this.approaches.push({
          nodeId: r.nodeId,
          fromLat: r.fromLat,
          fromLng: r.fromLng,
          toLat: r.toLat,
          toLng: r.toLng,
          lanes: r.lanes,
          isPedestrian: r.isPedestrian,
          armIndex,
        });
      });
    }

    // Deterministic traversal for graphics list index ↔ approach order.
    this.approaches.sort((a, b) => a.nodeId - b.nodeId || a.armIndex - b.armIndex);

    this.rebuild();
  }

  /** Sync phase states; only re-colours affected approach sprites. */
  updateStates(updates: LightStateUpdate[]): void {
    for (const upd of updates) {
      this.lightPhases.set(upd.intersectionId, upd.phase);
      this.junctionArmSignals.set(
        upd.intersectionId,
        upd.junctionArmPhases !== undefined ? upd.junctionArmPhases : null,
      );

      const gfxList = this.spritesByNode.get(upd.intersectionId);
      if (!gfxList) continue;

      const zoom = this.map.getZoom();
      const scale = this.scaleForZoom(zoom);
      const hr = HOUSING_R_REF * scale;
      const br = BULB_R_REF * scale;

      const armSignals = this.junctionArmSignals.get(upd.intersectionId);
      let gfxPos = 0;
      for (const ap of this.approaches) {
        if (ap.nodeId !== upd.intersectionId) continue;

        if (ap.isPedestrian) {
          if (gfxPos + 1 >= gfxList.length) break;
          const carPhase = upd.phase;
          this.redrawSignalHead(gfxList[gfxPos], hr, br, carPhase);
          const pedPhase = carPhase === 2 ? 0 : carPhase === 0 ? 2 : 1;
          this.redrawPedestrianSignal(gfxList[gfxPos + 1], hr * 1.2, br * 1.2, pedPhase);
          gfxPos += 2;
        } else {
          if (gfxPos >= gfxList.length) break;
          const bulbPhase =
            armSignals && armSignals[ap.armIndex] !== undefined
              ? armSignals[ap.armIndex]!
              : upd.phase;
          this.redrawSignalHead(gfxList[gfxPos], hr, br, bulbPhase);
          gfxPos += 1;
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

    const scale   = this.scaleForZoom(zoom);
    const hr      = HOUSING_R_REF * scale;
    const br      = BULB_R_REF * scale;
    const staticGfx = new PIXI.Graphics(); // shared batch for stop lines + zebras
    this.overlay.trafficLights.addChild(staticGfx);

    for (const ap of this.approaches) {
      if (this.hiddenNodeIds.has(ap.nodeId)) continue;

      const arms = this.junctionArmSignals.get(ap.nodeId);
      const phase =
        !ap.isPedestrian && arms && arms[ap.armIndex] !== undefined
          ? arms[ap.armIndex]!
          : (this.lightPhases.get(ap.nodeId) ?? 0);

      // Project from/to
      const from = projectPoint(this.map, ap.fromLng, ap.fromLat);
      const to   = projectPoint(this.map, ap.toLng,   ap.toLat);

      const dx  = to.x - from.x;
      const dy  = to.y - from.y;
      const len = Math.hypot(dx, dy);
      if (len < 2) continue;

      const ux = dx / len; // unit vector along road (from → to)
      const uy = dy / len;
      // Right-perpendicular (for right-hand traffic)
      const rx = -uy;
      const ry =  ux;

      const hw = ap.lanes * HALF_PX_REF * scale; // half-width of road
      const sideOffset = hw + SIGNAL_SIDE_MARGIN_REF * scale;

      if (ap.isPedestrian) {
        // ── Pedestrian crossing: zebra stripes ──────────────────────────────
        // Draw across the full road width at the node position (t=1.0)
        const crossX = to.x;
        const crossY = to.y;
        const totalW = ZEBRA_STRIPE_W_REF * scale * ZEBRA_STRIPES * 2;
        // Centre the stripes on the node, extend ±hw perpendicular
        for (let s = 0; s < ZEBRA_STRIPES; s++) {
          const stripePx = ZEBRA_STRIPE_W_REF * scale;
          // Offset along road direction so stripes go from -totalW/2 to +totalW/2
          const off = -totalW / 2 + stripePx / 2 + s * stripePx * 2;
          const sx  = crossX + ux * off;
          const sy  = crossY + uy * off;
          staticGfx
            .moveTo(sx - rx * hw * 1.1, sy - ry * hw * 1.1)
            .lineTo(sx + rx * hw * 1.1, sy + ry * hw * 1.1)
            .stroke({ width: stripePx, color: 0xeeeeee, alpha: 0.9 });
        }

        // ── Stop line (before the zebra) ────────────────────────────────────
        const slx = from.x + ux * len * STOP_LINE_T;
        const sly = from.y + uy * len * STOP_LINE_T;
        staticGfx
          .moveTo(slx - rx * hw, sly - ry * hw)
          .lineTo(slx + rx * hw, sly + ry * hw)
          .stroke({ width: Math.max(1.5, 2 * scale), color: 0xffffff, alpha: 0.9 });

        // ── Car signal head (right side, before stop line) ──────────────────
        const shx = from.x + ux * len * SIGNAL_HEAD_T + rx * sideOffset;
        const shy = from.y + uy * len * SIGNAL_HEAD_T + ry * sideOffset;
        const carGfx = new PIXI.Graphics();
        this.drawSignalHead(carGfx, hr, br, phase);
        carGfx.x = shx; carGfx.y = shy;
        this.overlay.trafficLights.addChild(carGfx);

        // ── Pedestrian signal head (left side, at crossing) ─────────────────
        // Phase is INVERSE: when cars see RED (phase=0), pedestrians see GREEN
        const pedPhase = phase === 2 ? 0 : (phase === 0 ? 2 : 1);
        const phx = crossX - rx * (sideOffset + hr * 0.5) - ux * ZEBRA_STRIPE_W_REF * scale * ZEBRA_STRIPES;
        const phy = crossY - ry * (sideOffset + hr * 0.5) - uy * ZEBRA_STRIPE_W_REF * scale * ZEBRA_STRIPES;
        const pedGfx = new PIXI.Graphics();
        this.drawPedestrianSignal(pedGfx, hr * 1.2, br * 1.2, pedPhase);
        pedGfx.x = phx; pedGfx.y = phy;
        this.overlay.trafficLights.addChild(pedGfx);

        // Register BOTH heads for incremental updates
        const list = this.spritesByNode.get(ap.nodeId) ?? [];
        list.push(carGfx, pedGfx);
        this.spritesByNode.set(ap.nodeId, list);

      } else {
        // ── Regular traffic light ────────────────────────────────────────────
        const slx = from.x + ux * len * STOP_LINE_T;
        const sly = from.y + uy * len * STOP_LINE_T;
        staticGfx
          .moveTo(slx - rx * hw, sly - ry * hw)
          .lineTo(slx + rx * hw, sly + ry * hw)
          .stroke({ width: Math.max(1, 1.5 * scale), color: 0xffffff, alpha: 0.85 });

        const shx = from.x + ux * len * SIGNAL_HEAD_T + rx * sideOffset;
        const shy = from.y + uy * len * SIGNAL_HEAD_T + ry * sideOffset;
        const headGfx = new PIXI.Graphics();
        this.drawSignalHead(headGfx, hr, br, phase);
        headGfx.x = shx; headGfx.y = shy;
        this.overlay.trafficLights.addChild(headGfx);

        const list = this.spritesByNode.get(ap.nodeId) ?? [];
        list.push(headGfx);
        this.spritesByNode.set(ap.nodeId, list);
      }
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

  /**
   * Draw a pedestrian signal: square housing + circular bulb.
   * Green phase (2) = walking man (shown as bright green).
   * Red phase (0)   = stop (bright red).
   */
  private drawPedestrianSignal(gfx: PIXI.Graphics, hr: number, br: number, phase: number): void {
    // Square housing
    gfx.rect(-hr, -hr, hr * 2, hr * 2).fill({ color: 0x111111, alpha: 0.92 });
    // Coloured bulb
    gfx.circle(0, 0, br).fill({ color: this.phaseColor(phase), alpha: 1.0 });
    // Small "P" indicator — a slightly smaller inner ring
    if (phase === 2) {
      gfx.circle(0, 0, br * 0.45).fill({ color: 0x004400, alpha: 0.6 });
    }
  }

  private redrawPedestrianSignal(gfx: PIXI.Graphics, hr: number, br: number, phase: number): void {
    gfx.clear();
    this.drawPedestrianSignal(gfx, hr, br, phase);
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
