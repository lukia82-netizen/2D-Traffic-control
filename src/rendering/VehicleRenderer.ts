import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { VehicleState } from '../bridge/events';
import type { MapData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';
import { ROAD_TYPE_GROUP } from './RoadRenderer';


// ─── Vehicle visual constants ─────────────────────────────────────────────────

const VEHICLE_COLORS: Record<number, number> = {
  0: 0x4488ff, // Car   – blue
  1: 0xffcc44, // Van   – yellow
  2: 0xff8c00, // Bus   – orange
  3: 0x8b2500, // Truck – dark red
  4: 0xffd700, // Tram  – gold
};

const DOT_COLORS: Record<number, number> = {
  0: 0x66aaff, // Car
  1: 0xffdd66, // Van
  2: 0xffaa33, // Bus
  3: 0xcc4400, // Truck
  4: 0xffe066, // Tram – lighter gold
};

/**
 * Base pixel dimensions at spriteScale = 1.0.
 * CameraManager.getCarVisuals().spriteScale multiplies these.
 */
const VEHICLE_DIMS: Record<number, { w: number; h: number }> = {
  0: { w: 6,  h: 12 }, // Car
  1: { w: 7,  h: 16 }, // Van
  2: { w: 7,  h: 28 }, // Bus
  3: { w: 8,  h: 38 }, // Truck
  4: { w: 7,  h: 52 }, // Tram
};

/** Target vehicle width as a fraction of one lane width (all zoom levels). */
const VEHICLE_WIDTH_FILL: Record<number, number> = {
  0: 0.76, // car
  1: 0.84, // van
  2: 0.90, // bus
  3: 0.94, // truck
  4: 0.90, // tram
};

/** Vehicle length as a multiple of rendered width. */
const VEHICLE_LENGTH_FACTOR: Record<number, number> = {
  0: 1.9, // car
  1: 2.2, // van
  2: 2.8, // bus
  3: 3.2, // truck
  4: 4.2, // tram
};

/** Frustration thresholds for visual bubbles. */
const FRUSTRATION_CALM    = 40;
const FRUSTRATION_ANNOYED = 65;
const FRUSTRATION_ANGRY   = 85;
const FRUSTRATION_RAGE    = 99;

/** Bubble labels per threshold (annoyed, angry, raging, rage-quit) */
export const BUBBLE_LABELS = ['!', '!!', '!!!', '💢'] as const;
const BUBBLE_COLORS = [0xffdd00, 0xff8800, 0xff3300, 0xff0000] as const;
const BUBBLE_OFFSET_Y = 12;

const TUNNEL_ALPHA = 0.25;
const VEHICLE_TYPES = [0, 1, 2, 3, 4];

// ─── Helpers ─────────────────────────────────────────────────────────────────

function frustrationDotColor(frustration: number, typeColor: number): number {
  if (frustration >= FRUSTRATION_ANGRY)   return 0xff3300;
  if (frustration >= FRUSTRATION_ANNOYED) return 0xff8800;
  if (frustration >= FRUSTRATION_CALM)    return 0xffdd00;
  return typeColor;
}


// ─── Road-group spatial index ─────────────────────────────────────────────────

const LNG_SCALE = Math.cos(51.8 * Math.PI / 180); // ≈ 0.617

const RECHECK_THRESHOLD = 0.00008; // ≈ 9 m

interface EdgeLine {
  aLat: number; aLng: number;
  bLat: number; bLng: number;
  group: string;
}

function ptSegDist2(
  pLat: number, pLng: number,
  aLat: number, aLng: number,
  bLat: number, bLng: number,
): number {
  const dLat = bLat - aLat;
  const dLng = (bLng - aLng) * LNG_SCALE;
  const len2 = dLat * dLat + dLng * dLng;
  if (len2 === 0) {
    const eLat = pLat - aLat, eLng = (pLng - aLng) * LNG_SCALE;
    return eLat * eLat + eLng * eLng;
  }
  const pLat2 = pLat - aLat;
  const pLng2 = (pLng - aLng) * LNG_SCALE;
  const t = Math.max(0, Math.min(1, (pLat2 * dLat + pLng2 * dLng) / len2));
  const rLat = pLat2 - t * dLat;
  const rLng = pLng2 - t * dLng;
  return rLat * rLat + rLng * rLng;
}

// ─── Raw state per vehicle (no smoothing) ────────────────────────────────────

interface SmoothState {
  lat: number;
  lng: number;
  angle: number;
}

// ─── VehicleRenderer ─────────────────────────────────────────────────────────

/**
 * Renders moving vehicles on top of the MapLibre map via PixiJS.
 *
 * Two rendering modes managed by CameraManager:
 *  • dot    – single batched Graphics call per frame (ultra-fast for 3000+ cars)
 *  • sprite – textured rectangles with rotation (rich detail at street level)
 *
 * Smoothing: simple per-vehicle LERP applied every render frame.
 *  • LERP_STRAIGHT (0.4) on straight edges — gentle follow so timing jitter
 *    from the 60 Hz Rust→JS IPC doesn't cause frame-to-frame jumps.
 *  • LERP_ARC (0.9) on Bezier turn connectors — tight tracking of the
 *    analytically-exact Bezier position/angle from the Rust backend.
 *  • Instant snap on arc entry (t = 0) — eliminates any position lag from
 *    the straight-road LERP so the vehicle starts the arc from exactly P1.
 */
export class VehicleRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;
  private readonly camera: CameraManager;

  // ── Rectangle mode ─────────────────────────────────────────────────────────
  private readonly rectPools: Map<number, PIXI.Graphics[]> = new Map();
  private readonly activeRects: Map<number, PIXI.Graphics> = new Map();
  private readonly activeRectTypes: Map<number, number> = new Map();

  // ── Dot mode ────────────────────────────────────────────────────────────────
  private dotGraphics: PIXI.Graphics | null = null;

  // ── Frustration bubbles ──────────────────────────────────────────────────────
  private bubbleGraphics: PIXI.Graphics | null = null;

  // ── Raw state cache (direct Rust data, no smoothing) ─────────────────────────
  private readonly smoothState: Map<number, SmoothState> = new Map();
  private readonly prevOnConnector: Map<number, boolean> = new Map();

  // ── Road-group filtering ───────────────────────────────────────────────────
  private edgeIndex: EdgeLine[] = [];
  private readonly vehicleGroupCache: Map<number, string> = new Map();
  private readonly prevVehiclePos: Map<number, { lat: number; lng: number }> = new Map();
  private hiddenGroups: Set<string> = new Set();

  constructor(overlay: PixiOverlay, map: maplibregl.Map, camera: CameraManager) {
    this.overlay = overlay;
    this.map = map;
    this.camera = camera;
  }

  // ─── Road-group filtering API ──────────────────────────────────────────────

  setEdgeIndex(mapData: MapData): void {
    const nodeMap = new Map(mapData.nodes.map(n => [n.id, n]));
    this.edgeIndex = [];
    for (const edge of mapData.edges) {
      const a = nodeMap.get(edge.from);
      const b = nodeMap.get(edge.to);
      if (!a || !b) continue;
      this.edgeIndex.push({
        aLat: a.lat, aLng: a.lng,
        bLat: b.lat, bLng: b.lng,
        group: ROAD_TYPE_GROUP[edge.roadType] ?? 'residential',
      });
    }
    this.vehicleGroupCache.clear();
    this.prevVehiclePos.clear();
  }

  setHiddenGroups(groups: Set<string>): void {
    this.hiddenGroups = groups;
    this.vehicleGroupCache.clear();
  }

  // ─── Initialisation ────────────────────────────────────────────────────────

  async init(): Promise<void> {
    for (const typeId of VEHICLE_TYPES) {
      this.rectPools.set(typeId, []);
    }
    this.dotGraphics = new PIXI.Graphics();
    this.overlay.groundVehicles.addChild(this.dotGraphics);

    this.bubbleGraphics = new PIXI.Graphics();
    this.overlay.frustrationLayer.addChild(this.bubbleGraphics);
  }

  // ─── Frame update ──────────────────────────────────────────────────────────

  update(vehicles: Map<number, VehicleState>, infraMap: Map<number, string>): void {
    const { mode, spriteScale, dotRadius } = this.camera.getCarVisuals();

    if (mode === 'dot') {
      this.renderDots(vehicles, dotRadius);
      this.hideAllRects();
    } else {
      this.clearDots();
      this.renderSprites(vehicles, infraMap, spriteScale);
    }

    this.renderFrustrationBubbles(vehicles);
  }

  // ─── Raw passthrough (no smoothing) ──────────────────────────────────────

  /** Returns the raw Rust position and angle directly — zero smoothing. */
  private advanceSmooth(v: VehicleState): SmoothState {
    return { lat: v.lat, lng: v.lng, angle: v.angle };
  }

  // ─── Dot rendering ─────────────────────────────────────────────────────────

  private renderDots(vehicles: Map<number, VehicleState>, radius: number): void {
    const gfx = this.dotGraphics!;
    const bounds = this.map.getBounds();
    gfx.clear();

    for (const v of vehicles.values()) {
      if (this.isVehicleHidden(v)) continue;

      const s = this.advanceSmooth(v);

      if (
        s.lng < bounds.getWest() || s.lng > bounds.getEast() ||
        s.lat < bounds.getSouth() || s.lat > bounds.getNorth()
      ) continue;

      const typeId = v.vehicleType < VEHICLE_TYPES.length ? v.vehicleType : 0;
      const color  = frustrationDotColor(v.frustration, DOT_COLORS[typeId] ?? 0x4488ff);
      const px     = this.map.project([s.lng, s.lat]);

      // Right-hand lane offset applied in screen space.
      const offsetPx = this.camera.getLaneOffset() * (2 * v.lateralOffset + 1);
      const cx = px.x + Math.cos(s.angle) * offsetPx;
      const cy = px.y + Math.sin(s.angle) * offsetPx;

      gfx.circle(cx, cy, radius).fill({ color, alpha: 0.9 });
    }
  }

  private clearDots(): void {
    this.dotGraphics?.clear();
  }

  // ─── Frustration bubbles ───────────────────────────────────────────────────

  private renderFrustrationBubbles(vehicles: Map<number, VehicleState>): void {
    const gfx = this.bubbleGraphics!;
    const bounds = this.map.getBounds();
    gfx.clear();

    const now = performance.now();
    for (const v of vehicles.values()) {
      if (v.frustration < FRUSTRATION_CALM) continue;

      const s = this.smoothState.get(v.id) ?? { lat: v.lat, lng: v.lng };

      if (
        s.lng < bounds.getWest() || s.lng > bounds.getEast() ||
        s.lat < bounds.getSouth() || s.lat > bounds.getNorth()
      ) continue;

      let tier: number;
      if (v.frustration >= FRUSTRATION_RAGE)         tier = 3;
      else if (v.frustration >= FRUSTRATION_ANGRY)   tier = 2;
      else if (v.frustration >= FRUSTRATION_ANNOYED) tier = 1;
      else                                           tier = 0;

      const color = BUBBLE_COLORS[tier];
      const px    = this.map.project([s.lng, s.lat]);
      const cx    = px.x;
      const cy    = px.y - BUBBLE_OFFSET_Y;

      if (tier === 3 && Math.floor(now / 300) % 2 === 0) continue;

      const pulse  = 1 + 0.15 * Math.sin(now / (300 - tier * 60));
      const radius = (3 + tier) * pulse;

      gfx.circle(cx, cy, radius).fill({ color, alpha: 0.9 });
      gfx.circle(cx, cy, radius).stroke({ color: 0x000000, alpha: 0.3, width: 0.5 });
    }
  }

  // ─── Sprite rendering ──────────────────────────────────────────────────────

  private renderSprites(
    vehicles: Map<number, VehicleState>,
    infraMap: Map<number, string>,
    _spriteScale: number,
  ): void {
    const bounds      = this.map.getBounds();
    const laneWidthPx = this.camera.getLaneOffset() * 2;
    const renderedIds = new Set<number>();

    for (const [id, v] of vehicles) {
      if (this.isVehicleHidden(v)) continue;

      const s = this.advanceSmooth(v);

      if (
        s.lng < bounds.getWest() || s.lng > bounds.getEast() ||
        s.lat < bounds.getSouth() || s.lat > bounds.getNorth()
      ) continue;

      const typeId      = v.vehicleType < VEHICLE_TYPES.length ? v.vehicleType : 0;
      const dims        = VEHICLE_DIMS[typeId]        ?? VEHICLE_DIMS[0];
      const widthFill   = VEHICLE_WIDTH_FILL[typeId]  ?? VEHICLE_WIDTH_FILL[0];
      const lengthFactor= VEHICLE_LENGTH_FACTOR[typeId] ?? VEHICLE_LENGTH_FACTOR[0];
      const infraType   = infraMap.get(id) ?? 'normal';
      const isTunnel    = infraType === 'tunnel';
      const isBridge    = infraType === 'bridge';

      // Acquire or create rectangle graphic
      let rect = this.activeRects.get(id);
      if (!rect) {
        rect = this.acquireRect(typeId);
        this.activeRects.set(id, rect);
        this.activeRectTypes.set(id, typeId);
        if (isTunnel) {
          this.overlay.tunnelVehicles.addChild(rect);
        } else if (isBridge) {
          this.overlay.bridgeVehicles.addChild(rect);
        } else {
          this.overlay.groundVehicles.addChild(rect);
        }
      }

      const px = this.map.project([s.lng, s.lat]);

      // Right-hand lane offset — applied in screen space perpendicular to heading.
      // On a turn connector the Bezier axis is at road centre; we still apply the
      // lane offset so the vehicle stays in its lane visually throughout the turn.
      const laneOffset = this.camera.getLaneOffset() * (2 * v.lateralOffset + 1);
      const offsetX    = Math.cos(s.angle) * laneOffset;
      const offsetY    = Math.sin(s.angle) * laneOffset;

      rect.x        = px.x + offsetX;
      rect.y        = px.y + offsetY;
      rect.rotation = s.angle;

      const targetWidth  = Math.max(4, laneWidthPx * widthFill);
      const targetHeight = targetWidth * lengthFactor;
      rect.scale.set(targetWidth / dims.w, targetHeight / dims.h);
      rect.alpha   = isTunnel ? TUNNEL_ALPHA : 1;
      rect.visible = true;

      renderedIds.add(id);
    }

    // Return sprites for vehicles that left the viewport or were despawned
    for (const [id, rect] of this.activeRects) {
      if (!renderedIds.has(id)) {
        rect.visible = false;
        this.releaseRect(id, rect);
        this.activeRects.delete(id);
        this.activeRectTypes.delete(id);
        this.smoothState.delete(id);
        this.prevOnConnector.delete(id);
        this.vehicleGroupCache.delete(id);
        this.prevVehiclePos.delete(id);
      }
    }
  }

  private hideAllRects(): void {
    for (const rect of this.activeRects.values()) {
      rect.visible = false;
    }
  }

  // ─── Road-group lookup ─────────────────────────────────────────────────────

  private isVehicleHidden(v: VehicleState): boolean {
    if (this.hiddenGroups.size === 0 || this.edgeIndex.length === 0) return false;

    const prev  = this.prevVehiclePos.get(v.id);
    const moved = !prev ||
      Math.abs(v.lat - prev.lat) > RECHECK_THRESHOLD ||
      Math.abs(v.lng - prev.lng) > RECHECK_THRESHOLD;

    if (moved) {
      const group = this.resolveVehicleGroup(v.lat, v.lng);
      this.vehicleGroupCache.set(v.id, group);
      this.prevVehiclePos.set(v.id, { lat: v.lat, lng: v.lng });
    }

    const group = this.vehicleGroupCache.get(v.id) ?? 'residential';
    return this.hiddenGroups.has(group);
  }

  private resolveVehicleGroup(lat: number, lng: number): string {
    let bestDist2 = Infinity;
    let bestGroup = 'residential';
    for (const e of this.edgeIndex) {
      const d2 = ptSegDist2(lat, lng, e.aLat, e.aLng, e.bLat, e.bLng);
      if (d2 < bestDist2) { bestDist2 = d2; bestGroup = e.group; }
    }
    return bestGroup;
  }

  // ─── Sprite pool ───────────────────────────────────────────────────────────

  private acquireRect(typeId: number): PIXI.Graphics {
    const pool = this.rectPools.get(typeId);
    if (pool && pool.length > 0) {
      const rect = pool.pop()!;
      rect.visible = true;
      return rect;
    }
    const dims  = VEHICLE_DIMS[typeId] ?? VEHICLE_DIMS[0];
    const color = VEHICLE_COLORS[typeId] ?? 0xffffff;
    const rect  = new PIXI.Graphics();
    // Pivot at geometric centre: rect drawn symmetrically around (0,0).
    // sprite.anchor equivalent for Graphics — rotation is around origin = centre.
    rect
      .rect(-dims.w / 2, -dims.h / 2, dims.w, dims.h)
      .fill({ color, alpha: 1.0 });
    return rect;
  }

  private releaseRect(vehicleId: number, rect: PIXI.Graphics): void {
    rect.parent?.removeChild(rect);
    const typeId = this.activeRectTypes.get(vehicleId) ?? 0;
    const pool   = this.rectPools.get(typeId);
    if (pool) pool.push(rect);
    else rect.destroy();
  }

  // ─── Cleanup ───────────────────────────────────────────────────────────────

  destroy(): void {
    this.dotGraphics?.destroy();
    this.bubbleGraphics?.destroy();
    for (const rects of this.rectPools.values()) {
      for (const r of rects) r.destroy();
    }
    for (const r of this.activeRects.values()) r.destroy();
    this.rectPools.clear();
    this.activeRects.clear();
    this.activeRectTypes.clear();
    this.smoothState.clear();
    this.prevOnConnector.clear();
  }
}
