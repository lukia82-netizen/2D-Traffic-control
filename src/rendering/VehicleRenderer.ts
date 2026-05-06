import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { VehicleState } from '../bridge/events';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';

// ─── Lerp smoothing factor (0–1 per frame) ───────────────────────────────────
// 0.45 per frame at 60 fps ≈ ~99% of remaining distance in 5 frames.
// Keeps motion fluid when Tauri IPC delivers frames in bursts.
const GEO_LERP = 0.45;

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
  2: { w: 9,  h: 32 }, // Bus
  3: { w: 10, h: 48 }, // Truck
  4: { w: 8,  h: 56 }, // Tram  (20 m body)
};

/** Frustration thresholds for visual bubbles. */
const FRUSTRATION_CALM     = 40;
const FRUSTRATION_ANNOYED  = 65;
const FRUSTRATION_ANGRY    = 85;
const FRUSTRATION_RAGE     = 99;

/** Bubble labels per threshold (annoyed, angry, raging, rage-quit) */
export const BUBBLE_LABELS = ['!', '!!', '!!!', '💢'] as const;
/** Bubble colors per tier: annoyed, angry, raging, rage-quit */
const BUBBLE_COLORS = [0xffdd00, 0xff8800, 0xff3300, 0xff0000] as const;
/** Bubble vertical offset above the sprite (px) */
const BUBBLE_OFFSET_Y = 12;

const TUNNEL_ALPHA = 0.25;
const VEHICLE_TYPES = [0, 1, 2, 3, 4];

// ─── Helpers ─────────────────────────────────────────────────────────────────

/**
 * Choose a dot colour based on frustration level.
 * 0–40  → normal type colour
 * 40–65 → yellow
 * 65–85 → orange
 * 85–100→ red
 */
function frustrationDotColor(frustration: number, typeColor: number): number {
  if (frustration >= FRUSTRATION_ANGRY)  return 0xff3300;
  if (frustration >= FRUSTRATION_ANNOYED) return 0xff8800;
  if (frustration >= FRUSTRATION_CALM)   return 0xffdd00;
  return typeColor;
}

// ─── VehicleRenderer ─────────────────────────────────────────────────────────

/**
 * Renders moving vehicles on top of the MapLibre map via PixiJS.
 *
 * Two rendering modes managed by CameraManager:
 *  • dot    – single batched Graphics call per frame (ultra-fast for 3000+ cars)
 *  • sprite – textured rectangles with rotation (rich detail at street level)
 *
 * The mode switches automatically as the player zooms in/out.
 */
export class VehicleRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;
  private readonly camera: CameraManager;

  // ── Sprite mode ─────────────────────────────────────────────────────────────
  /** Pre-rendered textures per vehicle type */
  private readonly textures: Map<number, PIXI.Texture> = new Map();
  /** Pooled inactive sprites per vehicle type */
  private readonly spritePools: Map<number, PIXI.Sprite[]> = new Map();
  /** Currently active sprites keyed by vehicle id */
  private readonly activeSprites: Map<number, PIXI.Sprite> = new Map();

  // ── Dot mode ────────────────────────────────────────────────────────────────
  /** Single Graphics object reused every frame to batch-draw all dots */
  private dotGraphics: PIXI.Graphics | null = null;

  // ── Frustration bubbles ──────────────────────────────────────────────────────
  /** Single Graphics object reused every frame to draw frustration bubbles */
  private bubbleGraphics: PIXI.Graphics | null = null;

  // ── Lerp smoothing (geo space) ───────────────────────────────────────────────
  /** Smoothed geographic positions to eliminate flicker from IPC burst delivery */
  private readonly geoSmoothed: Map<number, { lat: number; lng: number }> = new Map();

  constructor(overlay: PixiOverlay, map: maplibregl.Map, camera: CameraManager) {
    this.overlay = overlay;
    this.map = map;
    this.camera = camera;
  }

  // ─── Initialisation ────────────────────────────────────────────────────────

  async init(): Promise<void> {
    // Build sprite textures for each vehicle type (small RenderTextures)
    for (const typeId of VEHICLE_TYPES) {
      const dims = VEHICLE_DIMS[typeId] ?? VEHICLE_DIMS[0];
      const color = VEHICLE_COLORS[typeId] ?? 0xffffff;

      const gfx = new PIXI.Graphics();
      // Rounded rect body
      gfx.roundRect(0, 0, dims.w, dims.h, 2).fill({ color, alpha: 1 });
      // Dark outline for contrast on any background
      gfx.roundRect(0, 0, dims.w, dims.h, 2)
        .stroke({ color: 0x000000, alpha: 0.5, width: 1 });
      // Windshield highlight (top 25% of body)
      gfx.roundRect(1, 1, dims.w - 2, dims.h * 0.25, 1)
        .fill({ color: 0xffffff, alpha: 0.25 });

      const rt = PIXI.RenderTexture.create({ width: dims.w, height: dims.h });
      this.overlay.app.renderer.render({ container: gfx, target: rt });
      gfx.destroy();

      this.textures.set(typeId, rt);
      this.spritePools.set(typeId, []);
    }

    // Dot graphics object lives in groundVehicles layer, below sprites
    this.dotGraphics = new PIXI.Graphics();
    this.overlay.groundVehicles.addChild(this.dotGraphics);

    // Bubble graphics lives in the dedicated frustration layer (above vehicles)
    this.bubbleGraphics = new PIXI.Graphics();
    this.overlay.frustrationLayer.addChild(this.bubbleGraphics);
  }

  // ─── Frame update ──────────────────────────────────────────────────────────

  /**
   * Called every PixiJS ticker frame.
   * Projects vehicle positions and renders them in the appropriate mode.
   */
  update(vehicles: Map<number, VehicleState>, infraMap: Map<number, string>): void {
    const { mode, spriteScale, dotRadius } = this.camera.getCarVisuals();

    if (mode === 'dot') {
      this.renderDots(vehicles, dotRadius);
      this.hideAllSprites();
    } else {
      this.clearDots();
      this.renderSprites(vehicles, infraMap, spriteScale);
    }

    // Draw frustration bubbles regardless of vehicle render mode
    this.renderFrustrationBubbles(vehicles);
  }

  // ─── Dot rendering ─────────────────────────────────────────────────────────

  private renderDots(vehicles: Map<number, VehicleState>, radius: number): void {
    const gfx = this.dotGraphics!;
    const bounds = this.map.getBounds();

    gfx.clear();

    for (const v of vehicles.values()) {
      const smooth = this.smoothGeo(v);

      // Frustum cull
      if (
        smooth.lng < bounds.getWest() || smooth.lng > bounds.getEast() ||
        smooth.lat < bounds.getSouth() || smooth.lat > bounds.getNorth()
      ) continue;

      const typeId = v.vehicleType < VEHICLE_TYPES.length ? v.vehicleType : 0;
      // Frustration tinting: calm→type color, annoyed→yellow, angry→orange, rage→red
      const color = frustrationDotColor(v.frustration, DOT_COLORS[typeId] ?? 0x4488ff);
      const px = this.map.project([smooth.lng, smooth.lat]);

      const offsetPx = this.camera.getLaneOffset();
      const cx = px.x + Math.cos(v.angle) * offsetPx;
      const cy = px.y + Math.sin(v.angle) * offsetPx;

      gfx.circle(cx, cy, radius).fill({ color, alpha: 0.9 });
    }
  }

  private clearDots(): void {
    this.dotGraphics?.clear();
  }

  // ─── Frustration bubbles ───────────────────────────────────────────────────

  /**
   * Draw small indicator bubbles above frustrated vehicles.
   * Tier system:
   *   40–65  → yellow  "!"
   *   65–85  → orange  "!!"
   *   85–99  → red     "!!!"
   *   ≥99    → red     "💢" + fast flicker
   *
   * Uses a single batched Graphics object so all bubbles cost ~1 draw call.
   */
  private renderFrustrationBubbles(vehicles: Map<number, VehicleState>): void {
    const gfx = this.bubbleGraphics!;
    const bounds = this.map.getBounds();
    gfx.clear();

    const now = performance.now();
    for (const v of vehicles.values()) {
      if (v.frustration < FRUSTRATION_CALM) continue;

      const smooth = this.geoSmoothed.get(v.id) ?? { lat: v.lat, lng: v.lng };

      // Frustum cull
      if (
        smooth.lng < bounds.getWest() || smooth.lng > bounds.getEast() ||
        smooth.lat < bounds.getSouth() || smooth.lat > bounds.getNorth()
      ) continue;

      // Pick tier
      let tier: number;
      if (v.frustration >= FRUSTRATION_RAGE)   tier = 3;
      else if (v.frustration >= FRUSTRATION_ANGRY)   tier = 2;
      else if (v.frustration >= FRUSTRATION_ANNOYED) tier = 1;
      else                                           tier = 0;

      const color = BUBBLE_COLORS[tier];
      const px = this.map.project([smooth.lng, smooth.lat]);
      const cx = px.x;
      const cy = px.y - BUBBLE_OFFSET_Y;

      // Flicker for rage tier
      if (tier === 3 && Math.floor(now / 300) % 2 === 0) continue;

      // Pulsing scale (simple sin wave per tier)
      const pulse = 1 + 0.15 * Math.sin(now / (300 - tier * 60));
      const radius = (3 + tier) * pulse;

      // Draw circle indicator
      gfx.circle(cx, cy, radius).fill({ color, alpha: 0.9 });
      // Tiny dot outline for visibility on light backgrounds
      gfx.circle(cx, cy, radius).stroke({ color: 0x000000, alpha: 0.3, width: 0.5 });
    }
  }

  // ─── Sprite rendering ──────────────────────────────────────────────────────

  private renderSprites(
    vehicles: Map<number, VehicleState>,
    infraMap: Map<number, string>,
    spriteScale: number,
  ): void {
    const bounds = this.map.getBounds();
    const renderedIds = new Set<number>();

    for (const [id, v] of vehicles) {
      const smooth = this.smoothGeo(v);

      // Frustum cull
      if (
        smooth.lng < bounds.getWest() || smooth.lng > bounds.getEast() ||
        smooth.lat < bounds.getSouth() || smooth.lat > bounds.getNorth()
      ) continue;

      const typeId = v.vehicleType < VEHICLE_TYPES.length ? v.vehicleType : 0;
      const dims = VEHICLE_DIMS[typeId] ?? VEHICLE_DIMS[0];
      const infraType = infraMap.get(id) ?? 'normal';
      const isTunnel = infraType === 'tunnel';
      const isBridge = infraType === 'bridge';

      const px = this.map.project([smooth.lng, smooth.lat]);

      // Acquire or create sprite
      let sprite = this.activeSprites.get(id);
      if (!sprite) {
        sprite = this.acquireSprite(typeId);
        this.activeSprites.set(id, sprite);
        if (isTunnel) {
          this.overlay.tunnelVehicles.addChild(sprite);
        } else if (isBridge) {
          this.overlay.bridgeVehicles.addChild(sprite);
        } else {
          this.overlay.groundVehicles.addChild(sprite);
        }
      }

      // Right-hand traffic lane offset.
      // Rust angle: atan2(dlng, dlat), so angle=0→North, π/2→East.
      // Screen heading vector = (sin θ, −cos θ).
      // Perpendicular 90° clockwise (right lane) = (cos θ, sin θ).
      const laneOffset = this.camera.getLaneOffset();
      const offsetX = Math.cos(v.angle) * laneOffset;
      const offsetY = Math.sin(v.angle) * laneOffset;

      sprite.x = px.x + offsetX;
      sprite.y = px.y + offsetY;
      // Rotation: angle=0 → sprite front (y=0 of texture) points North (up).
      // PixiJS clockwise rotation=π/2 → front points East. Matches Rust convention.
      sprite.rotation = v.angle;
      sprite.width  = dims.w * spriteScale;
      sprite.height = dims.h * spriteScale;
      sprite.anchor.set(0.5, 0.5);
      sprite.alpha = isTunnel ? TUNNEL_ALPHA : 1;
      sprite.visible = true;

      renderedIds.add(id);
    }

    // Return sprites for vehicles that left viewport or were removed
    for (const [id, sprite] of this.activeSprites) {
      if (!renderedIds.has(id)) {
        sprite.visible = false;
        this.releaseSprite(id, sprite);
        this.activeSprites.delete(id);
        this.geoSmoothed.delete(id);
      }
    }
  }

  private hideAllSprites(): void {
    for (const sprite of this.activeSprites.values()) {
      sprite.visible = false;
    }
    // Don't return to pool – they'll be reused when zooming back in
  }

  // ─── Geo smoothing (lerp) ──────────────────────────────────────────────────

  /**
   * Returns a smoothed lat/lng for vehicle `v`, lerping toward the latest
   * Rust position.  Eliminates visible jumps from IPC burst delivery and
   * edge-to-edge transitions.  Because smoothing is in geographic space,
   * camera pan/zoom never introduces lag.
   */
  private smoothGeo(v: VehicleState): { lat: number; lng: number } {
    const prev = this.geoSmoothed.get(v.id);
    if (!prev) {
      const pos = { lat: v.lat, lng: v.lng };
      this.geoSmoothed.set(v.id, pos);
      return pos;
    }
    const lat = prev.lat + (v.lat - prev.lat) * GEO_LERP;
    const lng = prev.lng + (v.lng - prev.lng) * GEO_LERP;
    prev.lat = lat;
    prev.lng = lng;
    return prev;
  }

  // ─── Sprite pool ───────────────────────────────────────────────────────────

  private acquireSprite(typeId: number): PIXI.Sprite {
    const pool = this.spritePools.get(typeId);
    if (pool && pool.length > 0) {
      const sprite = pool.pop()!;
      sprite.visible = true;
      return sprite;
    }
    const texture = this.textures.get(typeId) ?? PIXI.Texture.EMPTY;
    return new PIXI.Sprite(texture);
  }

  private releaseSprite(vehicleId: number, sprite: PIXI.Sprite): void {
    for (const [typeId, tex] of this.textures) {
      if (sprite.texture === tex) {
        sprite.parent?.removeChild(sprite);
        const pool = this.spritePools.get(typeId);
        if (pool) pool.push(sprite);
        return;
      }
    }
    sprite.destroy();
    void vehicleId;
  }

  // ─── Cleanup ───────────────────────────────────────────────────────────────

  destroy(): void {
    this.dotGraphics?.destroy();
    this.bubbleGraphics?.destroy();
    for (const sprites of this.spritePools.values()) {
      for (const s of sprites) s.destroy();
    }
    for (const s of this.activeSprites.values()) s.destroy();
    for (const t of this.textures.values()) t.destroy(true);
    this.spritePools.clear();
    this.activeSprites.clear();
    this.textures.clear();
    this.geoSmoothed.clear();
  }
}
