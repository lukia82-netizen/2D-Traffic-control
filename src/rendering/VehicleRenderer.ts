import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { VehicleState } from '../bridge/events';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';

// ─── Vehicle visual constants ─────────────────────────────────────────────────

const VEHICLE_COLORS: Record<number, number> = {
  0: 0x4488ff, // Car   – blue
  1: 0xffcc44, // Van   – yellow
  2: 0xff8c00, // Bus   – orange
  3: 0x8b2500, // Truck – dark red
};

const DOT_COLORS: Record<number, number> = {
  0: 0x66aaff, // Car   – lighter for dot visibility
  1: 0xffdd66,
  2: 0xffaa33,
  3: 0xcc4400,
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
};

const TUNNEL_ALPHA = 0.25;
const VEHICLE_TYPES = [0, 1, 2, 3];

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
  }

  // ─── Dot rendering ─────────────────────────────────────────────────────────

  private renderDots(vehicles: Map<number, VehicleState>, radius: number): void {
    const gfx = this.dotGraphics!;
    const bounds = this.map.getBounds();

    gfx.clear();

    for (const v of vehicles.values()) {
      // Frustum cull
      if (
        v.lng < bounds.getWest() || v.lng > bounds.getEast() ||
        v.lat < bounds.getSouth() || v.lat > bounds.getNorth()
      ) continue;

      const typeId = v.vehicleType < VEHICLE_TYPES.length ? v.vehicleType : 0;
      const color = DOT_COLORS[typeId] ?? 0x4488ff;
      const px = this.map.project([v.lng, v.lat]);

      // Right-hand traffic offset (small at dot scale, but keeps roads clean)
      const offsetPx = Math.max(1, radius * 0.8);
      const cx = px.x + (-Math.sin(v.angle)) * offsetPx;
      const cy = px.y + ( Math.cos(v.angle)) * offsetPx;

      gfx.circle(cx, cy, radius).fill({ color, alpha: 0.9 });
    }
  }

  private clearDots(): void {
    this.dotGraphics?.clear();
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
      // Frustum cull
      if (
        v.lng < bounds.getWest() || v.lng > bounds.getEast() ||
        v.lat < bounds.getSouth() || v.lat > bounds.getNorth()
      ) continue;

      const typeId = v.vehicleType < VEHICLE_TYPES.length ? v.vehicleType : 0;
      const dims = VEHICLE_DIMS[typeId] ?? VEHICLE_DIMS[0];
      const infraType = infraMap.get(id) ?? 'normal';
      const isTunnel = infraType === 'tunnel';
      const isBridge = infraType === 'bridge';

      const px = this.map.project([v.lng, v.lat]);

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

      // Right-hand traffic: offset vehicle to the right of its heading direction
      // so vehicles on opposite sides of the same road don't overlap.
      const laneOffset = Math.max(2, dims.w * spriteScale * 0.6);
      const offsetX = -Math.sin(v.angle) * laneOffset;
      const offsetY =  Math.cos(v.angle) * laneOffset;

      sprite.x = px.x + offsetX;
      sprite.y = px.y + offsetY;
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
      }
    }
  }

  private hideAllSprites(): void {
    for (const sprite of this.activeSprites.values()) {
      sprite.visible = false;
    }
    // Don't return to pool – they'll be reused when zooming back in
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
    for (const sprites of this.spritePools.values()) {
      for (const s of sprites) s.destroy();
    }
    for (const s of this.activeSprites.values()) s.destroy();
    for (const t of this.textures.values()) t.destroy(true);
    this.spritePools.clear();
    this.activeSprites.clear();
    this.textures.clear();
  }
}
