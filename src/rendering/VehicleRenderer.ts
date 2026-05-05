import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { VehicleState } from '../bridge/events';
import type { PixiOverlay } from './PixiOverlay';

// ─── Vehicle visual constants ─────────────────────────────────────────────────

const VEHICLE_COLORS: Record<number, number> = {
  0: 0x4488ff, // Car   – blue
  1: 0xffcc44, // Van   – yellow
  2: 0xff8c00, // Bus   – orange
  3: 0x8b2500, // Truck – dark red
};

/** Pixel dimensions at zoom 16; scaled in update() relative to current zoom. */
const VEHICLE_DIMS: Record<number, { w: number; h: number }> = {
  0: { w: 6, h: 12 },  // Car
  1: { w: 7, h: 16 },  // Van
  2: { w: 9, h: 32 },  // Bus
  3: { w: 10, h: 48 }, // Truck
};

const REFERENCE_ZOOM = 16;
const TUNNEL_ALPHA = 0.25;
const VEHICLE_TYPES = [0, 1, 2, 3];

// infra type strings as returned by Rust
const INFRA_TUNNEL = 'tunnel';
const INFRA_BRIDGE = 'bridge';

// ─── VehicleRenderer ─────────────────────────────────────────────────────────

export class VehicleRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  /** Sprite pool per vehicle type: [typeId] → free sprites */
  private readonly spritePools: Map<number, PIXI.Sprite[]> = new Map();
  /** Pre-rendered textures per vehicle type */
  private readonly textures: Map<number, PIXI.Texture> = new Map();
  /** Active sprites keyed by vehicle id */
  private readonly activeSprites: Map<number, PIXI.Sprite> = new Map();

  constructor(overlay: PixiOverlay, map: maplibregl.Map) {
    this.overlay = overlay;
    this.map = map;
  }

  async init(): Promise<void> {
    for (const typeId of VEHICLE_TYPES) {
      const dims = VEHICLE_DIMS[typeId] ?? VEHICLE_DIMS[0];
      const color = VEHICLE_COLORS[typeId] ?? 0xffffff;

      // Draw a small rounded rectangle per type into a RenderTexture
      const gfx = new PIXI.Graphics();
      gfx
        .roundRect(0, 0, dims.w, dims.h, 2)
        .fill({ color, alpha: 1 });

      const rt = PIXI.RenderTexture.create({
        width: dims.w,
        height: dims.h,
      });
      this.overlay.app.renderer.render({ container: gfx, target: rt });
      gfx.destroy();

      this.textures.set(typeId, rt);
      this.spritePools.set(typeId, []);
    }
  }

  /**
   * Called every frame.  Projects vehicles, manages sprite pool, updates
   * positions/rotations.
   */
  update(vehicles: Map<number, VehicleState>, infraMap: Map<number, string>): void {
    const bounds = this.map.getBounds();
    const zoomScale = Math.pow(2, this.map.getZoom() - REFERENCE_ZOOM);

    // Track which vehicle ids we rendered this frame
    const renderedIds = new Set<number>();

    for (const [id, v] of vehicles) {
      // Frustum cull
      if (
        v.lng < bounds.getWest() ||
        v.lng > bounds.getEast() ||
        v.lat < bounds.getSouth() ||
        v.lat > bounds.getNorth()
      ) {
        continue;
      }

      const typeId = v.vehicleType < VEHICLE_TYPES.length ? v.vehicleType : 0;
      const dims = VEHICLE_DIMS[typeId] ?? VEHICLE_DIMS[0];
      const infraType = infraMap.get(id) ?? 'normal';
      const isTunnel = infraType === INFRA_TUNNEL;
      const isBridge = infraType === INFRA_BRIDGE;

      // Project position
      const px = this.map.project([v.lng, v.lat]);

      // Acquire sprite from pool or create new
      let sprite = this.activeSprites.get(id);
      if (!sprite) {
        sprite = this.acquireSprite(typeId);
        this.activeSprites.set(id, sprite);

        // Add to correct layer
        if (isTunnel) {
          this.overlay.tunnelVehicles.addChild(sprite);
        } else if (isBridge) {
          this.overlay.bridgeVehicles.addChild(sprite);
        } else {
          this.overlay.groundVehicles.addChild(sprite);
        }
      }

      const scaledW = dims.w * zoomScale;
      const scaledH = dims.h * zoomScale;

      sprite.x = px.x;
      sprite.y = px.y;
      sprite.rotation = v.angle;
      sprite.width = scaledW;
      sprite.height = scaledH;
      sprite.anchor.set(0.5, 0.5);
      sprite.alpha = isTunnel ? TUNNEL_ALPHA : 1;
      sprite.visible = true;

      renderedIds.add(id);
    }

    // Return sprites for vehicles that left the viewport or were removed
    for (const [id, sprite] of this.activeSprites) {
      if (!renderedIds.has(id)) {
        sprite.visible = false;
        this.releaseSprite(id, sprite);
        this.activeSprites.delete(id);
      }
    }
  }

  // ─── Sprite pool helpers ────────────────────────────────────────────────────

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
    // Determine type from texture to return to the right pool
    for (const [typeId, tex] of this.textures) {
      if (sprite.texture === tex) {
        sprite.parent?.removeChild(sprite);
        const pool = this.spritePools.get(typeId);
        if (pool) pool.push(sprite);
        return;
      }
    }
    // Unknown type, just destroy
    sprite.destroy();
    void vehicleId; // suppress lint
  }

  destroy(): void {
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
