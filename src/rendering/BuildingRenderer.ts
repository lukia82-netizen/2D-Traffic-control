import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { projectPoint } from '../map/MapLibreSetup';

// Building colours (sketch aesthetic on dark background)
const BUILDING_FILL    = 0x2d2d4e; // dark blue-gray fill
const BUILDING_STROKE  = 0x4a4a70; // slightly lighter outline
const BUILDING_ALPHA   = 0.92;

// ─── Internal types ───────────────────────────────────────────────────────────

interface BuildingAABB {
  minLat: number;
  maxLat: number;
  minLng: number;
  maxLng: number;
}

// ─── BuildingRenderer ─────────────────────────────────────────────────────────

/**
 * Draws building footprints fetched from OSM as filled PixiJS polygons.
 * Rebuilds every camera move so polygons stay aligned with the projection.
 *
 * Performance optimization:
 * Calculates and caches AABB (min/max lat/lng) for each building polygon and performs
 * viewport bounds culling with 50% padding to skip projection and rendering of off-screen buildings.
 */
export class BuildingRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  private gfx: PIXI.Graphics | null = null;

  /** Cached AABB bounds per building ID to avoid recomputing min/max lat/lng on camera updates. */
  private readonly buildingBounds: Map<number, BuildingAABB> = new Map();

  constructor(overlay: PixiOverlay, map: maplibregl.Map) {
    this.overlay = overlay;
    this.map = map;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  build(mapData: MapData): void {
    this.rebuild(mapData);
  }

  rebuildOnCameraChange(mapData: MapData): void {
    this.rebuild(mapData);
  }

  // ─── Rendering ─────────────────────────────────────────────────────────────

  private rebuild(mapData: MapData): void {
    if (!this.gfx) {
      this.gfx = new PIXI.Graphics();
      this.overlay.buildings.addChild(this.gfx);
    }

    const gfx = this.gfx;
    gfx.clear();

    if (!mapData.buildings || mapData.buildings.length === 0) return;

    // Viewport bounds culling: hoist map bounds and apply 50% padding margin to avoid popping
    const mapBounds = this.map.getBounds();
    const west = mapBounds.getWest();
    const east = mapBounds.getEast();
    const south = mapBounds.getSouth();
    const north = mapBounds.getNorth();

    const lngMargin = (east - west) * 0.5;
    const latMargin = (north - south) * 0.5;
    const minLng = west - lngMargin;
    const maxLng = east + lngMargin;
    const minLat = south - latMargin;
    const maxLat = north + latMargin;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      // Get or compute cached building AABB
      let aabb = this.buildingBounds.get(building.id);
      if (!aabb) {
        let bMinLat = Infinity;
        let bMaxLat = -Infinity;
        let bMinLng = Infinity;
        let bMaxLng = -Infinity;

        for (const [lat, lng] of building.polygon) {
          if (lat < bMinLat) bMinLat = lat;
          if (lat > bMaxLat) bMaxLat = lat;
          if (lng < bMinLng) bMinLng = lng;
          if (lng > bMaxLng) bMaxLng = lng;
        }

        aabb = { minLat: bMinLat, maxLat: bMaxLat, minLng: bMinLng, maxLng: bMaxLng };
        this.buildingBounds.set(building.id, aabb);
      }

      // Cull off-screen buildings whose AABB does not overlap padded viewport
      if (
        aabb.maxLng < minLng ||
        aabb.minLng > maxLng ||
        aabb.maxLat < minLat ||
        aabb.minLat > maxLat
      ) {
        continue;
      }

      // Project each vertex [lat, lng] → screen {x, y}
      const pts: number[] = [];
      for (const [lat, lng] of building.polygon) {
        const { x, y } = projectPoint(this.map, lng, lat);
        pts.push(x, y);
      }

      gfx.poly(pts).fill({ color: BUILDING_FILL, alpha: BUILDING_ALPHA });
      gfx.poly(pts).stroke({ color: BUILDING_STROKE, width: 0.8, alpha: 0.8 });
    }
  }

  // ─── Cleanup ───────────────────────────────────────────────────────────────

  destroy(): void {
    this.gfx?.destroy();
    this.buildingBounds.clear();
  }
}
