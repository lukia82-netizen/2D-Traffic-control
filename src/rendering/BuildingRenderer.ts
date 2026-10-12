import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { projectPoint } from '../map/MapLibreSetup';

// Building colours (sketch aesthetic on dark background)
const BUILDING_FILL    = 0x2d2d4e; // dark blue-gray fill
const BUILDING_STROKE  = 0x4a4a70; // slightly lighter outline
const BUILDING_ALPHA   = 0.92;

// ─── BuildingRenderer ─────────────────────────────────────────────────────────

/**
 * Draws building footprints fetched from OSM as filled PixiJS polygons.
 * Rebuilds every camera move so polygons stay aligned with the projection.
 */
interface BuildingAABB {
  minLat: number;
  maxLat: number;
  minLng: number;
  maxLng: number;
}

export class BuildingRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  private gfx: PIXI.Graphics | null = null;
  /** Caches pre-calculated AABB min/max coordinates for building polygons. */
  private readonly boundsCache = new WeakMap<object, BuildingAABB>();

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

    // Extract map viewport bounds with 20% padding margin for AABB viewport culling.
    // Skips coordinate projection and PixiJS graphics draws for off-screen buildings.
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();

    const latSpan = north - south;
    const marginRatio = 0.2; // 20% padding

    const minLat = south - latSpan * marginRatio;
    const maxLat = north + latSpan * marginRatio;

    // Handle antimeridian crossing (west > east) safely by skipping longitude culling
    const isAntimeridian = west > east;
    let minLng = -Infinity;
    let maxLng = Infinity;

    if (!isAntimeridian) {
      const lngSpan = east - west;
      minLng = west - lngSpan * marginRatio;
      maxLng = east + lngSpan * marginRatio;
    }

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      let aabb = this.boundsCache.get(building);
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
        this.boundsCache.set(building, aabb);
      }

      // Skip off-screen buildings outside padded viewport bounds
      if (
        aabb.maxLat < minLat ||
        aabb.minLat > maxLat ||
        aabb.maxLng < minLng ||
        aabb.minLng > maxLng
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
  }
}
