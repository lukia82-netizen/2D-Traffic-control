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

interface BuildingBounds {
  minLat: number;
  maxLat: number;
  minLng: number;
  maxLng: number;
}

/**
 * Draws building footprints fetched from OSM as filled PixiJS polygons.
 * Rebuilds every camera move so polygons stay aligned with the projection.
 */
export class BuildingRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  private gfx: PIXI.Graphics | null = null;
  /** Performance optimization: Cache AABB bounds per building polygon to allow O(1) viewport culling. */
  private buildingBoundsCache = new WeakMap<object, BuildingBounds>();

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

    // Extract map viewport bounds and add a 50% margin to prevent visual popping during fast panning/zooming.
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();

    const lngMargin = Math.max(0.002, (east - west) * 0.5);
    const latMargin = Math.max(0.002, (north - south) * 0.5);

    const minLng = west - lngMargin;
    const maxLng = east + lngMargin;
    const minLat = south - latMargin;
    const maxLat = north + latMargin;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      // Check viewport culling using cached AABB bounds
      let bBounds = this.buildingBoundsCache.get(building);
      if (!bBounds) {
        let bMinLat = Infinity, bMaxLat = -Infinity;
        let bMinLng = Infinity, bMaxLng = -Infinity;
        for (const [lat, lng] of building.polygon) {
          if (lat < bMinLat) bMinLat = lat;
          if (lat > bMaxLat) bMaxLat = lat;
          if (lng < bMinLng) bMinLng = lng;
          if (lng > bMaxLng) bMaxLng = lng;
        }
        bBounds = { minLat: bMinLat, maxLat: bMaxLat, minLng: bMinLng, maxLng: bMaxLng };
        this.buildingBoundsCache.set(building, bBounds);
      }

      // Skip buildings completely outside the extended viewport bounds to avoid redundant projection calculations
      if (
        bBounds.maxLat < minLat ||
        bBounds.minLat > maxLat ||
        bBounds.maxLng < minLng ||
        bBounds.minLng > maxLng
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
