import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, BuildingData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { projectPoint } from '../map/MapLibreSetup';

// Building colours (sketch aesthetic on dark background)
const BUILDING_FILL    = 0x2d2d4e; // dark blue-gray fill
const BUILDING_STROKE  = 0x4a4a70; // slightly lighter outline
const BUILDING_ALPHA   = 0.92;

interface AABB {
  minLat: number;
  maxLat: number;
  minLng: number;
  maxLng: number;
}

// ─── BuildingRenderer ─────────────────────────────────────────────────────────

/**
 * Draws building footprints fetched from OSM as filled PixiJS polygons.
 * Rebuilds every camera move so polygons stay aligned with the projection.
 */
export class BuildingRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  private gfx: PIXI.Graphics | null = null;
  /** Cache pre-calculated geographic bounding boxes per building polygon to avoid recomputation on camera movement */
  private readonly boundsCache = new WeakMap<BuildingData, AABB>();

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

  // ─── Helpers ───────────────────────────────────────────────────────────────

  /**
   * Precompute and cache the geographic bounding box (AABB) of a building polygon.
   * building.polygon contains [lat, lng] vertices.
   */
  private getBuildingBounds(building: BuildingData): AABB {
    let cached = this.boundsCache.get(building);
    if (!cached) {
      let minLat = Infinity, maxLat = -Infinity;
      let minLng = Infinity, maxLng = -Infinity;

      for (const [lat, lng] of building.polygon) {
        if (lat < minLat) minLat = lat;
        if (lat > maxLat) maxLat = lat;
        if (lng < minLng) minLng = lng;
        if (lng > maxLng) maxLng = lng;
      }

      cached = { minLat, maxLat, minLng, maxLng };
      this.boundsCache.set(building, cached);
    }
    return cached;
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

    // Viewport geographic bounds culling with 20% margin padding to prevent pop-in during panning
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();

    const lngPadding = Math.abs(east - west) * 0.2;
    const latPadding = Math.abs(north - south) * 0.2;

    const minViewLng = west - lngPadding;
    const maxViewLng = east + lngPadding;
    const minViewLat = south - latPadding;
    const maxViewLat = north + latPadding;
    const crossesAntimeridian = west > east;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      // Skip off-screen buildings prior to coordinate projection and PixiJS graphics creation
      const bBounds = this.getBuildingBounds(building);
      if (!crossesAntimeridian) {
        if (
          bBounds.maxLat < minViewLat ||
          bBounds.minLat > maxViewLat ||
          bBounds.maxLng < minViewLng ||
          bBounds.minLng > maxViewLng
        ) {
          continue;
        }
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
