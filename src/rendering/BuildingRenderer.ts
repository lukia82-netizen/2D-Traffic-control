import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, BuildingData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { projectPoint } from '../map/MapLibreSetup';

// Building colours (sketch aesthetic on dark background)
const BUILDING_FILL    = 0x2d2d4e; // dark blue-gray fill
const BUILDING_STROKE  = 0x4a4a70; // slightly lighter outline
const BUILDING_ALPHA   = 0.92;

interface BoundingBox {
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

  /**
   * Performance optimization: Caches pre-calculated AABB bounding boxes for building
   * polygons using WeakMap to avoid recalculating min/max bounds on each camera move.
   */
  private readonly buildingBounds = new WeakMap<BuildingData, BoundingBox>();

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

  // ─── Internal Helpers ───────────────────────────────────────────────────────

  /**
   * Lazily calculates or retrieves cached bounding box for a building polygon.
   */
  private getBuildingBounds(building: BuildingData): BoundingBox {
    let bounds = this.buildingBounds.get(building);
    if (!bounds) {
      let minLat = Infinity;
      let maxLat = -Infinity;
      let minLng = Infinity;
      let maxLng = -Infinity;

      for (const [lat, lng] of building.polygon) {
        if (lat < minLat) minLat = lat;
        if (lat > maxLat) maxLat = lat;
        if (lng < minLng) minLng = lng;
        if (lng > maxLng) maxLng = lng;
      }

      bounds = { minLat, maxLat, minLng, maxLng };
      this.buildingBounds.set(building, bounds);
    }
    return bounds;
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

    // Viewport bounds culling with 20% padding margin to avoid projecting off-screen buildings
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();

    const lngPadding = (east - west) * 0.2;
    const latPadding = (north - south) * 0.2;

    const paddedWest = west - lngPadding;
    const paddedEast = east + lngPadding;
    const paddedSouth = south - latPadding;
    const paddedNorth = north + latPadding;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      const b = this.getBuildingBounds(building);

      // Perform fast AABB bounding box check against padded viewport
      if (
        b.maxLng < paddedWest ||
        b.minLng > paddedEast ||
        b.maxLat < paddedSouth ||
        b.minLat > paddedNorth
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
