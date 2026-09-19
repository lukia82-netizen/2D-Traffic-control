import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, BuildingData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { projectPoint } from '../map/MapLibreSetup';

// Building colours (sketch aesthetic on dark background)
const BUILDING_FILL    = 0x2d2d4e; // dark blue-gray fill
const BUILDING_STROKE  = 0x4a4a70; // slightly lighter outline
const BUILDING_ALPHA   = 0.92;

interface BuildingBounds {
  minLng: number;
  maxLng: number;
  minLat: number;
  maxLat: number;
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
  /** Cached AABB bounds per building polygon to avoid recalculating min/max on camera moves. */
  private buildingBounds: Map<BuildingData, BuildingBounds> = new Map();

  constructor(overlay: PixiOverlay, map: maplibregl.Map) {
    this.overlay = overlay;
    this.map = map;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  build(mapData: MapData): void {
    this.computeBounds(mapData);
    this.rebuild(mapData);
  }

  rebuildOnCameraChange(mapData: MapData): void {
    this.rebuild(mapData);
  }

  private computeBounds(mapData: MapData): void {
    this.buildingBounds.clear();
    if (!mapData.buildings) return;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      let minLng = Infinity;
      let maxLng = -Infinity;
      let minLat = Infinity;
      let maxLat = -Infinity;

      for (const [lat, lng] of building.polygon) {
        if (lng < minLng) minLng = lng;
        if (lng > maxLng) maxLng = lng;
        if (lat < minLat) minLat = lat;
        if (lat > maxLat) maxLat = lat;
      }

      this.buildingBounds.set(building, { minLng, maxLng, minLat, maxLat });
    }
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

    // Viewport bounds culling with 50% coordinate margin to avoid popping artifacts during panning
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();

    const lngMargin = Math.max((east - west) * 0.5, 0.001);
    const latMargin = Math.max((north - south) * 0.5, 0.001);

    const minViewLng = west - lngMargin;
    const maxViewLng = east + lngMargin;
    const minViewLat = south - latMargin;
    const maxViewLat = north + latMargin;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      let bBounds = this.buildingBounds.get(building);
      if (!bBounds) {
        // Fallback or lazy compute if bounds weren't precomputed
        let minLng = Infinity;
        let maxLng = -Infinity;
        let minLat = Infinity;
        let maxLat = -Infinity;
        for (const [lat, lng] of building.polygon) {
          if (lng < minLng) minLng = lng;
          if (lng > maxLng) maxLng = lng;
          if (lat < minLat) minLat = lat;
          if (lat > maxLat) maxLat = lat;
        }
        bBounds = { minLng, maxLng, minLat, maxLat };
        this.buildingBounds.set(building, bBounds);
      }

      // Check if building AABB intersects extended viewport AABB
      if (
        bBounds.maxLng < minViewLng ||
        bBounds.minLng > maxViewLng ||
        bBounds.maxLat < minViewLat ||
        bBounds.minLat > maxViewLat
      ) {
        continue; // Off-screen, skip projectPoint and PixiJS shape creation
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
