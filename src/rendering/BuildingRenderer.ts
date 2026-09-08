import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, BuildingData } from '../bridge/commands';
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
export class BuildingRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  private gfx: PIXI.Graphics | null = null;

  /** Cache of pre-calculated bounding boxes per building to speed up viewport culling. */
  private buildingBounds: Map<BuildingData, { minLng: number; maxLng: number; minLat: number; maxLat: number }> = new Map();

  constructor(overlay: PixiOverlay, map: maplibregl.Map) {
    this.overlay = overlay;
    this.map = map;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  build(mapData: MapData): void {
    this.buildingBounds.clear();
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

    // Viewport bounding box culling: extract map bounds with a 5% margin
    // to skip projecting and drawing buildings completely outside the viewport.
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();

    const lngMargin = Math.abs(east - west) * 0.05;
    const latMargin = Math.abs(north - south) * 0.05;

    const cullWest = west - lngMargin;
    const cullEast = east + lngMargin;
    const cullSouth = south - latMargin;
    const cullNorth = north + latMargin;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      let bBounds = this.buildingBounds.get(building);
      if (!bBounds) {
        let minLng = Infinity, maxLng = -Infinity;
        let minLat = Infinity, maxLat = -Infinity;
        for (const [lat, lng] of building.polygon) {
          if (lng < minLng) minLng = lng;
          if (lng > maxLng) maxLng = lng;
          if (lat < minLat) minLat = lat;
          if (lat > maxLat) maxLat = lat;
        }
        bBounds = { minLng, maxLng, minLat, maxLat };
        this.buildingBounds.set(building, bBounds);
      }

      // Fast AABB check: skip off-screen buildings
      if (
        bBounds.maxLng < cullWest ||
        bBounds.minLng > cullEast ||
        bBounds.maxLat < cullSouth ||
        bBounds.minLat > cullNorth
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
    this.buildingBounds.clear();
    this.gfx?.destroy();
  }
}
