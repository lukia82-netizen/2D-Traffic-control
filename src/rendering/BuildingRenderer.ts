import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, BuildingData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { projectPoint } from '../map/MapLibreSetup';

// Building colours (sketch aesthetic on dark background)
const BUILDING_FILL    = 0x2d2d4e; // dark blue-gray fill
const BUILDING_STROKE  = 0x4a4a70; // slightly lighter outline
const BUILDING_ALPHA   = 0.92;

/** Precomputed bounding box entry for fast viewport culling. */
interface CachedBuilding {
  building: BuildingData;
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
  /** Cached buildings with precomputed bounding boxes to avoid per-frame recalculations. */
  private cachedBuildings: CachedBuilding[] = [];

  constructor(overlay: PixiOverlay, map: maplibregl.Map) {
    this.overlay = overlay;
    this.map = map;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  build(mapData: MapData): void {
    this.updateBuildingCache(mapData);
    this.rebuild();
  }

  rebuildOnCameraChange(_mapData: MapData): void {
    this.rebuild();
  }

  // ─── Cache Management ───────────────────────────────────────────────────────

  /** Precomputes geographic bounding boxes for all buildings on map load. */
  private updateBuildingCache(mapData: MapData): void {
    this.cachedBuildings = [];
    if (!mapData.buildings || mapData.buildings.length === 0) return;

    for (const building of mapData.buildings) {
      if (!building.polygon || building.polygon.length < 3) continue;

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

      this.cachedBuildings.push({
        building,
        minLat,
        maxLat,
        minLng,
        maxLng,
      });
    }
  }

  // ─── Rendering ─────────────────────────────────────────────────────────────

  private rebuild(): void {
    if (!this.gfx) {
      this.gfx = new PIXI.Graphics();
      this.overlay.buildings.addChild(this.gfx);
    }

    const gfx = this.gfx;
    gfx.clear();

    if (this.cachedBuildings.length === 0) return;

    // Viewport bounding box culling (with 10% padding margin to avoid pop-in on pan/zoom)
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();

    const padLng = (east - west) * 0.1;
    const padLat = (north - south) * 0.1;
    const minLng = west - padLng;
    const maxLng = east + padLng;
    const minLat = south - padLat;
    const maxLat = north + padLat;

    for (const cached of this.cachedBuildings) {
      // Skip off-screen buildings outside the padded viewport bounds
      if (
        cached.maxLat < minLat ||
        cached.minLat > maxLat ||
        cached.maxLng < minLng ||
        cached.minLng > maxLng
      ) {
        continue;
      }

      // Project each vertex [lat, lng] → screen {x, y}
      const pts: number[] = [];
      for (const [lat, lng] of cached.building.polygon) {
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
    this.cachedBuildings = [];
  }
}
