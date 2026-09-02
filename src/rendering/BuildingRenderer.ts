import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, BuildingData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { projectPoint } from '../map/MapLibreSetup';

// Building colours (sketch aesthetic on dark background)
const BUILDING_FILL    = 0x2d2d4e; // dark blue-gray fill
const BUILDING_STROKE  = 0x4a4a70; // slightly lighter outline
const BUILDING_ALPHA   = 0.92;

interface BuildingWithBBox {
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
  private cachedBuildings: BuildingWithBBox[] = [];
  private cachedMapDataBuildings: BuildingData[] | null = null;

  constructor(overlay: PixiOverlay, map: maplibregl.Map) {
    this.overlay = overlay;
    this.map = map;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  build(mapData: MapData): void {
    this.cacheBuildings(mapData);
    this.rebuild();
  }

  rebuildOnCameraChange(mapData: MapData): void {
    if (mapData && mapData.buildings && mapData.buildings !== this.cachedMapDataBuildings) {
      this.cacheBuildings(mapData);
    }
    this.rebuild();
  }

  /**
   * Precomputes bounding boxes for all buildings to enable fast viewport culling.
   */
  private cacheBuildings(mapData: MapData): void {
    this.cachedMapDataBuildings = mapData.buildings || null;
    this.cachedBuildings = [];
    if (!mapData.buildings || mapData.buildings.length === 0) return;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

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

    // Viewport bounding box culling (with 5% coordinate padding)
    // Avoids expensive MapLibre projections and PixiJS polygon draws for off-screen buildings
    const bounds = this.map.getBounds();
    const west  = bounds.getWest();
    const east  = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();

    const lngPad = (east - west) * 0.05;
    const latPad = (north - south) * 0.05;

    const minLngVisible = west - lngPad;
    const maxLngVisible = east + lngPad;
    const minLatVisible = south - latPad;
    const maxLatVisible = north + latPad;

    for (const item of this.cachedBuildings) {
      // Cull off-screen buildings before projecting coordinates or generating PixiJS shapes
      if (
        item.maxLng < minLngVisible ||
        item.minLng > maxLngVisible ||
        item.maxLat < minLatVisible ||
        item.minLat > maxLatVisible
      ) {
        continue;
      }

      // Project each vertex [lat, lng] → screen {x, y}
      const pts: number[] = [];
      for (const [lat, lng] of item.building.polygon) {
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
    this.cachedMapDataBuildings = null;
  }
}
