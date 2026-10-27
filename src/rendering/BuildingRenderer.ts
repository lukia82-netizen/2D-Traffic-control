import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { BuildingData, MapData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { projectPoint } from '../map/MapLibreSetup';

// Building colours (sketch aesthetic on dark background)
const BUILDING_FILL    = 0x2d2d4e; // dark blue-gray fill
const BUILDING_STROKE  = 0x4a4a70; // slightly lighter outline
const BUILDING_ALPHA   = 0.92;

interface BuildingWithBbox {
  building: BuildingData;
  minLat: number;
  maxLat: number;
  minLng: number;
  maxLng: number;
}

// ─── BuildingRenderer ─────────────────────────────────────────────────────────

/**
 * Draws building footprints fetched from OSM as filled PixiJS polygons.
 * Precomputes geographic bounding boxes on map load and performs viewport
 * culling with a 10% padding margin on camera moves to skip off-screen calculations.
 */
export class BuildingRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  private gfx: PIXI.Graphics | null = null;
  private cachedBuildings: BuildingWithBbox[] = [];

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
    if (this.cachedBuildings.length === 0 && mapData.buildings?.length) {
      this.cacheBuildings(mapData);
    }
    this.rebuild();
  }

  // ─── Precomputation ────────────────────────────────────────────────────────

  private cacheBuildings(mapData: MapData): void {
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

    // Viewport geographic bounding box culling (with 10% padding margin)
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();
    const lngPad = (east - west) * 0.1;
    const latPad = (north - south) * 0.1;
    const minLng = west - lngPad;
    const maxLng = east + lngPad;
    const minLat = south - latPad;
    const maxLat = north + latPad;

    for (const item of this.cachedBuildings) {
      // Skip off-screen buildings outside the viewport bounds
      if (
        item.maxLat < minLat ||
        item.minLat > maxLat ||
        item.maxLng < minLng ||
        item.minLng > maxLng
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
  }
}
