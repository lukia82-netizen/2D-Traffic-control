import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { projectPoint } from '../map/MapLibreSetup';

// Building colours (sketch aesthetic on dark background)
const BUILDING_FILL    = 0x2d2d4e; // dark blue-gray fill
const BUILDING_STROKE  = 0x4a4a70; // slightly lighter outline
const BUILDING_ALPHA   = 0.92;

interface BuildingBounds {
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
  /** Cached AABB min/max coordinates for each building polygon to accelerate viewport culling. */
  private buildingBounds: Map<object, BuildingBounds> = new Map();
  private lastMapData: MapData | null = null;

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

  /**
   * Pre-computes AABB bounding boxes for each building polygon once when map data arrives.
   */
  private updateBuildingBounds(mapData: MapData): void {
    this.buildingBounds.clear();
    if (!mapData.buildings) return;

    for (const building of mapData.buildings) {
      if (!building.polygon || building.polygon.length === 0) continue;

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

      this.buildingBounds.set(building, { minLat, maxLat, minLng, maxLng });
    }
  }

  // ─── Rendering ─────────────────────────────────────────────────────────────

  private rebuild(mapData: MapData): void {
    if (this.lastMapData !== mapData) {
      this.updateBuildingBounds(mapData);
      this.lastMapData = mapData;
    }

    if (!this.gfx) {
      this.gfx = new PIXI.Graphics();
      this.overlay.buildings.addChild(this.gfx);
    }

    const gfx = this.gfx;
    gfx.clear();

    if (!mapData.buildings || mapData.buildings.length === 0) return;

    // Viewport bounding box culling (with 50% padding)
    // Hoisting map bounds extraction and applying viewport culling avoids expensive
    // MapLibre projectPoint calls and PixiJS polygon paths for off-screen buildings.
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();
    const lngPad = (east - west) * 0.5;
    const latPad = (north - south) * 0.5;

    const minViewLng = west - lngPad;
    const maxViewLng = east + lngPad;
    const minViewLat = south - latPad;
    const maxViewLat = north + latPad;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      // Fast AABB viewport culling check
      const bbox = this.buildingBounds.get(building);
      if (bbox) {
        if (
          bbox.maxLng < minViewLng ||
          bbox.minLng > maxViewLng ||
          bbox.maxLat < minViewLat ||
          bbox.minLat > maxViewLat
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
    this.buildingBounds.clear();
    this.lastMapData = null;
  }
}
