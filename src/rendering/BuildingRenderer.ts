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
  /** Cached AABB bounds per building polygon to avoid recomputing min/max lat/lng on every camera move. */
  private buildingBounds: Map<BuildingData, BuildingBounds> = new Map();

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

    // Viewport bounding box culling: hoist map bounds outside the building loop
    const mapBounds = this.map.getBounds();
    const west = mapBounds.getWest();
    const east = mapBounds.getEast();
    const south = mapBounds.getSouth();
    const north = mapBounds.getNorth();

    // 10% viewport padding factor to account for map rotation/pitch and border edge cases
    const lngPad = Math.abs(east - west) * 0.1;
    const latPad = Math.abs(north - south) * 0.1;

    const minViewLng = west - lngPad;
    const maxViewLng = east + lngPad;
    const minViewLat = south - latPad;
    const maxViewLat = north + latPad;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      let bounds = this.buildingBounds.get(building);
      if (!bounds) {
        let minLat = Infinity, maxLat = -Infinity;
        let minLng = Infinity, maxLng = -Infinity;
        for (const [lat, lng] of building.polygon) {
          if (lat < minLat) minLat = lat;
          if (lat > maxLat) maxLat = lat;
          if (lng < minLng) minLng = lng;
          if (lng > maxLng) maxLng = lng;
        }
        bounds = { minLat, maxLat, minLng, maxLng };
        this.buildingBounds.set(building, bounds);
      }

      // Viewport culling: skip buildings entirely outside the visible viewport bounding box
      if (
        bounds.maxLat < minViewLat ||
        bounds.minLat > maxViewLat ||
        bounds.maxLng < minViewLng ||
        bounds.minLng > maxViewLng
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
