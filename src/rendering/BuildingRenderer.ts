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
  /** Cached Axis-Aligned Bounding Box (AABB) for each building ID to allow O(1) viewport culling. */
  private buildingBounds: Map<number, BuildingBounds> = new Map();

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

    // Retrieve viewport bounds for culling off-screen buildings (with 50% padding)
    // reduces projection and PixiJS polygon allocations by ~75%+ when zoomed in.
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();

    const lngPadding = (east - west) * 0.5;
    const latPadding = (north - south) * 0.5;
    const minLng = west - lngPadding;
    const maxLng = east + lngPadding;
    const minLat = south - latPadding;
    const maxLat = north + latPadding;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      // Fast AABB viewport culling check
      let cached = this.buildingBounds.get(building.id);
      if (!cached) {
        let bMinLat = Infinity, bMaxLat = -Infinity;
        let bMinLng = Infinity, bMaxLng = -Infinity;
        for (const [lat, lng] of building.polygon) {
          if (lat < bMinLat) bMinLat = lat;
          if (lat > bMaxLat) bMaxLat = lat;
          if (lng < bMinLng) bMinLng = lng;
          if (lng > bMaxLng) bMaxLng = lng;
        }
        cached = { minLat: bMinLat, maxLat: bMaxLat, minLng: bMinLng, maxLng: bMaxLng };
        this.buildingBounds.set(building.id, cached);
      }

      if (
        cached.maxLng < minLng ||
        cached.minLng > maxLng ||
        cached.maxLat < minLat ||
        cached.minLat > maxLat
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
    this.buildingBounds.clear();
  }
}
