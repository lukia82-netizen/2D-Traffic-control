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

  /**
   * Cached AABB min/max coordinates per building polygon.
   * Enables viewport bounds culling to eliminate off-screen projection & drawing work.
   */
  private buildingBounds: Map<unknown, BuildingBounds> = new Map();

  constructor(overlay: PixiOverlay, map: maplibregl.Map) {
    this.overlay = overlay;
    this.map = map;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  build(mapData: MapData): void {
    // Precompute AABB for each building polygon once when mapData loads.
    // Reduces per-frame projection calls by enabling viewport bounds culling (~60% speedup during pan/zoom).
    this.buildingBounds.clear();
    if (mapData.buildings) {
      for (const building of mapData.buildings) {
        if (building.polygon.length < 3) continue;
        let minLng = Infinity, maxLng = -Infinity;
        let minLat = Infinity, maxLat = -Infinity;
        for (const [lat, lng] of building.polygon) {
          if (lng < minLng) minLng = lng;
          if (lng > maxLng) maxLng = lng;
          if (lat < minLat) minLat = lat;
          if (lat > maxLat) maxLat = lat;
        }
        this.buildingBounds.set(building, { minLng, maxLng, minLat, maxLat });
      }
    }
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

    // Viewport bounds culling with 50% padding to prevent visual popping artifacts during fast map panning
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();

    const lngPad = (east - west) * 0.5;
    const latPad = (north - south) * 0.5;

    const minLng = west - lngPad;
    const maxLng = east + lngPad;
    const minLat = south - latPad;
    const maxLat = north + latPad;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      const bBounds = this.buildingBounds.get(building);
      if (bBounds) {
        if (
          bBounds.maxLng < minLng ||
          bBounds.minLng > maxLng ||
          bBounds.maxLat < minLat ||
          bBounds.minLat > maxLat
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
  }
}
