import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, BuildingData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { projectPoint } from '../map/MapLibreSetup';

// Building colours (sketch aesthetic on dark background)
const BUILDING_FILL    = 0x2d2d4e; // dark blue-gray fill
const BUILDING_STROKE  = 0x4a4a70; // slightly lighter outline
const BUILDING_ALPHA   = 0.92;

interface BuildingAABB {
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
  /** WeakMap cache for pre-calculated building AABBs (min/max lng & lat). */
  private boundsCache = new WeakMap<BuildingData, BuildingAABB>();

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

    // Viewport bounds culling with 20% padding margin to avoid projecting off-screen buildings
    const bounds = this.map.getBounds();
    let minLng = -Infinity, maxLng = Infinity, minLat = -Infinity, maxLat = Infinity;
    if (bounds) {
      const west = bounds.getWest();
      const east = bounds.getEast();
      const south = bounds.getSouth();
      const north = bounds.getNorth();

      const padLng = (east - west) * 0.2;
      const padLat = (north - south) * 0.2;
      minLng = west - padLng;
      maxLng = east + padLng;
      minLat = south - padLat;
      maxLat = north + padLat;
    }

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      let box = this.boundsCache.get(building);
      if (!box) {
        let bMinLng = Infinity;
        let bMaxLng = -Infinity;
        let bMinLat = Infinity;
        let bMaxLat = -Infinity;

        for (const [lat, lng] of building.polygon) {
          if (lng < bMinLng) bMinLng = lng;
          if (lng > bMaxLng) bMaxLng = lng;
          if (lat < bMinLat) bMinLat = lat;
          if (lat > bMaxLat) bMaxLat = lat;
        }

        box = { minLng: bMinLng, maxLng: bMaxLng, minLat: bMinLat, maxLat: bMaxLat };
        this.boundsCache.set(building, box);
      }

      // Skip off-screen buildings
      if (
        box.maxLng < minLng ||
        box.minLng > maxLng ||
        box.maxLat < minLat ||
        box.minLat > maxLat
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
