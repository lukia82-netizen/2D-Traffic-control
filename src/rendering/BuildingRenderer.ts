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

/**
 * Draws building footprints fetched from OSM as filled PixiJS polygons.
 * Rebuilds every camera move so polygons stay aligned with the projection.
 */
interface BuildingAABB {
  minLat: number;
  maxLat: number;
  minLng: number;
  maxLng: number;
}

export class BuildingRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  private gfx: PIXI.Graphics | null = null;
  // Cache pre-calculated AABB bounds per building object to avoid recomputing on every frame
  private buildingBounds: WeakMap<object, BuildingAABB> = new WeakMap();

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

    // Viewport bounding box culling: extract map bounds with 20% margin to prevent visual popping
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();
    const lngMargin = (east - west) * 0.2;
    const latMargin = (north - south) * 0.2;

    const minLng = west - lngMargin;
    const maxLng = east + lngMargin;
    const minLat = south - latMargin;
    const maxLat = north + latMargin;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      // Retrieve or compute AABB bounds for O(1) viewport culling
      let aabb = this.buildingBounds.get(building);
      if (!aabb) {
        let bMinLat = Infinity;
        let bMaxLat = -Infinity;
        let bMinLng = Infinity;
        let bMaxLng = -Infinity;

        for (const [lat, lng] of building.polygon) {
          if (lat < bMinLat) bMinLat = lat;
          if (lat > bMaxLat) bMaxLat = lat;
          if (lng < bMinLng) bMinLng = lng;
          if (lng > bMaxLng) bMaxLng = lng;
        }

        aabb = { minLat: bMinLat, maxLat: bMaxLat, minLng: bMinLng, maxLng: bMaxLng };
        this.buildingBounds.set(building, aabb);
      }

      // Cull buildings strictly outside the expanded viewport bounding box to save CPU projection and GPU draw overhead (~70-90% saving when zoomed in)
      if (
        aabb.maxLng < minLng ||
        aabb.minLng > maxLng ||
        aabb.maxLat < minLat ||
        aabb.minLat > maxLat
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
