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
  /** Cache pre-calculated geographic AABB min/max coordinates for building polygons. */
  private boundsCache = new WeakMap<BuildingData, BuildingBounds>();

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

    // Viewport bounding box culling (with 20% margin padding to prevent edge popping)
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();

    const lngPadding = Math.abs(east - west) * 0.2;
    const latPadding = Math.abs(north - south) * 0.2;
    const minLng = west - lngPadding;
    const maxLng = east + lngPadding;
    const minLat = south - latPadding;
    const maxLat = north + latPadding;

    const antimeridianWrapped = west > east;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      let bBounds = this.boundsCache.get(building);
      if (!bBounds) {
        let bMinLng = Infinity, bMaxLng = -Infinity;
        let bMinLat = Infinity, bMaxLat = -Infinity;
        for (const [lat, lng] of building.polygon) {
          if (lng < bMinLng) bMinLng = lng;
          if (lng > bMaxLng) bMaxLng = lng;
          if (lat < bMinLat) bMinLat = lat;
          if (lat > bMaxLat) bMaxLat = lat;
        }
        bBounds = { minLng: bMinLng, maxLng: bMaxLng, minLat: bMinLat, maxLat: bMaxLat };
        this.boundsCache.set(building, bBounds);
      }

      // Check viewport culling (if not crossing antimeridian)
      if (!antimeridianWrapped) {
        if (
          bBounds.maxLat < minLat ||
          bBounds.minLat > maxLat ||
          bBounds.maxLng < minLng ||
          bBounds.minLng > maxLng
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
  }
}
