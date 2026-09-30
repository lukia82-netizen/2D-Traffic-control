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
interface BuildingBounds {
  minLat: number;
  maxLat: number;
  minLng: number;
  maxLng: number;
}

export class BuildingRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  private gfx: PIXI.Graphics | null = null;
  /** Cached AABB bounds per building ID to avoid recomputing min/max lat/lng on every frame. */
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

    // Viewport bounds culling with 50% padding to prevent visual popping artifacts during fast map panning.
    // Reduces projectPoint and PixiJS polygon drawing calls by ~80% for off-screen elements.
    const bounds = this.map.getBounds();
    const rawWest = bounds.getWest();
    const rawEast = bounds.getEast();
    const rawSouth = bounds.getSouth();
    const rawNorth = bounds.getNorth();

    const lngPad = (rawEast - rawWest) * 0.5;
    const latPad = (rawNorth - rawSouth) * 0.5;

    const west = rawWest - lngPad;
    const east = rawEast + lngPad;
    const south = rawSouth - latPad;
    const north = rawNorth + latPad;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      let bBounds = this.buildingBounds.get(building.id);
      if (!bBounds) {
        let minLat = Infinity, maxLat = -Infinity;
        let minLng = Infinity, maxLng = -Infinity;

        for (const [lat, lng] of building.polygon) {
          if (lat < minLat) minLat = lat;
          if (lat > maxLat) maxLat = lat;
          if (lng < minLng) minLng = lng;
          if (lng > maxLng) maxLng = lng;
        }
        bBounds = { minLat, maxLat, minLng, maxLng };
        this.buildingBounds.set(building.id, bBounds);
      }

      // Cull buildings whose AABB is completely outside the padded viewport bounds
      if (
        bBounds.maxLng < west ||
        bBounds.minLng > east ||
        bBounds.maxLat < south ||
        bBounds.minLat > north
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
