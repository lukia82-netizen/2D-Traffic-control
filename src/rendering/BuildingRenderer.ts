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
  minLng: number;
  maxLng: number;
  minLat: number;
  maxLat: number;
}

export class BuildingRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  private gfx: PIXI.Graphics | null = null;

  /** Precomputed geographic bounding boxes for each building polygon to enable fast viewport culling. */
  private buildingBounds: Map<unknown, BuildingAABB> = new Map();

  constructor(overlay: PixiOverlay, map: maplibregl.Map) {
    this.overlay = overlay;
    this.map = map;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  build(mapData: MapData): void {
    this.precomputeBounds(mapData);
    this.rebuild(mapData);
  }

  private precomputeBounds(mapData: MapData): void {
    this.buildingBounds.clear();
    if (!mapData.buildings) return;

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

      this.buildingBounds.set(building, { minLat, maxLat, minLng, maxLng });
    }
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

    // Retrieve viewport bounds once per rebuild and add margin (50% padding)
    // to prevent visual popping during fast panning.
    const bounds = this.map.getBounds();
    const w = bounds.getWest();
    const e = bounds.getEast();
    const s = bounds.getSouth();
    const n = bounds.getNorth();

    const lngPad = (e - w) * 0.5;
    const latPad = (n - s) * 0.5;

    const minWest = w - lngPad;
    const maxEast = e + lngPad;
    const minSouth = s - latPad;
    const maxNorth = n + latPad;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      // Fast viewport culling check using precomputed building AABB
      const b = this.buildingBounds.get(building);
      if (b) {
        if (b.maxLng < minWest || b.minLng > maxEast || b.maxLat < minSouth || b.minLat > maxNorth) {
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
