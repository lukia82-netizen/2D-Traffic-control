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
  /**
   * Cached axis-aligned bounding boxes per building.
   * Enables viewport culling during camera moves to avoid projecting off-screen buildings.
   */
  private buildingBounds: Map<BuildingData, BuildingBounds> = new Map();

  constructor(overlay: PixiOverlay, map: maplibregl.Map) {
    this.overlay = overlay;
    this.map = map;
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  build(mapData: MapData): void {
    this.buildingBounds.clear();
    if (mapData.buildings) {
      for (const b of mapData.buildings) {
        let minLng = Infinity;
        let maxLng = -Infinity;
        let minLat = Infinity;
        let maxLat = -Infinity;
        for (const [lat, lng] of b.polygon) {
          if (lng < minLng) minLng = lng;
          if (lng > maxLng) maxLng = lng;
          if (lat < minLat) minLat = lat;
          if (lat > maxLat) maxLat = lat;
        }
        this.buildingBounds.set(b, { minLng, maxLng, minLat, maxLat });
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

    // Viewport bounds culling with ~300m (0.003°) padding to prevent visual popping
    const bounds = this.map.getBounds();
    const minLng = bounds.getWest() - 0.003;
    const maxLng = bounds.getEast() + 0.003;
    const minLat = bounds.getSouth() - 0.003;
    const maxLat = bounds.getNorth() + 0.003;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      // Viewport culling check using cached building bounds
      const b = this.buildingBounds.get(building);
      if (b && (b.maxLng < minLng || b.minLng > maxLng || b.maxLat < minLat || b.minLat > maxLat)) {
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
