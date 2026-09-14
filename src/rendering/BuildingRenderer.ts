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
  /** Cached AABB per building to avoid re-calculating vertex bounds on camera movements. */
  private buildingBounds: Map<number, BuildingBounds> = new Map();

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

    // Fast viewport bounds culling: hoist map viewport coordinates outside the loop.
    // Add 50% coordinate margin padding to avoid popping during fast panning/zooming.
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();
    const lngPadding = (east - west) * 0.5;
    const latPadding = (north - south) * 0.5;
    const minViewLng = west - lngPadding;
    const maxViewLng = east + lngPadding;
    const minViewLat = south - latPadding;
    const maxViewLat = north + latPadding;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      // Get or compute cached geographic AABB for the building footprint
      let bbox = this.buildingBounds.get(building.id);
      if (!bbox) {
        let minLng = Infinity, maxLng = -Infinity, minLat = Infinity, maxLat = -Infinity;
        for (const [lat, lng] of building.polygon) {
          if (lng < minLng) minLng = lng;
          if (lng > maxLng) maxLng = lng;
          if (lat < minLat) minLat = lat;
          if (lat > maxLat) maxLat = lat;
        }
        bbox = { minLng, maxLng, minLat, maxLat };
        this.buildingBounds.set(building.id, bbox);
      }

      // Early exit if building bounding box does not intersect padded viewport
      if (
        bbox.maxLng < minViewLng ||
        bbox.minLng > maxViewLng ||
        bbox.maxLat < minViewLat ||
        bbox.minLat > maxViewLat
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
