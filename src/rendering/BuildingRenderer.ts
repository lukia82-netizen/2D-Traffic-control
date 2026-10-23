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
  /** Cached geographic AABB per building to avoid re-calculating on every camera frame. */
  private readonly boundsCache = new WeakMap<BuildingData, BuildingBounds>();

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

  // ─── Bounds Cache ──────────────────────────────────────────────────────────

  private getBounds(building: BuildingData): BuildingBounds {
    let bounds = this.boundsCache.get(building);
    if (!bounds) {
      let minLat = Infinity, maxLat = -Infinity;
      let minLng = Infinity, maxLng = -Infinity;
      for (const [lng, lat] of building.polygon) {
        if (lat < minLat) minLat = lat;
        if (lat > maxLat) maxLat = lat;
        if (lng < minLng) minLng = lng;
        if (lng > maxLng) maxLng = lng;
      }
      bounds = { minLat, maxLat, minLng, maxLng };
      this.boundsCache.set(building, bounds);
    }
    return bounds;
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

    // Viewport bounding box culling: skip projecting/drawing off-screen buildings
    const bounds = this.map.getBounds();
    const west = bounds.getWest();
    const east = bounds.getEast();
    const south = bounds.getSouth();
    const north = bounds.getNorth();

    // 20% padding margin to avoid clipping buildings near viewport edges
    const padLng = (east - west) * 0.2;
    const padLat = (north - south) * 0.2;
    const minLng = west - padLng;
    const maxLng = east + padLng;
    const minLat = south - padLat;
    const maxLat = north + padLat;
    const isWrapped = east < west; // Antimeridian safety fallback

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      if (!isWrapped) {
        const b = this.getBounds(building);
        if (
          b.maxLng < minLng ||
          b.minLng > maxLng ||
          b.maxLat < minLat ||
          b.minLat > maxLat
        ) {
          continue;
        }
      }

      // Project each vertex [lng, lat] → screen {x, y}
      const pts: number[] = [];
      for (const [lng, lat] of building.polygon) {
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
