import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData, BuildingData } from '../bridge/commands';
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
export interface BuildingBounds {
  minLat: number;
  maxLat: number;
  minLng: number;
  maxLng: number;
}

/** Computes the axis-aligned bounding box (AABB) in geographical coordinates for a building polygon. */
export function computeBuildingBounds(polygon: [number, number][]): BuildingBounds {
  let minLat = Infinity, maxLat = -Infinity;
  let minLng = Infinity, maxLng = -Infinity;
  for (const [lat, lng] of polygon) {
    if (lat < minLat) minLat = lat;
    if (lat > maxLat) maxLat = lat;
    if (lng < minLng) minLng = lng;
    if (lng > maxLng) maxLng = lng;
  }
  return { minLat, maxLat, minLng, maxLng };
}

export class BuildingRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  private gfx: PIXI.Graphics | null = null;
  /** Cached AABB bounds for each building polygon to enable fast viewport culling. */
  private readonly buildingBounds = new Map<BuildingData, BuildingBounds>();

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

    // Retrieve viewport bounds with 50% padding to prevent pop-in during fast panning
    let viewMinLng = -Infinity, viewMaxLng = Infinity;
    let viewMinLat = -Infinity, viewMaxLat = Infinity;
    const bounds = this.map.getBounds?.();
    if (bounds) {
      const west = bounds.getWest();
      const east = bounds.getEast();
      const south = bounds.getSouth();
      const north = bounds.getNorth();
      const lngPad = Math.max((east - west) * 0.5, 0.001);
      const latPad = Math.max((north - south) * 0.5, 0.001);
      viewMinLng = west - lngPad;
      viewMaxLng = east + lngPad;
      viewMinLat = south - latPad;
      viewMaxLat = north + latPad;
    }

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      // Retrieve or compute cached building bounding box
      let bbox = this.buildingBounds.get(building);
      if (!bbox) {
        bbox = computeBuildingBounds(building.polygon);
        this.buildingBounds.set(building, bbox);
      }

      // Viewport culling check: skip buildings strictly outside padded viewport
      if (
        bbox.maxLng < viewMinLng ||
        bbox.minLng > viewMaxLng ||
        bbox.maxLat < viewMinLat ||
        bbox.minLat > viewMaxLat
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
