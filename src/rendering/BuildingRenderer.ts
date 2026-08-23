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
export class BuildingRenderer {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;

  private gfx: PIXI.Graphics | null = null;

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

    // Viewport bounding box culling (10% margin padding)
    // Avoids projecting coordinates and creating PixiJS geometry for off-screen buildings.
    const bounds = this.map.getBounds();
    const w = bounds.getWest();
    const e = bounds.getEast();
    const s = bounds.getSouth();
    const n = bounds.getNorth();
    const padLng = (e - w) * 0.1;
    const padLat = (n - s) * 0.1;
    const minLng = w - padLng;
    const maxLng = e + padLng;
    const minLat = s - padLat;
    const maxLat = n + padLat;

    for (const building of mapData.buildings) {
      if (building.polygon.length < 3) continue;

      // Quick bounding box check for polygon vertices
      let bMinLng = Infinity;
      let bMaxLng = -Infinity;
      let bMinLat = Infinity;
      let bMaxLat = -Infinity;

      for (let i = 0; i < building.polygon.length; i++) {
        const pt = building.polygon[i];
        const lat = pt[0];
        const lng = pt[1];
        if (lng < bMinLng) bMinLng = lng;
        if (lng > bMaxLng) bMaxLng = lng;
        if (lat < bMinLat) bMinLat = lat;
        if (lat > bMaxLat) bMaxLat = lat;
      }

      // Cull off-screen building footprint
      if (bMaxLng < minLng || bMinLng > maxLng || bMaxLat < minLat || bMinLat > maxLat) {
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
