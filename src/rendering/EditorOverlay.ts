import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import type { MapData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { projectPoint } from '../map/MapLibreSetup';

export class EditorOverlay {
  private readonly overlay: PixiOverlay;
  private readonly map: maplibregl.Map;
  private readonly handlesGfx: PIXI.Graphics;
  private readonly guidesGfx: PIXI.Graphics;
  private readonly selectionGfx: PIXI.Graphics;
  private enabled = false;

  constructor(overlay: PixiOverlay, map: maplibregl.Map) {
    this.overlay = overlay;
    this.map = map;
    this.handlesGfx = new PIXI.Graphics();
    this.guidesGfx = new PIXI.Graphics();
    this.selectionGfx = new PIXI.Graphics();
    this.overlay.editorOverlay.addChild(this.guidesGfx);
    this.overlay.editorOverlay.addChild(this.selectionGfx);
    this.overlay.editorOverlay.addChild(this.handlesGfx);
  }

  setEnabled(enabled: boolean): void {
    this.enabled = enabled;
    this.overlay.editorOverlay.visible = enabled;
  }

  redrawHandles(mapData: MapData): void {
    this.handlesGfx.clear();
    if (!this.enabled) return;
    for (const n of mapData.nodes) {
      const p = projectPoint(this.map, n.lng, n.lat);
      this.handlesGfx.circle(p.x, p.y, 5).fill({ color: 0x67e8f9, alpha: 0.95 });
      this.handlesGfx.circle(p.x, p.y, 7).stroke({ color: 0x0f172a, width: 2, alpha: 0.9 });
    }
  }

  drawAlignmentGuide(pxX: number | null, pxY: number | null): void {
    this.guidesGfx.clear();
    if (!this.enabled) return;
    if (pxX !== null) {
      this.guidesGfx.moveTo(pxX, 0).lineTo(pxX, this.overlay.height).stroke({ color: 0xfacc15, width: 1.5, alpha: 0.9 });
    }
    if (pxY !== null) {
      this.guidesGfx.moveTo(0, pxY).lineTo(this.overlay.width, pxY).stroke({ color: 0xfacc15, width: 1.5, alpha: 0.9 });
    }
  }

  clearGuides(): void {
    this.guidesGfx.clear();
  }

  drawSelectedEdge(mapData: MapData, edgeIndex: number | null): void {
    this.selectionGfx.clear();
    if (!this.enabled || edgeIndex === null) return;
    const edge = mapData.edges[edgeIndex];
    if (!edge) return;
    const from = mapData.nodes.find((n) => n.id === edge.from);
    const to = mapData.nodes.find((n) => n.id === edge.to);
    if (!from || !to) return;
    const p1 = projectPoint(this.map, from.lng, from.lat);
    const p2 = projectPoint(this.map, to.lng, to.lat);
    this.selectionGfx.moveTo(p1.x, p1.y).lineTo(p2.x, p2.y).stroke({ color: 0x22d3ee, width: 8, alpha: 0.8 });
  }
}

