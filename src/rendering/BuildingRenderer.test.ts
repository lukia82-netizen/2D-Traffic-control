import { describe, expect, it, vi } from 'vitest';
import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import { BuildingRenderer } from './BuildingRenderer';
import type { MapData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';

describe('BuildingRenderer', () => {
  function createMockMap(bounds: { west: number; east: number; south: number; north: number }) {
    return {
      getBounds: () => ({
        getWest: () => bounds.west,
        getEast: () => bounds.east,
        getSouth: () => bounds.south,
        getNorth: () => bounds.north,
      }),
      project: vi.fn(([lng, lat]: [number, number]) => ({ x: lng * 10, y: lat * 10 })),
    } as unknown as maplibregl.Map;
  }

  function createMockOverlay(): PixiOverlay {
    return {
      buildings: new PIXI.Container(),
    } as unknown as PixiOverlay;
  }

  it('projects and renders buildings within viewport bounds', () => {
    const mockMap = createMockMap({ west: 10, east: 20, south: 50, north: 60 });
    const mockOverlay = createMockOverlay();
    const renderer = new BuildingRenderer(mockOverlay, mockMap);

    const mapData: MapData = {
      nodes: [],
      edges: [],
      spawnPoints: [],
      bbox: [10, 50, 20, 60],
      buildings: [
        {
          id: 1,
          buildingType: 'residential',
          polygon: [
            [55, 15], // [lat, lng]
            [55, 16],
            [56, 16],
            [56, 15],
          ],
        },
      ],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      lanes: [],
      conflictAreas: [],
    };

    renderer.build(mapData);

    expect(mockMap.project).toHaveBeenCalled();
    // 4 vertices projected
    expect(mockMap.project).toHaveBeenCalledTimes(4);
  });

  it('culls buildings outside viewport bounds and avoids map.project calls', () => {
    // Viewport is west: 10, east: 20, south: 50, north: 60
    const mockMap = createMockMap({ west: 10, east: 20, south: 50, north: 60 });
    const mockOverlay = createMockOverlay();
    const renderer = new BuildingRenderer(mockOverlay, mockMap);

    const mapData: MapData = {
      nodes: [],
      edges: [],
      spawnPoints: [],
      bbox: [0, 0, 100, 100],
      buildings: [
        {
          id: 10, // Completely off-screen building (lng 100+, lat 100+)
          buildingType: 'commercial',
          polygon: [
            [100, 100],
            [100, 101],
            [101, 101],
            [101, 100],
          ],
        },
      ],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      lanes: [],
      conflictAreas: [],
    };

    renderer.build(mapData);

    // Should cull the off-screen building and skip map.project entirely
    expect(mockMap.project).not.toHaveBeenCalled();
  });

  it('clears internal bounds cache on destroy', () => {
    const mockMap = createMockMap({ west: 10, east: 20, south: 50, north: 60 });
    const mockOverlay = createMockOverlay();
    const renderer = new BuildingRenderer(mockOverlay, mockMap);

    renderer.destroy();
    expect(mockOverlay.buildings.children.length).toBe(0);
  });
});
