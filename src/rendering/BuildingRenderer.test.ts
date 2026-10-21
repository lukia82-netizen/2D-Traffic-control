import { describe, expect, it, vi } from 'vitest';
import * as PIXI from 'pixi.js';
import type { MapData, BuildingData } from '../bridge/commands';
import { BuildingRenderer } from './BuildingRenderer';

// Mock maplibre-gl
vi.mock('maplibre-gl', () => {
  return {
    default: {
      Map: vi.fn().mockImplementation(() => ({
        getBounds: vi.fn(),
        project: vi.fn((lnglat: { lng: number; lat: number }) => ({
          x: lnglat.lng * 10,
          y: lnglat.lat * 10,
        })),
      })),
    },
  };
});

describe('BuildingRenderer', () => {
  it('culls buildings outside padded viewport bounds and draws visible ones', () => {
    const mockContainer = {
      addChild: vi.fn(),
    };
    const mockOverlay = {
      buildings: mockContainer,
    } as any;

    const mockBounds = {
      getWest: () => 10,
      getEast: () => 20,
      getSouth: () => 10,
      getNorth: () => 20,
    };

    const mockMap = {
      getBounds: () => mockBounds,
      project: (lnglat: { lng: number; lat: number }) => ({
        x: lnglat.lng * 10,
        y: lnglat.lat * 10,
      }),
    } as any;

    const renderer = new BuildingRenderer(mockOverlay, mockMap);

    // Building 1: Inside viewport (lat: 15, lng: 15)
    const visibleBuilding: BuildingData = {
      id: 1,
      polygon: [
        [15, 15],
        [15, 16],
        [16, 16],
        [16, 15],
      ],
      buildingType: 'residential',
    };

    // Building 2: Far outside viewport (lat: 100, lng: 100)
    const culledBuilding: BuildingData = {
      id: 2,
      polygon: [
        [100, 100],
        [100, 101],
        [101, 101],
        [101, 100],
      ],
      buildingType: 'commercial',
    };

    const mapData: MapData = {
      nodes: [],
      edges: [],
      spawnPoints: [],
      bbox: [0, 0, 0, 0],
      buildings: [visibleBuilding, culledBuilding],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      lanes: [],
      conflictAreas: [],
    };

    renderer.build(mapData);

    expect(mockContainer.addChild).toHaveBeenCalledTimes(1);
    const gfx = (renderer as any).gfx as PIXI.Graphics;
    expect(gfx).toBeDefined();

    // Verify boundsCache cached both buildings
    const boundsCache = (renderer as any).boundsCache as WeakMap<BuildingData, any>;
    expect(boundsCache.get(visibleBuilding)).toEqual({
      minLat: 15,
      maxLat: 16,
      minLng: 15,
      maxLng: 16,
    });
    expect(boundsCache.get(culledBuilding)).toEqual({
      minLat: 100,
      maxLat: 101,
      minLng: 100,
      maxLng: 101,
    });
  });
});
