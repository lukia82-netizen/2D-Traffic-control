import { describe, expect, it, vi } from 'vitest';
import { BuildingRenderer } from './BuildingRenderer';
import type { MapData, BuildingData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import maplibregl from 'maplibre-gl';

describe('BuildingRenderer', () => {
  it('culls off-screen buildings and projects on-screen buildings', () => {
    const mockContainer = {
      addChild: vi.fn(),
    };

    const mockOverlay = {
      buildings: mockContainer,
    } as unknown as PixiOverlay;

    const mockBounds = {
      getWest: () => 10.0,
      getEast: () => 20.0,
      getSouth: () => 50.0,
      getNorth: () => 60.0,
    };

    const mockProject = vi.fn().mockImplementation(([lng, lat]: [number, number]) => ({
      x: (lng - 10) * 10,
      y: (60 - lat) * 10,
    }));

    const mockMap = {
      getBounds: () => mockBounds,
      project: mockProject,
    } as unknown as maplibregl.Map;

    const insideBuilding: BuildingData = {
      id: 1,
      polygon: [
        [55.0, 15.0],
        [55.0, 16.0],
        [56.0, 15.0],
      ],
      buildingType: 'residential',
    };

    const outsideBuilding: BuildingData = {
      id: 2,
      polygon: [
        [10.0, 100.0],
        [10.0, 101.0],
        [11.0, 100.0],
      ],
      buildingType: 'commercial',
    };

    const mapData: MapData = {
      nodes: [],
      edges: [],
      spawnPoints: [],
      bbox: [10, 50, 20, 60],
      buildings: [insideBuilding, outsideBuilding],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      lanes: [],
      conflictAreas: [],
    };

    const renderer = new BuildingRenderer(mockOverlay, mockMap);
    renderer.build(mapData);

    // Inside building has 3 vertices so mockProject should be called 3 times.
    // Outside building should be culled and NOT projected.
    expect(mockProject).toHaveBeenCalledTimes(3);

    // Verify coordinates passed to mockProject belong to insideBuilding ([lng, lat])
    expect(mockProject).toHaveBeenNthCalledWith(1, [15.0, 55.0]);
    expect(mockProject).toHaveBeenNthCalledWith(2, [16.0, 55.0]);
    expect(mockProject).toHaveBeenNthCalledWith(3, [15.0, 56.0]);
  });
});
