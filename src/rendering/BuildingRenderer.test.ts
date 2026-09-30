import { describe, it, expect, vi } from 'vitest';
import { BuildingRenderer } from './BuildingRenderer';
import type { MapData } from '../bridge/commands';

describe('BuildingRenderer', () => {
  it('renders buildings inside viewport bounds and culls buildings outside padded bounds', () => {
    // Mock PixiOverlay
    const mockContainer = {
      addChild: vi.fn(),
    };
    const mockOverlay = {
      buildings: mockContainer,
    } as any;

    // Mock MapLibre Map
    const mockMap = {
      getBounds: vi.fn().mockReturnValue({
        getWest: () => 10,
        getEast: () => 20,
        getSouth: () => 50,
        getNorth: () => 60,
      }),
      project: vi.fn().mockImplementation(([lng, lat]) => ({ x: lng * 10, y: lat * 10 })),
    } as any;

    const renderer = new BuildingRenderer(mockOverlay, mockMap);

    const mapData: MapData = {
      nodes: [],
      edges: [],
      lanes: [],
      buildings: [
        {
          id: 1,
          buildingType: 'residential',
          // Inside viewport (lat 55, lng 15)
          polygon: [
            [55, 15],
            [55, 16],
            [56, 16],
            [56, 15],
          ],
        },
        {
          id: 2,
          buildingType: 'commercial',
          // Far outside viewport (lat 100, lng 100)
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
      conflictAreas: [],
      spawnPoints: [],
      bbox: [10, 50, 20, 60],
    };

    renderer.build(mapData);

    // mockMap.project should be called 4 times for building 1, but NOT for building 2
    expect(mockMap.project).toHaveBeenCalledTimes(4);
    expect(mockMap.project).toHaveBeenCalledWith([15, 55]);
  });

  it('uses cached building bounds across multiple rebuild calls', () => {
    const mockContainer = {
      addChild: vi.fn(),
    };
    const mockOverlay = {
      buildings: mockContainer,
    } as any;

    const mockMap = {
      getBounds: vi.fn().mockReturnValue({
        getWest: () => 10,
        getEast: () => 20,
        getSouth: () => 50,
        getNorth: () => 60,
      }),
      project: vi.fn().mockImplementation(([lng, lat]) => ({ x: lng * 10, y: lat * 10 })),
    } as any;

    const renderer = new BuildingRenderer(mockOverlay, mockMap);

    const mapData: MapData = {
      nodes: [],
      edges: [],
      lanes: [],
      buildings: [
        {
          id: 1,
          buildingType: 'residential',
          polygon: [
            [55, 15],
            [55, 16],
            [56, 16],
          ],
        },
      ],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      conflictAreas: [],
      spawnPoints: [],
      bbox: [10, 50, 20, 60],
    };

    renderer.build(mapData);
    expect(mockMap.project).toHaveBeenCalledTimes(3);

    // Rebuild on camera change
    renderer.rebuildOnCameraChange(mapData);
    expect(mockMap.project).toHaveBeenCalledTimes(6);

    renderer.destroy();
  });
});
