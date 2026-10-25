import { describe, expect, it, vi } from 'vitest';
import type { MapData } from '../bridge/commands';
import { BuildingRenderer } from './BuildingRenderer';
import { projectPoint } from '../map/MapLibreSetup';

vi.mock('../map/MapLibreSetup', () => ({
  projectPoint: vi.fn((_map: unknown, lng: number, lat: number) => ({
    x: lng * 100,
    y: lat * 100,
  })),
}));

describe('BuildingRenderer', () => {
  it('precomputes building bounds and culls off-screen buildings during rebuild', () => {
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
        getSouth: () => 10,
        getNorth: () => 20,
      }),
    } as any;

    const renderer = new BuildingRenderer(mockOverlay, mockMap);

    const mapData: MapData = {
      nodes: [],
      edges: [],
      spawnPoints: [],
      bbox: [0, 0, 60, 60],
      buildings: [
        {
          id: 1,
          buildingType: 'residential',
          // On-screen building (within 10..20 viewport bounds)
          polygon: [
            [15, 15],
            [15, 16],
            [16, 16],
            [16, 15],
          ],
        },
        {
          id: 2,
          buildingType: 'commercial',
          // Off-screen building (far outside viewport)
          polygon: [
            [50, 50],
            [50, 51],
            [51, 51],
            [51, 50],
          ],
        },
      ],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      lanes: [],
      conflictAreas: [],
    };

    const projectPointMock = vi.mocked(projectPoint);
    projectPointMock.mockClear();

    renderer.build(mapData);

    // Only 4 calls for the 4 vertices of the on-screen building (building id 1)
    expect(projectPointMock).toHaveBeenCalledTimes(4);
    expect(projectPointMock).toHaveBeenCalledWith(mockMap, 15, 15);
    expect(projectPointMock).not.toHaveBeenCalledWith(mockMap, 50, 50);

    // Clean up
    renderer.destroy();
  });
});
