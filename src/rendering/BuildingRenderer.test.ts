import { describe, expect, it, vi } from 'vitest';
import { BuildingRenderer } from './BuildingRenderer';
import type { MapData } from '../bridge/commands';

describe('BuildingRenderer', () => {
  it('renders visible buildings and culls off-screen buildings', () => {
    // Mock PixiOverlay buildings container
    const mockContainer = {
      addChild: vi.fn(),
    };
    const mockOverlay: any = {
      buildings: mockContainer,
    };

    // Viewport bounds: lat 10..20, lng 10..20
    const mockBounds = {
      getWest: () => 10,
      getEast: () => 20,
      getSouth: () => 10,
      getNorth: () => 20,
    };

    const mockMap: any = {
      getBounds: () => mockBounds,
      project: vi.fn((coords: [number, number]) => ({ x: coords[0] * 10, y: coords[1] * 10 })),
    };

    const renderer = new BuildingRenderer(mockOverlay, mockMap);

    const mapData = {
      buildings: [
        {
          id: 1,
          polygon: [
            [15, 15],
            [16, 15],
            [16, 16],
            [15, 16],
          ],
          buildingType: 'residential',
        },
        {
          id: 2,
          polygon: [
            [50, 50],
            [51, 50],
            [51, 51],
            [50, 51],
          ],
          buildingType: 'commercial',
        },
      ],
    } as MapData;

    renderer.build(mapData);

    // Map project should be called only for the visible building vertices (4 calls)
    // and NOT for the offscreen building vertices.
    expect(mockMap.project).toHaveBeenCalledTimes(4);
    expect(mockMap.project).toHaveBeenCalledWith([15, 15]);
    expect(mockMap.project).toHaveBeenCalledWith([16, 15]);
    expect(mockMap.project).toHaveBeenCalledWith([16, 16]);
    expect(mockMap.project).toHaveBeenCalledWith([15, 16]);
    expect(mockMap.project).not.toHaveBeenCalledWith([50, 50]);
  });
});
