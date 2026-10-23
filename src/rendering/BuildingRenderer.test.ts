import { describe, expect, it, vi } from 'vitest';
import { BuildingRenderer } from './BuildingRenderer';
import type { MapData, BuildingData } from '../bridge/commands';

// Mock pixi.js Graphics
const mockGfx = {
  clear: vi.fn(),
  poly: vi.fn().mockReturnThis(),
  fill: vi.fn().mockReturnThis(),
  stroke: vi.fn().mockReturnThis(),
  destroy: vi.fn(),
};

vi.mock('pixi.js', () => ({
  Graphics: vi.fn().mockImplementation(() => mockGfx),
}));

describe('BuildingRenderer', () => {
  it('culls off-screen buildings and renders on-screen buildings', () => {
    mockGfx.poly.mockClear();

    const overlay = {
      buildings: {
        addChild: vi.fn(),
      },
    } as any;

    const mockBounds = {
      getWest: () => 10,
      getEast: () => 20,
      getSouth: () => 10,
      getNorth: () => 20,
    };

    const map = {
      getBounds: () => mockBounds,
      project: (coords: [number, number]) => ({ x: coords[0] * 10, y: coords[1] * 10 }),
    } as any;

    const renderer = new BuildingRenderer(overlay, map);

    const visibleBuilding: BuildingData = {
      id: 1,
      polygon: [
        [15, 15],
        [16, 15],
        [16, 16],
      ],
      buildingType: 'residential',
    };

    const offscreenBuilding: BuildingData = {
      id: 2,
      polygon: [
        [100, 100],
        [101, 100],
        [101, 101],
      ],
      buildingType: 'commercial',
    };

    const mapData: MapData = {
      nodes: [],
      edges: [],
      spawnPoints: [],
      bbox: [0, 0, 0, 0],
      buildings: [visibleBuilding, offscreenBuilding],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      lanes: [],
      conflictAreas: [],
    };

    renderer.build(mapData);

    expect(overlay.buildings.addChild).toHaveBeenCalledWith(mockGfx);
    // Each building calls gfx.poly() twice (once for fill, once for stroke).
    // So 1 visible building = 2 poly calls; 2 buildings would be 4 calls.
    expect(mockGfx.poly).toHaveBeenCalledTimes(2);
  });
});
