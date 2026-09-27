import { describe, expect, it, vi } from 'vitest';
import { BuildingRenderer } from './BuildingRenderer';
import type { MapData } from '../bridge/commands';

vi.mock('pixi.js', () => {
  return {
    Graphics: vi.fn(),
  };
});

describe('BuildingRenderer', () => {
  it('culls buildings outside the padded viewport bounds', async () => {
    const mockGraphics = {
      clear: vi.fn(),
      poly: vi.fn().mockReturnThis(),
      fill: vi.fn().mockReturnThis(),
      stroke: vi.fn().mockReturnThis(),
      destroy: vi.fn(),
    };

    const PIXI = await import('pixi.js');
    (PIXI.Graphics as any).mockImplementation(() => mockGraphics);

    const mockOverlay = {
      buildings: {
        addChild: vi.fn(),
      },
    } as any;

    const mockMap = {
      getBounds: () => ({
        getWest: () => 10,
        getEast: () => 20,
        getSouth: () => 10,
        getNorth: () => 20,
      }),
      project: (lngLat: [number, number]) => ({ x: lngLat[0] * 10, y: lngLat[1] * 10 }),
    } as any;

    const renderer = new BuildingRenderer(mockOverlay, mockMap);

    const mapData: MapData = {
      nodes: [],
      edges: [],
      spawnPoints: [],
      bbox: [10, 10, 20, 20],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      lanes: [],
      conflictAreas: [],
      buildings: [
        {
          id: 1,
          buildingType: 'residential',
          // Inside viewport bounds (lat: 15, lng: 15)
          polygon: [
            [15, 15],
            [15, 16],
            [16, 16],
          ],
        },
        {
          id: 2,
          buildingType: 'commercial',
          // Far outside viewport bounds (lat: 100, lng: 100)
          polygon: [
            [100, 100],
            [100, 101],
            [101, 101],
          ],
        },
      ],
    };

    renderer.build(mapData);

    // Each rendered building calls poly twice (fill + stroke)
    // 1 rendered building = 2 poly calls. If unculled, 2 buildings = 4 poly calls.
    expect(mockGraphics.poly).toHaveBeenCalledTimes(2);

    renderer.destroy();
    expect(mockGraphics.destroy).toHaveBeenCalled();
  });
});
