import { describe, expect, it, vi } from 'vitest';
import type { MapData, BuildingData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { BuildingRenderer } from './BuildingRenderer';

describe('BuildingRenderer', () => {
  function createMockOverlay(): PixiOverlay {
    return {
      buildings: {
        addChild: vi.fn(),
      },
    } as unknown as PixiOverlay;
  }

  function createMockMap(
    bounds = { west: 10, east: 20, south: 10, north: 20 },
  ) {
    return {
      getBounds: () => ({
        getWest: () => bounds.west,
        getEast: () => bounds.east,
        getSouth: () => bounds.south,
        getNorth: () => bounds.north,
      }),
      project: vi.fn((coords: [number, number]) => ({
        x: coords[0] * 10,
        y: coords[1] * 10,
      })),
    };
  }

  it('renders buildings inside padded viewport and skips buildings outside viewport', () => {
    const mockOverlay = createMockOverlay();
    const mockMap = createMockMap({ west: 10, east: 20, south: 10, north: 20 });
    const renderer = new BuildingRenderer(mockOverlay, mockMap as any);

    // Viewport: west=10, east=20 (width=10, 20% pad = 2 => minLng=8, maxLng=22)
    // south=10, north=20 (height=10, 20% pad = 2 => minLat=8, maxLat=22)

    const insideBuilding: BuildingData = {
      id: 1,
      buildingType: 'residential',
      // polygon is [lat, lng]
      polygon: [
        [15, 15],
        [15, 16],
        [16, 16],
        [16, 15],
      ],
    };

    const farOutsideBuilding: BuildingData = {
      id: 2,
      buildingType: 'commercial',
      polygon: [
        [50, 50],
        [50, 51],
        [51, 51],
        [51, 50],
      ],
    };

    const mapData: MapData = {
      nodes: [],
      edges: [],
      spawnPoints: [],
      bbox: [0, 0, 30, 30],
      buildings: [insideBuilding, farOutsideBuilding],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      lanes: [],
      conflictAreas: [],
    };

    renderer.build(mapData);

    // map.project should be called 4 times (for the 4 vertices of the inside building),
    // and 0 times for the far outside building!
    expect(mockMap.project).toHaveBeenCalledTimes(4);
  });
});
