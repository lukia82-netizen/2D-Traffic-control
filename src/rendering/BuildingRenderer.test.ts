import { describe, it, expect, vi, beforeEach } from 'vitest';
import { BuildingRenderer } from './BuildingRenderer';
import type { MapData } from '../bridge/commands';

// Mock PixiJS
vi.mock('pixi.js', () => {
  class MockGraphics {
    poly = vi.fn().mockReturnThis();
    fill = vi.fn().mockReturnThis();
    stroke = vi.fn().mockReturnThis();
    clear = vi.fn().mockReturnThis();
    destroy = vi.fn();
  }

  class MockContainer {
    addChild = vi.fn();
  }

  return {
    Graphics: MockGraphics,
    Container: MockContainer,
  };
});

// Mock MapLibreSetup projectPoint
vi.mock('../map/MapLibreSetup', () => ({
  projectPoint: vi.fn((_map: unknown, lng: number, lat: number) => ({
    x: lng * 10,
    y: lat * 10,
  })),
}));

describe('BuildingRenderer', () => {
  let mockOverlay: any;
  let mockMap: any;
  let renderer: BuildingRenderer;

  beforeEach(() => {
    mockOverlay = {
      buildings: {
        addChild: vi.fn(),
      },
    };

    mockMap = {
      getBounds: vi.fn().mockReturnValue({
        getWest: () => 10,
        getEast: () => 20,
        getSouth: () => 10,
        getNorth: () => 20,
      }),
      project: vi.fn(),
    };

    renderer = new BuildingRenderer(mockOverlay, mockMap);
  });

  const mockMapData: MapData = {
    nodes: [],
    edges: [],
    lanes: [],
    spawnPoints: [],
    bbox: [0, 0, 0, 0],
    restrictions: [],
    tramStops: [],
    turnConnectors: [],
    conflictAreas: [],
    buildings: [
      {
        id: 1,
        buildingType: 'residential',
        polygon: [
          [15, 15],
          [16, 15],
          [16, 16],
          [15, 16],
        ],
      },
      {
        id: 2, // Off-screen building
        buildingType: 'commercial',
        polygon: [
          [50, 50],
          [51, 50],
          [51, 51],
          [50, 51],
        ],
      },
    ],
  };

  it('precomputes bounding boxes and culls off-screen buildings during build', async () => {
    const { projectPoint } = await import('../map/MapLibreSetup');
    vi.clearAllMocks();

    renderer.build(mockMapData);

    // Only building 1 (on-screen, [15,15]) should have its vertices projected.
    // Building 2 ([50,50]) is far outside viewport bounds [10,20] and should be culled.
    expect(projectPoint).toHaveBeenCalledTimes(4);
  });

  it('clears cached buildings upon destroy', async () => {
    const { projectPoint } = await import('../map/MapLibreSetup');
    renderer.build(mockMapData);

    renderer.destroy();
    vi.clearAllMocks();

    renderer.rebuildOnCameraChange(mockMapData);

    // Should re-cache and project again
    expect(projectPoint).toHaveBeenCalledTimes(4);
  });
});
