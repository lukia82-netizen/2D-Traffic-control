import { describe, expect, it, vi } from 'vitest';
import type { MapData, BuildingData } from '../bridge/commands';
import { BuildingRenderer } from './BuildingRenderer';

// Mock PIXI.Graphics
vi.mock('pixi.js', () => {
  class MockGraphics {
    public polyCalls: number[][] = [];
    public clearCallCount = 0;
    public destroyed = false;

    clear() {
      this.clearCallCount++;
    }

    poly(pts: number[]) {
      this.polyCalls.push(pts);
      return this;
    }

    fill() {
      return this;
    }

    stroke() {
      return this;
    }

    destroy() {
      this.destroyed = true;
    }
  }

  return {
    Graphics: MockGraphics,
  };
});

describe('BuildingRenderer', () => {
  function createMockMap(bounds = { west: 10, east: 20, south: 10, north: 20 }) {
    return {
      getBounds: () => ({
        getWest: () => bounds.west,
        getEast: () => bounds.east,
        getSouth: () => bounds.south,
        getNorth: () => bounds.north,
      }),
      project: ([lng, lat]: [number, number]) => ({
        x: lng * 10,
        y: lat * 10,
      }),
    } as any;
  }

  function createMockOverlay() {
    return {
      buildings: {
        addChild: vi.fn(),
      },
    } as any;
  }

  const sampleBuildings: BuildingData[] = [
    {
      id: 1,
      buildingType: 'residential',
      polygon: [
        [15, 15], // lat, lng
        [15, 16],
        [16, 16],
        [16, 15],
      ],
    },
    {
      id: 2,
      buildingType: 'commercial',
      polygon: [
        [100, 100], // far off-screen lat, lng
        [100, 101],
        [101, 101],
        [101, 100],
      ],
    },
  ];

  const mapData: MapData = {
    nodes: [],
    edges: [],
    spawnPoints: [],
    bbox: [0, 0, 0, 0],
    buildings: sampleBuildings,
    restrictions: [],
    tramStops: [],
    turnConnectors: [],
    lanes: [],
    conflictAreas: [],
  };

  it('renders visible buildings and culls off-screen buildings', () => {
    const mockMap = createMockMap({ west: 10, east: 20, south: 10, north: 20 });
    const mockOverlay = createMockOverlay();

    const renderer = new BuildingRenderer(mockOverlay, mockMap);
    renderer.build(mapData);

    const gfx = (renderer as any).gfx;
    expect(gfx).toBeDefined();
    // Building 1 is inside bounds (lat 15-16, lng 15-16), Building 2 is far away (lat 100-101, lng 100-101)
    // `gfx.poly` is called twice per building (once for fill, once for stroke)
    expect(gfx.polyCalls.length).toBe(2);

    // Verify projected points for Building 1: polygon coordinates [lat, lng] mapped via projectPoint(lng, lat)
    // point 0: lng=15 -> x=150, lat=15 -> y=150
    expect(gfx.polyCalls[0]).toEqual([150, 150, 160, 150, 160, 160, 150, 160]);
  });

  it('updates rendering when camera changes', () => {
    let currentBounds = { west: 10, east: 20, south: 10, north: 20 };
    const mockMap = {
      getBounds: () => ({
        getWest: () => currentBounds.west,
        getEast: () => currentBounds.east,
        getSouth: () => currentBounds.south,
        getNorth: () => currentBounds.north,
      }),
      project: ([lng, lat]: [number, number]) => ({
        x: lng * 10,
        y: lat * 10,
      }),
    } as any;

    const mockOverlay = createMockOverlay();
    const renderer = new BuildingRenderer(mockOverlay, mockMap);
    renderer.build(mapData);

    const gfx = (renderer as any).gfx;
    expect(gfx.polyCalls.length).toBe(2);

    // Pan camera to center around building 2 (lat 100-101, lng 100-101)
    currentBounds = { west: 95, east: 105, south: 95, north: 105 };
    gfx.polyCalls = [];

    renderer.rebuildOnCameraChange(mapData);

    // Now building 2 is on-screen, building 1 is off-screen
    expect(gfx.polyCalls.length).toBe(2);
    expect(gfx.polyCalls[0]).toEqual([1000, 1000, 1010, 1000, 1010, 1010, 1000, 1010]);
  });

  it('cleans up resources on destroy', () => {
    const mockMap = createMockMap();
    const mockOverlay = createMockOverlay();
    const renderer = new BuildingRenderer(mockOverlay, mockMap);

    renderer.build(mapData);
    const gfx = (renderer as any).gfx;

    renderer.destroy();
    expect(gfx.destroyed).toBe(true);
  });
});
