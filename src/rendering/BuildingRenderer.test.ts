import { describe, it, expect, vi, beforeEach } from 'vitest';
import { BuildingRenderer } from './BuildingRenderer';
import type { BuildingData, MapData } from '../bridge/commands';
import { projectPoint } from '../map/MapLibreSetup';

vi.mock('../map/MapLibreSetup', () => ({
  projectPoint: vi.fn(() => ({ x: 100, y: 100 })),
}));

describe('BuildingRenderer', () => {
  let mockOverlay: any;
  let mockMap: any;
  let mockGfx: any;

  beforeEach(() => {
    vi.clearAllMocks();

    mockGfx = {
      clear: vi.fn().mockReturnThis(),
      poly: vi.fn().mockReturnThis(),
      fill: vi.fn().mockReturnThis(),
      stroke: vi.fn().mockReturnThis(),
      destroy: vi.fn(),
    };

    mockOverlay = {
      buildings: {
        addChild: vi.fn(),
      },
    };

    mockMap = {
      getBounds: vi.fn(() => ({
        getWest: () => 10.0,
        getEast: () => 20.0,
        getSouth: () => 10.0,
        getNorth: () => 20.0,
      })),
    };
  });

  it('renders visible buildings inside viewport bounds', () => {
    const renderer = new BuildingRenderer(mockOverlay, mockMap);
    (renderer as any).gfx = mockGfx;

    const visibleBuilding: BuildingData = {
      id: 1,
      polygon: [
        [15.0, 15.0],
        [15.1, 15.0],
        [15.1, 15.1],
        [15.0, 15.1],
      ],
      buildingType: 'residential',
    };

    const mapData = {
      nodes: [],
      edges: [],
      lanes: [],
      buildings: [visibleBuilding],
    } as unknown as MapData;

    renderer.build(mapData);

    expect(mockGfx.clear).toHaveBeenCalled();
    expect(projectPoint).toHaveBeenCalledTimes(4);
    expect(mockGfx.poly).toHaveBeenCalled();
    expect(mockGfx.fill).toHaveBeenCalled();
    expect(mockGfx.stroke).toHaveBeenCalled();
  });

  it('culls off-screen buildings outside viewport bounds', () => {
    const renderer = new BuildingRenderer(mockOverlay, mockMap);
    (renderer as any).gfx = mockGfx;

    const offscreenBuilding: BuildingData = {
      id: 2,
      polygon: [
        [50.0, 50.0],
        [50.1, 50.0],
        [50.1, 50.1],
        [50.0, 50.1],
      ],
      buildingType: 'commercial',
    };

    const mapData = {
      nodes: [],
      edges: [],
      lanes: [],
      buildings: [offscreenBuilding],
    } as unknown as MapData;

    renderer.build(mapData);

    expect(mockGfx.clear).toHaveBeenCalled();
    expect(projectPoint).not.toHaveBeenCalled();
    expect(mockGfx.poly).not.toHaveBeenCalled();
  });

  it('caches computed AABB bounds for building polygons', () => {
    const renderer = new BuildingRenderer(mockOverlay, mockMap);
    (renderer as any).gfx = mockGfx;

    const building: BuildingData = {
      id: 3,
      polygon: [
        [15.0, 15.0],
        [15.1, 15.0],
        [15.1, 15.1],
      ],
      buildingType: 'office',
    };

    const getBoundsSpy = vi.spyOn(renderer as any, 'getBuildingBounds');

    const mapData = {
      nodes: [],
      edges: [],
      lanes: [],
      buildings: [building],
    } as unknown as MapData;

    renderer.build(mapData);
    renderer.rebuildOnCameraChange(mapData);

    expect(getBoundsSpy).toHaveBeenCalledTimes(2);
    // The bounds object returned in both calls should be identical due to WeakMap caching
    const firstCallResult = getBoundsSpy.mock.results[0].value;
    const secondCallResult = getBoundsSpy.mock.results[1].value;
    expect(firstCallResult).toBe(secondCallResult);
  });
});
