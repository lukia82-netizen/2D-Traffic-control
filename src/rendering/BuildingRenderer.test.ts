import { describe, expect, it, vi } from 'vitest';
import * as PIXI from 'pixi.js';
import type { MapData } from '../bridge/commands';
import { BuildingRenderer } from './BuildingRenderer';

describe('BuildingRenderer', () => {
  const createMockMap = (west = 10, south = 10, east = 20, north = 20) => {
    return {
      getBounds: () => ({
        getWest: () => west,
        getEast: () => east,
        getSouth: () => south,
        getNorth: () => north,
      }),
      project: (coords: [number, number]) => ({
        x: coords[0] * 10,
        y: coords[1] * 10,
      }),
    } as any;
  };

  const createMockOverlay = () => {
    const buildingsContainer = new PIXI.Container();
    return {
      buildings: buildingsContainer,
    } as any;
  };

  const sampleMapData: MapData = {
    nodes: [],
    edges: [],
    spawnPoints: [],
    bbox: [0, 0, 100, 100],
    buildings: [
      {
        id: 1,
        // Inside viewport bounds (10-20)
        polygon: [
          [12, 12],
          [12, 18],
          [18, 18],
          [18, 12],
        ],
        buildingType: 'residential',
      },
      {
        id: 2,
        // Completely outside viewport bounds (10-20)
        polygon: [
          [50, 50],
          [50, 60],
          [60, 60],
          [60, 50],
        ],
        buildingType: 'commercial',
      },
    ],
    restrictions: [],
    tramStops: [],
    turnConnectors: [],
    lanes: [],
    conflictAreas: [],
  };

  it('projects and renders visible buildings while culling off-screen buildings', () => {
    const map = createMockMap(10, 10, 20, 20);
    const projectSpy = vi.spyOn(map, 'project');
    const overlay = createMockOverlay();

    const renderer = new BuildingRenderer(overlay, map);
    renderer.build(sampleMapData);

    // Building 1 has 4 vertices inside viewport -> 4 project calls
    // Building 2 is culled -> 0 project calls for building 2
    expect(projectSpy).toHaveBeenCalledTimes(4);

    // Verify calls were only for building 1 coordinates [lat, lng] -> project([lng, lat])
    expect(projectSpy).toHaveBeenNthCalledWith(1, [12, 12]);
    expect(projectSpy).toHaveBeenNthCalledWith(2, [18, 12]);
    expect(projectSpy).toHaveBeenNthCalledWith(3, [18, 18]);
    expect(projectSpy).toHaveBeenNthCalledWith(4, [12, 18]);
  });

  it('re-renders correctly on camera move when viewport shifts', () => {
    const map = createMockMap(10, 10, 20, 20);
    const overlay = createMockOverlay();

    const renderer = new BuildingRenderer(overlay, map);
    renderer.build(sampleMapData);

    const projectSpy = vi.spyOn(map, 'project');

    // Shift camera to encompass building 2 (50-60)
    map.getBounds = () => ({
      getWest: () => 45,
      getEast: () => 65,
      getSouth: () => 45,
      getNorth: () => 65,
    });

    renderer.rebuildOnCameraChange(sampleMapData);

    // Now building 1 should be culled and building 2 projected (4 vertices)
    expect(projectSpy).toHaveBeenCalledTimes(4);
    expect(projectSpy).toHaveBeenNthCalledWith(1, [50, 50]);
  });

  it('cleans up resources on destroy', () => {
    const map = createMockMap();
    const overlay = createMockOverlay();

    const renderer = new BuildingRenderer(overlay, map);
    renderer.build(sampleMapData);

    expect(() => renderer.destroy()).not.toThrow();
  });
});
