import { describe, it, expect, vi } from 'vitest';
import { BuildingRenderer } from './BuildingRenderer';
import type { MapData, BuildingData } from '../bridge/commands';

// Mock PIXI.Graphics
vi.mock('pixi.js', () => {
  class Graphics {
    clear() { return this; }
    poly() { return this; }
    fill() { return this; }
    stroke() { return this; }
    destroy() {}
  }
  return { Graphics };
});

describe('BuildingRenderer', () => {
  it('culls off-screen buildings and renders visible ones', () => {
    const addChildMock = vi.fn();

    const mockOverlay: any = {
      buildings: {
        addChild: addChildMock,
      },
    };

    const mockMapBounds = {
      getWest: () => 10,
      getEast: () => 20,
      getSouth: () => 10,
      getNorth: () => 20,
    };

    const projectedPoints: [number, number][] = [];
    const mockMap: any = {
      getBounds: () => mockMapBounds,
      project: (coords: { lng: number; lat: number } | [number, number]) => {
        const lng = Array.isArray(coords) ? coords[0] : coords.lng;
        const lat = Array.isArray(coords) ? coords[1] : coords.lat;
        projectedPoints.push([lng, lat]);
        return { x: lng * 10, y: lat * 10 };
      },
    };

    const renderer = new BuildingRenderer(mockOverlay, mockMap);

    // Visible building inside [10..20, 10..20].
    // Note: building.polygon vertices are [lat, lng]
    const visibleBuilding: BuildingData = {
      id: 1,
      polygon: [
        [15, 15], // lat=15, lng=15
        [15, 16], // lat=15, lng=16
        [16, 16], // lat=16, lng=16
      ],
      buildingType: 'residential',
    };

    // Off-screen building far away at [50..51, 50..51]
    const offScreenBuilding: BuildingData = {
      id: 2,
      polygon: [
        [50, 50],
        [50, 51],
        [51, 51],
      ],
      buildingType: 'commercial',
    };

    const mapData: MapData = {
      nodes: [],
      edges: [],
      spawnPoints: [],
      bbox: [0, 0, 0, 0],
      buildings: [visibleBuilding, offScreenBuilding],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      lanes: [],
      conflictAreas: [],
    };

    renderer.build(mapData);

    expect(addChildMock).toHaveBeenCalledTimes(1);
    // Only vertices from visibleBuilding should be projected as [lng, lat]
    expect(projectedPoints.length).toBe(3);
    expect(projectedPoints).toEqual([
      [15, 15],
      [16, 15],
      [16, 16],
    ]);
  });

  it('caches bounding box calculation across camera rebuilds', () => {
    const addChildMock = vi.fn();
    const mockOverlay: any = {
      buildings: { addChild: addChildMock },
    };

    const mockMapBounds = {
      getWest: () => 10,
      getEast: () => 20,
      getSouth: () => 10,
      getNorth: () => 20,
    };

    let projectCount = 0;
    const mockMap: any = {
      getBounds: () => mockMapBounds,
      project: () => {
        projectCount++;
        return { x: 100, y: 100 };
      },
    };

    const renderer = new BuildingRenderer(mockOverlay, mockMap);

    const building: BuildingData = {
      id: 1,
      polygon: [
        [12, 12],
        [12, 14],
        [14, 14],
      ],
      buildingType: 'residential',
    };

    const mapData: MapData = {
      nodes: [],
      edges: [],
      spawnPoints: [],
      bbox: [0, 0, 0, 0],
      buildings: [building],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      lanes: [],
      conflictAreas: [],
    };

    renderer.build(mapData);
    expect(projectCount).toBe(3);

    // Rebuild on camera change — bounding box should be retrieved from cache
    renderer.rebuildOnCameraChange(mapData);
    expect(projectCount).toBe(6);
  });
});
