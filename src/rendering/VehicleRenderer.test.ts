import { describe, expect, it, vi } from 'vitest';
import { VehicleRenderer } from './VehicleRenderer';
import type { MapData } from '../bridge/commands';
import type { VehicleState } from '../bridge/events';

// Mock PIXI.js to avoid WebGL / environment issues in unit tests
vi.mock('pixi.js', () => {
  class Graphics {
    parent = { removeChild: vi.fn() };
    circle = vi.fn().mockReturnThis();
    fill = vi.fn().mockReturnThis();
    stroke = vi.fn().mockReturnThis();
    rect = vi.fn().mockReturnThis();
    clear = vi.fn().mockReturnThis();
    destroy = vi.fn();
    x = 0;
    y = 0;
    rotation = 0;
    scale = { set: vi.fn() };
    alpha = 1;
    visible = true;
  }
  return {
    Graphics,
  };
});

describe('VehicleRenderer', () => {
  const mockMap: any = {
    getCenter: vi.fn(() => ({ lng: 18.6, lat: 54.3 })),
    project: vi.fn((coords: any) => {
      // Coords could be [lng, lat] or an object with {lng, lat}
      const lng = Array.isArray(coords) ? coords[0] : coords.lng;
      const lat = Array.isArray(coords) ? coords[1] : coords.lat;
      return { x: lng * 10, y: lat * 10 };
    }),
    getBounds: vi.fn(() => ({
      getWest: () => 18.5,
      getEast: () => 18.7,
      getSouth: () => 54.2,
      getNorth: () => 54.4,
    })),
  };

  const mockCamera: any = {
    getCarVisuals: vi.fn(() => ({ mode: 'dot', spriteScale: 1.0, dotRadius: 3 })),
  };

  const mockOverlay: any = {
    groundVehicles: { addChild: vi.fn() },
    tunnelVehicles: { addChild: vi.fn() },
    bridgeVehicles: { addChild: vi.fn() },
    frustrationLayer: { addChild: vi.fn() },
  };

  const sampleMapData: MapData = {
    nodes: [
      { id: 1, lat: 54.31, lng: 18.61, intersectionType: 'priority' },
      { id: 2, lat: 54.32, lng: 18.62, intersectionType: 'priority' },
    ],
    edges: [
      {
        from: 1,
        to: 2,
        lanes: 1,
        maxSpeed: 50,
        oneway: true,
        infraType: 'normal',
        layer: 0,
        lengthM: 100,
        roadType: 'motorway', // Maps to 'motorway' group
        laneDirections: ['straight'],
      },
    ],
    spawnPoints: [],
    bbox: [18.5, 54.2, 18.7, 54.4],
    buildings: [],
    restrictions: [],
    tramStops: [],
    turnConnectors: [],
    lanes: [
      {
        id: 101,
        width: 3.5,
        connections: [],
        conflictAreas: [],
        points: [],
        lengthM: 100,
        fromNodeOsmId: 1,
        toNodeOsmId: 2,
        laneIndex: 0,
        isConnector: false,
      },
    ],
    conflictAreas: [],
  };

  it('populates laneGroupCache based on lane and edge node OSM IDs', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    renderer.setEdgeIndex(sampleMapData);

    // Using isVehicleHidden via hiddenGroups to indirectly verify laneGroupCache O(1) resolution
    renderer.setHiddenGroups(new Set(['motorway']));

    const testVehicle: VehicleState = {
      id: 1,
      lat: 54.315,
      lng: 18.615,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: 101,
    };

    // Since the lane 101 maps to 'motorway' group (via edge from 1 to 2 with motorway roadType),
    // and 'motorway' is in hiddenGroups, the vehicle should be hidden.
    const isHidden = (renderer as any).isVehicleHidden(testVehicle);
    expect(isHidden).toBe(true);
  });

  it('falls back to spatial search when currentLaneId is null or missing in cache', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    renderer.setEdgeIndex(sampleMapData);
    renderer.setHiddenGroups(new Set(['motorway']));

    const testVehicle: VehicleState = {
      id: 2,
      lat: 54.315, // Geographically close to edge from 1 to 2
      lng: 18.615,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: null, // triggers fallback to spatial resolve
    };

    const isHidden = (renderer as any).isVehicleHidden(testVehicle);
    expect(isHidden).toBe(true);
  });

  it('clears caches and releases objects on destroy', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    renderer.setEdgeIndex(sampleMapData);

    renderer.destroy();

    // Verify cache has been cleared
    expect((renderer as any).laneGroupCache.size).toBe(0);
    expect((renderer as any).renderedIds.size).toBe(0);
  });
});
