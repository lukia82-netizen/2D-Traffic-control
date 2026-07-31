import { vi, describe, it, expect } from 'vitest';
import { VehicleRenderer } from './VehicleRenderer';
import type { MapData } from '../bridge/commands';
import type { VehicleState } from '../bridge/events';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';
import maplibregl from 'maplibre-gl';

// Mock maplibre-gl so we can instantiate the Map
vi.mock('maplibre-gl', () => {
  return {
    default: {
      Map: class {
        constructor(_config: any) {}
        getCenter() {
          return { lng: 0, lat: 0 };
        }
        project(coord: any) {
          return { x: coord.lng * 100, y: coord.lat * 100 };
        }
        getBounds() {
          return {
            getWest() { return -10; },
            getEast() { return 10; },
            getSouth() { return -10; },
            getNorth() { return 10; },
          };
        }
      }
    }
  };
});

// Mock PIXI elements
vi.mock('pixi.js', () => {
  return {
    Graphics: class {
      circle() { return this; }
      fill() { return this; }
      stroke() { return this; }
      clear() { return this; }
      addChild() {}
      removeChild() {}
      destroy() {}
    },
    Container: class {
      addChild() {}
      removeChild() {}
      destroy() {}
    }
  };
});

describe('VehicleRenderer', () => {
  const mockOverlay = {
    groundVehicles: { addChild: vi.fn(), removeChild: vi.fn() },
    tunnelVehicles: { addChild: vi.fn(), removeChild: vi.fn() },
    bridgeVehicles: { addChild: vi.fn(), removeChild: vi.fn() },
    frustrationLayer: { addChild: vi.fn(), removeChild: vi.fn() },
  } as any as PixiOverlay;

  const mockCamera = {
    getCarVisuals: () => ({
      mode: 'dot',
      spriteScale: 1,
      dotRadius: 5,
    }),
  } as any as CameraManager;

  const mockMap = new maplibregl.Map({} as any);

  const sampleMapData: MapData = {
    nodes: [
      { id: 1, lat: 1.0, lng: 1.0, intersectionType: 'none' },
      { id: 2, lat: 1.0, lng: 2.0, intersectionType: 'none' },
      { id: 3, lat: 2.0, lng: 1.0, intersectionType: 'none' },
      { id: 4, lat: 2.0, lng: 2.0, intersectionType: 'none' }
    ],
    edges: [
      {
        from: 1,
        to: 2,
        roadType: 'motorway',
        lanes: 2,
        maxSpeed: 100,
        oneway: true,
        infraType: 'normal',
        layer: 0,
        lengthM: 100,
        laneDirections: ['straight']
      },
      {
        from: 3,
        to: 4,
        roadType: 'residential',
        lanes: 1,
        maxSpeed: 30,
        oneway: true,
        infraType: 'normal',
        layer: 0,
        lengthM: 100,
        laneDirections: ['straight']
      }
    ],
    lanes: [
      {
        id: 100,
        width: 3.5,
        connections: [],
        conflictAreas: [],
        points: [[1.0, 1.0], [1.0, 2.0]],
        lengthM: 100,
        fromNodeOsmId: 1,
        toNodeOsmId: 2,
        laneIndex: 0,
        isConnector: false
      },
      {
        id: 200,
        width: 3.5,
        connections: [],
        conflictAreas: [],
        points: [[2.0, 1.0], [2.0, 2.0]],
        lengthM: 100,
        fromNodeOsmId: 3,
        toNodeOsmId: 4,
        laneIndex: 0,
        isConnector: false
      }
    ],
    spawnPoints: [],
    bbox: [0, 0, 3, 3],
    buildings: [],
    restrictions: [],
    tramStops: [],
    turnConnectors: [],
    conflictAreas: []
  };

  it('populates laneGroupCache on setEdgeIndex and clears on destroy', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    renderer.setEdgeIndex(sampleMapData);

    // Access private laneGroupCache map using any casting
    const cache = (renderer as any).laneGroupCache as Map<number, string>;
    expect(cache.size).toBe(2);
    expect(cache.get(100)).toBe('motorway'); // motorway maps to motorway
    expect(cache.get(200)).toBe('residential'); // residential maps to residential

    renderer.destroy();
    expect(cache.size).toBe(0);
  });

  it('determines vehicle hidden state using laneGroupCache in O(1)', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    renderer.setEdgeIndex(sampleMapData);
    renderer.setHiddenGroups(new Set(['motorway']));

    // Vehicle on motorway lane (id: 100)
    const vehicleOnMotorway: VehicleState = {
      id: 1,
      lat: 1.0,
      lng: 1.5,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      frustration: 0,
      lateralOffset: 0,
      currentLaneId: 100
    };

    // Vehicle on residential lane (id: 200)
    const vehicleOnResidential: VehicleState = {
      id: 2,
      lat: 2.0,
      lng: 1.5,
      angle: 0,
      speed: 5,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      frustration: 0,
      lateralOffset: 0,
      currentLaneId: 200
    };

    // Since 'motorway' is hidden, vehicle 1 should be hidden, vehicle 2 should not be hidden
    expect((renderer as any).isVehicleHidden(vehicleOnMotorway)).toBe(true);
    expect((renderer as any).isVehicleHidden(vehicleOnResidential)).toBe(false);
  });

  it('falls back to spatial search if currentLaneId is null or missing from cache', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    renderer.setEdgeIndex(sampleMapData);
    renderer.setHiddenGroups(new Set(['motorway']));

    // Vehicle near motorway with null lane ID (e.g. at junction or connector fallback)
    const vehicleNearMotorway: VehicleState = {
      id: 3,
      lat: 1.01,
      lng: 1.51,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      frustration: 0,
      lateralOffset: 0,
      currentLaneId: null
    };

    expect((renderer as any).isVehicleHidden(vehicleNearMotorway)).toBe(true);
  });
});
