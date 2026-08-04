import { describe, expect, it } from 'vitest';
import { VehicleRenderer } from './VehicleRenderer';
import type { MapData } from '../bridge/commands';
import type { VehicleState } from '../bridge/events';

describe('VehicleRenderer', () => {
  const mockOverlay = {} as any;
  const mockMap = {} as any;
  const mockCamera = {} as any;

  const createMockMapData = (): MapData => ({
    nodes: [
      { id: 1, lat: 50.0, lng: 20.0, intersectionType: 'Plain' },
      { id: 2, lat: 50.01, lng: 20.01, intersectionType: 'Plain' },
      { id: 3, lat: 50.02, lng: 20.02, intersectionType: 'Plain' },
    ],
    edges: [
      {
        from: 1,
        to: 2,
        roadType: 'motorway',
        lanes: 1,
        maxSpeed: 120,
        oneway: true,
        infraType: 'normal',
        layer: 0,
        lengthM: 100,
        laneDirections: [],
      },
      {
        from: 2,
        to: 3,
        roadType: 'primary',
        lanes: 1,
        maxSpeed: 70,
        oneway: true,
        infraType: 'normal',
        layer: 0,
        lengthM: 100,
        laneDirections: [],
      },
    ],
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
      {
        id: 102,
        width: 3.5,
        connections: [],
        conflictAreas: [],
        points: [],
        lengthM: 100,
        fromNodeOsmId: 2,
        toNodeOsmId: 3,
        laneIndex: 0,
        isConnector: false,
      },
    ],
    spawnPoints: [],
    bbox: [0, 0, 0, 0],
    buildings: [],
    restrictions: [],
    tramStops: [],
    turnConnectors: [],
    conflictAreas: [],
  });

  const createMockVehicle = (id: number, currentLaneId: number | null, lat = 50.005, lng = 20.005): VehicleState => ({
    id,
    lat,
    lng,
    angle: 0,
    speed: 0,
    vehicleType: 0,
    driverProfile: 0,
    tripKind: 0,
    currentLane: 0,
    onTurnConnector: false,
    frustration: 0,
    lateralOffset: 0,
    currentLaneId,
  });

  it('populates laneGroupCache correctly from edges and lanes', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    const mapData = createMockMapData();

    renderer.setEdgeIndex(mapData);

    const laneGroupCache = renderer['laneGroupCache'];
    expect(laneGroupCache.get(101)).toBe('motorway');
    expect(laneGroupCache.get(102)).toBe('primary');
  });

  it('hides vehicles in hidden groups using O(1) laneGroupCache lookup', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    const mapData = createMockMapData();

    renderer.setEdgeIndex(mapData);
    renderer.setHiddenGroups(new Set(['motorway']));

    const v1 = createMockVehicle(1, 101); // in motorway, should be hidden
    const v2 = createMockVehicle(2, 102); // in primary, should not be hidden

    expect(renderer['isVehicleHidden'](v1)).toBe(true);
    expect(renderer['isVehicleHidden'](v2)).toBe(false);
  });

  it('falls back to spatial lookup when currentLaneId is null', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    const mapData = createMockMapData();

    renderer.setEdgeIndex(mapData);
    renderer.setHiddenGroups(new Set(['motorway']));

    // Vehicle at 50.005, 20.005 is closer to edge 1-2 (motorway)
    const v1 = createMockVehicle(1, null, 50.005, 20.005);
    // Vehicle at 50.015, 20.015 is closer to edge 2-3 (primary)
    const v2 = createMockVehicle(2, null, 50.015, 20.015);

    expect(renderer['isVehicleHidden'](v1)).toBe(true);
    expect(renderer['isVehicleHidden'](v2)).toBe(false);
  });

  it('falls back to spatial lookup when currentLaneId is not in laneGroupCache', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    const mapData = createMockMapData();

    renderer.setEdgeIndex(mapData);
    renderer.setHiddenGroups(new Set(['motorway']));

    // Vehicle with lane ID 999 (not in cache), but physically on motorway segment
    const v = createMockVehicle(1, 999, 50.005, 20.005);

    expect(renderer['isVehicleHidden'](v)).toBe(true);
  });

  it('clears laneGroupCache on destroy', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    const mapData = createMockMapData();

    renderer.setEdgeIndex(mapData);
    expect(renderer['laneGroupCache'].size).toBe(2);

    renderer.destroy();
    expect(renderer['laneGroupCache'].size).toBe(0);
  });
});
