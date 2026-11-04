import { describe, expect, it, vi } from 'vitest';
import { VehicleRenderer } from './VehicleRenderer';
import type { MapData } from '../bridge/commands';
import type { VehicleState } from '../bridge/events';

describe('VehicleRenderer road group filtering', () => {
  const mockOverlay = {
    groundVehicles: { addChild: vi.fn() },
    frustrationLayer: { addChild: vi.fn() },
    tunnelVehicles: { addChild: vi.fn() },
    bridgeVehicles: { addChild: vi.fn() },
    width: 800,
    height: 600,
    app: { renderer: {} },
  } as any;

  const mockMap = {
    getBounds: () => ({
      getWest: () => 10,
      getEast: () => 20,
      getSouth: () => 50,
      getNorth: () => 60,
    }),
    getCenter: () => ({ lng: 15, lat: 55 }),
    project: (coords: any) => ({ x: coords[0] || coords.lng, y: coords[1] || coords.lat }),
  } as any;

  const mockCamera = {
    getCarVisuals: () => ({ mode: 'dot', spriteScale: 1, dotRadius: 3 }),
  } as any;

  const mockMapData: MapData = {
    nodes: [
      { id: 1, lat: 51.0, lng: 19.0, intersectionType: 'plain' },
      { id: 2, lat: 51.1, lng: 19.1, intersectionType: 'plain' },
      { id: 3, lat: 52.0, lng: 19.0, intersectionType: 'plain' },
      { id: 4, lat: 52.1, lng: 19.1, intersectionType: 'plain' },
    ],
    edges: [
      { from: 1, to: 2, roadType: 'motorway', lanes: 2, infraType: 'normal', lengthM: 100, oneway: true, maxSpeed: 100, layer: 0, laneDirections: [] },
      { from: 3, to: 4, roadType: 'residential', lanes: 1, infraType: 'normal', lengthM: 50, oneway: false, maxSpeed: 50, layer: 0, laneDirections: [] },
    ],
    lanes: [
      {
        id: 101,
        width: 3.5,
        connections: [],
        conflictAreas: [],
        points: [[51.0, 19.0], [51.1, 19.1]],
        lengthM: 100,
        fromNodeOsmId: 1,
        toNodeOsmId: 2,
        laneIndex: 0,
        isConnector: false,
      },
      {
        id: 102,
        width: 3.0,
        connections: [],
        conflictAreas: [],
        points: [[52.0, 19.0], [52.1, 19.1]],
        lengthM: 50,
        fromNodeOsmId: 3,
        toNodeOsmId: 4,
        laneIndex: 0,
        isConnector: false,
      },
    ],
    spawnPoints: [],
    bbox: [18, 50, 20, 53],
    buildings: [],
    restrictions: [],
    tramStops: [],
    turnConnectors: [],
    conflictAreas: [],
  };

  it('populates laneGroupCache and filters vehicles by currentLaneId in O(1)', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    renderer.setEdgeIndex(mockMapData);

    const vehicleMotorway: VehicleState = {
      id: 1,
      lat: 51.05,
      lng: 19.05,
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

    const vehicleResidential: VehicleState = {
      id: 2,
      lat: 52.05,
      lng: 19.05,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: 102,
    };

    // When no groups are hidden, neither vehicle is hidden
    renderer.setHiddenGroups(new Set());
    expect((renderer as any).isVehicleHidden(vehicleMotorway)).toBe(false);
    expect((renderer as any).isVehicleHidden(vehicleResidential)).toBe(false);

    // Hide motorway group
    renderer.setHiddenGroups(new Set(['motorway']));
    expect((renderer as any).isVehicleHidden(vehicleMotorway)).toBe(true);
    expect((renderer as any).isVehicleHidden(vehicleResidential)).toBe(false);

    // Hide residential group
    renderer.setHiddenGroups(new Set(['residential']));
    expect((renderer as any).isVehicleHidden(vehicleMotorway)).toBe(false);
    expect((renderer as any).isVehicleHidden(vehicleResidential)).toBe(true);
  });

  it('falls back to spatial distance calculation when currentLaneId is not in laneGroupCache', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    renderer.setEdgeIndex(mockMapData);
    renderer.setHiddenGroups(new Set(['motorway']));

    const vehicleUnknownLane: VehicleState = {
      id: 3,
      lat: 51.05,
      lng: 19.05,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: null,
    };

    // Position is close to motorway edge (1 -> 2), should resolve to motorway and be hidden
    expect((renderer as any).isVehicleHidden(vehicleUnknownLane)).toBe(true);
  });
});
