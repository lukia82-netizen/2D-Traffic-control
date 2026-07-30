import { describe, it, expect, vi } from 'vitest';
import { VehicleRenderer } from './VehicleRenderer';
import type { MapData } from '../bridge/commands';
import type { VehicleState } from '../bridge/events';

// Mock PIXI and MapLibre
vi.mock('pixi.js', () => {
  class Graphics {
    circle = vi.fn().mockReturnThis();
    rect = vi.fn().mockReturnThis();
    fill = vi.fn().mockReturnThis();
    stroke = vi.fn().mockReturnThis();
    clear = vi.fn().mockReturnThis();
    destroy = vi.fn();
    scale = { set: vi.fn() };
    parent = { removeChild: vi.fn() };
    visible = true;
    x = 0;
    y = 0;
    rotation = 0;
    alpha = 1;
  }
  return {
    Graphics,
  };
});

vi.mock('maplibre-gl', () => {
  return {
    default: {
      Map: class {
        getBounds = vi.fn().mockReturnValue({
          getWest: () => 10,
          getEast: () => 20,
          getSouth: () => 10,
          getNorth: () => 20,
        });
        project = vi.fn().mockReturnValue({ x: 100, y: 100 });
        getCenter = vi.fn().mockReturnValue({ lng: 15, lat: 15 });
      },
    },
  };
});

describe('VehicleRenderer - laneGroupCache & isVehicleHidden', () => {
  const mockOverlay = {
    groundVehicles: { addChild: vi.fn() },
    tunnelVehicles: { addChild: vi.fn() },
    bridgeVehicles: { addChild: vi.fn() },
    frustrationLayer: { addChild: vi.fn() },
  } as any;

  const mockMap = {
    getBounds: vi.fn().mockReturnValue({
      getWest: () => 10,
      getEast: () => 20,
      getSouth: () => 10,
      getNorth: () => 20,
    }),
    project: vi.fn().mockReturnValue({ x: 100, y: 100 }),
    getCenter: vi.fn().mockReturnValue({ lng: 15, lat: 15 }),
  } as any;

  const mockCamera = {
    getCarVisuals: vi.fn().mockReturnValue({
      mode: 'dot',
      spriteScale: 1.0,
      dotRadius: 4.0,
    }),
  } as any;

  it('correctly maps lanes to road groups and leverages cache in isVehicleHidden', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);

    // Mock map data with edges and lanes
    const mapData: MapData = {
      nodes: [
        { id: 1, lat: 15.0, lng: 15.0, intersectionType: 'none' },
        { id: 2, lat: 15.01, lng: 15.01, intersectionType: 'none' },
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
      bbox: [10, 10, 20, 20],
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

    renderer.setEdgeIndex(mapData);

    // Initial hidden groups is empty, so nothing hidden
    const vehicle: VehicleState = {
      id: 1,
      lat: 15.005,
      lng: 15.005,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: 101, // Corresponds to the lane in cache
    };

    // No groups hidden, should not be hidden
    expect((renderer as any).isVehicleHidden(vehicle)).toBe(false);

    // Set 'motorway' as hidden
    renderer.setHiddenGroups(new Set(['motorway']));

    // Now the vehicle should be hidden because lane 101 -> motorway group
    expect((renderer as any).isVehicleHidden(vehicle)).toBe(true);

    // If currentLaneId is null, it should fall back to spatial lookup
    const vehicleWithoutLane: VehicleState = {
      ...vehicle,
      currentLaneId: null,
    };

    // Spatial lookup finds the edge with 'motorway' roadType
    expect((renderer as any).isVehicleHidden(vehicleWithoutLane)).toBe(true);
  });

  it('correctly clears laneGroupCache on destroy', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);

    const mapData: MapData = {
      nodes: [
        { id: 1, lat: 15.0, lng: 15.0, intersectionType: 'none' },
        { id: 2, lat: 15.01, lng: 15.01, intersectionType: 'none' },
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
          roadType: 'motorway',
          laneDirections: ['straight'],
        },
      ],
      spawnPoints: [],
      bbox: [10, 10, 20, 20],
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

    renderer.setEdgeIndex(mapData);
    expect((renderer as any).laneGroupCache.size).toBe(1);

    renderer.destroy();
    expect((renderer as any).laneGroupCache.size).toBe(0);
  });
});
