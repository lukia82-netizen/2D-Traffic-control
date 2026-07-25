import { describe, it, expect, beforeEach, vi } from 'vitest';
import { VehicleRenderer } from './VehicleRenderer';
import type { VehicleState } from '../bridge/events';
import type { MapData } from '../bridge/commands';
import maplibregl from 'maplibre-gl';

// Mock PIXI and MapLibre
vi.mock('pixi.js', () => {
  class MockGraphics {
    rect = vi.fn().mockReturnThis();
    fill = vi.fn().mockReturnThis();
    circle = vi.fn().mockReturnThis();
    stroke = vi.fn().mockReturnThis();
    clear = vi.fn().mockReturnThis();
    scale = { set: vi.fn() };
    visible = true;
    x = 0;
    y = 0;
    rotation = 0;
    alpha = 1;
    destroy = vi.fn();
    parent = { removeChild: vi.fn() };
  }
  return {
    Graphics: MockGraphics,
  };
});

vi.mock('maplibre-gl', () => {
  class MockMap {
    getCenter = vi.fn().mockReturnValue({ lng: 10, lat: 20 });
    project = vi.fn().mockReturnValue({ x: 100, y: 200 });
    getBounds = vi.fn().mockReturnValue({
      getWest: () => 5,
      getEast: () => 15,
      getSouth: () => 15,
      getNorth: () => 25,
    });
  }
  return {
    default: {
      Map: MockMap,
    },
  };
});

// A small mock overlay
const mockOverlay = {
  groundVehicles: { addChild: vi.fn() },
  tunnelVehicles: { addChild: vi.fn() },
  bridgeVehicles: { addChild: vi.fn() },
  frustrationLayer: { addChild: vi.fn() },
} as any;

// A small mock camera
const mockCamera = {
  getCarVisuals: vi.fn().mockReturnValue({ mode: 'dot', spriteScale: 1, dotRadius: 4 }),
} as any;

describe('VehicleRenderer Road-Group Optimization Tests', () => {
  let mapMock: any;
  let renderer: VehicleRenderer;

  beforeEach(() => {
    vi.restoreAllMocks();
    mapMock = new maplibregl.Map({} as any);
    renderer = new VehicleRenderer(mockOverlay, mapMock, mockCamera);
  });

  it('populates and clears the lane group cache during setEdgeIndex and destroy', () => {
    const mockMapData: MapData = {
      nodes: [
        { id: 1, lat: 51.5, lng: -0.1, intersectionType: 'normal' },
        { id: 2, lat: 51.51, lng: -0.11, intersectionType: 'normal' },
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
      bbox: [0, 0, 1, 1],
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

    renderer.setEdgeIndex(mockMapData);

    // Retrieve hidden state for a vehicle mapped to lane 101
    const vehicle: VehicleState = {
      id: 1,
      lat: 51.505,
      lng: -0.105,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: 101, // Mapped in laneGroupCache to motorway's group ('motorway')
    };

    // By default not hidden since we don't have hidden groups
    expect((renderer as any).isVehicleHidden(vehicle)).toBe(false);

    // If motorway is hidden, the vehicle should be hidden instantly via cache
    renderer.setHiddenGroups(new Set(['motorway']));
    expect((renderer as any).isVehicleHidden(vehicle)).toBe(true);

    // Clear and check destroy
    renderer.destroy();
    expect((renderer as any).laneGroupCache.size).toBe(0);
  });

  it('falls back to spatial lookup when currentLaneId is null or missing in cache', () => {
    const mockMapData: MapData = {
      nodes: [
        { id: 1, lat: 51.5, lng: -0.1, intersectionType: 'normal' },
        { id: 2, lat: 51.51, lng: -0.11, intersectionType: 'normal' },
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
      bbox: [0, 0, 1, 1],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      lanes: [], // No lanes means no cache entries
      conflictAreas: [],
    };

    renderer.setEdgeIndex(mockMapData);
    renderer.setHiddenGroups(new Set(['motorway']));

    const vehicleWithNullLane: VehicleState = {
      id: 2,
      lat: 51.505,
      lng: -0.105,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: null, // should fall back to spatial
    };

    expect((renderer as any).isVehicleHidden(vehicleWithNullLane)).toBe(true);

    const vehicleWithUnknownLane: VehicleState = {
      id: 3,
      lat: 51.505,
      lng: -0.105,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: 999, // Unknown lane ID, should fall back to spatial
    };

    expect((renderer as any).isVehicleHidden(vehicleWithUnknownLane)).toBe(true);
  });
});
