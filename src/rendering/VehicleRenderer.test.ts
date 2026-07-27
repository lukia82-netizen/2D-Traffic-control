import { describe, expect, it, vi, beforeEach } from 'vitest';
import maplibregl from 'maplibre-gl';
import { VehicleRenderer } from './VehicleRenderer';
import type { MapData } from '../bridge/commands';
import type { VehicleState } from '../bridge/events';

vi.mock('maplibre-gl', () => {
  return {
    default: {
      Map: vi.fn().mockImplementation(() => {
        return {
          getBounds: vi.fn().mockReturnValue({
            getWest: () => 10,
            getEast: () => 20,
            getSouth: () => 10,
            getNorth: () => 20,
          }),
          getCenter: vi.fn().mockReturnValue({ lng: 15, lat: 15 }),
          project: vi.fn().mockReturnValue({ x: 100, y: 100 }),
        };
      }),
    },
  };
});

vi.mock('pixi.js', () => {
  class MockGraphics {
    scale = { set: vi.fn() };
    parent = { removeChild: vi.fn() };
    visible = true;
    x = 0;
    y = 0;
    rotation = 0;
    alpha = 1;

    clear = vi.fn().mockReturnThis();
    circle = vi.fn().mockReturnThis();
    fill = vi.fn().mockReturnThis();
    stroke = vi.fn().mockReturnThis();
    rect = vi.fn().mockReturnThis();
    destroy = vi.fn();
  }

  return {
    Graphics: MockGraphics,
  };
});

describe('VehicleRenderer', () => {
  let map: maplibregl.Map;
  let overlay: any;
  let camera: any;
  let renderer: VehicleRenderer;

  beforeEach(async () => {
    map = new maplibregl.Map({} as any);
    overlay = {
      groundVehicles: { addChild: vi.fn() },
      tunnelVehicles: { addChild: vi.fn() },
      bridgeVehicles: { addChild: vi.fn() },
      frustrationLayer: { addChild: vi.fn() },
    };
    camera = {
      getCarVisuals: vi.fn().mockReturnValue({
        mode: 'dot',
        spriteScale: 1.0,
        dotRadius: 3,
      }),
    };
    renderer = new VehicleRenderer(overlay, map, camera);
    await renderer.init();
  });

  const mockMapData: MapData = {
    nodes: [
      { id: 1, lat: 15, lng: 15, intersectionType: 'none' },
      { id: 2, lat: 16, lng: 16, intersectionType: 'none' },
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
        roadType: 'residential',
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
        id: 100,
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

  it('populates laneGroupCache mapping lane ID to road group', () => {
    renderer.setEdgeIndex(mockMapData);

    const laneGroupCache = (renderer as any).laneGroupCache as Map<number, string>;
    expect(laneGroupCache).toBeDefined();
    expect(laneGroupCache.get(100)).toBe('residential');
  });

  it('resolves hidden state using O(1) laneGroupCache with currentLaneId', () => {
    renderer.setEdgeIndex(mockMapData);

    const vehicle: VehicleState = {
      id: 42,
      lat: 15.5,
      lng: 15.5,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: 100,
    };

    // Initially no hidden groups
    expect(renderer['isVehicleHidden'](vehicle)).toBe(false);

    // Set hidden groups to include 'residential'
    renderer.setHiddenGroups(new Set(['residential']));
    expect(renderer['isVehicleHidden'](vehicle)).toBe(true);

    // Unknown lane ID should fallback to spatial search
    const unknownLaneVehicle: VehicleState = {
      ...vehicle,
      currentLaneId: 999, // Should trigger spatial search
    };
    expect(renderer['isVehicleHidden'](unknownLaneVehicle)).toBe(true);
  });

  it('falls back to spatial search when currentLaneId is null', () => {
    renderer.setEdgeIndex(mockMapData);
    renderer.setHiddenGroups(new Set(['residential']));

    const vehicle: VehicleState = {
      id: 42,
      lat: 15.5,
      lng: 15.5,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: null, // Forces spatial search fallback
    };

    expect(renderer['isVehicleHidden'](vehicle)).toBe(true);
  });

  it('clears cache on destroy and setEdgeIndex', () => {
    renderer.setEdgeIndex(mockMapData);
    expect((renderer as any).laneGroupCache.size).toBeGreaterThan(0);

    renderer.destroy();
    expect((renderer as any).laneGroupCache.size).toBe(0);

    // Reinitialize & check setEdgeIndex clearing behavior
    renderer.setEdgeIndex(mockMapData);
    expect((renderer as any).laneGroupCache.size).toBeGreaterThan(0);

    // Empty map data
    renderer.setEdgeIndex({ ...mockMapData, lanes: [] });
    expect((renderer as any).laneGroupCache.size).toBe(0);
  });
});
