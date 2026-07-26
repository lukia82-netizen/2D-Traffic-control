import { describe, expect, it, vi, beforeEach } from 'vitest';
import { VehicleRenderer } from './VehicleRenderer';
import type { MapData } from '../bridge/commands';
import type { VehicleState } from '../bridge/events';
import maplibregl from 'maplibre-gl';

// Mock pixi.js to avoid canvas/WebGL requirements in Node
vi.mock('pixi.js', () => {
  const mockGraphics = vi.fn().mockImplementation(() => {
    return {
      addChild: vi.fn(),
      removeChild: vi.fn(),
      clear: vi.fn(),
      circle: vi.fn().mockReturnThis(),
      fill: vi.fn().mockReturnThis(),
      stroke: vi.fn().mockReturnThis(),
      destroy: vi.fn(),
      rect: vi.fn().mockReturnThis(),
      scale: { set: vi.fn() },
      visible: true,
      x: 0,
      y: 0,
      rotation: 0,
      alpha: 1,
    };
  });
  return {
    Graphics: mockGraphics,
  };
});

// Mock maplibre-gl to avoid browser requirements
vi.mock('maplibre-gl', () => {
  const mockMapClass = vi.fn().mockImplementation(() => {
    return {
      getBounds: vi.fn().mockReturnValue({
        getWest: () => -120,
        getEast: () => 120,
        getSouth: () => -60,
        getNorth: () => 60,
      }),
      getCenter: vi.fn().mockReturnValue({ lng: 10, lat: 20 }),
      project: vi.fn().mockReturnValue({ x: 50, y: 50 }),
    };
  });
  return {
    default: {
      Map: mockMapClass,
    },
  };
});

describe('VehicleRenderer', () => {
  let mockOverlay: any;
  let mockMap: any;
  let mockCamera: any;
  let renderer: VehicleRenderer;

  beforeEach(() => {
    mockOverlay = {
      groundVehicles: { addChild: vi.fn() },
      frustrationLayer: { addChild: vi.fn() },
      tunnelVehicles: { addChild: vi.fn() },
      bridgeVehicles: { addChild: vi.fn() },
    };

    // Use empty object cast as any as per maplibre-gl constructor guidelines
    mockMap = new maplibregl.Map({} as any);

    mockCamera = {
      getCarVisuals: vi.fn().mockReturnValue({
        mode: 'dot',
        spriteScale: 1.0,
        dotRadius: 4.0,
      }),
    };

    renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
  });

  it('populates laneGroupCache on setEdgeIndex and resolves O(1) in isVehicleHidden', () => {
    const mapData: MapData = {
      nodes: [
        { id: 101, lat: 10.0, lng: 10.0, intersectionType: 'plain' },
        { id: 102, lat: 10.1, lng: 10.1, intersectionType: 'plain' },
      ],
      edges: [
        {
          from: 101,
          to: 102,
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
      bbox: [9.9, 9.9, 10.2, 10.2],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      conflictAreas: [],
      lanes: [
        {
          id: 5001,
          width: 3.5,
          connections: [],
          conflictAreas: [],
          points: [],
          lengthM: 100,
          fromNodeOsmId: 101,
          toNodeOsmId: 102,
          laneIndex: 0,
          isConnector: false,
        },
      ],
    };

    renderer.setEdgeIndex(mapData);

    // Verify cache has populated lane 5001 -> residential group
    const vehicle: VehicleState = {
      id: 999,
      lat: 10.05,
      lng: 10.05,
      angle: 0.78,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0.0,
      frustration: 0,
      currentLaneId: 5001,
    };

    renderer.setHiddenGroups(new Set(['residential']));

    // With residential hidden, the vehicle on lane 5001 should be resolved as hidden (returns true)
    expect((renderer as any).isVehicleHidden(vehicle)).toBe(true);

    // If residential is not hidden, the vehicle should not be hidden (returns false)
    renderer.setHiddenGroups(new Set([]));
    expect((renderer as any).isVehicleHidden(vehicle)).toBe(false);
  });

  it('falls back to spatial search when lane group is not cached', () => {
    const mapData: MapData = {
      nodes: [
        { id: 101, lat: 10.0, lng: 10.0, intersectionType: 'plain' },
        { id: 102, lat: 10.1, lng: 10.1, intersectionType: 'plain' },
      ],
      edges: [
        {
          from: 101,
          to: 102,
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
      bbox: [9.9, 9.9, 10.2, 10.2],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      conflictAreas: [],
      lanes: [], // No lane group data
    };

    renderer.setEdgeIndex(mapData);

    const vehicle: VehicleState = {
      id: 999,
      lat: 10.05,
      lng: 10.05,
      angle: 0.78,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0.0,
      frustration: 0,
      currentLaneId: 9999, // Uncached lane ID
    };

    renderer.setHiddenGroups(new Set(['residential']));

    // Since lane 9999 is uncached, it falls back to O(E) spatial lookup and resolves to 'residential' edge, thus hidden (true)
    expect((renderer as any).isVehicleHidden(vehicle)).toBe(true);
  });

  it('clears all caches when destroy is called', () => {
    const mapData: MapData = {
      nodes: [
        { id: 101, lat: 10.0, lng: 10.0, intersectionType: 'plain' },
        { id: 102, lat: 10.1, lng: 10.1, intersectionType: 'plain' },
      ],
      edges: [
        {
          from: 101,
          to: 102,
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
      bbox: [9.9, 9.9, 10.2, 10.2],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      conflictAreas: [],
      lanes: [
        {
          id: 5001,
          width: 3.5,
          connections: [],
          conflictAreas: [],
          points: [],
          lengthM: 100,
          fromNodeOsmId: 101,
          toNodeOsmId: 102,
          laneIndex: 0,
          isConnector: false,
        },
      ],
    };

    renderer.setEdgeIndex(mapData);

    const rAny = renderer as any;
    expect(rAny.laneGroupCache.size).toBeGreaterThan(0);

    renderer.destroy();

    expect(rAny.laneGroupCache.size).toBe(0);
    expect(rAny.vehicleGroupCache.size).toBe(0);
    expect(rAny.prevVehiclePos.size).toBe(0);
  });
});
