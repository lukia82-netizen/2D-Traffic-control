import { describe, expect, it, vi } from 'vitest';
import { VehicleRenderer } from './VehicleRenderer';
import type { MapData } from '../bridge/commands';
import type { VehicleState } from '../bridge/events';
import maplibregl from 'maplibre-gl';

// Mock maplibre-gl and pixi.js if necessary
vi.mock('maplibre-gl', () => {
  return {
    default: {
      Map: vi.fn().mockImplementation(() => {
        return {
          getCenter: vi.fn().mockReturnValue({ lng: 20.0, lat: 50.0 }),
          project: vi.fn().mockReturnValue({ x: 100, y: 100 }),
          getBounds: vi.fn().mockReturnValue({
            getWest: () => 19.9,
            getEast: () => 20.2,
            getSouth: () => 49.9,
            getNorth: () => 50.2,
          }),
        };
      }),
    },
  };
});

vi.mock('pixi.js', () => {
  return {
    Graphics: vi.fn().mockImplementation(() => {
      return {
        addChild: vi.fn(),
        clear: vi.fn(),
        circle: vi.fn().mockReturnThis(),
        fill: vi.fn().mockReturnThis(),
        stroke: vi.fn().mockReturnThis(),
        rect: vi.fn().mockReturnThis(),
        destroy: vi.fn(),
        parent: { removeChild: vi.fn() },
      };
    }),
  };
});

const mockOverlay = {
  groundVehicles: { addChild: vi.fn() },
  frustrationLayer: { addChild: vi.fn() },
  tunnelVehicles: { addChild: vi.fn() },
  bridgeVehicles: { addChild: vi.fn() },
} as any;

const mockCamera = {
  getCarVisuals: vi.fn().mockReturnValue({
    mode: 'dot',
    spriteScale: 1.0,
    dotRadius: 5,
  }),
} as any;

describe('VehicleRenderer', () => {
  it('populates laneGroupCache and resolves road group via lane ID', () => {
    const mockMap = new maplibregl.Map({} as any);
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);

    const mapData: MapData = {
      nodes: [
        { id: 1, lat: 50.0, lng: 20.0, intersectionType: 'normal' },
        { id: 2, lat: 50.1, lng: 20.1, intersectionType: 'normal' },
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
          roadType: 'primary',
          laneDirections: ['straight'],
        },
      ],
      lanes: [
        {
          id: 42,
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
      spawnPoints: [],
      bbox: [19.9, 49.9, 20.2, 50.2],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      conflictAreas: [],
    };

    renderer.setEdgeIndex(mapData);

    // Verify lane group cache is populated
    expect((renderer as any).laneGroupCache.get(42)).toBe('primary');

    // Test vehicle hidden state resolution when the group is not hidden
    renderer.setHiddenGroups(new Set());
    const v: VehicleState = {
      id: 101,
      lat: 50.05,
      lng: 20.05,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 1,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: 42,
    };
    expect((renderer as any).isVehicleHidden(v)).toBe(false);

    // Test vehicle hidden state resolution when the group is hidden
    renderer.setHiddenGroups(new Set(['primary']));
    expect((renderer as any).isVehicleHidden(v)).toBe(true);
  });

  it('falls back to spatial search when currentLaneId is null', () => {
    const mockMap = new maplibregl.Map({} as any);
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);

    const mapData: MapData = {
      nodes: [
        { id: 1, lat: 50.0, lng: 20.0, intersectionType: 'normal' },
        { id: 2, lat: 50.1, lng: 20.1, intersectionType: 'normal' },
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
          roadType: 'secondary',
          laneDirections: ['straight'],
        },
      ],
      lanes: [],
      spawnPoints: [],
      bbox: [19.9, 49.9, 20.2, 50.2],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      conflictAreas: [],
    };

    renderer.setEdgeIndex(mapData);

    // Test vehicle hidden state resolution with null lane ID (spatial lookup)
    const v: VehicleState = {
      id: 102,
      lat: 50.05,
      lng: 20.05,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 1,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: null,
    };

    renderer.setHiddenGroups(new Set());
    expect((renderer as any).isVehicleHidden(v)).toBe(false);

    renderer.setHiddenGroups(new Set(['secondary']));
    expect((renderer as any).isVehicleHidden(v)).toBe(true);
  });

  it('clears caches on destroy', () => {
    const mockMap = new maplibregl.Map({} as any);
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);

    const mapData: MapData = {
      nodes: [
        { id: 1, lat: 50.0, lng: 20.0, intersectionType: 'normal' },
        { id: 2, lat: 50.1, lng: 20.1, intersectionType: 'normal' },
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
          roadType: 'primary',
          laneDirections: ['straight'],
        },
      ],
      lanes: [
        {
          id: 42,
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
      spawnPoints: [],
      bbox: [19.9, 49.9, 20.2, 50.2],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      conflictAreas: [],
    };

    renderer.setEdgeIndex(mapData);
    expect((renderer as any).laneGroupCache.size).toBeGreaterThan(0);

    renderer.destroy();

    expect((renderer as any).laneGroupCache.size).toBe(0);
    expect((renderer as any).vehicleGroupCache.size).toBe(0);
    expect((renderer as any).prevVehiclePos.size).toBe(0);
    expect((renderer as any).edgeIndex.length).toBe(0);
  });
});
