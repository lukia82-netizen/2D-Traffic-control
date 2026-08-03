import { describe, expect, it, vi, beforeEach } from 'vitest';
import maplibregl from 'maplibre-gl';
import { VehicleRenderer } from './VehicleRenderer';
import type { MapData } from '../bridge/commands';
import type { VehicleState } from '../bridge/events';

// Mock PIXI.js
vi.mock('pixi.js', () => {
  const Graphics = vi.fn().mockImplementation(() => {
    return {
      circle: vi.fn().mockReturnThis(),
      rect: vi.fn().mockReturnThis(),
      fill: vi.fn().mockReturnThis(),
      stroke: vi.fn().mockReturnThis(),
      clear: vi.fn().mockReturnThis(),
      destroy: vi.fn(),
      addChild: vi.fn(),
      removeChild: vi.fn(),
      parent: { removeChild: vi.fn() },
      visible: true,
      scale: { set: vi.fn() },
      x: 0,
      y: 0,
      rotation: 0,
      alpha: 1,
    };
  });
  return {
    Graphics,
  };
});

// Mock maplibre-gl
vi.mock('maplibre-gl', () => {
  const Map = vi.fn().mockImplementation(() => {
    return {
      getCenter: vi.fn().mockReturnValue({ lng: 10, lat: 20 }),
      project: vi.fn().mockReturnValue({ x: 100, y: 200 }),
      getBounds: vi.fn().mockReturnValue({
        getWest: vi.fn().mockReturnValue(5),
        getEast: vi.fn().mockReturnValue(15),
        getSouth: vi.fn().mockReturnValue(15),
        getNorth: vi.fn().mockReturnValue(25),
      }),
    };
  });
  return {
    default: {
      Map,
    },
  };
});

describe('VehicleRenderer Lane Group Optimization', () => {
  let mockOverlay: any;
  let mockMap: any;
  let mockCamera: any;
  let renderer: VehicleRenderer;

  beforeEach(() => {
    mockOverlay = {
      groundVehicles: {
        addChild: vi.fn(),
      },
      tunnelVehicles: {
        addChild: vi.fn(),
      },
      bridgeVehicles: {
        addChild: vi.fn(),
      },
      frustrationLayer: {
        addChild: vi.fn(),
      },
    };

    mockMap = new maplibregl.Map({} as any);

    mockCamera = {
      getCarVisuals: vi.fn().mockReturnValue({
        mode: 'dot',
        spriteScale: 1.0,
        dotRadius: 4.0,
      }),
    };

    renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    renderer.init();
  });

  const mockMapData: MapData = {
    nodes: [
      { id: 1, lat: 51.8, lng: 16.8, intersectionType: 'normal' },
      { id: 2, lat: 51.81, lng: 16.81, intersectionType: 'normal' },
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
    bbox: [0, 0, 0, 0],
    buildings: [],
    restrictions: [],
    tramStops: [],
    turnConnectors: [],
    conflictAreas: [],
  };

  const vehicleState: VehicleState = {
    id: 1001,
    lat: 51.805,
    lng: 16.805,
    angle: 0,
    speed: 10,
    vehicleType: 0,
    driverProfile: 1,
    tripKind: 0,
    currentLane: 0,
    onTurnConnector: false,
    frustration: 0,
    lateralOffset: 0,
    currentLaneId: 42,
  };

  it('populates laneGroupCache on setEdgeIndex', () => {
    renderer.setEdgeIndex(mockMapData);

    const laneGroupCache = (renderer as any).laneGroupCache;
    expect(laneGroupCache.get(42)).toBe('motorway');
  });

  it('resolves road group using laneGroupCache and hides vehicle if group is in hiddenGroups', () => {
    renderer.setEdgeIndex(mockMapData);
    renderer.setHiddenGroups(new Set(['motorway']));

    const isHidden = (renderer as any).isVehicleHidden(vehicleState);
    expect(isHidden).toBe(true);
  });

  it('falls back to spatial search if lane ID is not in laneGroupCache', () => {
    renderer.setEdgeIndex(mockMapData);
    renderer.setHiddenGroups(new Set(['motorway']));

    const unknownVehicle = {
      ...vehicleState,
      currentLaneId: 9999, // not in laneGroupCache
    };

    const isHidden = (renderer as any).isVehicleHidden(unknownVehicle);
    expect(isHidden).toBe(true); // Should resolve to 'motorway' via spatial proximity to node 1 -> node 2 edge
  });

  it('clears caches and state on destroy', () => {
    renderer.setEdgeIndex(mockMapData);
    renderer.setHiddenGroups(new Set(['motorway']));

    // Trigger spatial cache too
    const unknownVehicle = {
      ...vehicleState,
      currentLaneId: null,
    };
    (renderer as any).isVehicleHidden(unknownVehicle);

    expect((renderer as any).laneGroupCache.size).toBeGreaterThan(0);
    expect((renderer as any).vehicleGroupCache.size).toBeGreaterThan(0);
    expect((renderer as any).prevVehiclePos.size).toBeGreaterThan(0);

    renderer.destroy();

    expect((renderer as any).laneGroupCache.size).toBe(0);
    expect((renderer as any).vehicleGroupCache.size).toBe(0);
    expect((renderer as any).prevVehiclePos.size).toBe(0);
  });
});
