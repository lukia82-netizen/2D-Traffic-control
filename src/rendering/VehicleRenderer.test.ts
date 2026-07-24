import { describe, expect, it } from 'vitest';
import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import { VehicleRenderer } from './VehicleRenderer';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';
import type { MapData, NodeData, EdgeData, LaneData } from '../bridge/commands';
import type { VehicleState } from '../bridge/events';

// Create a helper to construct dummy nodes/edges/lanes
function createNode(id: number, lat: number, lng: number): NodeData {
  return { id, lat, lng, intersectionType: 'none' };
}

function createEdge(from: number, to: number, roadType: string): EdgeData {
  return {
    from,
    to,
    lanes: 1,
    maxSpeed: 50,
    oneway: true,
    infraType: 'normal',
    layer: 0,
    lengthM: 100,
    roadType,
    laneDirections: ['straight'],
  };
}

function createLane(id: number, fromNodeOsmId: number, toNodeOsmId: number): LaneData {
  return {
    id,
    width: 3.5,
    connections: [],
    conflictAreas: [],
    points: [],
    lengthM: 100,
    fromNodeOsmId,
    toNodeOsmId,
    laneIndex: 0,
    isConnector: false,
  };
}

function createMockMap(): maplibregl.Map {
  return {
    getCenter: () => ({ lng: 16.575, lat: 51.845 }),
    project: (_coords: any) => ({ x: 100, y: 100 }),
    getBounds: () => ({
      getWest: () => 16.57,
      getEast: () => 16.58,
      getSouth: () => 51.84,
      getNorth: () => 51.85,
    }),
  } as unknown as maplibregl.Map;
}

function createMockOverlay(): PixiOverlay {
  return {
    groundVehicles: {
      addChild: (_child: PIXI.Container) => {},
      removeChild: (_child: PIXI.Container) => {},
    },
    tunnelVehicles: {
      addChild: (_child: PIXI.Container) => {},
      removeChild: (_child: PIXI.Container) => {},
    },
    bridgeVehicles: {
      addChild: (_child: PIXI.Container) => {},
      removeChild: (_child: PIXI.Container) => {},
    },
    frustrationLayer: {
      addChild: (_child: PIXI.Container) => {},
      removeChild: (_child: PIXI.Container) => {},
    },
  } as unknown as PixiOverlay;
}

function createMockCamera(): CameraManager {
  return {
    getCarVisuals: () => ({ mode: 'dot', spriteScale: 1, dotRadius: 2 }),
    zoom: 15,
  } as unknown as CameraManager;
}

describe('VehicleRenderer Lane to Road Group Mapping & Cache', () => {
  it('should correctly map lanes to road groups during setEdgeIndex', () => {
    const map: maplibregl.Map = createMockMap();
    const overlay: PixiOverlay = createMockOverlay();
    const camera: CameraManager = createMockCamera();

    const renderer = new VehicleRenderer(overlay, map, camera);

    // Create MapData with primary and residential roads
    const nodes = [
      createNode(1, 51.841, 16.571),
      createNode(2, 51.842, 16.572),
      createNode(3, 51.843, 16.573),
      createNode(4, 51.844, 16.574),
    ];
    const edges = [
      createEdge(1, 2, 'primary'),
      createEdge(3, 4, 'residential'),
    ];
    const lanes = [
      createLane(1001, 1, 2),
      createLane(2002, 3, 4),
    ];

    const mapData: MapData = {
      nodes,
      edges,
      lanes,
      spawnPoints: [],
      bbox: [16.57, 51.84, 16.58, 51.85],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      conflictAreas: [],
    };

    renderer.setEdgeIndex(mapData);

    const laneGroupCache = (renderer as any).laneGroupCache as Map<number, string>;
    expect(laneGroupCache).toBeDefined();
    expect(laneGroupCache.get(1001)).toBe('primary');
    expect(laneGroupCache.get(2002)).toBe('residential');
  });

  it('should resolve hidden state correctly via laneGroupCache when currentLaneId is present', () => {
    const map: maplibregl.Map = createMockMap();
    const overlay: PixiOverlay = createMockOverlay();
    const camera: CameraManager = createMockCamera();

    const renderer = new VehicleRenderer(overlay, map, camera);

    const nodes = [
      createNode(1, 51.841, 16.571),
      createNode(2, 51.842, 16.572),
      createNode(3, 10.0, 10.0),
      createNode(4, 10.1, 10.1),
    ];
    const edges = [
      createEdge(1, 2, 'primary'),
      createEdge(3, 4, 'residential'),
    ];
    const lanes = [
      createLane(1001, 1, 2),
      createLane(2002, 3, 4),
    ];

    const mapData: MapData = {
      nodes,
      edges,
      lanes,
      spawnPoints: [],
      bbox: [16.57, 51.84, 16.58, 51.85],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      conflictAreas: [],
    };

    renderer.setEdgeIndex(mapData);

    // Hide primary road group
    renderer.setHiddenGroups(new Set(['primary']));

    const v1: VehicleState = {
      id: 42,
      lat: 51.8415,
      lng: 16.5715,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: 1001, // matches the primary lane
    };

    const isHidden1 = (renderer as any).isVehicleHidden(v1);
    expect(isHidden1).toBe(true);

    const v2: VehicleState = {
      ...v1,
      id: 43,
      currentLaneId: 9999, // unknown lane id, should fall back to spatial
    };

    // Since v2 coordinate (51.8415, 16.5715) is very close to primary edge (1,2),
    // spatial resolution should also resolve it to 'primary' and hide it.
    const isHidden2 = (renderer as any).isVehicleHidden(v2);
    expect(isHidden2).toBe(true);

    // Put v3 very close to edge (3,4) (residential), spatial check should resolve to residential
    const v3: VehicleState = {
      ...v1,
      id: 44,
      lat: 10.05,
      lng: 10.05,
      currentLaneId: null, // no lane id, should fall back to spatial
    };
    const isHidden3 = (renderer as any).isVehicleHidden(v3);
    // Since primary is hidden, but residential is not, it should NOT be hidden.
    expect(isHidden3).toBe(false);
  });

  it('should clear caches upon calling destroy', () => {
    const map: maplibregl.Map = createMockMap();
    const overlay: PixiOverlay = createMockOverlay();
    const camera: CameraManager = createMockCamera();

    const renderer = new VehicleRenderer(overlay, map, camera);

    const nodes = [createNode(1, 51.841, 16.571), createNode(2, 51.842, 16.572)];
    const edges = [createEdge(1, 2, 'primary')];
    const lanes = [createLane(1001, 1, 2)];

    const mapData: MapData = {
      nodes,
      edges,
      lanes,
      spawnPoints: [],
      bbox: [16.57, 51.84, 16.58, 51.85],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      conflictAreas: [],
    };

    renderer.setEdgeIndex(mapData);

    const laneGroupCache = (renderer as any).laneGroupCache as Map<number, string>;
    expect(laneGroupCache.size).toBeGreaterThan(0);

    renderer.destroy();

    expect(laneGroupCache.size).toBe(0);
  });
});
