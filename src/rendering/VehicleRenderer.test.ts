import { describe, expect, it, vi } from 'vitest';
import { VehicleRenderer } from './VehicleRenderer';
import type { MapData } from '../bridge/commands';
import type { VehicleState } from '../bridge/events';

describe('VehicleRenderer Road-Group Filtering and Cache Lookups', () => {
  it('correctly populates laneGroupCache and resolves groups', () => {
    // Mock PixiOverlay
    const mockContainer = {
      addChild: vi.fn(),
      removeChild: vi.fn(),
      destroy: vi.fn(),
    };
    const mockOverlay = {
      groundVehicles: mockContainer,
      tunnelVehicles: mockContainer,
      bridgeVehicles: mockContainer,
      frustrationLayer: mockContainer,
    } as any;

    // Mock maplibre map
    const mockMap = {
      getCenter: vi.fn(() => ({ lng: 21.0122, lat: 52.2297 })),
      project: vi.fn(() => ({ x: 100, y: 100 })),
      getBounds: vi.fn(() => ({
        getWest: () => 21.0,
        getEast: () => 21.1,
        getSouth: () => 52.2,
        getNorth: () => 52.3,
      })),
    } as any;

    // Mock camera
    const mockCamera = {
      getCarVisuals: vi.fn(() => ({
        mode: 'dot',
        spriteScale: 1,
        dotRadius: 5,
      })),
    } as any;

    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    renderer.init();

    // Mock MapData
    const mockMapData: MapData = {
      nodes: [
        { id: 1, lat: 52.2290, lng: 21.0120, intersectionType: 'traffic_light' },
        { id: 2, lat: 52.2300, lng: 21.0130, intersectionType: 'priority' },
      ],
      edges: [
        {
          from: 1,
          to: 2,
          lanes: 1,
          maxSpeed: 50,
          oneway: true,
          infraType: 'road',
          layer: 0,
          lengthM: 100,
          roadType: 'primary',
          laneDirections: ['straight'],
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
      ],
      spawnPoints: [],
      bbox: [21.0, 52.2, 21.1, 52.3],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      conflictAreas: [],
    };

    renderer.setEdgeIndex(mockMapData);

    // Verify cache is populated
    const laneGroupCache = (renderer as any).laneGroupCache;
    expect(laneGroupCache).toBeDefined();
    expect(laneGroupCache.get(101)).toBe('primary');

    // Test hidden logic
    renderer.setHiddenGroups(new Set(['primary']));
    const vehicle: VehicleState = {
      id: 42,
      lat: 52.2295,
      lng: 21.0125,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      frustration: 0,
      lateralOffset: 0,
      currentLaneId: 101,
    };

    // Since currentLaneId matches the hidden group, isVehicleHidden should return true.
    const isHidden = (renderer as any).isVehicleHidden(vehicle);
    expect(isHidden).toBe(true);

    // Clean up
    renderer.destroy();
    expect(laneGroupCache.size).toBe(0);
  });
});
