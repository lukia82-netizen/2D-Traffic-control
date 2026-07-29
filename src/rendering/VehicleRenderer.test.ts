import { describe, expect, it, vi } from 'vitest';
import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import { VehicleRenderer } from './VehicleRenderer';
import type { MapData } from '../bridge/commands';
import type { VehicleState } from '../bridge/events';

// Mock maplibre-gl and pixi.js
vi.mock('maplibre-gl', () => {
  return {
    default: {
      Map: vi.fn().mockImplementation(() => {
        return {
          getCenter: vi.fn().mockReturnValue({ lng: 21.0122, lat: 52.2297 }),
          project: vi.fn().mockReturnValue({ x: 100, y: 100 }),
          getBounds: vi.fn().mockReturnValue({
            getWest: vi.fn().mockReturnValue(20.0),
            getEast: vi.fn().mockReturnValue(22.0),
            getSouth: vi.fn().mockReturnValue(51.0),
            getNorth: vi.fn().mockReturnValue(53.0),
          }),
        };
      }),
    },
  };
});

vi.mock('pixi.js', () => {
  const Container = vi.fn().mockImplementation(() => {
    return {
      addChild: vi.fn(),
      removeChild: vi.fn(),
    };
  });
  const Graphics = vi.fn().mockImplementation(() => {
    return {
      clear: vi.fn().mockReturnThis(),
      circle: vi.fn().mockReturnThis(),
      fill: vi.fn().mockReturnThis(),
      stroke: vi.fn().mockReturnThis(),
      rect: vi.fn().mockReturnThis(),
      destroy: vi.fn(),
      parent: {
        removeChild: vi.fn(),
      },
    };
  });
  return {
    Container,
    Graphics,
  };
});

describe('VehicleRenderer Lane to Road Group Caching', () => {
  it('correctly maps lanes to road groups and resolves hidden state', async () => {
    const mockMap = new maplibregl.Map({} as any);
    const mockOverlay = {
      groundVehicles: new PIXI.Container(),
      tunnelVehicles: new PIXI.Container(),
      bridgeVehicles: new PIXI.Container(),
      frustrationLayer: new PIXI.Container(),
    } as any;
    const mockCamera = {
      getCarVisuals: vi.fn().mockReturnValue({ mode: 'dot', spriteScale: 1, dotRadius: 4 }),
    } as any;

    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    await renderer.init();

    const mapData: MapData = {
      nodes: [
        { id: 1, lat: 52.2, lng: 21.0, intersectionType: 'normal' },
        { id: 2, lat: 52.3, lng: 21.1, intersectionType: 'normal' },
        { id: 3, lat: 40.2, lng: 40.0, intersectionType: 'normal' },
        { id: 4, lat: 40.3, lng: 40.1, intersectionType: 'normal' },
      ],
      edges: [
        {
          from: 1,
          to: 2,
          lanes: 2,
          maxSpeed: 50,
          oneway: true,
          infraType: 'normal',
          layer: 0,
          lengthM: 100,
          roadType: 'motorway',
          laneDirections: ['straight', 'straight'],
        },
        {
          from: 3,
          to: 4,
          lanes: 1,
          maxSpeed: 30,
          oneway: true,
          infraType: 'normal',
          layer: 0,
          lengthM: 100,
          roadType: 'residential',
          laneDirections: ['straight'],
        },
      ],
      spawnPoints: [],
      bbox: [20, 51, 22, 53],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
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
        {
          id: 43,
          width: 3.5,
          connections: [],
          conflictAreas: [],
          points: [],
          lengthM: 100,
          fromNodeOsmId: 3,
          toNodeOsmId: 4,
          laneIndex: 0,
          isConnector: false,
        },
      ],
      conflictAreas: [],
    };

    renderer.setEdgeIndex(mapData);

    // Set motorway as hidden, but residential remains visible
    renderer.setHiddenGroups(new Set(['motorway']));

    const visibleVehicle: VehicleState = {
      id: 101,
      lat: 40.25,
      lng: 40.05,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 1,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: 43, // Residential lane ID -> should be visible
    };

    const hiddenVehicle: VehicleState = {
      id: 102,
      lat: 52.25,
      lng: 21.05,
      angle: 0,
      speed: 10,
      vehicleType: 0,
      driverProfile: 1,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      lateralOffset: 0,
      frustration: 0,
      currentLaneId: 42, // Pre-mapped lane ID (motorway -> hidden)
    };

    const isHiddenFn = (renderer as any).isVehicleHidden.bind(renderer);

    expect(isHiddenFn(hiddenVehicle)).toBe(true);
    expect(isHiddenFn(visibleVehicle)).toBe(false);

    // After clearing / destroying, the cache should be cleared
    renderer.destroy();
  });
});
