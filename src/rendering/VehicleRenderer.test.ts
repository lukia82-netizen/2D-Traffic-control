import { vi, describe, expect, it } from 'vitest';
import maplibregl from 'maplibre-gl';
import { VehicleRenderer } from './VehicleRenderer';
import type { MapData } from '../bridge/commands';
import type { VehicleState } from '../bridge/events';

vi.mock('maplibre-gl', () => {
  return {
    default: {
      Map: class {
        constructor() {}
        getCenter() {
          return { lng: 21.0122, lat: 52.2297 };
        }
        project(lngLat: [number, number]) {
          return { x: lngLat[0] * 100, y: lngLat[1] * 100 };
        }
        getBounds() {
          return {
            getWest: () => -180,
            getEast: () => 180,
            getSouth: () => -90,
            getNorth: () => 90,
          };
        }
      }
    }
  };
});

vi.mock('pixi.js', () => {
  class MockGraphics {
    parent = null;
    x = 0;
    y = 0;
    rotation = 0;
    scale = { set: () => {} };
    alpha = 1;
    visible = true;
    circle() { return this; }
    rect() { return this; }
    fill() { return this; }
    stroke() { return this; }
    clear() { return this; }
    destroy() {}
  }
  return {
    Graphics: MockGraphics,
  };
});

describe('VehicleRenderer Road-Group Filtering', () => {
  const mockOverlay = {
    groundVehicles: { addChild: () => {}, removeChild: () => {} },
    tunnelVehicles: { addChild: () => {}, removeChild: () => {} },
    bridgeVehicles: { addChild: () => {}, removeChild: () => {} },
    frustrationLayer: { addChild: () => {}, removeChild: () => {} }
  } as any;

  const mockCamera = {
    getCarVisuals: () => ({ mode: 'dot', spriteScale: 1, dotRadius: 3 })
  } as any;

  const mapData: MapData = {
    nodes: [
      { id: 1, lat: 52.0, lng: 21.0, intersectionType: 'normal' },
      { id: 2, lat: 52.1, lng: 21.1, intersectionType: 'normal' },
      { id: 3, lat: 52.2, lng: 21.2, intersectionType: 'normal' },
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
      {
        from: 2,
        to: 3,
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
    lanes: [
      {
        id: 101,
        width: 3.5,
        connections: [],
        conflictAreas: [],
        points: [[21.0, 52.0], [21.1, 52.1]],
        lengthM: 100,
        fromNodeOsmId: 1,
        toNodeOsmId: 2,
        laneIndex: 0,
        isConnector: false,
      },
      {
        id: 102,
        width: 3.5,
        connections: [],
        conflictAreas: [],
        points: [[21.1, 52.1], [21.2, 52.2]],
        lengthM: 100,
        fromNodeOsmId: 2,
        toNodeOsmId: 3,
        laneIndex: 0,
        isConnector: false,
      },
    ],
    spawnPoints: [],
    bbox: [21.0, 52.0, 21.2, 52.2],
    buildings: [],
    restrictions: [],
    tramStops: [],
    turnConnectors: [],
    conflictAreas: [],
  };

  const createVehicle = (id: number, lat: number, lng: number, currentLaneId: number | null): VehicleState => ({
    id,
    lat,
    lng,
    angle: 0,
    speed: 10,
    vehicleType: 0,
    driverProfile: 1,
    tripKind: 0,
    currentLane: 0,
    onTurnConnector: false,
    frustration: 0,
    lateralOffset: 0,
    currentLaneId,
  });

  it('populates laneGroupCache on setEdgeIndex', () => {
    const map = new maplibregl.Map({} as any);
    const renderer = new VehicleRenderer(mockOverlay, map, mockCamera);

    renderer.setEdgeIndex(mapData);

    // laneGroupCache is private, but we can verify its correctness via isVehicleHidden
    // Let's hide the 'primary' group
    renderer.setHiddenGroups(new Set(['primary']));

    const vPrimary = createVehicle(1, 52.05, 21.05, 101);
    const vResidential = createVehicle(2, 52.15, 21.15, 102);

    // vPrimary has currentLaneId = 101 (which is mapped to 'primary')
    // and 'primary' is hidden, so vPrimary should be hidden.
    expect((renderer as any).isVehicleHidden(vPrimary)).toBe(true);

    // vResidential has currentLaneId = 102 (which is mapped to 'residential')
    // and 'residential' is NOT hidden, so vResidential should NOT be hidden.
    expect((renderer as any).isVehicleHidden(vResidential)).toBe(false);
  });

  it('falls back to spatial search if currentLaneId is null', () => {
    const map = new maplibregl.Map({} as any);
    const renderer = new VehicleRenderer(mockOverlay, map, mockCamera);

    renderer.setEdgeIndex(mapData);
    renderer.setHiddenGroups(new Set(['primary']));

    // Vehicle with null currentLaneId but spatially close to edge 1-2 (primary)
    const vPrimarySpatial = createVehicle(3, 52.01, 21.01, null);
    expect((renderer as any).isVehicleHidden(vPrimarySpatial)).toBe(true);

    // Vehicle with null currentLaneId but spatially close to edge 2-3 (residential)
    const vResidentialSpatial = createVehicle(4, 52.19, 21.19, null);
    expect((renderer as any).isVehicleHidden(vResidentialSpatial)).toBe(false);
  });

  it('clears cache on destroy', () => {
    const map = new maplibregl.Map({} as any);
    const renderer = new VehicleRenderer(mockOverlay, map, mockCamera);

    renderer.setEdgeIndex(mapData);
    expect((renderer as any).laneGroupCache.size).toBe(2);

    renderer.destroy();
    expect((renderer as any).laneGroupCache.size).toBe(0);
  });
});
