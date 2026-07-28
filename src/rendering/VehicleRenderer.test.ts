import { describe, expect, it, vi } from 'vitest';
import { VehicleRenderer } from './VehicleRenderer';

// Mock PIXI so we don't need a real WebGL context
vi.mock('pixi.js', () => {
  return {
    Graphics: class {
      circle() { return this; }
      fill() { return this; }
      stroke() { return this; }
      clear() { return this; }
      rect() { return this; }
      destroy() {}
    }
  };
});

// Mock maplibre-gl
vi.mock('maplibre-gl', () => {
  return {
    default: {
      Map: class {
        getCenter() { return { lat: 50.0, lng: 19.0 }; }
        project() { return { x: 100, y: 100 }; }
        getBounds() {
          return {
            getWest: () => 18.9,
            getEast: () => 19.1,
            getSouth: () => 49.9,
            getNorth: () => 50.1,
          };
        }
      }
    }
  };
});

describe('VehicleRenderer', () => {
  const mockOverlay = {
    groundVehicles: { addChild: vi.fn() },
    frustrationLayer: { addChild: vi.fn() },
    tunnelVehicles: { addChild: vi.fn() },
    bridgeVehicles: { addChild: vi.fn() },
  } as any;

  const mockMap = {
    getCenter: () => ({ lat: 50.0, lng: 19.0 }),
    project: (coord: any) => ({ x: 100, y: 100 }),
    getBounds: () => ({
      getWest: () => 18.9,
      getEast: () => 19.1,
      getSouth: () => 49.9,
      getNorth: () => 50.1,
    }),
  } as any;

  const mockCamera = {
    getCarVisuals: () => ({ mode: 'dot', spriteScale: 1.0, dotRadius: 4 }),
  } as any;

  it('correctly maps lanes to road groups and leverages O(1) cache lookup', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);

    const mapData = {
      nodes: [
        { id: 1, lat: 50.0, lng: 19.0, intersectionType: 'plain' },
        { id: 2, lat: 50.1, lng: 19.1, intersectionType: 'plain' }
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
          roadType: 'residential'
        }
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
          isConnector: false
        }
      ],
      spawnPoints: [],
      bbox: [19.0, 50.0, 19.1, 50.1],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      conflictAreas: []
    } as any;

    renderer.setEdgeIndex(mapData);

    // Verify laneGroupCache has the mapping
    const laneGroupCache = (renderer as any).laneGroupCache;
    expect(laneGroupCache.get(42)).toBe('residential');

    // Set hidden groups
    renderer.setHiddenGroups(new Set(['residential']));

    // When currentLaneId matches the hidden group, the vehicle should be hidden
    const hiddenVehicle = { id: 1, currentLaneId: 42, lat: 50.05, lng: 19.05 } as any;
    expect((renderer as any).isVehicleHidden(hiddenVehicle)).toBe(true);

    // When currentLaneId matches a non-hidden group, it should not be hidden
    const visibleVehicle = { id: 2, currentLaneId: 99, lat: 50.05, lng: 19.05 } as any;
    // (Since 99 is not in cache, it falls back to spatial lookup.
    // The closest (and only) edge has group 'residential', which is hidden.
    // Let's verify that invisible vehicle is still resolved spatially.)
    expect((renderer as any).isVehicleHidden(visibleVehicle)).toBe(true);

    // If we set a hidden group that doesn't match the resolved edge, it should be visible
    renderer.setHiddenGroups(new Set(['motorway']));
    expect((renderer as any).isVehicleHidden(visibleVehicle)).toBe(false);
  });

  it('consistent cleanup in destroy method', () => {
    const renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    const mapData = {
      nodes: [{ id: 1, lat: 50.0, lng: 19.0 }],
      edges: [{ from: 1, to: 1, roadType: 'residential' }],
      lanes: [{ id: 10, fromNodeOsmId: 1, toNodeOsmId: 1 }]
    } as any;

    renderer.setEdgeIndex(mapData);
    expect((renderer as any).laneGroupCache.size).toBe(1);

    renderer.destroy();
    expect((renderer as any).laneGroupCache.size).toBe(0);
    expect((renderer as any).vehicleGroupCache.size).toBe(0);
    expect((renderer as any).prevVehiclePos.size).toBe(0);
  });
});
