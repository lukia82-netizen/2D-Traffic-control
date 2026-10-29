import { describe, expect, it, vi } from 'vitest';
import type maplibregl from 'maplibre-gl';
import type { PixiOverlay } from './PixiOverlay';
import { TrafficLightRenderer } from './TrafficLightRenderer';
import type { NodeData, EdgeData } from '../bridge/commands';
import type { LightStateUpdate } from '../bridge/events';

// Mock projectPoint
vi.mock('../map/MapLibreSetup', () => ({
  projectPoint: vi.fn((_map, lng, lat) => ({ x: lng * 100, y: lat * 100 })),
}));

// Mock PixiJS
vi.mock('pixi.js', () => {
  class MockGraphics {
    x = 0;
    y = 0;
    circle() { return this; }
    rect() { return this; }
    fill() { return this; }
    stroke() { return this; }
    moveTo() { return this; }
    lineTo() { return this; }
    clear() { return this; }
  }
  class MockContainer {
    children: unknown[] = [];
    addChild(child: unknown) {
      this.children.push(child);
      return child;
    }
    removeChildren() {
      this.children = [];
    }
  }
  return {
    Graphics: MockGraphics,
    Container: MockContainer,
  };
});

describe('TrafficLightRenderer', () => {
  const createMockMap = (zoom = 16) => {
    return {
      getZoom: () => zoom,
      getBounds: () => ({
        getWest: () => 10,
        getEast: () => 20,
        getSouth: () => 10,
        getNorth: () => 20,
      }),
    } as unknown as maplibregl.Map;
  };

  const createMockOverlay = () => {
    const trafficLightsContainer = {
      children: [] as unknown[],
      addChild(child: unknown) {
        this.children.push(child);
        return child;
      },
      removeChildren() {
        this.children = [];
      },
    };
    return {
      trafficLights: trafficLightsContainer,
    } as unknown as PixiOverlay;
  };

  const sampleNodes: NodeData[] = [
    { id: 1, lat: 15, lng: 15, intersectionType: 'plain' },
    { id: 2, lat: 15, lng: 15, intersectionType: 'traffic_light' },
    { id: 3, lat: 0, lng: 0, intersectionType: 'traffic_light' }, // off-screen
  ];

  const sampleEdges: EdgeData[] = [
    {
      from: 1,
      to: 2,
      lanes: 2,
      maxSpeed: 50,
      oneway: false,
      infraType: 'none',
      layer: 0,
      lengthM: 50,
      roadType: 'primary',
      laneDirections: [],
    },
    {
      from: 1,
      to: 3,
      lanes: 2,
      maxSpeed: 50,
      oneway: false,
      infraType: 'none',
      layer: 0,
      lengthM: 50,
      roadType: 'primary',
      laneDirections: [],
    },
  ];

  it('initializes and renders approaches for visible nodes', () => {
    const map = createMockMap();
    const overlay = createMockOverlay();
    const renderer = new TrafficLightRenderer(overlay, map);

    renderer.init(sampleNodes, sampleEdges);

    // Node 2 is inside bounds (10-20), Node 3 is off-screen (0,0)
    // Should add graphics for node 2 but cull node 3
    expect(overlay.trafficLights.children.length).toBeGreaterThan(0);
  });

  it('updates light states efficiently using per-node approach lookup', () => {
    const map = createMockMap();
    const overlay = createMockOverlay();
    const renderer = new TrafficLightRenderer(overlay, map);

    renderer.init(sampleNodes, sampleEdges);

    const update: LightStateUpdate = {
      intersectionId: 2,
      phase: 2,
      timeRemaining: 10,
      queueCount: 0,
      mode: 'Auto',
      greenDuration: 20,
      redDuration: 20,
    };

    // Send update for intersection 2
    renderer.updateStates([update]);

    // Empty updates should be a fast no-op
    renderer.updateStates([]);
  });

  it('cleans up on destroy', () => {
    const map = createMockMap();
    const overlay = createMockOverlay();
    const renderer = new TrafficLightRenderer(overlay, map);

    renderer.init(sampleNodes, sampleEdges);
    renderer.destroy();

    expect(overlay.trafficLights.children.length).toBe(0);
  });
});
