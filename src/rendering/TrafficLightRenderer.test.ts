import { describe, it, expect, vi, beforeEach } from 'vitest';
import * as PIXI from 'pixi.js';
import type { PixiOverlay } from './PixiOverlay';
import { TrafficLightRenderer } from './TrafficLightRenderer';
import type { NodeData, EdgeData } from '../bridge/commands';
import type { LightStateUpdate } from '../bridge/events';

// Mock MapLibre Map
const mockMap = {
  getZoom: vi.fn().mockReturnValue(16),
  project: vi.fn().mockImplementation((coords: [number, number]) => ({
    x: coords[0] * 10,
    y: coords[1] * 10,
  })),
} as any;

describe('TrafficLightRenderer', () => {
  let mockOverlay: PixiOverlay;
  let renderer: TrafficLightRenderer;

  beforeEach(() => {
    vi.clearAllMocks();
    mockOverlay = {
      trafficLights: new PIXI.Container(),
    } as any;
    renderer = new TrafficLightRenderer(mockOverlay, mockMap);
  });

  it('initializes approaches correctly and builds graphics', () => {
    const nodes: NodeData[] = [
      { id: 1, lat: 51.5, lng: -0.1, intersectionType: 'none' },
      { id: 2, lat: 51.51, lng: -0.1, intersectionType: 'traffic_light' },
    ];
    const edges: EdgeData[] = [
      {
        from: 1,
        to: 2,
        roadType: 'primary',
        lanes: 2,
        lengthM: 100,
        oneway: true,
        maxSpeed: 50,
        infraType: 'none',
        layer: 0,
        laneDirections: ['straight', 'straight'],
      },
    ];

    renderer.init(nodes, edges);

    expect(mockOverlay.trafficLights.children.length).toBeGreaterThan(0);
  });

  it('updates traffic light states efficiently with O(1) approach lookups', () => {
    const nodes: NodeData[] = [
      { id: 1, lat: 51.5, lng: -0.1, intersectionType: 'none' },
      { id: 2, lat: 51.51, lng: -0.1, intersectionType: 'traffic_light' },
    ];
    const edges: EdgeData[] = [
      {
        from: 1,
        to: 2,
        roadType: 'primary',
        lanes: 2,
        lengthM: 100,
        oneway: true,
        maxSpeed: 50,
        infraType: 'none',
        layer: 0,
        laneDirections: ['straight', 'straight'],
      },
    ];

    renderer.init(nodes, edges);
    mockMap.getZoom.mockClear();

    const updates: LightStateUpdate[] = [
      {
        intersectionId: 2,
        phase: 2,
        timeRemaining: 10,
        queueCount: 0,
        mode: 'auto',
        greenDuration: 15,
        redDuration: 15,
      },
      {
        intersectionId: 2,
        phase: 0,
        timeRemaining: 15,
        queueCount: 1,
        mode: 'auto',
        greenDuration: 15,
        redDuration: 15,
      },
    ];

    expect(() => renderer.updateStates(updates)).not.toThrow();
    // getZoom should be called once for the entire update batch regardless of how many updates are in the array
    expect(mockMap.getZoom).toHaveBeenCalledTimes(1);
  });

  it('handles empty updates array gracefully without calling getZoom', () => {
    renderer.updateStates([]);
    expect(mockMap.getZoom).not.toHaveBeenCalled();
  });
});
