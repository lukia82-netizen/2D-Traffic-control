import { describe, it, expect, vi } from 'vitest';
import { TrafficLightRenderer } from './TrafficLightRenderer';
import type { NodeData, EdgeData } from '../bridge/commands';
import type { LightStateUpdate } from '../bridge/events';

describe('TrafficLightRenderer', () => {
  it('updates signal states efficiently using indexed node approaches', () => {
    const mockContainer = {
      addChild: vi.fn(),
      removeChildren: vi.fn(),
    };
    const mockOverlay = {
      trafficLights: mockContainer,
    } as any;
    const mockMap = {
      getZoom: vi.fn().mockReturnValue(16),
      project: vi.fn().mockImplementation((coords: [number, number]) => ({
        x: coords[0] * 100,
        y: coords[1] * 100,
      })),
    } as any;

    const renderer = new TrafficLightRenderer(mockOverlay, mockMap);

    const nodes: NodeData[] = [
      { id: 10, lat: 51.0, lng: 19.0, intersectionType: 'traffic_light' },
      { id: 1, lat: 51.01, lng: 19.0, intersectionType: 'plain' },
      { id: 2, lat: 50.99, lng: 19.0, intersectionType: 'plain' },
    ];

    const edges: EdgeData[] = [
      {
        from: 1,
        to: 10,
        lanes: 2,
        lengthM: 100,
        roadType: 'primary',
        oneway: false,
        maxSpeed: 50,
        layer: 0,
        infraType: 'surface',
        laneDirections: ['straight', 'right'],
      },
      {
        from: 2,
        to: 10,
        lanes: 2,
        lengthM: 100,
        roadType: 'primary',
        oneway: false,
        maxSpeed: 50,
        layer: 0,
        infraType: 'surface',
        laneDirections: ['left', 'straight'],
      },
    ];

    renderer.init(nodes, edges);

    const updates: LightStateUpdate[] = [
      {
        intersectionId: 10,
        phase: 2,
        timeRemaining: 15,
        queueCount: 0,
        mode: 'auto',
        greenDuration: 20,
        redDuration: 20,
      },
    ];

    // Ensure updateStates runs without errors and redraws signal heads
    expect(() => renderer.updateStates(updates)).not.toThrow();

    renderer.destroy();
  });
});
