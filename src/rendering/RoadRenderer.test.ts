import { describe, it, expect, vi, beforeEach } from 'vitest';
import * as PIXI from 'pixi.js';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';
import { RoadRenderer } from './RoadRenderer';
import type { MapData } from '../bridge/commands';

describe('RoadRenderer', () => {
  let mockOverlay: PixiOverlay;
  let mockMap: any;
  let mockCamera: CameraManager;
  let renderer: RoadRenderer;

  beforeEach(() => {
    vi.clearAllMocks();
    mockOverlay = {
      roads: new PIXI.Container(),
    } as any;

    // Viewport: lng [-0.1, 0.1], lat [51.4, 51.6]
    // with 10% padding: lng [-0.12, 0.12], lat [51.38, 51.62]
    mockMap = {
      getBounds: vi.fn().mockReturnValue({
        getWest: () => -0.1,
        getEast: () => 0.1,
        getSouth: () => 51.4,
        getNorth: () => 51.6,
      }),
      project: vi.fn().mockImplementation((coords: [number, number]) => ({
        x: (coords[0] + 0.1) * 1000,
        y: (coords[1] - 51.4) * 1000,
      })),
    };

    mockCamera = {
      zoom: 16,
    } as any;

    renderer = new RoadRenderer(mockOverlay, mockMap, mockCamera);
  });

  it('draws visible roads and junction circles', () => {
    const mapData: MapData = {
      nodes: [
        { id: 1, lat: 51.5, lng: 0.0, intersectionType: 'plain' },
        { id: 2, lat: 51.51, lng: 0.0, intersectionType: 'plain' },
      ],
      edges: [
        {
          from: 1,
          to: 2,
          roadType: 'primary',
          lanes: 2,
          lengthM: 1000,
          oneway: false,
          maxSpeed: 50,
          infraType: 'none',
          layer: 0,
          laneDirections: ['straight', 'straight'],
        },
      ],
      buildings: [],
    } as any;

    renderer.build(mapData);

    // Overlay roads should contain graphics child
    expect(mockOverlay.roads.children.length).toBe(1);
    // Both endpoints are visible, project should be called for edge from & to
    expect(mockMap.project).toHaveBeenCalledTimes(2);
  });

  it('culls off-screen edges without projecting them', () => {
    const mapData: MapData = {
      nodes: [
        // Visible edge
        { id: 1, lat: 51.5, lng: 0.0, intersectionType: 'plain' },
        { id: 2, lat: 51.51, lng: 0.0, intersectionType: 'plain' },
        // Far off-screen edge (both endpoints north > 52.0)
        { id: 3, lat: 52.5, lng: 0.0, intersectionType: 'plain' },
        { id: 4, lat: 52.51, lng: 0.0, intersectionType: 'plain' },
      ],
      edges: [
        {
          from: 1,
          to: 2,
          roadType: 'primary',
          lanes: 2,
          lengthM: 1000,
          oneway: false,
          maxSpeed: 50,
          infraType: 'none',
          layer: 0,
          laneDirections: ['straight', 'straight'],
        },
        {
          from: 3,
          to: 4,
          roadType: 'primary',
          lanes: 2,
          lengthM: 1000,
          oneway: false,
          maxSpeed: 50,
          infraType: 'none',
          layer: 0,
          laneDirections: ['straight', 'straight'],
        },
      ],
      buildings: [],
    } as any;

    renderer.build(mapData);

    // Only nodes 1 and 2 should be projected; nodes 3 and 4 should be culled
    expect(mockMap.project).toHaveBeenCalledTimes(2);
  });

  it('reuses projected screen coordinates in Step 2 for junction circles', () => {
    const mapData: MapData = {
      nodes: [
        { id: 1, lat: 51.5, lng: 0.0, intersectionType: 'plain' },
        { id: 2, lat: 51.51, lng: 0.0, intersectionType: 'traffic_light' },
        { id: 3, lat: 51.52, lng: 0.0, intersectionType: 'plain' },
      ],
      edges: [
        {
          from: 1,
          to: 2,
          roadType: 'primary',
          lanes: 2,
          lengthM: 1000,
          oneway: false,
          maxSpeed: 50,
          infraType: 'none',
          layer: 0,
          laneDirections: ['straight', 'straight'],
        },
        {
          from: 2,
          to: 3,
          roadType: 'primary',
          lanes: 2,
          lengthM: 1000,
          oneway: false,
          maxSpeed: 50,
          infraType: 'none',
          layer: 0,
          laneDirections: ['straight', 'straight'],
        },
      ],
      buildings: [],
    } as any;

    renderer.build(mapData);

    // Edge 1 has 2 endpoints (project 2x). Edge 2 has 2 endpoints (project 2x).
    // Junction circles must NOT call project again. Total calls must be 4.
    expect(mockMap.project).toHaveBeenCalledTimes(4);
  });
});
