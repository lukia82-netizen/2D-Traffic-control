import { describe, it, expect, vi, beforeEach } from 'vitest';
import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import { RoadRenderer } from './RoadRenderer';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';
import type { MapData } from '../bridge/commands';

describe('RoadRenderer', () => {
  let mockMap: maplibregl.Map;
  let mockOverlay: PixiOverlay;
  let mockCamera: CameraManager;
  let roadContainer: PIXI.Container;

  beforeEach(() => {
    roadContainer = new PIXI.Container();

    mockOverlay = {
      roads: roadContainer,
    } as unknown as PixiOverlay;

    mockCamera = {
      zoom: 16,
    } as unknown as CameraManager;

    mockMap = {
      getBounds: vi.fn().mockReturnValue({
        getWest: () => 18.0,
        getEast: () => 18.1,
        getSouth: () => 54.0,
        getNorth: () => 54.1,
      }),
      project: vi.fn((coords: [number, number] | { lng: number; lat: number }) => {
        const lng = Array.isArray(coords) ? coords[0] : coords.lng;
        const lat = Array.isArray(coords) ? coords[1] : coords.lat;
        return {
          x: (lng - 18.0) * 10000,
          y: (54.1 - lat) * 10000,
        };
      }),
    } as unknown as maplibregl.Map;
  });

  function createMockMapData(nodes: MapData['nodes'], edges: MapData['edges']): MapData {
    return {
      nodes,
      edges,
      spawnPoints: [],
      bbox: [18.0, 54.0, 18.1, 54.1],
      buildings: [],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      lanes: [],
      conflictAreas: [],
    };
  }

  it('renders visible road edges within viewport bounds', () => {
    const mapData = createMockMapData(
      [
        { id: 1, lat: 54.05, lng: 18.05, intersectionType: 'plain' },
        { id: 2, lat: 54.06, lng: 18.06, intersectionType: 'plain' },
      ],
      [
        {
          from: 1,
          to: 2,
          roadType: 'primary',
          lanes: 2,
          lengthM: 100,
          infraType: 'none',
          oneway: false,
          maxSpeed: 50,
          layer: 0,
          laneDirections: ['straight', 'straight'],
        },
      ],
    );

    const renderer = new RoadRenderer(mockOverlay, mockMap, mockCamera);
    renderer.build(mapData);

    expect(mockMap.project).toHaveBeenCalled();
    // 2 endpoint projections for the visible edge (fromNode and toNode)
    expect(mockMap.project).toHaveBeenCalledTimes(2);
  });

  it('culls off-screen road edges whose endpoints lie outside the viewport', () => {
    const mapData = createMockMapData(
      [
        { id: 1, lat: 55.0, lng: 19.0, intersectionType: 'plain' },
        { id: 2, lat: 55.1, lng: 19.1, intersectionType: 'plain' },
      ],
      [
        {
          from: 1,
          to: 2,
          roadType: 'primary',
          lanes: 2,
          lengthM: 100,
          infraType: 'none',
          oneway: false,
          maxSpeed: 50,
          layer: 0,
          laneDirections: ['straight', 'straight'],
        },
      ],
    );

    const renderer = new RoadRenderer(mockOverlay, mockMap, mockCamera);
    renderer.build(mapData);

    // Edge is far outside viewport bounds, so map.project should not be called for edge endpoints
    expect(mockMap.project).not.toHaveBeenCalled();
  });
});
