import { describe, expect, it, vi } from 'vitest';
import type maplibregl from 'maplibre-gl';
import type { MapData, BuildingData } from '../bridge/commands';
import type { PixiOverlay } from './PixiOverlay';
import { BuildingRenderer } from './BuildingRenderer';

describe('BuildingRenderer viewport culling', () => {
  it('renders visible buildings and culls off-screen buildings', () => {
    const projectedCoords: Array<[number, number]> = [];
    const mockMap = {
      getBounds: () => ({
        getWest: () => 10,
        getEast: () => 20,
        getSouth: () => 10,
        getNorth: () => 20,
      }),
      project: vi.fn((coords: [number, number]) => {
        projectedCoords.push(coords);
        return { x: 100, y: 100 };
      }),
    } as unknown as maplibregl.Map;

    const mockOverlay = {
      buildings: {
        addChild: vi.fn(),
      },
    } as unknown as PixiOverlay;

    const insideBuilding: BuildingData = {
      id: 1,
      polygon: [
        [15, 15],
        [15, 16],
        [16, 16],
      ], // [lat, lng] inside [10, 20] viewport
      buildingType: 'residential',
    };

    const outsideBuilding: BuildingData = {
      id: 2,
      polygon: [
        [50, 50],
        [50, 51],
        [51, 51],
      ], // [lat, lng] far outside viewport
      buildingType: 'commercial',
    };

    const mapData: MapData = {
      nodes: [],
      edges: [],
      spawnPoints: [],
      bbox: [10, 10, 20, 20],
      buildings: [insideBuilding, outsideBuilding],
      restrictions: [],
      tramStops: [],
      turnConnectors: [],
      lanes: [],
      conflictAreas: [],
    };

    const renderer = new BuildingRenderer(mockOverlay, mockMap);
    renderer.build(mapData);

    // Verify only vertices from insideBuilding were projected (3 vertices)
    expect(mockMap.project).toHaveBeenCalledTimes(3);
    expect(projectedCoords).toEqual([
      [15, 15],
      [16, 15],
      [16, 16],
    ]);

    renderer.destroy();
  });
});
