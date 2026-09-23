import { describe, it, expect } from 'vitest';
import { computeBuildingBounds, BuildingBounds } from './BuildingRenderer';

describe('computeBuildingBounds', () => {
  it('correctly calculates min/max lat and lng for a polygon', () => {
    const polygon: [number, number][] = [
      [52.2297, 21.0122],
      [52.2300, 21.0125],
      [52.2295, 21.0130],
      [52.2290, 21.0120],
    ];

    const bounds: BuildingBounds = computeBuildingBounds(polygon);

    expect(bounds.minLat).toBe(52.2290);
    expect(bounds.maxLat).toBe(52.2300);
    expect(bounds.minLng).toBe(21.0120);
    expect(bounds.maxLng).toBe(21.0130);
  });
});
