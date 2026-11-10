import { describe, it, expect, vi, beforeEach } from 'vitest';
import * as PIXI from 'pixi.js';
import { VehicleRenderer } from './VehicleRenderer';
import type { PixiOverlay } from './PixiOverlay';
import type { CameraManager } from './CameraManager';
import type { VehicleState } from '../bridge/events';

describe('VehicleRenderer', () => {
  let mockOverlay: PixiOverlay;
  let mockMap: any;
  let mockCamera: CameraManager;
  let renderer: VehicleRenderer;

  beforeEach(async () => {
    mockOverlay = {
      groundVehicles: new PIXI.Container(),
      tunnelVehicles: new PIXI.Container(),
      bridgeVehicles: new PIXI.Container(),
      frustrationLayer: new PIXI.Container(),
    } as any;

    mockMap = {
      getBounds: vi.fn().mockReturnValue({
        getWest: vi.fn().mockReturnValue(10),
        getEast: vi.fn().mockReturnValue(20),
        getSouth: vi.fn().mockReturnValue(50),
        getNorth: vi.fn().mockReturnValue(60),
      }),
      getCenter: vi.fn().mockReturnValue({ lat: 55, lng: 15 }),
      project: vi.fn().mockImplementation(([lng, lat]: [number, number]) => ({
        x: (lng - 10) * 10,
        y: (lat - 50) * 10,
      })),
    };

    mockCamera = {
      getCarVisuals: vi.fn().mockReturnValue({
        mode: 'sprite',
        spriteScale: 1.0,
        dotRadius: 3,
      }),
    } as any;

    renderer = new VehicleRenderer(mockOverlay, mockMap, mockCamera);
    await renderer.init();
  });

  const createVehicle = (id: number, lat: number, lng: number, frustration = 0): VehicleState => ({
    id,
    lat,
    lng,
    angle: 0,
    speed: 10,
    vehicleType: 0,
    driverProfile: 0,
    tripKind: 0,
    currentLane: 0,
    onTurnConnector: false,
    lateralOffset: 0,
    frustration,
    currentLaneId: null,
  });

  it('renders sprites within viewport bounds and culls off-screen vehicles', () => {
    const vehicles = new Map<number, VehicleState>([
      [1, createVehicle(1, 55, 15)], // Inside bounds (10-20 lng, 50-60 lat)
      [2, createVehicle(2, 70, 25)], // Outside bounds
    ]);
    const infraMap = new Map<number, string>();

    renderer.update(vehicles, infraMap);

    // Only vehicle 1 should be rendered as a sprite in groundVehicles
    expect(mockOverlay.groundVehicles.children.length).toBe(2); // dotGraphics (from init) + 1 active sprite
    const activeSprite = mockOverlay.groundVehicles.children[1] as PIXI.Graphics;
    expect(activeSprite.visible).toBe(true);
  });

  it('renders dots when camera mode is dot', () => {
    (mockCamera.getCarVisuals as any).mockReturnValue({
      mode: 'dot',
      spriteScale: 1.0,
      dotRadius: 4,
    });

    const vehicles = new Map<number, VehicleState>([
      [1, createVehicle(1, 55, 15)],
    ]);
    const infraMap = new Map<number, string>();

    renderer.update(vehicles, infraMap);

    // Sprites shouldn't be added to groundVehicles
    expect(mockOverlay.groundVehicles.children.length).toBe(1); // dotGraphics only
  });

  it('cleans up sprite and pooling resources on destroy', () => {
    const vehicles = new Map<number, VehicleState>([
      [1, createVehicle(1, 55, 15)],
    ]);
    const infraMap = new Map<number, string>();

    renderer.update(vehicles, infraMap);
    expect(() => renderer.destroy()).not.toThrow();
  });
});
