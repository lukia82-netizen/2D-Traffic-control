import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import { Channel } from '@tauri-apps/api/core';
import type { MapData } from './bridge/commands';
import {
  loadMap,
  startSimulation,
  setTimeScale,
} from './bridge/commands';
import {
  parseVehicleFrame,
  listenCongestionUpdates,
  listenLightStateChanges,
} from './bridge/events';
import type { VehicleState, CongestionData, LightStateUpdate } from './bridge/events';
import { PixiOverlay } from './rendering/PixiOverlay';
import { CameraManager } from './rendering/CameraManager';
import { RoadRenderer } from './rendering/RoadRenderer';
import { BuildingRenderer } from './rendering/BuildingRenderer';
import { VehicleRenderer } from './rendering/VehicleRenderer';
import { InfraRenderer } from './rendering/InfraRenderer';
import { CongestionRenderer } from './rendering/CongestionRenderer';
import { UIRenderer } from './rendering/UIRenderer';
import { TrafficLightUI } from './traffic/TrafficLightUI';
import { GameClockUI } from './time/GameClockUI';

// Kraków Śródmieście – ~1 km × 1 km centred on Rynek Główny (19.9368, 50.0614)
// 0.007° lng ≈ 500 m, 0.0045° lat ≈ 500 m  →  total 1 km × 1 km
// west, south, east, north
const DEFAULT_BBOX: [number, number, number, number] = [19.930, 50.057, 19.944, 50.066];

// ─── Demo road network ─────────────────────────────────────────────────────
// Used as a fallback when the Overpass API is not reachable (e.g., corporate
// firewall). Generates a simple 5×5 grid of roads around the Kraków centre.

function buildDemoMapData(): MapData {
  const CX = 19.940;       // centre longitude
  const CY = 50.060;       // centre latitude
  const STEP_LNG = 0.004;  // ~300 m
  const STEP_LAT = 0.003;

  const COLS = 5;
  const ROWS = 5;

  const nodes: MapData['nodes'] = [];
  const edges: MapData['edges'] = [];
  const spawnPoints: MapData['spawnPoints'] = [];

  const nid = (r: number, c: number): number => r * COLS + c;

  for (let r = 0; r < ROWS; r++) {
    for (let c = 0; c < COLS; c++) {
      nodes.push({
        id: nid(r, c),
        lat: CY + (r - Math.floor(ROWS / 2)) * STEP_LAT,
        lng: CX + (c - Math.floor(COLS / 2)) * STEP_LNG,
        intersectionType: 'traffic_light',
      });
    }
  }

  const addEdgePair = (a: number, b: number): void => {
    edges.push({ from: a, to: b, lanes: 2, maxSpeed: 50, oneway: false, infraType: 'normal', layer: 0, lengthM: 300, roadType: 'residential' });
    edges.push({ from: b, to: a, lanes: 2, maxSpeed: 50, oneway: false, infraType: 'normal', layer: 0, lengthM: 300, roadType: 'residential' });
  };

  for (let r = 0; r < ROWS; r++) {
    for (let c = 0; c < COLS - 1; c++) addEdgePair(nid(r, c), nid(r, c + 1));
  }
  for (let r = 0; r < ROWS - 1; r++) {
    for (let c = 0; c < COLS; c++) addEdgePair(nid(r, c), nid(r + 1, c));
  }

  for (let c = 0; c < COLS; c++) {
    spawnPoints.push([nodes[nid(0, c)].lat, nodes[nid(0, c)].lng]);
    spawnPoints.push([nodes[nid(ROWS - 1, c)].lat, nodes[nid(ROWS - 1, c)].lng]);
  }
  for (let r = 1; r < ROWS - 1; r++) {
    spawnPoints.push([nodes[nid(r, 0)].lat, nodes[nid(r, 0)].lng]);
    spawnPoints.push([nodes[nid(r, COLS - 1)].lat, nodes[nid(r, COLS - 1)].lng]);
  }

  return { nodes, edges, spawnPoints, bbox: DEFAULT_BBOX, buildings: [] };
}

// Simulation starts at 06:00 (game seconds since midnight)
const GAME_START_TIME_S = 6 * 3600;

// ─── Game ─────────────────────────────────────────────────────────────────────

/**
 * Top-level coordinator.  Owns all sub-systems and the main ticker loop.
 *
 * Lifecycle:
 *   const game = new Game(map, pixiOverlay);
 *   await game.init();
 */
export class Game {
  private readonly map: maplibregl.Map;
  private readonly overlay: PixiOverlay;

  private camera!: CameraManager;
  private roadRenderer!: RoadRenderer;
  private buildingRenderer!: BuildingRenderer;
  private vehicleRenderer!: VehicleRenderer;
  private infraRenderer!: InfraRenderer;
  private congestionRenderer!: CongestionRenderer;
  private uiRenderer!: UIRenderer;
  private trafficLightUI!: TrafficLightUI;
  private gameClockUI!: GameClockUI;

  private mapData: MapData | null = null;
  private readonly vehicles: Map<number, VehicleState> = new Map();

  // infra type per vehicle id (populated from map data by Rust; we approximate
  // from edge data on the frontend until Rust sends per-vehicle infra)
  private readonly vehicleInfraMap: Map<number, string> = new Map();

  // Accumulated game time in seconds
  private gameTimeS: number = GAME_START_TIME_S;

  // Unlisten callbacks for Tauri events
  private unlistenCongestion: (() => void) | null = null;
  private unlistenLights: (() => void) | null = null;

  // Whether the Rust backend is available (desktop Tauri vs browser dev mode)
  private tauriAvailable = false;

  constructor(map: maplibregl.Map, overlay: PixiOverlay) {
    this.map = map;
    this.overlay = overlay;
  }

  // ─── Initialisation ────────────────────────────────────────────────────────

  async init(): Promise<void> {
    // Detect Tauri environment
    this.tauriAvailable = typeof (window as unknown as Record<string, unknown>)['__TAURI_INTERNALS__'] !== 'undefined';

    // Instantiate sub-systems
    this.camera = new CameraManager(this.map);
    this.buildingRenderer = new BuildingRenderer(this.overlay, this.map);
    this.roadRenderer = new RoadRenderer(this.overlay, this.map, this.camera);
    this.vehicleRenderer = new VehicleRenderer(this.overlay, this.map, this.camera);
    this.infraRenderer = new InfraRenderer(this.overlay, this.map, this.camera);
    this.congestionRenderer = new CongestionRenderer(
      this.overlay,
      this.map,
      document.getElementById('hud-overlay')!,
    );
    this.uiRenderer = new UIRenderer();
    this.trafficLightUI = new TrafficLightUI(this.map);
    this.gameClockUI = new GameClockUI();

    // Init vehicle textures
    await this.vehicleRenderer.init();

    // Init HUD controls
    this.gameClockUI.init();

    if (this.tauriAvailable) {
      await this.loadMapData();
      await this.subscribeToEvents();
      await this.startRustSimulation();
    } else {
      // Dev / browser mode: show placeholder notification
      this.uiRenderer.showNotification(
        'Running in browser – Tauri backend not available',
        'warning',
      );
    }

    // Hook infra rebuild on every map camera move
    this.map.on('render', () => {
      if (this.mapData) {
        this.buildingRenderer.rebuildOnCameraChange(this.mapData);
        this.roadRenderer.rebuildOnCameraChange(this.mapData);
        this.infraRenderer.rebuildOnCameraChange(this.mapData);
      }
    });

    // Start the PixiJS ticker
    this.overlay.app.ticker.add((ticker) => this.gameLoop(ticker));
  }

  // ─── Map loading ───────────────────────────────────────────────────────────

  private async loadMapData(): Promise<void> {
    this.uiRenderer.showNotification('Loading map data…', 'info');
    try {
      this.mapData = await loadMap(DEFAULT_BBOX);
      this.uiRenderer.showNotification(
        `Map loaded – ${this.mapData.nodes.length} nodes, ${this.mapData.edges.length} edges`,
        'info',
      );
    } catch (err) {
      console.warn('Overpass API unavailable, using demo road network:', err);
      this.mapData = buildDemoMapData();
      this.uiRenderer.showNotification(
        'Offline mode – demo road network (5×5 grid)',
        'warning',
      );
    }
    this.buildingRenderer.build(this.mapData);
    this.roadRenderer.build(this.mapData);
    this.infraRenderer.buildStaticLayer(this.mapData);
    this.trafficLightUI.init(this.mapData.nodes);
  }

  // ─── Event subscriptions ───────────────────────────────────────────────────

  private async subscribeToEvents(): Promise<void> {
    this.unlistenCongestion = await listenCongestionUpdates((data) =>
      this.onCongestionUpdate(data),
    );
    this.unlistenLights = await listenLightStateChanges((data) =>
      this.onLightStateChange(data),
    );
  }

  // ─── Simulation start ──────────────────────────────────────────────────────

  private async startRustSimulation(): Promise<void> {
    const channel = new Channel<string>();
    channel.onmessage = (data: string) => this.onVehicleFrame(data);

    try {
      await startSimulation(channel);
      // Sync the initial time scale – GameClockUI defaults to 60 but Rust starts at 1.0
      await setTimeScale(this.gameClockUI.timeScale);
    } catch (err) {
      console.error('Failed to start simulation:', err);
      this.uiRenderer.showNotification('Simulation start failed', 'error');
    }
  }

  // ─── Tauri event handlers ──────────────────────────────────────────────────

  private onVehicleFrame(data: string): void {
    const states = parseVehicleFrame(data);

    // Merge into vehicles map
    for (const v of states) {
      this.vehicles.set(v.id, v);
    }

    // Advance game time based on real elapsed time × timeScale
    // (We rely on the ticker's deltaMS in gameLoop; this handler only updates state)
  }

  private onCongestionUpdate(data: CongestionData[]): void {
    if (this.mapData) {
      this.congestionRenderer.update(data, this.mapData);
    }
  }

  private onLightStateChange(data: LightStateUpdate[]): void {
    this.trafficLightUI.updateLightState(data);
  }

  // ─── Main game loop ────────────────────────────────────────────────────────

  private gameLoop(ticker: PIXI.Ticker): void {
    if (this.gameClockUI.paused) return;

    // Advance game clock
    const realDeltaS = ticker.deltaMS / 1000;
    this.gameTimeS += realDeltaS * this.gameClockUI.timeScale;

    // Update clock HUD
    this.gameClockUI.updateClock(this.gameTimeS);

    // Render vehicles
    this.vehicleRenderer.update(this.vehicles, this.vehicleInfraMap);

    // Update satisfaction bar from vehicle states
    this.updateSatisfaction();
  }

  // ─── Derived state ─────────────────────────────────────────────────────────

  private updateSatisfaction(): void {
    if (this.vehicles.size === 0) return;

    let total = 0;
    for (const v of this.vehicles.values()) {
      total += v.satisfaction;
    }
    const avg = total / this.vehicles.size;

    this.uiRenderer.updateSatisfaction(avg);
    this.uiRenderer.updateVehicleCount(this.vehicles.size);
  }

  // ─── Cleanup ───────────────────────────────────────────────────────────────

  destroy(): void {
    this.unlistenCongestion?.();
    this.unlistenLights?.();
    this.buildingRenderer.destroy();
    this.roadRenderer.destroy();
    this.vehicleRenderer.destroy();
    this.infraRenderer.destroy();
    this.congestionRenderer.destroy();
  }
}
