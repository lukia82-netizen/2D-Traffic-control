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
  listenGameOver,
} from './bridge/events';
import type { VehicleState, CongestionData, LightStateUpdate, GameOverPayload } from './bridge/events';
import { PixiOverlay } from './rendering/PixiOverlay';
import { CameraManager } from './rendering/CameraManager';
import { RoadRenderer } from './rendering/RoadRenderer';
import { BuildingRenderer } from './rendering/BuildingRenderer';
import { VehicleRenderer } from './rendering/VehicleRenderer';
import { InfraRenderer } from './rendering/InfraRenderer';
import { CongestionRenderer } from './rendering/CongestionRenderer';
import { UIRenderer } from './rendering/UIRenderer';
import { TrafficLightUI } from './traffic/TrafficLightUI';
import { TrafficLightRenderer } from './rendering/TrafficLightRenderer';
import { GameClockUI } from './time/GameClockUI';
import { SandboxUI } from './ui/SandboxUI';
import { LESZNO_BBOX } from './map/MapLibreSetup';

// ─── Mode: always start in SANDBOX ───────────────────────────────────────────
// Sandbox uses Leszno, skips building rendering (big perf win), shows
// the layer/legend panel.  A full game menu will replace this later.
const SANDBOX_MODE = true;

// Sandbox default city bbox (Leszno ~2km × 2km)
const DEFAULT_BBOX: [number, number, number, number] = LESZNO_BBOX;

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
    const dirs = ['left', 'straight'];
    edges.push({ from: a, to: b, lanes: 2, maxSpeed: 50, oneway: false, infraType: 'normal', layer: 0, lengthM: 300, roadType: 'residential', laneDirections: dirs });
    edges.push({ from: b, to: a, lanes: 2, maxSpeed: 50, oneway: false, infraType: 'normal', layer: 0, lengthM: 300, roadType: 'residential', laneDirections: ['straight', 'right'] });
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

  return { nodes, edges, spawnPoints, bbox: DEFAULT_BBOX, buildings: [], restrictions: [], tramStops: [] };
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
  private trafficLightRenderer!: TrafficLightRenderer;
  private gameClockUI!: GameClockUI;

  private mapData: MapData | null = null;
  private readonly vehicles: Map<number, VehicleState> = new Map();

  // infra type per vehicle id
  private readonly vehicleInfraMap: Map<number, string> = new Map();

  // Accumulated game time in seconds
  private gameTimeS: number = GAME_START_TIME_S;

  // Unlisten callbacks for Tauri events
  private unlistenCongestion: (() => void) | null = null;
  private unlistenLights: (() => void) | null = null;
  private unlistenGameOver: (() => void) | null = null;

  // Scoring
  private score = 0;
  private gameOver = false;

  // Whether the Rust backend is available (desktop Tauri vs browser dev mode)
  private tauriAvailable = false;

  // Sandbox mode
  private sandboxUI: SandboxUI | null = null;
  private vehiclesVisible = true;

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
    this.trafficLightRenderer = new TrafficLightRenderer(this.overlay, this.map);
    this.gameClockUI = new GameClockUI();

    // Init vehicle textures
    await this.vehicleRenderer.init();

    // Init HUD controls
    this.gameClockUI.init();

    // ── Sandbox UI ────────────────────────────────────────────────────────
    if (SANDBOX_MODE) {
      this.sandboxUI = new SandboxUI();
      this.wireSandboxUI();
    }

    if (this.tauriAvailable) {
      await this.loadMapData();
      await this.subscribeToEvents();
      await this.startRustSimulation();
    } else {
      this.uiRenderer.showNotification(
        'Running in browser – Tauri backend not available',
        'warning',
      );
    }

    // Hook infra rebuild on every map camera move
    this.map.on('render', () => {
      if (this.mapData) {
        if (!SANDBOX_MODE) this.buildingRenderer.rebuildOnCameraChange(this.mapData);
        this.roadRenderer.rebuildOnCameraChange(this.mapData);
        this.infraRenderer.rebuildOnCameraChange(this.mapData);
        this.trafficLightRenderer.rebuildOnCameraChange();
      }
    });

    // Start the PixiJS ticker
    this.overlay.app.ticker.add((ticker) => this.gameLoop(ticker));
  }

  // ─── Sandbox wiring ────────────────────────────────────────────────────────

  private wireSandboxUI(): void {
    const ui = this.sandboxUI!;

    ui.onLayerToggle = (group, visible) => {
      this.roadRenderer.setGroupVisible(group, visible);
    };

    ui.onOsmModeToggle = (enabled) => {
      this.roadRenderer.setOsmMode(enabled);
    };

    ui.onVehicleToggle = (visible) => {
      this.vehiclesVisible = visible;
      this.overlay.groundVehicles.visible = visible;
      this.overlay.bridgeVehicles.visible = visible;
      this.overlay.tunnelVehicles.visible = visible;
    };

    ui.onBuildingToggle = (visible) => {
      this.overlay.buildings.visible = visible;
      if (visible && this.mapData) {
        this.buildingRenderer.build(this.mapData);
      }
    };
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
    if (!SANDBOX_MODE) this.buildingRenderer.build(this.mapData);
    this.roadRenderer.build(this.mapData);
    this.infraRenderer.buildStaticLayer(this.mapData);
    this.trafficLightUI.init(this.mapData.nodes);
    this.trafficLightRenderer.init(this.mapData.nodes);
  }

  // ─── Event subscriptions ───────────────────────────────────────────────────

  private async subscribeToEvents(): Promise<void> {
    this.unlistenCongestion = await listenCongestionUpdates((data) =>
      this.onCongestionUpdate(data),
    );
    this.unlistenLights = await listenLightStateChanges((data) =>
      this.onLightStateChange(data),
    );
    this.unlistenGameOver = await listenGameOver((data) =>
      this.onGameOver(data),
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
    this.trafficLightRenderer.updateStates(data);
  }

  private onGameOver(data: GameOverPayload): void {
    this.gameOver = true;
    // Compute average frustration from current vehicles
    let avgFrustration = 0;
    if (this.vehicles.size > 0) {
      let total = 0;
      for (const v of this.vehicles.values()) total += v.frustration;
      avgFrustration = total / this.vehicles.size;
    }
    this.uiRenderer.showGameOver(
      data.reason,
      data.reason === 'avg_frustration' ? avgFrustration : data.value,
      this.score,
      data.timestampGame,
    );
  }

  // ─── Main game loop ────────────────────────────────────────────────────────

  private gameLoop(ticker: PIXI.Ticker): void {
    // Animate oneway arrows regardless of pause state
    this.infraRenderer.update(ticker.deltaMS);

    if (this.gameClockUI.paused || this.gameOver) return;

    // Advance game clock
    const realDeltaS = ticker.deltaMS / 1000;
    this.gameTimeS += realDeltaS * this.gameClockUI.timeScale;

    // Update clock HUD
    this.gameClockUI.updateClock(this.gameTimeS);

    // Render vehicles
    if (this.vehiclesVisible) {
      this.vehicleRenderer.update(this.vehicles, this.vehicleInfraMap);
    }

    // Update sandbox stats
    if (this.sandboxUI) {
      this.sandboxUI.update(this.vehicles.size, this.overlay.app.ticker.FPS);
    }

    // Update satisfaction bar from vehicle states
    this.updateSatisfaction();

    // Update score
    this.updateScore(realDeltaS);
  }

  // ─── Derived state ─────────────────────────────────────────────────────────

  private updateSatisfaction(): void {
    if (this.vehicles.size === 0) return;

    let total = 0;
    for (const v of this.vehicles.values()) {
      total += v.frustration;
    }
    const avg = total / this.vehicles.size;

    // UIRenderer.updateSatisfaction now shows frustration (0=calm, 100=rage)
    this.uiRenderer.updateSatisfaction(avg);
    this.uiRenderer.updateVehicleCount(this.vehicles.size);
  }

  /**
   * Score increases each second based on:
   *  - Number of active vehicles (more = harder = more points)
   *  - Inverse of average frustration (calm traffic = max points)
   * Score rate: vehicles × (1 - avg_frustration/100) × 10 pts/s
   */
  private updateScore(realDeltaS: number): void {
    if (this.vehicles.size === 0) return;
    let total = 0;
    for (const v of this.vehicles.values()) total += v.frustration;
    const avgFrustration = total / this.vehicles.size;

    const rate = this.vehicles.size * (1 - avgFrustration / 100) * 10;
    this.score += rate * realDeltaS;
    this.uiRenderer.updateScore(this.score);
  }

  // ─── Cleanup ───────────────────────────────────────────────────────────────

  destroy(): void {
    this.unlistenCongestion?.();
    this.unlistenLights?.();
    this.unlistenGameOver?.();
    this.sandboxUI?.destroy();
    this.buildingRenderer.destroy();
    this.roadRenderer.destroy();
    this.vehicleRenderer.destroy();
    this.infraRenderer.destroy();
    this.trafficLightRenderer.destroy();
    this.congestionRenderer.destroy();
  }
}
