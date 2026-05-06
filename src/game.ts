import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import { Channel } from '@tauri-apps/api/core';
import type { MapData } from './bridge/commands';
import {
  loadMap,
  startSimulation,
  setTimeScale,
  setMaxVehicles,
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
import { SandboxUI, CITY_PRESETS } from './ui/SandboxUI';
import { LESZNO_BBOX } from './map/MapLibreSetup';
import { ROAD_TYPE_GROUP } from './rendering/RoadRenderer';

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
  private currentBbox: [number, number, number, number] = DEFAULT_BBOX;
  /** null = OSM map; string = sandbox grid type ('mixed'|'one_lane'|…) */
  private currentGridMode: string | null = 'mixed';

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

    // In sandbox, buildings layer starts empty and hidden
    this.overlay.buildings.visible = false;

    ui.onLayerToggle = (group, visible) => {
      this.roadRenderer.setGroupVisible(group, visible);
      // Sync to VehicleRenderer so vehicles on hidden roads also disappear
      this.vehicleRenderer.setHiddenGroups(this.roadRenderer.getHiddenGroups());
      // Sync to TrafficLight renderers so lights on fully-hidden nodes disappear
      const hiddenNodes = this.computeHiddenNodeIds();
      this.trafficLightRenderer.setHiddenNodeIds(hiddenNodes);
      this.trafficLightUI.setHiddenNodeIds(hiddenNodes);
    };

    ui.onMaxVehiclesChange = (count) => {
      if (this.tauriAvailable) {
        setMaxVehicles(count).catch(console.error);
      }
    };

    ui.onReloadMap = (center, sizeM) => {
      // Degrees per metre at given latitude
      const lat = center[1];
      const dLat = (sizeM / 2) / 111320;
      const dLng = (sizeM / 2) / (111320 * Math.cos(lat * Math.PI / 180));
      const bbox: [number, number, number, number] = [
        center[0] - dLng, lat - dLat,
        center[0] + dLng, lat + dLat,
      ];
      const cityName = CITY_PRESETS.find(c => c.center[0] === center[0] && c.center[1] === center[1])?.name ?? 'Custom';
      this.reloadMap(bbox, sizeM, cityName, this.currentGridMode);
    };

    ui.onMapModeChange = (forceSandbox) => {
      this.currentGridMode = forceSandbox;
    };

    ui.onOsmModeToggle = (enabled) => {
      this.roadRenderer.setOsmMode(enabled);
    };

    ui.onMapBgToggle = (visible) => {
      const mapEl = document.getElementById('map-container');
      if (mapEl) mapEl.style.opacity = visible ? '1' : '0';
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
        // Build only on first enable; subsequent toggles just show/hide
        this.buildingRenderer.build(this.mapData);
      }
    };
  }

  // ─── Map loading ───────────────────────────────────────────────────────────

  private async loadMapData(): Promise<void> {
    this.uiRenderer.showNotification('Loading map data…', 'info');
    try {
      this.mapData = await loadMap(this.currentBbox, this.currentGridMode);
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
    this.vehicleRenderer.setEdgeIndex(this.mapData);
    if (SANDBOX_MODE) {
      this.vehicleRenderer.setHiddenGroups(this.roadRenderer.getHiddenGroups());
      const hiddenNodes = this.computeHiddenNodeIds();
      this.trafficLightRenderer.setHiddenNodeIds(hiddenNodes);
      this.trafficLightUI.setHiddenNodeIds(hiddenNodes);
    }
    this.trafficLightUI.init(this.mapData.nodes);
    this.trafficLightRenderer.init(this.mapData.nodes);
  }

  // ─── Map reload (sandbox dynamic area) ────────────────────────────────────

  private async reloadMap(
    bbox: [number, number, number, number],
    sizeM: number,
    cityName: string,
    forceSandbox?: string | null,
  ): Promise<void> {
    this.currentBbox = bbox;
    this.vehicles.clear();

    const modeLabel = forceSandbox
      ? `Siatka (${forceSandbox})`
      : `${cityName} ${sizeM >= 1000 ? sizeM / 1000 + ' km' : sizeM + ' m'}`;
    this.uiRenderer.showNotification(`Przeładowanie – ${modeLabel}…`, 'info');

    try {
      this.mapData = await loadMap(bbox, forceSandbox);
      this.uiRenderer.showNotification(
        `Map reloaded – ${this.mapData.nodes.length} nodes, ${this.mapData.edges.length} edges`,
        'info',
      );
    } catch (err) {
      console.warn('Overpass unavailable on reload:', err);
      this.uiRenderer.showNotification('Reload failed – kept old map', 'error');
      this.sandboxUI?.setLoadingDone(cityName, sizeM);
      return; // keep old map
    }

    // Re-center the camera
    const cx = (bbox[0] + bbox[2]) / 2;
    const cy = (bbox[1] + bbox[3]) / 2;
    this.map.setCenter([cx, cy]);

    this.roadRenderer.build(this.mapData);
    this.infraRenderer.buildStaticLayer(this.mapData);
    this.vehicleRenderer.setEdgeIndex(this.mapData);
    this.vehicleRenderer.setHiddenGroups(this.roadRenderer.getHiddenGroups());

    const hiddenNodes = this.computeHiddenNodeIds();
    this.trafficLightRenderer.setHiddenNodeIds(hiddenNodes);
    this.trafficLightUI.setHiddenNodeIds(hiddenNodes);
    this.trafficLightUI.init(this.mapData.nodes);
    this.trafficLightRenderer.init(this.mapData.nodes);

    this.sandboxUI?.setLoadingDone(cityName, sizeM);
  }

  // ─── Hidden-node helper ────────────────────────────────────────────────────

  /**
   * Returns the set of node IDs where ALL connected road groups are currently
   * hidden.  Used to sync traffic-light visibility with road layer toggles.
   */
  private computeHiddenNodeIds(): Set<number> {
    if (!this.mapData) return new Set();
    const hiddenGroups = this.roadRenderer.getHiddenGroups();
    const nodeGroups = new Map<number, Set<string>>();
    for (const edge of this.mapData.edges) {
      const group = (ROAD_TYPE_GROUP as Record<string, string>)[edge.roadType] ?? 'residential';
      for (const nodeId of [edge.from, edge.to]) {
        if (!nodeGroups.has(nodeId)) nodeGroups.set(nodeId, new Set());
        nodeGroups.get(nodeId)!.add(group);
      }
    }
    const hidden = new Set<number>();
    for (const [nodeId, groups] of nodeGroups) {
      if ([...groups].every(g => hiddenGroups.has(g))) {
        hidden.add(nodeId);
      }
    }
    return hidden;
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

    // Replace map contents: remove IDs absent from this frame (despawned), add/update the rest.
    const frameIds = new Set<number>();
    for (const v of states) frameIds.add(v.id);
    for (const id of this.vehicles.keys()) {
      if (!frameIds.has(id)) this.vehicles.delete(id);
    }
    for (const v of states) this.vehicles.set(v.id, v);
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
