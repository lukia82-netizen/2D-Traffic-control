import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import { Channel } from '@tauri-apps/api/core';
import type { MapData } from './bridge/commands';
import {
  loadMap,
  startSimulation,
} from './bridge/commands';
import {
  parseVehicleFrame,
  listenCongestionUpdates,
  listenLightStateChanges,
} from './bridge/events';
import type { VehicleState, CongestionData, LightStateUpdate } from './bridge/events';
import { PixiOverlay } from './rendering/PixiOverlay';
import { VehicleRenderer } from './rendering/VehicleRenderer';
import { InfraRenderer } from './rendering/InfraRenderer';
import { CongestionRenderer } from './rendering/CongestionRenderer';
import { UIRenderer } from './rendering/UIRenderer';
import { TrafficLightUI } from './traffic/TrafficLightUI';
import { GameClockUI } from './time/GameClockUI';

// Default Kraków bbox: [west, south, east, north]
const DEFAULT_BBOX: [number, number, number, number] = [19.925, 50.052, 19.955, 50.068];

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
    this.vehicleRenderer = new VehicleRenderer(this.overlay, this.map);
    this.infraRenderer = new InfraRenderer(this.overlay, this.map);
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
        this.infraRenderer.rebuildOnCameraChange(this.mapData);
      }
    });

    // Start the PixiJS ticker
    this.overlay.app.ticker.add((ticker) => this.gameLoop(ticker));
  }

  // ─── Map loading ───────────────────────────────────────────────────────────

  private async loadMapData(): Promise<void> {
    try {
      this.uiRenderer.showNotification('Loading map data…', 'info');
      this.mapData = await loadMap(DEFAULT_BBOX);
      this.infraRenderer.buildStaticLayer(this.mapData);
      this.trafficLightUI.init(this.mapData.nodes);
      this.uiRenderer.showNotification(
        `Map loaded – ${this.mapData.nodes.length} nodes, ${this.mapData.edges.length} edges`,
        'info',
      );
    } catch (err) {
      console.error('Failed to load map:', err);
      const message = err instanceof Error ? err.message : String(err);
      this.uiRenderer.showNotification(`Map load failed — ${message}`, 'error');
    }
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
    this.vehicleRenderer.destroy();
    this.infraRenderer.destroy();
    this.congestionRenderer.destroy();
  }
}
