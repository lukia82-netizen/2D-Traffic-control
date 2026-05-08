import * as PIXI from 'pixi.js';
import maplibregl from 'maplibre-gl';
import { Channel } from '@tauri-apps/api/core';
import type { MapData } from './bridge/commands';
import {
  loadMap,
  startSimulation,
  setTimeScale,
  setMaxVehicles,
  setDebugVehicle,
  setEditorTool,
  editorMoveNode,
  editorConnect,
  editorExtrude,
  editorDeleteEdge,
  editorUpdateEdgeTags,
  editorUndo,
  editorRedo,
  saveMapOverrides,
  setDebugVisualization,
} from './bridge/commands';
import {
  parseVehicleFrame,
  listenCongestionUpdates,
  listenLightStateChanges,
  listenGameOver,
  listenIdmDebug,
  listenDebugVisualization,
} from './bridge/events';
import type {
  VehicleState,
  CongestionData,
  LightStateUpdate,
  GameOverPayload,
  IdmDebugPayload,
  DebugVisualizationPayload,
} from './bridge/events';
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
import { EditorOverlay } from './rendering/EditorOverlay';
import { GameClockUI } from './time/GameClockUI';
import { SandboxUI, CITY_PRESETS } from './ui/SandboxUI';
import { MapScenarioEditorUI, type ScenarioData } from './ui/MapScenarioEditorUI';
import { LESZNO_BBOX } from './map/MapLibreSetup';
import { ROAD_TYPE_GROUP } from './rendering/RoadRenderer';
import { MapBboxPicker } from './map/MapBboxPicker';

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

  return {
    nodes,
    edges,
    spawnPoints,
    bbox: DEFAULT_BBOX,
    buildings: [],
    restrictions: [],
    tramStops: [],
    turnConnectors: [],
    lanes: [],
    conflictAreas: [],
  };
}

// Simulation starts at 06:00 (game seconds since midnight)
const GAME_START_TIME_S = 6 * 3600;
const TURN_DEBUG_STORAGE_KEY = 'debug_turn_connectors_visible';
const TURN_DEBUG_ACTIVE_ONLY_STORAGE_KEY = 'debug_turn_connectors_active_only';
const TURN_CONNECTOR_ENTRY_M = 30;
const TURN_CONNECTOR_EXIT_M = 30;
const TURN_CONNECTOR_MIN_ANGLE_RAD = 0.35;
const TURN_CONNECTOR_ACTIVE_MAX_DIST_M = 12;
// Debug arcs drawn at road centre (0 offset) — matches where vehicles travel
// on a connector (laneOffset is zeroed in VehicleRenderer while onTurnConnector).
const TURN_CONNECTOR_DEBUG_LANE_OFFSET_M = 0;

interface TurnConnectorPath {
  points: [number, number][];
  p1: [number, number];
  ctrl: [number, number];
  p2: [number, number];
}

type ObbDebugMode = 'visual' | 'physical';

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
  private editorOverlay!: EditorOverlay;

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
  private unlistenIdmDebug: (() => void) | null = null;
  private unlistenDebugVisualization: (() => void) | null = null;
  private selectedVehicleId: number | null = null;
  private selectedRoutePoints: [number, number][] = [];
  private selectedThreatPoint: [number, number] | null = null;
  private selectedStopLinePoint: [number, number] | null = null;
  private selectedTurnEntryPoint: [number, number] | null = null;
  private selectedThreatShapeLengthM = 0;
  /** From latest idm_debug: hood [lng,lat] for HUD threat line matching Rust. */
  private selectedHudHoodLngLat: [number, number] | null = null;
  private debugRouteGfx: PIXI.Graphics | null = null;
  private debugVizGfx: PIXI.Graphics | null = null;
  private debugConflictLabels: PIXI.Container | null = null;
  /** Full-map CP + threat overlay (Rust `debug_visualization`). */
  private debugVisualizationEnabled = false;
  private obbDebugMode: ObbDebugMode = 'visual';
  private latestDebugVisualization: DebugVisualizationPayload | null = null;
  private turnConnectorGfx: PIXI.Graphics | null = null;
  private turnConnectorPaths: TurnConnectorPath[] = [];
  private showTurnConnectors = false;
  private showTurnConnectorsActiveOnly = false;

  // Scoring
  private score = 0;
  private gameOver = false;

  // Whether the Rust backend is available (desktop Tauri vs browser dev mode)
  private tauriAvailable = false;

  // Sandbox mode
  private sandboxUI: SandboxUI | null = null;
  private mapScenarioEditorUI: MapScenarioEditorUI | null = null;
  private vehiclesVisible = true;
  private currentBbox: [number, number, number, number] = DEFAULT_BBOX;
  /** null = real OSM data; string = sandbox grid type ('mixed'|'one_lane'|'single_road'|…) */
  private currentGridMode: string | null = 'single_road';
  private bboxPicker: MapBboxPicker | null = null;
  private editorMode = true;
  private editorTool: 'select' | 'move_node' | 'add_road' | 'delete' = 'move_node';
  private selectedEdgeIndex: number | null = null;
  private dragNodeId: number | null = null;
  private dragLastSentAt = 0;
  private connectFromNodeId: number | null = null;
  private nextCustomNodeId = 1_000_000;
  private edgeEditorPanel: HTMLDivElement | null = null;
  private toolSwitchPanel: HTMLDivElement | null = null;

  constructor(map: maplibregl.Map, overlay: PixiOverlay) {
    this.map = map;
    this.overlay = overlay;
    this.bboxPicker = new MapBboxPicker(this.map);
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
    this.editorOverlay = new EditorOverlay(this.overlay, this.map);
    this.gameClockUI = new GameClockUI();
    this.debugRouteGfx = new PIXI.Graphics();
    this.turnConnectorGfx = new PIXI.Graphics();
    this.debugVizGfx = new PIXI.Graphics();
    this.debugConflictLabels = new PIXI.Container();
    this.overlay.congestionLayer.addChild(this.debugRouteGfx);
    this.overlay.congestionLayer.addChild(this.turnConnectorGfx);
    this.overlay.congestionLayer.addChild(this.debugVizGfx);
    this.overlay.congestionLayer.addChild(this.debugConflictLabels);

    window.addEventListener('keydown', (ev) => {
      if (!this.tauriAvailable) return;
      const t = ev.target as HTMLElement | null;
      if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.isContentEditable)) return;
      if (ev.key === 'd' || ev.key === 'D') {
        ev.preventDefault();
        const next = !this.debugVisualizationEnabled;
        void this.setDebugVisualizationMode(next);
        this.sandboxUI?.setChecked('debug-visualization', next);
      } else if (ev.key === 'o' || ev.key === 'O') {
        ev.preventDefault();
        this.obbDebugMode = this.obbDebugMode === 'visual' ? 'physical' : 'visual';
        this.uiRenderer.showNotification(
          `OBB debug mode: ${this.obbDebugMode.toUpperCase()} (O = switch)`,
          'info',
        );
        if (this.debugVisualizationEnabled && this.latestDebugVisualization) {
          this.redrawFullDebugVisualization();
        }
      }
    });
    await this.vehicleRenderer.init();

    // Init HUD controls
    this.gameClockUI.init();

    // ── Sandbox UI ────────────────────────────────────────────────────────
    if (SANDBOX_MODE) {
      this.sandboxUI = new SandboxUI();
      this.wireSandboxUI();
      this.mapScenarioEditorUI = new MapScenarioEditorUI();
      this.wireMapScenarioEditorUI();
    }
    this.showTurnConnectors = localStorage.getItem(TURN_DEBUG_STORAGE_KEY) === '1';
    this.showTurnConnectorsActiveOnly =
      localStorage.getItem(TURN_DEBUG_ACTIVE_ONLY_STORAGE_KEY) === '1';
    if (this.turnConnectorGfx) {
      this.turnConnectorGfx.visible = this.showTurnConnectors;
    }

    if (this.tauriAvailable) {
      await setEditorTool(this.editorTool);
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
        this.redrawSelectedRoute();
        this.redrawTurnConnectors();
        if (this.debugVisualizationEnabled && this.latestDebugVisualization) {
          this.redrawFullDebugVisualization();
        }
      }
    });
    this.map.on('click', (e) => {
      if (this.editorMode && this.mapData) {
        if (this.handleEditorClick(e.point.x, e.point.y)) return;
      }
      const consumed = this.trafficLightUI.handleMapClick(
        e.lngLat.lng,
        e.lngLat.lat,
        e.point.x,
        e.point.y,
        this.tauriAvailable,
      );
      if (consumed) return;
      void this.selectVehicleAtScreenPoint(e.point.x, e.point.y);
    });
    this.bindEditorPointerHandlers();
    this.bindEditorKeyboardHandlers();
    this.initEdgeEditorPanel();
    this.initToolSwitchPanel();

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
    // Apply the UI's default immediately so the backend cap matches the displayed value.
    if (this.tauriAvailable) {
      setMaxVehicles(20).catch(console.error);
    }

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
    ui.onBboxPickRequest = () => {
      this.uiRenderer.showNotification('Zaznacz obszar na mapie przeciągnięciem', 'info');
      this.bboxPicker?.start(async (bbox) => {
        const centerLng = (bbox[0] + bbox[2]) / 2;
        const centerLat = (bbox[1] + bbox[3]) / 2;
        const latSizeM = (bbox[3] - bbox[1]) * 111320;
        const lngSizeM = (bbox[2] - bbox[0]) * 111320 * Math.cos(centerLat * Math.PI / 180);
        const sizeM = Math.max(100, Math.round(Math.max(latSizeM, lngSizeM)));
        const cityName = CITY_PRESETS.find(c =>
          Math.abs(c.center[0] - centerLng) < 0.02 &&
          Math.abs(c.center[1] - centerLat) < 0.02,
        )?.name ?? 'Custom BBOX';
        await this.reloadMap(bbox, sizeM, cityName, this.currentGridMode);
      });
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
    ui.onTurnConnectorsToggle = (visible) => {
      this.showTurnConnectors = visible;
      localStorage.setItem(TURN_DEBUG_STORAGE_KEY, visible ? '1' : '0');
      if (this.turnConnectorGfx) {
        this.turnConnectorGfx.visible = visible;
      }
      this.redrawTurnConnectors();
    };
    ui.onTurnConnectorsActiveOnlyToggle = (activeOnly) => {
      this.showTurnConnectorsActiveOnly = activeOnly;
      localStorage.setItem(TURN_DEBUG_ACTIVE_ONLY_STORAGE_KEY, activeOnly ? '1' : '0');
      this.redrawTurnConnectors();
    };
    ui.onDebugVisualizationToggle = (enabled) => {
      void this.setDebugVisualizationMode(enabled);
    };
    ui.setChecked('turn-connectors', this.showTurnConnectors);
    ui.setChecked('turn-connectors-active-only', this.showTurnConnectorsActiveOnly);
    ui.setChecked('debug-visualization', false);
  }

  private wireMapScenarioEditorUI(): void {
    const editor = this.mapScenarioEditorUI;
    if (!editor) return;
    editor.onApplyMap = (mapData) => {
      this.applyCustomMapData(mapData);
      this.uiRenderer.showNotification('Wczytano mapę z edytora', 'info');
    };
    editor.onApplyScenario = (scenario) => {
      this.applyScenario(scenario);
      this.uiRenderer.showNotification(`Scenariusz uruchomiony: ${scenario.name}`, 'info');
    };
    if (this.mapData) {
      editor.setMapData(this.mapData);
    }
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
      console.warn('Failed to load live map data; using offline demo network:', err);
      const message = err instanceof Error ? err.message : String(err);
      this.mapData = buildDemoMapData();
      this.uiRenderer.showNotification(
        `Live map unavailable — demo network active (${message})`,
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
    this.trafficLightRenderer.init(this.mapData.nodes, this.mapData.edges);
    this.rebuildTurnConnectorPaths();
    this.redrawTurnConnectors();
    this.editorOverlay.setEnabled(this.editorMode);
    this.editorOverlay.redrawHandles(this.mapData);
    this.mapScenarioEditorUI?.setMapData(this.mapData);
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
    this.trafficLightRenderer.init(this.mapData.nodes, this.mapData.edges);
    this.rebuildTurnConnectorPaths();
    this.redrawTurnConnectors();
    this.mapScenarioEditorUI?.setMapData(this.mapData);

    this.sandboxUI?.setLoadingDone(cityName, sizeM);
  }

  private applyCustomMapData(mapData: MapData): void {
    if (!(mapData as Partial<MapData>).turnConnectors) {
      (mapData as MapData).turnConnectors = [];
    }
    this.mapData = mapData;
    this.vehicles.clear();
    this.gameOver = false;
    this.roadRenderer.build(mapData);
    this.infraRenderer.buildStaticLayer(mapData);
    this.vehicleRenderer.setEdgeIndex(mapData);
    if (this.overlay.buildings.visible) {
      this.buildingRenderer.build(mapData);
    }
    const hiddenNodes = this.computeHiddenNodeIds();
    this.trafficLightRenderer.setHiddenNodeIds(hiddenNodes);
    this.trafficLightUI.setHiddenNodeIds(hiddenNodes);
    this.trafficLightUI.init(mapData.nodes);
    this.trafficLightRenderer.init(mapData.nodes, mapData.edges);
    this.rebuildTurnConnectorPaths();
    this.redrawTurnConnectors();
    this.editorOverlay.redrawHandles(mapData);
  }

  private applyScenario(scenario: ScenarioData): void {
    this.applyCustomMapData(scenario.mapData);
    this.gameTimeS = scenario.startTimeS;
    this.gameClockUI.updateClock(this.gameTimeS);
    this.gameClockUI.setTimeScaleValue(scenario.timeScale);
    if (this.tauriAvailable) {
      setMaxVehicles(scenario.maxVehicles).catch(console.error);
    }
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
    this.unlistenIdmDebug = await listenIdmDebug((data) =>
      this.onIdmDebug(data),
    );
    this.unlistenDebugVisualization = await listenDebugVisualization((data) =>
      this.onDebugVisualization(data),
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

  private onIdmDebug(data: IdmDebugPayload): void {
    this.uiRenderer.updateIdmDebug(data);
    if (this.selectedVehicleId !== null && data.vehicleId === this.selectedVehicleId) {
      this.uiRenderer.updateVehicleTelemetrySelected({
        vehicleId: data.vehicleId,
        speed: data.speed,
        desiredSpeed: data.desiredSpeed,
        acceleration: data.acceleration,
        distanceToLeader: data.distanceToLeader,
      });
    }
    if (this.selectedVehicleId === null || data.vehicleId === this.selectedVehicleId) {
      this.selectedVehicleId = data.vehicleId;
      this.selectedRoutePoints = data.routePoints ?? [];
      this.selectedThreatPoint = data.threatPoint ?? null;
      this.selectedStopLinePoint = data.stopLinePoint ?? null;
      this.selectedTurnEntryPoint = data.turnEntryPoint ?? null;
      this.selectedThreatShapeLengthM = data.shapeLengthM ?? 0;
      this.selectedHudHoodLngLat = data.hoodLngLat ?? null;
      this.redrawSelectedRoute();
    }
  }

  private onDebugVisualization(data: DebugVisualizationPayload): void {
    this.latestDebugVisualization = data;
  }

  /** Toggle CP / IDM map overlay — syncs Sandbox checkbox + key D. */
  private async setDebugVisualizationMode(enabled: boolean): Promise<void> {
    this.debugVisualizationEnabled = enabled;
    if (this.debugVizGfx) {
      this.debugVizGfx.visible = enabled;
    }
    if (this.debugConflictLabels) {
      this.debugConflictLabels.visible = enabled;
    }
    if (!enabled) {
      this.latestDebugVisualization = null;
      this.debugVizGfx?.clear();
      this.debugConflictLabels?.removeChildren();
    }
    if (this.tauriAvailable) {
      await setDebugVisualization(enabled).catch(console.error);
    }
  }

  private redrawFullDebugVisualization(): void {
    if (!this.debugVisualizationEnabled || !this.latestDebugVisualization) return;
    const gfx = this.debugVizGfx;
    const lbl = this.debugConflictLabels;
    if (!gfx || !lbl) return;

    gfx.clear();
    lbl.removeChildren();

    const DATA = this.latestDebugVisualization;
    const VEHICLE_WIDTH_FILL: Record<number, number> = {
      0: 0.76, 1: 0.84, 2: 0.90, 3: 0.94, 4: 0.90,
    };
    const VEHICLE_LENGTH_FACTOR: Record<number, number> = {
      0: 1.9, 1: 2.2, 2: 2.8, 3: 3.2, 4: 4.2,
    };
    const metersToPixelsAt = (lng: number, lat: number, meters: number): number => {
      const p0 = this.map.project([lng, lat]);
      const p1 = this.map.project([lng + meters / 111_320.0, lat]);
      return Math.hypot(p1.x - p0.x, p1.y - p0.y);
    };
    const vehicleVisualObb = (vehicleId: number): { center: { x: number; y: number }; corners: { x: number; y: number }[] } | null => {
      const v = this.vehicles.get(vehicleId);
      if (!v) return null;
      const s = { lng: v.lng, lat: v.lat, angle: v.angle };
      const px = this.map.project([s.lng, s.lat]);
      const cx = px.x;
      const cy = px.y;
      const laneWidthPx = this.camera.getLaneOffset() * 2;
      const widthFill = VEHICLE_WIDTH_FILL[v.vehicleType] ?? VEHICLE_WIDTH_FILL[0];
      const lengthFactor = VEHICLE_LENGTH_FACTOR[v.vehicleType] ?? VEHICLE_LENGTH_FACTOR[0];
      const width = Math.max(4, laneWidthPx * widthFill);
      const length = width * lengthFactor;
      const hx = width * 0.5;
      const hy = length * 0.5;
      const c = Math.cos(s.angle);
      const si = Math.sin(s.angle);
      const rotate = (lx: number, ly: number) => ({ x: cx + lx * c - ly * si, y: cy + lx * si + ly * c });
      return {
        center: { x: cx, y: cy },
        corners: [rotate(-hx, -hy), rotate(hx, -hy), rotate(hx, hy), rotate(-hx, hy)],
      };
    };
    const touchedConflictIds = new Set<number>();
    let selectedRouteCpIds: Set<number> | null = null;
    if (this.selectedVehicleId != null) {
      const thSel = DATA.vehicleThreats.find((t) => t.vehicleId === this.selectedVehicleId);
      if (thSel) selectedRouteCpIds = new Set(thSel.routeConflictPointIds ?? []);
    }
    for (const th of DATA.vehicleThreats) {
      for (const id of th.collidingConflictPointIds ?? []) touchedConflictIds.add(id);
    }
    const laneColor = 0x22ff22;
    const laneLegend = new Map<string, { color: number; label: string }>();
    for (const lp of DATA.lanePaths ?? []) {
      if (!lp.points || lp.points.length < 2) continue;
      const pts = lp.points.map(([lng, lat]) => this.map.project([lng, lat]));
      const c = laneColor;
      if (!laneLegend.has(lp.lanePathId)) {
        const parts = lp.lanePathId.split(':');
        const label = parts.length === 4
          ? `${parts[0]}->${parts[1]} | L${parts[2]}->L${parts[3]}`
          : lp.lanePathId;
        laneLegend.set(lp.lanePathId, { color: c, label });
      }
      gfx.moveTo(pts[0].x, pts[0].y);
      for (let i = 1; i < pts.length; i++) gfx.lineTo(pts[i].x, pts[i].y);
      gfx.stroke({ color: c, alpha: 0.8, width: 1.4 });
    }
    if (laneLegend.size > 0) {
      const title = new PIXI.Text({
        text: 'Lane Paths (color legend)',
        style: { fontFamily: 'Inter, Segoe UI, sans-serif', fontSize: 10, fill: 0xe5e7eb },
      });
      title.x = 12;
      title.y = 10;
      title.alpha = 0.95;
      lbl.addChild(title);
      let row = 0;
      for (const item of laneLegend.values()) {
        if (row >= 12) break; // keep overlay readable
        const y = 26 + row * 14;
        gfx.moveTo(12, y + 5);
        gfx.lineTo(28, y + 5);
        gfx.stroke({ color: item.color, alpha: 0.95, width: 2.2 });
        const tx = new PIXI.Text({
          text: item.label,
          style: { fontFamily: 'Inter, Segoe UI, sans-serif', fontSize: 9, fill: 0xcbd5e1 },
        });
        tx.x = 34;
        tx.y = y - 1;
        tx.alpha = 0.94;
        lbl.addChild(tx);
        row++;
      }
    }

    for (const cp of DATA.conflictPoints) {
      if (selectedRouteCpIds && !selectedRouteCpIds.has(cp.id)) continue;
      const p = this.map.project([cp.lng, cp.lat]);
      const reserved = cp.reservedBy !== null && cp.reservedBy !== undefined;
      const touchingObb = cp.collidingWithObb || touchedConflictIds.has(cp.id);
      const radiusPx = Math.max(3.5, metersToPixelsAt(cp.lng, cp.lat, cp.radiusM));
      gfx.circle(p.x, p.y, radiusPx);
      gfx.fill({ color: touchingObb ? 0xfacc15 : reserved ? 0xdc2626 : 0x22c55e, alpha: 0.2 });
      gfx.stroke({ color: touchingObb ? 0xfacc15 : reserved ? 0xdc2626 : 0x22c55e, alpha: 0.95, width: touchingObb ? 2.5 : 1.8 });
      gfx.circle(p.x, p.y, reserved ? 4.5 : 3.0);
      gfx.fill({ color: touchingObb ? 0xf59e0b : reserved ? 0xdc2626 : 0x22c55e, alpha: 0.95 });
      gfx.stroke({ color: 0x0f172a, alpha: 0.85, width: 1.2 });

      if (reserved) {
        const t = new PIXI.Text({
          text: `Reserved by Car ID: ${cp.reservedBy}`,
          style: { fontFamily: 'Inter, Segoe UI, sans-serif', fontSize: 9, fill: 0xfff1f2 },
        });
        t.x = p.x + 8;
        t.y = p.y - 14;
        t.alpha = 0.95;
        lbl.addChild(t);
      }
    }

    const drawDashed = (x0: number, y0: number, x1: number, y1: number, w: number, col: number) => {
      const dx = x1 - x0;
      const dy = y1 - y0;
      const len = Math.hypot(dx, dy);
      if (len < 4) return;
      const ux = dx / len;
      const uy = dy / len;
      const dash = 5;
      const gap = 4;
      let t = 0;
      let on = true;
      let cx = x0;
      let cy = y0;
      while (t < len) {
        const step = on ? dash : gap;
        const nt = Math.min(len, t + step);
        const nx = x0 + ux * nt;
        const ny = y0 + uy * nt;
        if (on) {
          gfx.moveTo(cx, cy);
          gfx.lineTo(nx, ny);
        }
        cx = nx;
        cy = ny;
        t = nt;
        on = !on;
      }
      gfx.stroke({ color: col, alpha: 0.95, width: w });
    };

    for (const th of DATA.vehicleThreats) {
      const visualObb = vehicleVisualObb(th.vehicleId);
      const physicalCenter = this.map.project(th.centerLngLat);
      const pc = this.obbDebugMode === 'visual'
        ? (visualObb?.center ?? physicalCenter)
        : physicalCenter;
      gfx.circle(pc.x, pc.y, 2.8);
      gfx.fill({ color: 0xf8fafc, alpha: 0.95 });
      gfx.stroke({ color: 0x111827, alpha: 0.9, width: 1.2 });
      const [lngH, latH] = th.hoodLngLat;
      const p0 = this.map.project([lngH, latH]);
      const pComfort = this.map.project(th.comfortBrakeEndLngLat);
      const pEmergency = this.map.project(th.emergencyBrakeEndLngLat);
      const pRight = this.map.project(th.rightArrowLngLat);
      const [lngR, latR] = th.rearBumperLngLat;
      const pb = this.map.project([lngR, latR]);
      gfx.moveTo(pb.x - 4, pb.y - 4);
      gfx.lineTo(pb.x + 4, pb.y + 4);
      gfx.moveTo(pb.x + 4, pb.y - 4);
      gfx.lineTo(pb.x - 4, pb.y + 4);
      gfx.stroke({ color: 0x3b82f6, alpha: 0.98, width: 2 });
      // Stopping-distance probes ahead of hood:
      // green = comfortable braking, red = emergency braking.
      gfx.moveTo(p0.x, p0.y);
      gfx.lineTo(pComfort.x, pComfort.y);
      gfx.stroke({ color: 0x22c55e, alpha: 0.75, width: 2 });
      gfx.moveTo(p0.x, p0.y);
      gfx.lineTo(pEmergency.x, pEmergency.y);
      gfx.stroke({ color: 0xef4444, alpha: 0.82, width: 2.4 });
      if (th.emergencyBrakingActive) {
        const et = new PIXI.Text({
          text: 'EMERGENCY',
          style: { fontFamily: 'Inter, Segoe UI, sans-serif', fontSize: 10, fill: 0xfca5a5 },
        });
        et.x = p0.x - 28;
        et.y = p0.y - 38;
        et.alpha = 0.96;
        lbl.addChild(et);
      }
      if (this.obbDebugMode === 'visual' && visualObb) {
        const obb = visualObb.corners;
        const obbTouched = (th.collidingConflictPointIds?.length ?? 0) > 0;
        gfx.moveTo(obb[0].x, obb[0].y);
        for (let i = 1; i < obb.length; i++) gfx.lineTo(obb[i].x, obb[i].y);
        gfx.lineTo(obb[0].x, obb[0].y);
        gfx.stroke({ color: obbTouched ? 0xfacc15 : 0x38bdf8, alpha: 0.95, width: obbTouched ? 2.8 : 1.6 });
      } else if (this.obbDebugMode === 'physical' && th.obbCorners && th.obbCorners.length >= 4) {
        const obb = th.obbCorners.map(([lng, lat]) => this.map.project([lng, lat]));
        const obbTouched = (th.collidingConflictPointIds?.length ?? 0) > 0;
        gfx.moveTo(obb[0].x, obb[0].y);
        for (let i = 1; i < obb.length; i++) gfx.lineTo(obb[i].x, obb[i].y);
        gfx.lineTo(obb[0].x, obb[0].y);
        gfx.stroke({ color: obbTouched ? 0xfacc15 : 0xa78bfa, alpha: 0.95, width: obbTouched ? 2.8 : 1.6 });
      }

      // Blue right-arrow from hood: priority sector probe.
      const arrowCol = th.rightArrowActive ? 0x3b82f6 : 0x9ca3af;
      gfx.moveTo(p0.x, p0.y);
      gfx.lineTo(pRight.x, pRight.y);
      gfx.stroke({ color: arrowCol, alpha: 0.95, width: 2 });
      const ahx = pRight.x - p0.x;
      const ahy = pRight.y - p0.y;
      const ahl = Math.hypot(ahx, ahy) || 1;
      const ux = ahx / ahl;
      const uy = ahy / ahl;
      const wing = 5;
      gfx.moveTo(pRight.x, pRight.y);
      gfx.lineTo(pRight.x - ux * 8 - uy * wing, pRight.y - uy * 8 + ux * wing);
      gfx.moveTo(pRight.x, pRight.y);
      gfx.lineTo(pRight.x - ux * 8 + uy * wing, pRight.y - uy * 8 - ux * wing);
      gfx.stroke({ color: th.rightArrowActive ? 0x60a5fa : 0xd1d5db, alpha: 0.95, width: 2 });

      if (th.hasSignalPriority) {
        const shield = new PIXI.Text({
          text: '🛡',
          style: { fontFamily: 'Segoe UI Emoji, Apple Color Emoji, sans-serif', fontSize: 12, fill: 0xe5e7eb },
        });
        shield.x = p0.x - 6;
        shield.y = p0.y - 36;
        shield.alpha = 0.95;
        lbl.addChild(shield);
      }

      if (th.reservationPath && th.reservationPath.length >= 2) {
        const rp = th.reservationPath.map(([lng, lat]) => this.map.project([lng, lat]));
        gfx.moveTo(rp[0].x, rp[0].y);
        for (let i = 1; i < rp.length; i++) gfx.lineTo(rp[i].x, rp[i].y);
        gfx.stroke({ color: 0x22d3ee, alpha: 0.9, width: 3 });
      }

      if (th.debugState) {
        const txt = new PIXI.Text({
          text: th.debugState,
          style: { fontFamily: 'Inter, Segoe UI, sans-serif', fontSize: 10, fill: 0xfef08a },
        });
        txt.x = p0.x - 22;
        txt.y = p0.y - 24;
        txt.alpha = 0.95;
        lbl.addChild(txt);
      }

      if (!th.threatLngLat) continue;
      const p1 = this.map.project(th.threatLngLat);
      const thick = th.lineStyle === 'thick';
      const dashed = th.lineStyle === 'dashed';
      const lw = thick ? 5 : dashed ? 2 : 2.5;
      const col = 0xef4444;
      if (dashed) {
        drawDashed(p0.x, p0.y, p1.x, p1.y, lw, col);
      } else {
        gfx.moveTo(p0.x, p0.y);
        gfx.lineTo(p1.x, p1.y);
        gfx.stroke({ color: col, alpha: 0.95, width: lw });
      }

      if (th.yieldToVehicleLngLat) {
        const py = this.map.project(th.yieldToVehicleLngLat);
        const ycol = (!th.rightArrowActive && th.hasSignalPriority) ? 0x9ca3af : 0x22c55e;
        gfx.moveTo(p0.x, p0.y);
        gfx.lineTo(py.x, py.y);
        gfx.stroke({ color: ycol, alpha: 0.95, width: 2.2 });
        const ytxt = new PIXI.Text({
          text: `YIELD${th.yieldToVehicleId != null ? ` #${th.yieldToVehicleId}` : ''}`,
          style: {
            fontFamily: 'Inter, Segoe UI, sans-serif',
            fontSize: 10,
            fill: (!th.rightArrowActive && th.hasSignalPriority) ? 0xe5e7eb : 0x86efac,
          },
        });
        ytxt.x = (p0.x + py.x) * 0.5 + 4;
        ytxt.y = (p0.y + py.y) * 0.5 - 10;
        ytxt.alpha = 0.95;
        lbl.addChild(ytxt);
      }

    }
  }

  private async selectVehicleAtScreenPoint(x: number, y: number): Promise<void> {
    if (this.vehicles.size === 0) {
      return;
    }
    let bestId: number | null = null;
    let bestDist = Number.POSITIVE_INFINITY;
    const MAX_PICK_PX = 28;
    for (const v of this.vehicles.values()) {
      const p = this.map.project([v.lng, v.lat]);
      const dx = p.x - x;
      const dy = p.y - y;
      const d = Math.hypot(dx, dy);
      if (d < bestDist) {
        bestDist = d;
        bestId = v.id;
      }
    }
    if (bestId !== null && bestDist <= MAX_PICK_PX) {
      this.selectedVehicleId = bestId;
      await setDebugVehicle(bestId).catch(console.error);
      this.uiRenderer.showNotification(`Debug vehicle #${bestId}`, 'info');
    } else {
      this.selectedVehicleId = null;
      this.selectedRoutePoints = [];
      this.selectedThreatPoint = null;
      this.selectedStopLinePoint = null;
      this.selectedTurnEntryPoint = null;
      this.selectedThreatShapeLengthM = 0;
      this.selectedHudHoodLngLat = null;
      this.redrawSelectedRoute();
      this.uiRenderer.updateVehicleTelemetrySelected({
        vehicleId: null,
        speed: 0,
        desiredSpeed: 0,
        acceleration: 0,
        distanceToLeader: 0,
      });
      await setDebugVehicle(null).catch(console.error);
    }
  }

  private redrawSelectedRoute(): void {
    if (!this.debugRouteGfx) return;
    this.debugRouteGfx.clear();
    if (this.selectedRoutePoints.length < 2) return;
    const pts = this.selectedRoutePoints.map(([lng, lat]) => this.map.project([lng, lat]));
    this.debugRouteGfx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < pts.length; i++) {
      this.debugRouteGfx.lineTo(pts[i].x, pts[i].y);
    }
    this.debugRouteGfx.stroke({ color: 0x22d3ee, alpha: 0.95, width: 3 });

    // Red line: current IDM braking target (virtual leader / conflict point / stop line).
    if (this.selectedThreatPoint && this.selectedVehicleId !== null) {
      const v = this.vehicles.get(this.selectedVehicleId);
      if (v) {
        const hood = this.selectedHudHoodLngLat
          ?? this.computeVehicleHoodLngLat(v, this.selectedThreatShapeLengthM);
        const p0 = this.map.project(hood);
        const p1 = this.map.project(this.selectedThreatPoint);
        this.debugRouteGfx.moveTo(p0.x, p0.y);
        this.debugRouteGfx.lineTo(p1.x, p1.y);
        this.debugRouteGfx.stroke({ color: 0xef4444, alpha: 1.0, width: 3 });
      }
    }

    if (this.selectedStopLinePoint) {
      const ps = this.map.project(this.selectedStopLinePoint);
      this.debugRouteGfx.circle(ps.x, ps.y, 6);
      this.debugRouteGfx.fill({ color: 0xf59e0b, alpha: 0.95 });
      this.debugRouteGfx.stroke({ color: 0x0f172a, alpha: 0.95, width: 2 });
    }
    if (this.selectedTurnEntryPoint) {
      const pe = this.map.project(this.selectedTurnEntryPoint);
      this.debugRouteGfx.rect(pe.x - 5, pe.y - 5, 10, 10);
      this.debugRouteGfx.fill({ color: 0xa78bfa, alpha: 0.95 });
      this.debugRouteGfx.stroke({ color: 0x0f172a, alpha: 0.95, width: 2 });
    }
  }

  private computeVehicleHoodLngLat(v: VehicleState, lengthM: number): [number, number] {
    const half = Math.max(1.0, lengthM * 0.5);
    // Rust heading convention: angle = atan2(d_lng, d_lat)
    // => north component = cos(angle), east component = sin(angle).
    const northM = Math.cos(v.angle) * half;
    const eastM = Math.sin(v.angle) * half;
    const lat = v.lat + northM / 111_320.0;
    const lng = v.lng + eastM / 71_700.0;
    return [lng, lat];
  }

  private rebuildTurnConnectorPaths(): void {
    this.turnConnectorPaths = [];
    if (!this.mapData) return;
    const backendTurnConnectors = this.mapData.turnConnectors ?? [];
    if (backendTurnConnectors.length > 0) {
      this.turnConnectorPaths = backendTurnConnectors.map((tc) => ({
        points: tc.bezierLut,
        p1: tc.bezierLut[0] ?? [0, 0],
        ctrl: tc.bezierLut[Math.floor(tc.bezierLut.length / 2)] ?? [0, 0],
        p2: tc.bezierLut[tc.bezierLut.length - 1] ?? [0, 0],
      }));
      return;
    }

    const nodeById = new Map(this.mapData.nodes.map((n) => [n.id, n]));
    const incomingByNode = new Map<number, typeof this.mapData.edges>();
    const outgoingByNode = new Map<number, typeof this.mapData.edges>();

    for (const edge of this.mapData.edges) {
      if (!incomingByNode.has(edge.to)) incomingByNode.set(edge.to, []);
      if (!outgoingByNode.has(edge.from)) outgoingByNode.set(edge.from, []);
      incomingByNode.get(edge.to)!.push(edge);
      outgoingByNode.get(edge.from)!.push(edge);
    }

    for (const junction of this.mapData.nodes) {
      const incoming = incomingByNode.get(junction.id) ?? [];
      const outgoing = outgoingByNode.get(junction.id) ?? [];
      if (incoming.length === 0 || outgoing.length === 0) continue;

      for (const inEdge of incoming) {
        for (const outEdge of outgoing) {
          if (inEdge.from === outEdge.to) continue;
          const inSrc = nodeById.get(inEdge.from);
          const outTgt = nodeById.get(outEdge.to);
          if (!inSrc || !outTgt) continue;
          const angle = this.turnAngleRad(inSrc.lng, inSrc.lat, junction.lng, junction.lat, outTgt.lng, outTgt.lat);
          if (angle < TURN_CONNECTOR_MIN_ANGLE_RAD) continue;
          const path = this.buildTurnConnectorPath(inSrc, junction, outTgt, inEdge.lengthM, outEdge.lengthM);
          if (path.points.length >= 2) this.turnConnectorPaths.push(path);
        }
      }

    }
  }

  private redrawTurnConnectors(): void {
    const gfx = this.turnConnectorGfx;
    if (!gfx) return;
    gfx.clear();
    if (!this.showTurnConnectors || this.turnConnectorPaths.length === 0) return;

    const activeVehicles = this.showTurnConnectorsActiveOnly
      ? [...this.vehicles.values()].filter((v) => v.onTurnConnector)
      : [];

    const drawPath = (pathPoints: [number, number][]): void => {
      const pts = pathPoints.map(([lng, lat]) => this.map.project([lng, lat]));
      if (pts.length < 2) return;
      gfx.moveTo(pts[0].x, pts[0].y);
      for (let i = 1; i < pts.length; i++) {
        gfx.lineTo(pts[i].x, pts[i].y);
      }
    };

    for (const path of this.turnConnectorPaths) {
      if (
        this.showTurnConnectorsActiveOnly &&
        !this.connectorPathIsActive(path.points, activeVehicles)
      ) {
        continue;
      }
      drawPath(path.points);
    }
    // Outer glow-like under-stroke for high contrast against roads/map.
    gfx.stroke({ color: 0x111827, alpha: 0.9, width: 7 });

    for (const path of this.turnConnectorPaths) {
      if (
        this.showTurnConnectorsActiveOnly &&
        !this.connectorPathIsActive(path.points, activeVehicles)
      ) {
        continue;
      }
      drawPath(path.points);
    }
    gfx.stroke({ color: 0x22d3ee, alpha: 1.0, width: 4 });

    // Control-point markers: P1 (green), C (yellow), P2 (magenta).
    for (const path of this.turnConnectorPaths) {
      if (
        this.showTurnConnectorsActiveOnly &&
        !this.connectorPathIsActive(path.points, activeVehicles)
      ) {
        continue;
      }
      this.drawTurnMarker(gfx, path.p1, 0x22c55e, 5);
      this.drawTurnMarker(gfx, path.ctrl, 0xfacc15, 5);
      this.drawTurnMarker(gfx, path.p2, 0xe879f9, 5);
    }
  }

  private connectorPathIsActive(
    points: [number, number][],
    activeVehicles: VehicleState[],
  ): boolean {
    for (const v of activeVehicles) {
      for (const [lng, lat] of points) {
        if (this.geoDistApproxM(v.lat, v.lng, lat, lng) <= TURN_CONNECTOR_ACTIVE_MAX_DIST_M) {
          return true;
        }
      }
    }
    return false;
  }

  private buildTurnConnectorPath(
    inSrc: { lng: number; lat: number },
    junction: { lng: number; lat: number },
    outTgt: { lng: number; lat: number },
    inLenM: number,
    outLenM: number,
  ): TurnConnectorPath {
    const entryT = Math.max(0, Math.min(1, 1 - TURN_CONNECTOR_ENTRY_M / Math.max(inLenM, 1)));
    const exitT = Math.max(0, Math.min(1, TURN_CONNECTOR_EXIT_M / Math.max(outLenM, 1)));
    const p1BaseLng = inSrc.lng + (junction.lng - inSrc.lng) * entryT;
    const p1BaseLat = inSrc.lat + (junction.lat - inSrc.lat) * entryT;
    const p2BaseLng = junction.lng + (outTgt.lng - junction.lng) * exitT;
    const p2BaseLat = junction.lat + (outTgt.lat - junction.lat) * exitT;

    const lngM = 71_700;
    const latM = 111_320;
    const inFxRaw = (junction.lng - inSrc.lng) * lngM;
    const inFyRaw = (junction.lat - inSrc.lat) * latM;
    const outFxRaw = (outTgt.lng - junction.lng) * lngM;
    const outFyRaw = (outTgt.lat - junction.lat) * latM;
    const inLen = Math.hypot(inFxRaw, inFyRaw) || 1e-9;
    const outLen = Math.hypot(outFxRaw, outFyRaw) || 1e-9;
    const inFx = inFxRaw / inLen;
    const inFy = inFyRaw / inLen;
    const outFx = outFxRaw / outLen;
    const outFy = outFyRaw / outLen;

    // Right-hand lane center offset for debug curve visualization.
    const inRx = inFy;
    const inRy = -inFx;
    const outRx = outFy;
    const outRy = -outFx;
    const p1Lng = p1BaseLng + (inRx * TURN_CONNECTOR_DEBUG_LANE_OFFSET_M) / lngM;
    const p1Lat = p1BaseLat + (inRy * TURN_CONNECTOR_DEBUG_LANE_OFFSET_M) / latM;
    const p2Lng = p2BaseLng + (outRx * TURN_CONNECTOR_DEBUG_LANE_OFFSET_M) / lngM;
    const p2Lat = p2BaseLat + (outRy * TURN_CONNECTOR_DEBUG_LANE_OFFSET_M) / latM;

    // Build control point from tangent-line intersection for smoother entry/exit heading.
    const p1x = p1Lng * lngM;
    const p1y = p1Lat * latM;
    const p2x = p2Lng * lngM;
    const p2y = p2Lat * latM;
    const det = inFx * (-outFy) - inFy * (-outFx);
    let ctrlLng = junction.lng;
    let ctrlLat = junction.lat;
    if (Math.abs(det) > 1e-9) {
      const dx = p2x - p1x;
      const dy = p2y - p1y;
      const t = (dx * (-outFy) - dy * (-outFx)) / det;
      const cx = p1x + t * inFx;
      const cy = p1y + t * inFy;
      ctrlLng = cx / lngM;
      ctrlLat = cy / latM;
    }

    const points: [number, number][] = [];
    const samples = 14;
    for (let i = 0; i <= samples; i++) {
      const t = i / samples;
      const u = 1 - t;
      const lng = u * u * p1Lng + 2 * u * t * ctrlLng + t * t * p2Lng;
      const lat = u * u * p1Lat + 2 * u * t * ctrlLat + t * t * p2Lat;
      points.push([lng, lat]);
    }
    return {
      points,
      p1: [p1Lng, p1Lat],
      ctrl: [ctrlLng, ctrlLat],
      p2: [p2Lng, p2Lat],
    };
  }

  private turnAngleRad(
    inLng: number,
    inLat: number,
    jLng: number,
    jLat: number,
    outLng: number,
    outLat: number,
  ): number {
    const ax = jLng - inLng;
    const ay = jLat - inLat;
    const bx = outLng - jLng;
    const by = outLat - jLat;
    const al = Math.hypot(ax, ay) || 1e-9;
    const bl = Math.hypot(bx, by) || 1e-9;
    const dot = Math.max(-1, Math.min(1, (ax / al) * (bx / bl) + (ay / al) * (by / bl)));
    return Math.acos(dot);
  }

  private geoDistApproxM(lat1: number, lng1: number, lat2: number, lng2: number): number {
    const dLatM = (lat2 - lat1) * 111_320;
    const dLngM = (lng2 - lng1) * 71_700;
    return Math.hypot(dLatM, dLngM);
  }

  private drawTurnMarker(
    gfx: PIXI.Graphics,
    lngLat: [number, number],
    color: number,
    radiusPx: number,
  ): void {
    const p = this.map.project(lngLat);
    gfx.circle(p.x, p.y, radiusPx);
    gfx.fill({ color, alpha: 0.95 });
    gfx.stroke({ color: 0x0b1020, alpha: 0.95, width: 2 });
  }

  private bindEditorPointerHandlers(): void {
    const canvas = this.overlay.app.canvas as HTMLCanvasElement;
    canvas.style.pointerEvents = 'auto';
    canvas.addEventListener('pointerdown', (e) => {
      if (!this.editorMode || !this.mapData) return;
      const nodeId = this.findNearestNodeId(e.clientX, e.clientY, 12);
      if (this.editorTool === 'move_node' && nodeId !== null) {
        this.dragNodeId = nodeId;
      } else if (this.editorTool === 'add_road' && nodeId !== null) {
        this.connectFromNodeId = nodeId;
      }
    });
    canvas.addEventListener('pointermove', (e) => {
      if (!this.editorMode || !this.mapData || this.dragNodeId === null) return;
      const lngLat = this.map.unproject([e.clientX, e.clientY]);
      const node = this.mapData.nodes.find((n) => n.id === this.dragNodeId);
      if (!node) return;
      const guide = this.findAlignmentGuide(node.id, lngLat.lng, lngLat.lat);
      const targetLng = guide.lng ?? lngLat.lng;
      const targetLat = guide.lat ?? lngLat.lat;
      const gpx = guide.lng !== null ? this.map.project([guide.lng, targetLat]).x : null;
      const gpy = guide.lat !== null ? this.map.project([targetLng, guide.lat]).y : null;
      this.editorOverlay.drawAlignmentGuide(gpx, gpy);
      const now = performance.now();
      if (now - this.dragLastSentAt > 24) {
        this.dragLastSentAt = now;
        editorMoveNode(this.dragNodeId, targetLat, targetLng, false)
          .then((m) => this.applyCustomMapData(m))
          .catch(console.error);
      }
    });
    canvas.addEventListener('pointerup', (e) => {
      if (!this.editorMode || !this.mapData) return;
      if (this.dragNodeId !== null) {
        const lngLat = this.map.unproject([e.clientX, e.clientY]);
        const nodeId = this.dragNodeId;
        this.dragNodeId = null;
        this.editorOverlay.clearGuides();
        editorMoveNode(nodeId, lngLat.lat, lngLat.lng, true)
          .then((m) => this.applyCustomMapData(m))
          .catch(console.error);
        return;
      }
      if (this.editorTool === 'add_road' && this.connectFromNodeId !== null) {
        const fromNodeId = this.connectFromNodeId;
        this.connectFromNodeId = null;
        const targetNode = this.findNearestNodeId(e.clientX, e.clientY, 12);
        if (targetNode !== null && targetNode !== fromNodeId) {
          editorConnect(fromNodeId, targetNode).then((m) => this.applyCustomMapData(m)).catch(console.error);
        } else {
          const ll = this.map.unproject([e.clientX, e.clientY]);
          editorExtrude(fromNodeId, this.nextCustomNodeId++, ll.lat, ll.lng)
            .then((m) => this.applyCustomMapData(m))
            .catch(console.error);
        }
      }
    });
  }

  private bindEditorKeyboardHandlers(): void {
    window.addEventListener('keydown', (e) => {
      if (!this.editorMode) return;
      if ((e.key === 'Delete' || e.key === 'Backspace') && this.mapData && this.selectedEdgeIndex !== null) {
        const edge = this.mapData.edges[this.selectedEdgeIndex];
        if (!edge) return;
        editorDeleteEdge(edge.from, edge.to).then((m) => this.applyCustomMapData(m)).catch(console.error);
      } else if (e.key.toLowerCase() === 's') {
        void saveMapOverrides();
      } else if (e.ctrlKey && e.key.toLowerCase() === 'z') {
        void editorUndo().then((m) => this.applyCustomMapData(m));
      } else if (e.ctrlKey && e.key.toLowerCase() === 'y') {
        void editorRedo().then((m) => this.applyCustomMapData(m));
      }
    });
  }

  private handleEditorClick(x: number, y: number): boolean {
    if (!this.mapData) return false;
    const idx = this.findNearestEdgeIndex(x, y, 8);
    this.selectedEdgeIndex = idx;
    this.editorOverlay.drawSelectedEdge(this.mapData, idx);
    if (idx !== null) {
      const edge = this.mapData.edges[idx];
      this.updateEdgeEditorPanel(edge);
      if (this.editorTool === 'delete') {
        void editorDeleteEdge(edge.from, edge.to).then((m) => this.applyCustomMapData(m));
        return true;
      }
      this.uiRenderer.showNotification(`Edge lanes=${edge.lanes} dir=${edge.oneway ? 'oneway' : 'both'}`, 'info');
      return true;
    }
    this.updateEdgeEditorPanel(null);
    return false;
  }

  private initEdgeEditorPanel(): void {
    const panel = document.createElement('div');
    panel.className = 'edge-editor-panel';
    panel.innerHTML = `
      <div class="edge-editor-title">Edge Selection</div>
      <div class="edge-editor-content">No edge selected</div>
      <div class="edge-editor-form hidden">
        <label>Lanes <input id="edge-lanes" type="number" min="1" max="8" value="2" /></label>
        <label>Direction
          <select id="edge-direction">
            <option value="both">both</option>
            <option value="oneway">oneway</option>
          </select>
        </label>
        <label>Lane directions (csv)
          <input id="edge-lane-directions" type="text" value="left,straight" />
        </label>
        <button id="edge-apply-btn">Apply tags</button>
      </div>
    `;
    document.body.appendChild(panel);
    this.edgeEditorPanel = panel;
    panel.querySelector('#edge-apply-btn')?.addEventListener('click', () => {
      if (!this.mapData || this.selectedEdgeIndex === null) return;
      const edge = this.mapData.edges[this.selectedEdgeIndex];
      if (!edge) return;
      const lanes = Number((panel.querySelector('#edge-lanes') as HTMLInputElement).value || '2');
      const dir = (panel.querySelector('#edge-direction') as HTMLSelectElement).value;
      const laneDirections = (panel.querySelector('#edge-lane-directions') as HTMLInputElement)
        .value
        .split(',')
        .map((s) => s.trim().toLowerCase())
        .filter(Boolean);
      editorUpdateEdgeTags(edge.from, edge.to, lanes, dir === 'oneway', laneDirections)
        .then((m) => this.applyCustomMapData(m))
        .catch(console.error);
    });
  }

  private updateEdgeEditorPanel(edge: MapData['edges'][number] | null): void {
    if (!this.edgeEditorPanel) return;
    const content = this.edgeEditorPanel.querySelector('.edge-editor-content');
    if (!content) return;
    if (!edge) {
      content.innerHTML = 'No edge selected';
      this.edgeEditorPanel.querySelector('.edge-editor-form')?.classList.add('hidden');
      return;
    }
    this.edgeEditorPanel.querySelector('.edge-editor-form')?.classList.remove('hidden');
    (this.edgeEditorPanel.querySelector('#edge-lanes') as HTMLInputElement).value = String(edge.lanes);
    (this.edgeEditorPanel.querySelector('#edge-direction') as HTMLSelectElement).value = edge.oneway ? 'oneway' : 'both';
    (this.edgeEditorPanel.querySelector('#edge-lane-directions') as HTMLInputElement).value = edge.laneDirections.join(',');
    content.innerHTML = `
      <div>Lanes: ${edge.lanes}</div>
      <div>Direction: ${edge.oneway ? 'oneway' : 'both'}</div>
      <div>Road type: ${edge.roadType}</div>
      <div>Max speed: ${(edge.maxSpeed * 3.6).toFixed(0)} km/h</div>
    `;
  }

  private initToolSwitchPanel(): void {
    const panel = document.createElement('div');
    panel.className = 'tool-switch-panel';
    panel.innerHTML = `
      <button data-tool="move_node">Move</button>
      <button data-tool="add_road">Add</button>
      <button data-tool="delete">Delete</button>
      <button data-tool="select">Select</button>
    `;
    document.body.appendChild(panel);
    this.toolSwitchPanel = panel;
    const buttons = [...panel.querySelectorAll('button')];
    const refresh = (): void => {
      buttons.forEach((btn) => {
        btn.classList.toggle('active', btn.getAttribute('data-tool') === this.editorTool);
      });
    };
    buttons.forEach((btn) => {
      btn.addEventListener('click', () => {
        this.editorTool = btn.getAttribute('data-tool') as typeof this.editorTool;
        void setEditorTool(this.editorTool);
        refresh();
      });
    });
    refresh();
  }

  private findNearestNodeId(x: number, y: number, maxDistPx: number): number | null {
    if (!this.mapData) return null;
    let best: number | null = null;
    let bestDist = Number.POSITIVE_INFINITY;
    for (const node of this.mapData.nodes) {
      const p = this.map.project([node.lng, node.lat]);
      const d = Math.hypot(p.x - x, p.y - y);
      if (d < bestDist && d <= maxDistPx) {
        bestDist = d;
        best = node.id;
      }
    }
    return best;
  }

  private findNearestEdgeIndex(x: number, y: number, maxDistPx: number): number | null {
    if (!this.mapData) return null;
    let best: number | null = null;
    let bestDist = Number.POSITIVE_INFINITY;
    this.mapData.edges.forEach((e, idx) => {
      const a = this.mapData!.nodes.find((n) => n.id === e.from);
      const b = this.mapData!.nodes.find((n) => n.id === e.to);
      if (!a || !b) return;
      const p1 = this.map.project([a.lng, a.lat]);
      const p2 = this.map.project([b.lng, b.lat]);
      const t = Math.max(0, Math.min(1, ((x - p1.x) * (p2.x - p1.x) + (y - p1.y) * (p2.y - p1.y)) / ((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2 + 1e-9)));
      const px = p1.x + (p2.x - p1.x) * t;
      const py = p1.y + (p2.y - p1.y) * t;
      const d = Math.hypot(px - x, py - y);
      if (d < bestDist && d <= maxDistPx) {
        bestDist = d;
        best = idx;
      }
    });
    return best;
  }

  private findAlignmentGuide(movingId: number, targetLng: number, targetLat: number): { lng: number | null; lat: number | null } {
    if (!this.mapData) return { lng: null, lat: null };
    let bestLng: number | null = null;
    let bestLat: number | null = null;
    for (const n of this.mapData.nodes) {
      if (n.id === movingId) continue;
      const pxA = this.map.project([n.lng, targetLat]);
      const pxB = this.map.project([targetLng, n.lat]);
      if (Math.abs(pxA.x - this.map.project([targetLng, targetLat]).x) < 8) bestLng = n.lng;
      if (Math.abs(pxB.y - this.map.project([targetLng, targetLat]).y) < 8) bestLat = n.lat;
    }
    return { lng: bestLng, lat: bestLat };
  }

  // ─── Main game loop ────────────────────────────────────────────────────────

  private gameLoop(ticker: PIXI.Ticker): void {
    // Animate oneway arrows regardless of pause state
    this.infraRenderer.update(ticker.deltaMS);

    if (this.debugVisualizationEnabled && this.latestDebugVisualization) {
      this.redrawFullDebugVisualization();
    }

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
    this.bboxPicker?.destroy();
    this.unlistenIdmDebug?.();
    this.unlistenDebugVisualization?.();
    this.sandboxUI?.destroy();
    this.debugRouteGfx?.destroy();
    this.debugVizGfx?.destroy();
    this.debugConflictLabels?.destroy();
    this.turnConnectorGfx?.destroy();
    this.mapScenarioEditorUI?.destroy();
    this.buildingRenderer.destroy();
    this.roadRenderer.destroy();
    this.vehicleRenderer.destroy();
    this.infraRenderer.destroy();
    this.trafficLightRenderer.destroy();
    this.congestionRenderer.destroy();
  }
}
