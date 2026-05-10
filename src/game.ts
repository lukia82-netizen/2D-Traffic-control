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
} from './bridge/commands';
import {
  parseVehicleFrame,
  listenCongestionUpdates,
  listenLightStateChanges,
  listenGameOver,
  listenLeaderDebug,
  listenIdmDebug,
} from './bridge/events';
import type {
  VehicleState,
  CongestionData,
  LightStateUpdate,
  GameOverPayload,
  IdmDebugPayload,
  LeaderDebugEntry,
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
import { LESZNO_BBOX, projectPointForOverlay } from './map/MapLibreSetup';
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
        intersectionType: 'plain',
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
/** Same widths/lengths as VehicleRenderer `VEHICLE_PHYS` (for pick-debug OBB). */
const DEBUG_PICK_PHYS_W_M = [1.8, 2.0, 2.5, 2.6, 2.4];
const DEBUG_PICK_PHYS_L_M = [4.5, 6.0, 12.0, 16.0, 20.0];

interface TurnConnectorPath {
  points: [number, number][];
  p1: [number, number];
  ctrl: [number, number];
  p2: [number, number];
  /** OSM node IDs used to pick the same direction colour as straight lane lines. */
  fromNodeOsmId: number;
  toNodeOsmId: number;
}

type AppMode = 'game' | 'editor' | 'sandbox';
interface GameOptions {
  editorOnly?: boolean;
  appMode?: AppMode;
}

/** Matches Rust `GEO_LNG_M` / `GEO_LAT_M` in `sim_loop.rs` — quick metre deltas for stitch orientation only. */
const ROUTE_HINT_GEO_LNG_M = 71700;
const ROUTE_HINT_GEO_LAT_M = 111320;

function routeHintMetreDelta(
  fromLl: readonly [number, number],
  toLl: readonly [number, number],
): { ex: number; ny: number } {
  return {
    ex: (toLl[0] - fromLl[0]) * ROUTE_HINT_GEO_LNG_M,
    ny: (toLl[1] - fromLl[1]) * ROUTE_HINT_GEO_LAT_M,
  };
}

function routeHintNormalize(v: { ex: number; ny: number }): { ex: number; ny: number } | null {
  const len = Math.hypot(v.ex, v.ny);
  if (len < 0.06) return null;
  return { ex: v.ex / len, ny: v.ny / len };
}

/** Flip polyline if its start tangent opposes nominal travel (`refEn` east/north metres). */
function orientLngLatPolylineForward(
  pts: readonly [number, number][],
  refEn: { ex: number; ny: number } | null,
): [number, number][] {
  if (!refEn || pts.length < 2) return [...pts];
  const d = routeHintMetreDelta(pts[0], pts[1]);
  const t = routeHintNormalize(d);
  if (!t || t.ex * refEn.ex + t.ny * refEn.ny >= -0.02) return [...pts];
  return [...pts].reverse();
}

function trimLeadingBacktrackTowardJoin(
  segIn: readonly [number, number][],
  joinLl: readonly [number, number],
  maxTrim: number,
): [number, number][] {
  const seg = [...segIn];
  let n = 0;
  while (n < maxTrim && seg.length >= 2) {
    const tf = routeHintNormalize(routeHintMetreDelta(joinLl, seg[0]));
    const ta = routeHintNormalize(routeHintMetreDelta(seg[0], seg[1]));
    if (!tf || !ta || tf.ex * ta.ex + tf.ny * ta.ny >= -0.25) break;
    seg.shift();
    n++;
  }
  return seg;
}

/**
 * Remove points from the **tail** of a road lane segment that overshoot past the connector's inset
 * entry point (CONNECTOR_ENDPOINT_INSET_M ≈ 7 m before the junction centre).
 * Without this, the road lane ends at the junction node centre while the connector starts 7 m
 * behind it, producing a backward zig-zag in the purple route path.
 */
function trimRoadLaneTailPastConnectorEntry(
  segIn: readonly [number, number][],
  connectorFirstLngLat: readonly [number, number],
): [number, number][] {
  const seg = [...segIn];
  while (seg.length > 1) {
    const last = seg[seg.length - 1]!;
    const prev = seg[seg.length - 2]!;
    const dir = routeHintNormalize(routeHintMetreDelta(prev, last));
    const toConn = routeHintNormalize(routeHintMetreDelta(last, connectorFirstLngLat));
    if (!dir || !toConn) break;
    if (dir.ex * toConn.ex + dir.ny * toConn.ny < -0.15) {
      seg.pop();
    } else {
      break;
    }
  }
  return seg;
}

/** After a connector, drop outbound road vertices that sit behind `joinLl` along `refEn` (junction-node spike). */
function trimLeadingOutboundSpikeAfterConnector(
  segIn: readonly [number, number][],
  joinLl: readonly [number, number],
  refEn: { ex: number; ny: number },
  maxTrim: number,
): [number, number][] {
  const seg = [...segIn];
  let n = 0;
  while (n < maxTrim && seg.length > 1) {
    const d = routeHintMetreDelta(joinLl, seg[0]!);
    const dot = d.ex * refEn.ex + d.ny * refEn.ny;
    if (dot < -0.35) {
      seg.shift();
      n++;
    } else {
      break;
    }
  }
  return seg;
}

function dedupeLngLatTail(out: [number, number][], p: [number, number]): void {
  const prev = out[out.length - 1];
  if (prev && Math.abs(prev[0] - p[0]) < 1e-10 && Math.abs(prev[1] - p[1]) < 1e-10) return;
  out.push(p);
}

/** Chord length in metres (local plane, same as Rust route hints). */
function lngLatChordMetres(a: readonly [number, number], b: readonly [number, number]): number {
  const d = routeHintMetreDelta(a, b);
  return Math.hypot(d.ex, d.ny);
}

/**
 * Insert points along each segment so no chord exceeds `maxStepM`.
 * `lineTo` between sparse vertices looks jagged; this keeps Pixi polyline close to the sampled curve.
 */
function densifyLngLatPolyline(
  pts: readonly [number, number][],
  maxStepM: number,
): [number, number][] {
  if (pts.length < 2 || maxStepM < 0.15) return [...pts];
  const out: [number, number][] = [[pts[0][0], pts[0][1]]];
  for (let i = 1; i < pts.length; i++) {
    const a = out[out.length - 1];
    const b = pts[i];
    const len = lngLatChordMetres(a, b);
    if (len < 0.02) continue;
    const steps = Math.max(1, Math.ceil(len / maxStepM));
    for (let s = 1; s < steps; s++) {
      const t = s / steps;
      out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
    }
    out.push([b[0], b[1]]);
  }
  return out;
}

/**
 * Quadratic Bezier in [lng, lat] (matches Rust `bezier_point_lat_lng` / debug control polyline).
 */
function sampleQuadraticBezierLngLat(
  p1: readonly [number, number],
  ctrl: readonly [number, number],
  p2: readonly [number, number],
  numSegments: number,
): [number, number][] {
  const n = Math.max(12, numSegments);
  const out: [number, number][] = [];
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    const u = 1 - t;
    const lng = u * u * p1[0] + 2 * u * t * ctrl[0] + t * t * p2[0];
    const lat = u * u * p1[1] + 2 * u * t * ctrl[1] + t * t * p2[1];
    out.push([lng, lat]);
  }
  return out;
}

/** If any hop is longer than `sparseIfOverM`, subdivide segments (~`targetStepM`). */
function densifyLngLatIfSparse(
  pts: readonly [number, number][],
  sparseIfOverM: number,
  targetStepM: number,
): [number, number][] {
  if (pts.length < 2) return [...pts];
  let maxJump = 0;
  for (let i = 1; i < pts.length; i++) {
    maxJump = Math.max(maxJump, lngLatChordMetres(pts[i - 1], pts[i]));
  }
  if (maxJump < sparseIfOverM) return [...pts];
  return densifyLngLatPolyline(pts, targetStepM);
}

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
  private unlistenLeaderDebug: (() => void) | null = null;
  private selectedVehicleId: number | null = null;
  private selectedRoutePoints: [number, number][] = [];
  private selectedThreatPoint: [number, number] | null = null;
  private selectedLookAheadPoint: [number, number] | null = null;
  private selectedStopLinePoint: [number, number] | null = null;
  private selectedTurnEntryPoint: [number, number] | null = null;
  private selectedThreatShapeLengthM = 0;
  /** From latest idm_debug: hood [lng,lat] for HUD threat line matching Rust. */
  private selectedHudHoodLngLat: [number, number] | null = null;
  /** Last `idm_debug` payload for the click-selected vehicle only (path + sensors). */
  private latestIdmDebugSelection: IdmDebugPayload | null = null;
  private debugRouteGfx: PIXI.Graphics | null = null;
  /** Floating ❗ + text above the picked vehicle while braking. */
  private pickDebugHud: PIXI.Container | null = null;
  private laneLinesGfx: PIXI.Graphics | null = null;
  /** Purple path + floating brake HUD for click-selected vehicle (D / sandbox). */
  private pickedVehicleDebugOverlayVisible = false;
  private turnConnectorGfx: PIXI.Graphics | null = null;
  private conflictGfx: PIXI.Graphics | null = null;
  private turnConnectorPaths: TurnConnectorPath[] = [];
  private showTurnConnectors = false;
  private showTurnConnectorsActiveOnly = false;
  private showLaneLines = true;

  // ── Traffic Motion Debug layer ─────────────────────────────────────────────
  private trafficDebugGfx: PIXI.Graphics | null = null;
  private trafficDebugLabels: PIXI.Container | null = null;
  /** «Tryb Debugowania Ruchu» — leader ID labels + future motion vectors. */
  private trafficMotionDebugEnabled = false;
  private latestLeaderDebug: LeaderDebugEntry[] = [];
  /** Fast O(1) lane lookup rebuilt whenever map data changes. */
  private laneById: Map<number, MapData['lanes'][0]> = new Map();

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
  /** null = real OSM data; string = sandbox grid type ('mixed'|'one_lane'|'single_intersection'|…) */
  private currentGridMode: string | null = null;
  // 3.5 m = standard European city lane (car 1.8m + 1.7m clearance between opposing cars)
  private currentLaneWidthM = 3.5;
  private bboxPicker: MapBboxPicker | null = null;
  private editorMode = false;
  private editorTool: 'select' | 'move_node' | 'add_road' | 'delete' = 'move_node';
  private selectedEdgeIndex: number | null = null;
  private dragNodeId: number | null = null;
  private dragLastSentAt = 0;
  private connectFromNodeId: number | null = null;
  private nextCustomNodeId = 1_000_000;
  private edgeEditorPanel: HTMLDivElement | null = null;
  private toolSwitchPanel: HTMLDivElement | null = null;
  private readonly editorOnly: boolean;
  private readonly appMode: AppMode;

  constructor(map: maplibregl.Map, overlay: PixiOverlay, options: GameOptions = {}) {
    this.map = map;
    this.overlay = overlay;
    this.bboxPicker = new MapBboxPicker(this.map);
    this.editorOnly = options.editorOnly ?? false;
    this.appMode = options.appMode ?? (this.editorOnly ? 'editor' : 'game');
    this.editorMode = this.appMode === 'editor';
    if (this.appMode === 'sandbox') {
      this.currentGridMode = 'single_intersection';
    }
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
    this.pickDebugHud = new PIXI.Container();
    this.turnConnectorGfx = new PIXI.Graphics();
    this.conflictGfx = new PIXI.Graphics();
    this.laneLinesGfx = new PIXI.Graphics();
    this.overlay.pickDebugLayer.addChild(this.debugRouteGfx);
    this.overlay.pickDebugLayer.addChild(this.pickDebugHud);
    this.overlay.congestionLayer.addChild(this.turnConnectorGfx);
    this.overlay.congestionLayer.addChild(this.conflictGfx);
    this.overlay.congestionLayer.addChild(this.laneLinesGfx);
    this.debugRouteGfx.visible = this.pickedVehicleDebugOverlayVisible;
    this.pickDebugHud.visible = this.pickedVehicleDebugOverlayVisible;

    // Traffic motion debug — lives in the topmost layer so it renders above all roads/vehicles
    this.trafficDebugGfx = new PIXI.Graphics();
    this.trafficDebugLabels = new PIXI.Container();
    this.trafficDebugGfx.visible = false;
    this.trafficDebugLabels.visible = false;
    this.overlay.trafficDebugLayer.addChild(this.trafficDebugGfx);
    this.overlay.trafficDebugLayer.addChild(this.trafficDebugLabels);

    window.addEventListener('keydown', (ev) => {
      const t = ev.target as HTMLElement | null;
      if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.isContentEditable)) return;
      if (ev.key === 'd' || ev.key === 'D') {
        ev.preventDefault();
        const next = !this.pickedVehicleDebugOverlayVisible;
        this.setPickedVehicleDebugOverlayVisible(next);
        this.sandboxUI?.setChecked('debug-visualization', next);
        return;
      }
      if (!this.tauriAvailable) return;
    });
    await this.vehicleRenderer.init();

    // Init HUD controls
    this.gameClockUI.init();
    this.enableHudPanelDragging();
    if (this.editorOnly) {
      this.hideSimulationHud();
    }

    // ── Sandbox UI ────────────────────────────────────────────────────────
    if (SANDBOX_MODE) {
      this.sandboxUI = new SandboxUI();
      this.wireSandboxUI();
      this.enableHudPanelDragging();
      if (this.editorMode) {
        this.mapScenarioEditorUI = new MapScenarioEditorUI();
        this.wireMapScenarioEditorUI();
        this.enableHudPanelDragging();
      }
    }
    this.showTurnConnectors = localStorage.getItem(TURN_DEBUG_STORAGE_KEY) === '1';
    this.showTurnConnectorsActiveOnly =
      localStorage.getItem(TURN_DEBUG_ACTIVE_ONLY_STORAGE_KEY) === '1';
    if (this.turnConnectorGfx) {
      this.turnConnectorGfx.visible = this.showTurnConnectors;
    }

    if (this.tauriAvailable) {
      if (this.editorMode) {
        await setEditorTool(this.editorTool);
      }
      await this.loadMapData();
      if (!this.editorOnly) {
        await this.subscribeToEvents();
        await this.startRustSimulation();
      } else {
        this.vehicles.clear();
        this.uiRenderer.showNotification('Editor mode active: simulation disabled', 'info');
      }
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
        this.redrawPickDebugHud();
        if (this.trafficMotionDebugEnabled) {
          this.redrawTrafficLeaderDebug();
        }
        this.redrawTurnConnectors();
        this.redrawLaneLines();
        this.redrawConflictAreas();
      }
      // Second Pixi surface: keep in sync with MapLibre (ticker alone can miss a frame).
      this.overlay.renderPickDebug();
    });
    // Vehicle pick uses MapLibre screen coords (same as Pixi overlay); #pixi-container stays pointer-events:none.
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
    if (this.editorMode) {
      this.bindEditorPointerHandlers();
      this.bindEditorKeyboardHandlers();
      this.initEdgeEditorPanel();
      this.initToolSwitchPanel();
    }

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
      if (this.tauriAvailable && !this.editorOnly) {
        setMaxVehicles(count).catch(console.error);
      }
    };
    // Apply the UI's default immediately so the backend cap matches the displayed value.
    if (this.tauriAvailable && !this.editorOnly) {
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
      if (this.appMode === 'sandbox') {
        this.currentGridMode = 'single_intersection';
      } else {
        this.currentGridMode = forceSandbox;
      }
      // Auto-reload so changing mode instantly rebuilds the road network.
      const sizeM = this.estimateCurrentBboxSizeM();
      const cityName = this.estimateCurrentCityName();
      void this.reloadMap(this.currentBbox, sizeM, cityName, this.currentGridMode);
    };
    ui.onLaneWidthChange = (laneWidthM) => {
      this.currentLaneWidthM = laneWidthM;
      const sizeM = this.estimateCurrentBboxSizeM();
      const cityName = this.estimateCurrentCityName();
      void this.reloadMap(this.currentBbox, sizeM, cityName, this.currentGridMode);
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
      this.setPickedVehicleDebugOverlayVisible(enabled);
    };
    ui.onLaneLinesToggle = (enabled) => {
      this.showLaneLines = enabled;
      if (this.laneLinesGfx) this.laneLinesGfx.visible = enabled;
      if (this.turnConnectorGfx) this.turnConnectorGfx.visible = enabled;
      if (this.conflictGfx) this.conflictGfx.visible = enabled;
      this.redrawLaneLines();
      this.redrawTurnConnectors();
      this.redrawConflictAreas();
    };
    ui.onTrafficDebugToggle = (enabled) => {
      void this.setTrafficDebugMode(enabled);
    };
    ui.setChecked('turn-connectors', this.showTurnConnectors);
    ui.setChecked('turn-connectors-active-only', this.showTurnConnectorsActiveOnly);
    ui.applyPersistedSettings();
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
      this.mapData = await loadMap(this.currentBbox, this.currentGridMode, this.currentLaneWidthM);
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
    this.rebuildLaneById();
    this.redrawTurnConnectors();
    this.redrawLaneLines();
    this.redrawConflictAreas();
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
      this.mapData = await loadMap(bbox, forceSandbox, this.currentLaneWidthM);
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
    this.rebuildLaneById();
    this.redrawTurnConnectors();
    this.redrawLaneLines();
    this.redrawConflictAreas();
    this.mapScenarioEditorUI?.setMapData(this.mapData);

    this.sandboxUI?.setLoadingDone(cityName, sizeM);
  }

  private estimateCurrentBboxSizeM(): number {
    const [west, south, east, north] = this.currentBbox;
    const centerLat = (south + north) * 0.5;
    const latSizeM = (north - south) * 111_320;
    const lngSizeM = (east - west) * 111_320 * Math.cos(centerLat * Math.PI / 180);
    return Math.max(100, Math.round(Math.max(latSizeM, lngSizeM)));
  }

  private estimateCurrentCityName(): string {
    const [west, south, east, north] = this.currentBbox;
    const centerLng = (west + east) * 0.5;
    const centerLat = (south + north) * 0.5;
    return CITY_PRESETS.find((c) =>
      Math.abs(c.center[0] - centerLng) < 0.02 &&
      Math.abs(c.center[1] - centerLat) < 0.02,
    )?.name ?? 'Custom';
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
    this.rebuildLaneById();
    this.redrawTurnConnectors();
    this.redrawLaneLines();
    this.redrawConflictAreas();
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

  // ─── Lane lookup cache ─────────────────────────────────────────────────────

  /** Rebuild O(1) lane lookup table from the current map data. */
  private rebuildLaneById(): void {
    this.laneById.clear();
    if (!this.mapData) return;
    for (const lane of this.mapData.lanes) {
      this.laneById.set(lane.id, lane);
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
    this.unlistenLeaderDebug = await listenLeaderDebug((data) => {
      this.latestLeaderDebug = data.entries;
    });
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

    if (this.selectedVehicleId !== null && !this.vehicles.has(this.selectedVehicleId)) {
      this.selectedVehicleId = null;
      this.latestIdmDebugSelection = null;
      this.selectedRoutePoints = [];
      this.selectedThreatPoint = null;
      this.selectedLookAheadPoint = null;
      this.selectedStopLinePoint = null;
      this.selectedTurnEntryPoint = null;
      this.selectedThreatShapeLengthM = 0;
      this.selectedHudHoodLngLat = null;
      this.redrawSelectedRoute();
      this.redrawPickDebugHud();
      this.uiRenderer.updateVehicleTelemetrySelected({
        vehicleId: null,
        speed: 0,
        desiredSpeed: 0,
        acceleration: 0,
        distanceToLeader: 0,
      });
      if (this.tauriAvailable) {
        void setDebugVehicle(null).catch(console.error);
      }
    }
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
    const distHud = data.gap ?? data.distanceToLeaderM ?? data.distanceToLeader ?? 0;
    this.uiRenderer.updateIdmDebug({
      ...data,
      distanceToLeader: distHud,
      brakeReason: data.brakeReason ?? null,
      laneRouteIds: data.laneRouteIds ?? [],
    });
    if (this.selectedVehicleId !== null && data.vehicleId === this.selectedVehicleId) {
      this.latestIdmDebugSelection = data;
      this.uiRenderer.updateVehicleTelemetrySelected({
        vehicleId: data.vehicleId,
        speed: data.speed,
        desiredSpeed: data.desiredSpeed,
        acceleration: data.acceleration,
        distanceToLeader: data.gap ?? data.distanceToLeaderM ?? data.distanceToLeader ?? 0,
        gap: data.gap,
        deltaV: data.deltaV,
        turnT: data.turnT,
        onCurve: data.onCurve,
        laneRouteIds: data.laneRouteIds ?? [],
        brakeReason: data.brakeReason ?? null,
        idmDecision: data.idmDecision,
        ttcSeconds: data.ttcSeconds ?? null,
        comfortBrakingDistanceM: data.comfortBrakingDistanceM,
      });
      this.selectedRoutePoints = data.routePoints ?? [];
      this.selectedThreatPoint = data.threatPoint ?? null;
      this.selectedLookAheadPoint = data.lookAheadPoint ?? null;
      this.selectedStopLinePoint = data.stopLinePoint ?? null;
      this.selectedTurnEntryPoint = data.turnEntryPoint ?? null;
      this.selectedThreatShapeLengthM = data.shapeLengthM ?? 0;
      this.selectedHudHoodLngLat = data.hoodLngLat ?? null;
      this.redrawSelectedRoute();
      this.redrawPickDebugHud();
    }
  }

  /** Show/hide click-selected path + IDM HUD overlay (D / sandbox). Purely client-side. */
  private setPickedVehicleDebugOverlayVisible(visible: boolean): void {
    this.pickedVehicleDebugOverlayVisible = visible;
    if (this.debugRouteGfx) this.debugRouteGfx.visible = visible;
    if (this.pickDebugHud) this.pickDebugHud.visible = visible;
    if (!visible) {
      this.debugRouteGfx?.clear();
      this.pickDebugHud?.removeChildren();
    } else {
      this.redrawSelectedRoute();
      this.redrawPickDebugHud();
    }
  }

  /** Enable/disable the "Tryb Debugowania Ruchu" traffic motion debug overlay. */
  private setTrafficDebugMode(enabled: boolean): void {
    this.trafficMotionDebugEnabled = enabled;
    if (this.trafficDebugGfx) this.trafficDebugGfx.visible = enabled;
    if (this.trafficDebugLabels) this.trafficDebugLabels.visible = enabled;
    if (!enabled) {
      this.trafficDebugGfx?.clear();
      this.trafficDebugLabels?.removeChildren();
      this.latestLeaderDebug = [];
    }
  }

  /** Floating labels: IDM vehicle leader vs same-edge lane predecessor (queued / slow vehicles). */
  private redrawTrafficLeaderDebug(): void {
    const layer = this.trafficDebugLabels;
    if (!layer || !this.trafficMotionDebugEnabled) return;
    layer.removeChildren();
    const z = this.map.getZoom();
    const fontSize = Math.max(9, Math.min(13, 9 + (15 - z) * 0.35));
    for (const e of this.latestLeaderDebug) {
      const v = this.vehicles.get(e.vehicleId);
      if (!v) continue;
      const px = this.map.project([v.lng, v.lat]);
      const idm = e.idmLeaderVehicleId != null ? `#${e.idmLeaderVehicleId}` : '—';
      const lane = e.laneLeaderVehicleId != null ? `#${e.laneLeaderVehicleId}` : '—';
      const lines = [`IDM→${idm}`, `pas ${lane}`];
      if (e.sensorMismatch) {
        lines.push('SENSOR');
      }
      const text = lines.join('\n');
      const baseStyle = {
        fontFamily: 'Inter, Segoe UI, sans-serif',
        fontSize,
        fill: e.sensorMismatch ? 0xff4444 : 0xf8fafc,
        stroke: { color: 0x0f172a, width: Math.max(3, fontSize * 0.22) },
      } as const;
      const t = new PIXI.Text({ text, style: baseStyle });
      t.anchor.set(0.5, 1);
      t.x = px.x;
      t.y = px.y - 18;
      layer.addChild(t);
    }
  }

  private async selectVehicleAtScreenPoint(x: number, y: number): Promise<void> {
    if (this.vehicles.size === 0) {
      return;
    }
    let bestId: number | null = null;
    let bestDist = Number.POSITIVE_INFINITY;
    const z = this.map.getZoom();
    const MAX_PICK_PX = Math.min(52, Math.max(22, 30 + (14 - z) * 3.5));
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
      // Auto-enable picked-vehicle overlay on successful click so users
      // immediately see route/circle/arrow markers without toggling panel state.
      if (!this.pickedVehicleDebugOverlayVisible) {
        this.setPickedVehicleDebugOverlayVisible(true);
        this.sandboxUI?.setChecked('debug-visualization', true);
      }
      this.selectedVehicleId = bestId;
      this.latestIdmDebugSelection = null;
      // Clear previous selection debug data immediately; otherwise a newly picked
      // vehicle can briefly render the old vehicle's route/markers.
      this.selectedRoutePoints = [];
      this.selectedThreatPoint = null;
      this.selectedLookAheadPoint = null;
      this.selectedStopLinePoint = null;
      this.selectedTurnEntryPoint = null;
      this.selectedThreatShapeLengthM = 0;
      this.selectedHudHoodLngLat = null;
      await setDebugVehicle(bestId).catch(console.error);
      this.uiRenderer.showNotification(`Debug: pojazd #${bestId} (trasa + sensory)`, 'info');
      this.redrawSelectedRoute();
      this.redrawPickDebugHud();
    } else {
      this.selectedVehicleId = null;
      this.latestIdmDebugSelection = null;
      this.selectedRoutePoints = [];
      this.selectedThreatPoint = null;
      this.selectedLookAheadPoint = null;
      this.selectedStopLinePoint = null;
      this.selectedTurnEntryPoint = null;
      this.selectedThreatShapeLengthM = 0;
      this.selectedHudHoodLngLat = null;
      this.redrawSelectedRoute();
      this.redrawPickDebugHud();
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

  /** MapLibre `project` is relative to the map container; align to `#pixi-pick-debug` if layouts differ. */
  private projectForPickDebugOverlay(lng: number, lat: number): { x: number; y: number } {
    const root = this.overlay.pickDebugApp?.canvas.parentElement ?? null;
    return projectPointForOverlay(this.map, lng, lat, root);
  }

  /** Thicker strokes when zoomed out so route / pick debug stays visible. */
  private debugStrokeScale(): number {
    const z = this.map.getZoom();
    return Math.max(1, Math.min(4, 1 + (15 - z) * 0.35));
  }

  private redrawSelectedRoute(): void {
    if (!this.debugRouteGfx) return;
    this.debugRouteGfx.clear();
    if (!this.pickedVehicleDebugOverlayVisible) return;
    if (this.selectedVehicleId === null) return;

    const PATH_PURPLE = 0xff44ff;
    const PATH_PURPLE_INNER = 0xf0abfc;
    const s = this.debugStrokeScale();

    const fallbackRouteFromLaneIds = (): [number, number][] => {
      const d = this.latestIdmDebugSelection;
      if (!d || d.vehicleId !== this.selectedVehicleId) return [];
      const v = this.vehicles.get(this.selectedVehicleId);
      const laneIds = d.laneRouteIds ?? [];
      if (!v || laneIds.length === 0) return [];

      /** Same stitch logic as Rust `build_debug_target_path_points` — prevents reversed connector polylines. */
      const out: [number, number][] = [[v.lng, v.lat]];
      let refEn: { ex: number; ny: number } | null = {
        ex: Math.sin(v.angle),
        ny: Math.cos(v.angle),
      };
      let prevLaneWasConnector = false;

      for (let laneIdx = 0; laneIdx < laneIds.length; laneIdx++) {
        const laneId = laneIds[laneIdx]!;
        const lane = this.laneById.get(laneId);
        if (!lane || lane.points.length === 0) continue;

        let seg: [number, number][] = [];
        for (const [lat, lng] of lane.points) {
          dedupeLngLatTail(seg, [lng, lat]);
        }
        if (seg.length === 0) continue;

        const joinLl = out[out.length - 1]!;
        if (seg.length >= 2) {
          seg = orientLngLatPolylineForward(seg, refEn);
          /** Connectors: do not trim — backend Kubro polyline must stay intact (Rust skips trim too). */
          const maxTrim = lane.isConnector ? 0 : 96;
          seg = trimLeadingBacktrackTowardJoin(seg, joinLl, maxTrim);
          if (!lane.isConnector && prevLaneWasConnector && refEn) {
            seg = trimLeadingOutboundSpikeAfterConnector(seg, joinLl, refEn, 64);
          }
          // If the next lane is a connector, trim this road lane's tail so it does not overshoot
          // past the connector's inset entry point (~7 m before the junction node centre).
          // Without this the road lane ends at the junction centre and the connector starts 7 m
          // behind it, creating a visible backward zig-zag in the purple route overlay.
          if (!lane.isConnector && seg.length > 1) {
            const nextLane = laneIds[laneIdx + 1] !== undefined
              ? this.laneById.get(laneIds[laneIdx + 1]!)
              : undefined;
            if (nextLane?.isConnector && nextLane.points.length > 0) {
              const [connFirstLat, connFirstLng] = nextLane.points[0]!;
              seg = trimRoadLaneTailPastConnectorEntry(seg, [connFirstLng, connFirstLat]);
            }
          }
        }
        /** Connectors: `lineTo` needs many vertices; sparse map data → quadratic sample from IDM debug, else chord subdivide. */
        if (lane.isConnector) {
          const bz = d.bezierControlPathLngLat;
          if (seg.length <= 4 && bz?.length === 3 && d.onCurve) {
            seg = sampleQuadraticBezierLngLat(bz[0], bz[1], bz[2], 28);
          } else {
            seg = densifyLngLatPolyline(seg, 0.6);
          }
        }
        for (const p of seg) {
          dedupeLngLatTail(out, p);
        }

        if (out.length >= 2) {
          const a = out[out.length - 2];
          const b = out[out.length - 1];
          refEn = routeHintNormalize(routeHintMetreDelta(a, b)) ?? refEn;
        }
        prevLaneWasConnector = lane.isConnector;
      }
      return out;
    };

    /** Prefer backend `routePoints` (Kurbo-sampled); lane-ID fallback if empty. Always avoid long `lineTo` chords. */
    let routeLngLat: [number, number][] = [];
    if (this.selectedRoutePoints.length >= 2) {
      routeLngLat = densifyLngLatIfSparse(this.selectedRoutePoints, 4.5, 0.85);
    } else {
      const fromIds = fallbackRouteFromLaneIds();
      if (fromIds.length >= 2) {
        routeLngLat = densifyLngLatIfSparse(fromIds, 4.5, 0.85);
      } else {
        routeLngLat = [];
      }
    }

    if (routeLngLat.length >= 2) {
      const pts = routeLngLat.map(([lng, lat]) => this.projectForPickDebugOverlay(lng, lat));
      this.debugRouteGfx.moveTo(pts[0].x, pts[0].y);
      for (let i = 1; i < pts.length; i++) {
        this.debugRouteGfx.lineTo(pts[i].x, pts[i].y);
      }
      this.debugRouteGfx.stroke({ color: 0x1e0524, alpha: 0.85, width: 7 * s });
      this.debugRouteGfx.moveTo(pts[0].x, pts[0].y);
      for (let i = 1; i < pts.length; i++) {
        this.debugRouteGfx.lineTo(pts[i].x, pts[i].y);
      }
      this.debugRouteGfx.stroke({ color: PATH_PURPLE, alpha: 0.98, width: 4.5 * s });
      this.debugRouteGfx.moveTo(pts[0].x, pts[0].y);
      for (let i = 1; i < pts.length; i++) {
        this.debugRouteGfx.lineTo(pts[i].x, pts[i].y);
      }
      this.debugRouteGfx.stroke({ color: PATH_PURPLE_INNER, alpha: 0.55, width: Math.max(1.2, 2 * s) });
      this.drawArrowsAlongPath(this.debugRouteGfx, pts, PATH_PURPLE, 1.8 * s);
    }

    // Bezier control polyline (P1 → control → P2) + direction arrows while on connector.
    const bz = this.latestIdmDebugSelection?.bezierControlPathLngLat;
    if (
      bz &&
      bz.length === 3
      && this.latestIdmDebugSelection?.vehicleId === this.selectedVehicleId
    ) {
      const P = bz.map(([lng, lat]) => this.projectForPickDebugOverlay(lng, lat));
      this.debugRouteGfx.moveTo(P[0].x, P[0].y);
      this.debugRouteGfx.lineTo(P[1].x, P[1].y);
      this.debugRouteGfx.lineTo(P[2].x, P[2].y);
      this.debugRouteGfx.stroke({ color: 0xfde047, alpha: 0.95, width: 2.8 * s });
      for (const [i, j] of [[0, 1], [1, 2]] as const) {
        this.drawArrowHeadOnSegment(
          this.debugRouteGfx,
          P[i].x,
          P[i].y,
          P[j].x,
          P[j].y,
          0.42,
          0xfacc15,
          2.2 * s,
        );
      }
      for (const k of [0, 1, 2] as const) {
        this.debugRouteGfx.circle(P[k].x, P[k].y, k === 1 ? 5 : 4);
        this.debugRouteGfx.fill({ color: k === 1 ? 0xf97316 : 0xfde68a, alpha: 0.95 });
        this.debugRouteGfx.stroke({ color: 0x0f172a, alpha: 0.9, width: 1.5 * s });
      }
    }

    // Red line: IDM obstacle anchor (leader / conflict / stop line / …).
    if (this.selectedThreatPoint && this.selectedVehicleId !== null) {
      const v = this.vehicles.get(this.selectedVehicleId);
      if (v) {
        const hood = this.selectedHudHoodLngLat
          ?? this.computeVehicleHoodLngLat(v, this.selectedThreatShapeLengthM);
        const p0 = this.projectForPickDebugOverlay(hood[0], hood[1]);
        const p1 = this.projectForPickDebugOverlay(
          this.selectedThreatPoint[0],
          this.selectedThreatPoint[1],
        );
        this.debugRouteGfx.moveTo(p0.x, p0.y);
        this.debugRouteGfx.lineTo(p1.x, p1.y);
        this.debugRouteGfx.stroke({ color: 0xef4444, alpha: 1.0, width: 3.2 * s });
        this.drawArrowHeadOnSegment(this.debugRouteGfx, p0.x, p0.y, p1.x, p1.y, 0.88, 0xfe2e2e, 2.2 * s);
      }
    }

    // Purple vector: steering look-ahead target from current hood position.
    if (this.selectedLookAheadPoint && this.selectedVehicleId !== null) {
      const v = this.vehicles.get(this.selectedVehicleId);
      if (v) {
        const hood = this.selectedHudHoodLngLat
          ?? this.computeVehicleHoodLngLat(v, this.selectedThreatShapeLengthM);
        const p0 = this.projectForPickDebugOverlay(hood[0], hood[1]);
        const p1 = this.projectForPickDebugOverlay(
          this.selectedLookAheadPoint[0],
          this.selectedLookAheadPoint[1],
        );
        this.debugRouteGfx.moveTo(p0.x, p0.y);
        this.debugRouteGfx.lineTo(p1.x, p1.y);
        this.debugRouteGfx.stroke({ color: 0xa78bfa, alpha: 1.0, width: 2.8 * s });
        this.drawArrowHeadOnSegment(this.debugRouteGfx, p0.x, p0.y, p1.x, p1.y, 0.9, 0xc084fc, 2.0 * s);
      }
    }
    // Explicit red leader line: ego -> IDM leader vehicle (who the car follows).
    const dbg = this.latestIdmDebugSelection;
    if (dbg && dbg.vehicleId === this.selectedVehicleId && this.selectedVehicleId !== null) {
      const ego = this.vehicles.get(this.selectedVehicleId);
      if (ego) {
        const leaderId = dbg.leaderVehicleId;
        if (leaderId != null && leaderId !== this.selectedVehicleId) {
          const leader = this.vehicles.get(leaderId);
          if (leader) {
            const p0 = this.projectForPickDebugOverlay(ego.lng, ego.lat);
            const p1 = this.projectForPickDebugOverlay(leader.lng, leader.lat);
            this.debugRouteGfx.moveTo(p0.x, p0.y);
            this.debugRouteGfx.lineTo(p1.x, p1.y);
            this.debugRouteGfx.stroke({ color: 0xff2d2d, alpha: 0.95, width: 3.4 * s });
            this.drawArrowHeadOnSegment(this.debugRouteGfx, p0.x, p0.y, p1.x, p1.y, 0.9, 0xff3b3b, 2.6 * s);
            this.debugRouteGfx.circle(p1.x, p1.y, 5.8 * Math.min(1.35, 0.85 + s * 0.1));
            this.debugRouteGfx.fill({ color: 0xff2d2d, alpha: 0.98 });
            this.debugRouteGfx.stroke({ color: 0x3f0b0b, alpha: 0.95, width: 1.8 * s });
          }
        }
      }
    }

    // Perception arrows: additional relations (conflict reserver, etc).
    if (dbg && dbg.vehicleId === this.selectedVehicleId && this.selectedVehicleId !== null) {
      const ego = this.vehicles.get(this.selectedVehicleId);
      if (ego) {
        const egoPx = this.projectForPickDebugOverlay(ego.lng, ego.lat);
        const drawTargetArrow = (targetId: number | null | undefined, color: number, dashed: boolean) => {
          if (targetId == null || targetId === this.selectedVehicleId) return;
          const target = this.vehicles.get(targetId);
          if (!target) return;
          const tp = this.projectForPickDebugOverlay(target.lng, target.lat);
          if (dashed) {
            this.drawDashedSegment(this.debugRouteGfx!, egoPx.x, egoPx.y, tp.x, tp.y, 9 * s, 7 * s, color, 2.0 * s);
          } else {
            this.debugRouteGfx!.moveTo(egoPx.x, egoPx.y);
            this.debugRouteGfx!.lineTo(tp.x, tp.y);
            this.debugRouteGfx!.stroke({ color, alpha: 0.9, width: 2.4 * s });
          }
          this.drawArrowHeadOnSegment(this.debugRouteGfx!, egoPx.x, egoPx.y, tp.x, tp.y, 0.9, color, 2.0 * s);
        };
        drawTargetArrow(dbg.conflictReserverId, 0xf59e0b, true);
      }
    }

    // Turn intent arrow near selected vehicle (straight / left / right).
    if (dbg && dbg.vehicleId === this.selectedVehicleId && this.selectedVehicleId !== null) {
      const picked = this.vehicles.get(this.selectedVehicleId);
      if (picked) {
        const p = this.projectForPickDebugOverlay(picked.lng, picked.lat);
        this.drawTurnIntentArrow(this.debugRouteGfx!, p.x, p.y, picked.angle, dbg.nextTurnIntent ?? 'straight', 0x34d399, s);
      }
    }

    if (this.selectedStopLinePoint) {
      const ps = this.projectForPickDebugOverlay(
        this.selectedStopLinePoint[0],
        this.selectedStopLinePoint[1],
      );
      this.debugRouteGfx.circle(ps.x, ps.y, 6 * Math.min(1.4, 0.85 + s * 0.12));
      this.debugRouteGfx.fill({ color: 0xf59e0b, alpha: 0.95 });
      this.debugRouteGfx.stroke({ color: 0x0f172a, alpha: 0.95, width: 2 * s });
    }
    if (this.selectedTurnEntryPoint) {
      const pe = this.projectForPickDebugOverlay(
        this.selectedTurnEntryPoint[0],
        this.selectedTurnEntryPoint[1],
      );
      const half = 5 * Math.min(1.35, 0.85 + s * 0.1);
      this.debugRouteGfx.rect(pe.x - half, pe.y - half, half * 2, half * 2);
      this.debugRouteGfx.fill({ color: 0xa78bfa, alpha: 0.95 });
      this.debugRouteGfx.stroke({ color: 0x0f172a, alpha: 0.95, width: 2 * s });
    }

    // Yellow OBB + pick reticle (same screen space as MapLibre `project`).
    const picked = this.vehicles.get(this.selectedVehicleId);
    if (picked) {
      const p = this.projectForPickDebugOverlay(picked.lng, picked.lat);
      const mc = this.map.getCenter();
      const p1m = this.projectForPickDebugOverlay(mc.lng, mc.lat);
      const p2m = this.projectForPickDebugOverlay(mc.lng, mc.lat + 1 / 111_320);
      const pxPerM = Math.abs(p2m.y - p1m.y);
      const typeId = picked.vehicleType < 5 ? picked.vehicleType : 0;
      const widthM = DEBUG_PICK_PHYS_W_M[typeId] ?? 1.8;
      const lengthM = DEBUG_PICK_PHYS_L_M[typeId] ?? 4.5;
      const hw = Math.max(2, pxPerM * widthM * 0.5);
      const hl = Math.max(4, pxPerM * lengthM * 0.5);
      const c = Math.cos(picked.angle);
      const si = Math.sin(picked.angle);
      const rot = (lx: number, ly: number) => ({ x: p.x + lx * c - ly * si, y: p.y + lx * si + ly * c });
      const obb = [rot(-hw, -hl), rot(hw, -hl), rot(hw, hl), rot(-hw, hl)];
      this.debugRouteGfx.moveTo(obb[0].x, obb[0].y);
      for (let i = 1; i < obb.length; i++) this.debugRouteGfx.lineTo(obb[i].x, obb[i].y);
      this.debugRouteGfx.closePath();
      this.debugRouteGfx.stroke({ color: 0xfacc15, alpha: 0.98, width: 2.5 * s });

      this.debugRouteGfx.circle(p.x, p.y, 20).stroke({ color: 0xff44ff, alpha: 0.98, width: 3.5 * s });
      const cross = 11 * Math.min(1.35, 0.85 + s * 0.1);
      this.debugRouteGfx.moveTo(p.x - cross, p.y);
      this.debugRouteGfx.lineTo(p.x + cross, p.y);
      this.debugRouteGfx.moveTo(p.x, p.y - cross);
      this.debugRouteGfx.lineTo(p.x, p.y + cross);
      this.debugRouteGfx.stroke({ color: 0xffffff, alpha: 0.95, width: 2.5 * s });
    }
  }

  /** Arrow head at fraction `tAlong` along segment a→b (screen px). */
  private drawArrowHeadOnSegment(
    gfx: PIXI.Graphics,
    ax: number,
    ay: number,
    bx: number,
    by: number,
    tAlong: number,
    color: number,
    lineWidth = 2.2,
  ): void {
    const mx = ax + (bx - ax) * tAlong;
    const my = ay + (by - ay) * tAlong;
    const dx = bx - ax;
    const dy = by - ay;
    const len = Math.hypot(dx, dy) || 1;
    const ux = dx / len;
    const uy = dy / len;
    const wing = 4.5;
    const back = 9;
    gfx.moveTo(mx + ux * 3, my + uy * 3);
    gfx.lineTo(mx - ux * back + (-uy) * wing, my - uy * back + ux * wing);
    gfx.moveTo(mx + ux * 3, my + uy * 3);
    gfx.lineTo(mx - ux * back - (-uy) * wing, my - uy * back - ux * wing);
    gfx.stroke({ color, alpha: 0.96, width: lineWidth });
  }

  private drawDashedSegment(
    gfx: PIXI.Graphics,
    ax: number,
    ay: number,
    bx: number,
    by: number,
    dashPx: number,
    gapPx: number,
    color: number,
    width: number,
  ): void {
    const dx = bx - ax;
    const dy = by - ay;
    const len = Math.hypot(dx, dy);
    if (len < 1) return;
    const ux = dx / len;
    const uy = dy / len;
    let t = 0;
    while (t < len) {
      const s0 = t;
      const s1 = Math.min(len, t + dashPx);
      gfx.moveTo(ax + ux * s0, ay + uy * s0);
      gfx.lineTo(ax + ux * s1, ay + uy * s1);
      t += dashPx + gapPx;
    }
    gfx.stroke({ color, alpha: 0.92, width });
  }

  private drawTurnIntentArrow(
    gfx: PIXI.Graphics,
    cx: number,
    cy: number,
    headingRad: number,
    intentRaw: string,
    color: number,
    scale: number,
  ): void {
    const intent = intentRaw.toLowerCase();
    const len = 34 * Math.min(1.5, 0.9 + scale * 0.1);
    const dirx = Math.sin(headingRad);
    const diry = -Math.cos(headingRad);
    const nx = -diry;
    const ny = dirx;
    const ox = cx + dirx * 20;
    const oy = cy + diry * 20;
    if (intent === 'left' || intent === 'right') {
      const sign = intent === 'left' ? -1 : 1;
      const mx = ox + dirx * (len * 0.45) + nx * sign * (len * 0.35);
      const ex = ox + dirx * (len * 0.2) + nx * sign * (len * 0.9);
      const my = oy + diry * (len * 0.45) + ny * sign * (len * 0.35);
      const ey = oy + diry * (len * 0.2) + ny * sign * (len * 0.9);
      gfx.moveTo(ox, oy);
      gfx.quadraticCurveTo(mx, my, ex, ey);
      gfx.stroke({ color, alpha: 0.96, width: 3 * scale });
      this.drawArrowHeadOnSegment(gfx, mx, my, ex, ey, 0.96, color, 2.3 * scale);
      return;
    }
    const ex = ox + dirx * len;
    const ey = oy + diry * len;
    gfx.moveTo(ox, oy);
    gfx.lineTo(ex, ey);
    gfx.stroke({ color, alpha: 0.96, width: 3 * scale });
    this.drawArrowHeadOnSegment(gfx, ox, oy, ex, ey, 0.92, color, 2.3 * scale);
  }

  /** Floating brake caption above picked vehicle (PIXIES, map-aligned). */
  private redrawPickDebugHud(): void {
    const hud = this.pickDebugHud;
    if (!hud) return;
    hud.removeChildren();
    if (!this.pickedVehicleDebugOverlayVisible) return;
    if (this.selectedVehicleId === null) return;
    const v = this.vehicles.get(this.selectedVehicleId);
    if (!v) return;
    const d = this.latestIdmDebugSelection;
    const px = this.projectForPickDebugOverlay(v.lng, v.lat);
    const braking = d && d.vehicleId === this.selectedVehicleId && d.acceleration <= -0.45;
    const reason =
      braking && d.brakeReason
        ? d.brakeReason
        : braking
          ? `${d.threatKind}${d.leaderVehicleId != null ? ` #${d.leaderVehicleId}` : ''}${d.conflictReserverId != null ? ` · reserver ${d.conflictReserverId}` : ''}`
          : null;
    if (reason) {
      const wrap = new PIXI.Container();
      wrap.x = px.x;
      wrap.y = px.y;
      const badge = new PIXI.Graphics();
      badge.circle(0, -30, 12).fill({ color: 0xdc2626, alpha: 0.96 }).stroke({ color: 0x450a0a, width: 1.6 });
      const bang = new PIXI.Text({
        text: '!',
        style: {
          fontFamily: 'Inter, Segoe UI, sans-serif',
          fontSize: 15,
          fill: 0xfff7ed,
          fontWeight: '800',
        },
      });
      bang.anchor.set(0.5, 0.55);
      bang.position.set(0, -30);
      const cap = new PIXI.Text({
        text: reason,
        style: {
          fontFamily: 'Inter, Segoe UI, sans-serif',
          fontSize: 11,
          fill: 0xff4d4d,
          align: 'center',
          fontWeight: '600',
        },
      });
      cap.anchor.set(0.5, 1);
      cap.y = -12;
      wrap.addChild(badge, bang, cap);
      hud.addChild(wrap);
    }
    if (d && d.vehicleId === this.selectedVehicleId) {
      const ids = d.laneRouteIds ?? [];
      const laneStr =
        ids.length > 0 ? ids.slice(0, 10).join(' → ') + (ids.length > 10 ? ' …' : '') : '—';
      const stats = new PIXI.Text({
        text:
          `v ${d.speed.toFixed(1)} / v₀ ${d.desiredSpeed.toFixed(1)} m/s\n`
          + `t ${d.turnT.toFixed(3)}${d.onCurve ? ' (łuk)' : ''}\n`
          + `pasy: ${laneStr}`,
        style: {
          fontFamily: 'Inter, Segoe UI, sans-serif',
          fontSize: 10,
          fill: 0xf1f5f9,
          align: 'left',
          stroke: { color: 0x0f172a, width: 3 },
        },
      });
      stats.anchor.set(0, 0);
      stats.x = px.x + 16;
      stats.y = px.y - 72;
      stats.alpha = 0.97;
      hud.addChild(stats);
    }
    if (d && d.vehicleId === this.selectedVehicleId && d.routePoints.length < 2) {
      const w = new PIXI.Text({
        text: '⚠ Krótka trasa / brak polylinii\n(sprawdź lane_route)',
        style: {
          fontFamily: 'Inter, Segoe UI, sans-serif',
          fontSize: 10,
          fill: 0xfbbf24,
          align: 'center',
        },
      });
      w.anchor.set(0.5, 0);
      w.x = px.x;
      w.y = px.y + 14;
      w.alpha = 0.95;
      hud.addChild(w);
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
    // Prefer backend-computed connector lanes (isConnector = true) which already have
    // correct lane-offset geometry produced by populate_lane_graph / build_lane_connector.
    const connectorLanes = this.mapData.lanes.filter((l) => l.isConnector);
    if (connectorLanes.length > 0) {
      this.turnConnectorPaths = connectorLanes.map((l) => {
        // Backend points are [lat, lng]; drawing needs [lng, lat].
        const pts = l.points.map(([lat, lng]) => [lng, lat] as [number, number]);
        const mid = pts[Math.floor(pts.length / 2)] ?? pts[0] ?? [0, 0];
        return {
          points: pts,
          p1: pts[0] ?? [0, 0],
          ctrl: mid,
          p2: pts[pts.length - 1] ?? [0, 0],
          fromNodeOsmId: l.fromNodeOsmId,
          toNodeOsmId: l.toNodeOsmId,
        };
      });
      return;
    }
    // Fallback: compute from edge graph (no lane offset, road-centerline only).
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
          if (inEdge.from === outEdge.to) continue;  // skip U-turns
          const inSrc = nodeById.get(inEdge.from);
          const outTgt = nodeById.get(outEdge.to);
          if (!inSrc || !outTgt) continue;
          const angle = this.turnAngleRad(inSrc.lng, inSrc.lat, junction.lng, junction.lat, outTgt.lng, outTgt.lat);
          if (angle < TURN_CONNECTOR_MIN_ANGLE_RAD) continue;
          const path = this.buildTurnConnectorPath(inSrc, junction, outTgt, inEdge.lengthM, outEdge.lengthM, inEdge.from, outEdge.to);
          if (path.points.length >= 2) this.turnConnectorPaths.push(path);
        }
      }
    }
  }

  private redrawTurnConnectors(): void {
    const gfx = this.turnConnectorGfx;
    if (!gfx) return;
    gfx.clear();
    if (!this.showLaneLines || this.turnConnectorPaths.length === 0) return;

    const s = this.debugStrokeScale();
    const activeVehicles = [...this.vehicles.values()].filter((v) => v.onTurnConnector);
    const activePaths = this.turnConnectorPaths.filter((p) =>
      this.connectorPathIsActive(p.points, activeVehicles),
    );
    if (activePaths.length === 0) return;

    // Active-connector highlight — bright cyan glow drawn on top of the base lane lines.
    const toScreen = (pts: [number, number][]) =>
      pts.map(([lng, lat]) => this.map.project([lng, lat]));
    const drawScreenPath = (screenPts: { x: number; y: number }[]): void => {
      if (screenPts.length < 2) return;
      gfx.moveTo(screenPts[0].x, screenPts[0].y);
      for (let i = 1; i < screenPts.length; i++) gfx.lineTo(screenPts[i].x, screenPts[i].y);
    };

    for (const path of activePaths) drawScreenPath(toScreen(path.points));
    gfx.stroke({ color: 0x111827, alpha: 0.7, width: 7 * s });
    for (const path of activePaths) drawScreenPath(toScreen(path.points));
    gfx.stroke({ color: 0x22d3ee, alpha: 0.9, width: 3.0 * s });
  }

  /** Draw direction arrows along a polyline on the given Graphics object. */
  private drawArrowsAlongPath(
    gfx: import('pixi.js').Graphics,
    screenPts: { x: number; y: number }[],
    color: number,
    lineWidth = 1.8,
  ): void {
    if (screenPts.length < 2) return;
    const spacingPx = 70;
    const minSegPx = 18;
    const arrowLen = 8;
    const wing = 3.5;
    let carried = 0;
    for (let i = 1; i < screenPts.length; i++) {
      const a = screenPts[i - 1];
      const b = screenPts[i];
      const vx = b.x - a.x;
      const vy = b.y - a.y;
      const segLen = Math.hypot(vx, vy);
      if (segLen < minSegPx) continue;
      const len = segLen;
      const dx = vx / len;
      const dy = vy / len;
      const px = -dy;
      const py = dx;
      let t = (spacingPx - carried) / segLen;
      while (t <= 1) {
        const x = a.x + vx * t;
        const y = a.y + vy * t;
        gfx.moveTo(x - dx * 2 + px * wing, y - dy * 2 + py * wing);
        gfx.lineTo(x + dx * arrowLen,      y + dy * arrowLen);
        gfx.lineTo(x - dx * 2 - px * wing, y - dy * 2 - py * wing);
        gfx.stroke({ color, alpha: 0.95, width: lineWidth });
        t += spacingPx / segLen;
      }
      const distFromLast = (1 - (t - spacingPx / segLen)) * segLen;
      carried = distFromLast > 0 ? distFromLast : 0;
      if (carried >= spacingPx) carried = 0;
    }
  }

  private redrawLaneLines(): void {
    const gfx = this.laneLinesGfx;
    if (!gfx) return;
    gfx.clear();
    if (!this.showLaneLines || !this.mapData) return;

    const s = this.debugStrokeScale();
    for (const lane of this.mapData.lanes) {
      if (!lane.points || lane.points.length < 2) continue;
      // Opposite directions on the same physical road get distinct colours.
      // Connector lanes use the same rule (fromNodeOsmId of the incoming road).
      const color = lane.fromNodeOsmId <= lane.toNodeOsmId ? 0xfacc15 : 0x22c55e;
      const screenPts = lane.points.map((p) => this.map.project([p[1], p[0]]));
      gfx.moveTo(screenPts[0].x, screenPts[0].y);
      for (let i = 1; i < screenPts.length; i++) gfx.lineTo(screenPts[i].x, screenPts[i].y);
      gfx.stroke({ color, alpha: 0.94, width: 2.2 * s });
      this.drawArrowsAlongPath(gfx, screenPts, color, 1.8 * s);
    }
  }

  /** Draw red dots at every ConflictArea (where two connector paths physically cross). */
  private redrawConflictAreas(): void {
    const gfx = this.conflictGfx;
    if (!gfx) return;
    gfx.clear();
    if (!this.showLaneLines || !this.mapData) return;
    const connectorLaneIds = new Set(
      this.mapData.lanes.filter((l) => l.isConnector).map((l) => l.id),
    );
    for (const area of this.mapData.conflictAreas) {
      // Only show conflict areas that involve at least one connector lane.
      const hasConnector = area.laneIds.some((id) => connectorLaneIds.has(id));
      if (!hasConnector) continue;
      const pt = this.map.project([area.centerLng, area.centerLat]);
      gfx.circle(pt.x, pt.y, 5);
      gfx.fill({ color: 0xef4444, alpha: 0.9 });
      gfx.circle(pt.x, pt.y, 5);
      gfx.stroke({ color: 0xffffff, alpha: 0.8, width: 1.2 });
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
    fromNodeOsmId = 0,
    toNodeOsmId = 1,
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
    /** Mid-chord control if tangents are parallel — never the raw junction node (avoids a path through centre). */
    let ctrlLng = (p1Lng + p2Lng) * 0.5;
    let ctrlLat = (p1Lat + p2Lat) * 0.5;
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
      fromNodeOsmId,
      toNodeOsmId,
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

  private bindEditorPointerHandlers(): void {
    this.map.on('mousedown', (e) => {
      if (!this.editorMode || !this.mapData) return;
      const nodeId = this.findNearestNodeId(e.point.x, e.point.y, 12);
      if (this.editorTool === 'move_node' && nodeId !== null) {
        this.dragNodeId = nodeId;
        this.map.dragPan.disable();
      } else if (this.editorTool === 'add_road' && nodeId !== null) {
        this.connectFromNodeId = nodeId;
      }
    });
    this.map.on('mousemove', (e) => {
      if (!this.editorMode || !this.mapData || this.dragNodeId === null) return;
      const lngLat = e.lngLat;
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
    this.map.on('mouseup', (e) => {
      if (!this.editorMode || !this.mapData) return;
      if (this.dragNodeId !== null) {
        const lngLat = e.lngLat;
        const nodeId = this.dragNodeId;
        this.dragNodeId = null;
        this.map.dragPan.enable();
        this.editorOverlay.clearGuides();
        editorMoveNode(nodeId, lngLat.lat, lngLat.lng, true)
          .then((m) => this.applyCustomMapData(m))
          .catch(console.error);
        return;
      }
      if (this.editorTool === 'add_road' && this.connectFromNodeId !== null) {
        const fromNodeId = this.connectFromNodeId;
        this.connectFromNodeId = null;
        const targetNode = this.findNearestNodeId(e.point.x, e.point.y, 12);
        if (targetNode !== null && targetNode !== fromNodeId) {
          editorConnect(fromNodeId, targetNode).then((m) => this.applyCustomMapData(m)).catch(console.error);
        } else {
          const ll = e.lngLat;
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
    this.makePanelDraggable(panel, '.edge-editor-title', 'edge-editor-panel');
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
    this.makePanelDraggable(panel, undefined, 'tool-switch-panel');
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

  private enableHudPanelDragging(): void {
    const targets: Array<{ selector: string; handle?: string; storageKey: string }> = [
      { selector: '#sandbox-panel', handle: '.sbx-header', storageKey: 'sandbox-panel' },
      { selector: '.editor-panel', handle: '.editor-titlebar', storageKey: 'editor-panel' },
      { selector: '#control-panel', handle: '#control-panel-header', storageKey: 'control-panel' },
      { selector: '#light-control-panel', handle: '#light-panel-header', storageKey: 'light-control-panel' },
      { selector: '#idm-debug-panel', storageKey: 'idm-debug-panel' },
      { selector: '#vehicle-telemetry-panel', storageKey: 'vehicle-telemetry-panel' },
      { selector: '#satisfaction-bar', storageKey: 'satisfaction-bar' },
      { selector: '#score-display', storageKey: 'score-display' },
      { selector: '#clock-display', storageKey: 'clock-display' },
    ];
    for (const t of targets) {
      const panel = document.querySelector(t.selector) as HTMLElement | null;
      if (!panel) continue;
      this.makePanelDraggable(panel, t.handle, t.storageKey);
    }
  }

  private makePanelDraggable(panel: HTMLElement, handleSelector?: string, storageKey?: string): void {
    if (panel.dataset.draggableInit === '1') return;
    panel.dataset.draggableInit = '1';

    const handle = handleSelector
      ? (panel.querySelector(handleSelector) as HTMLElement | null) ?? panel
      : panel;
    handle.style.cursor = 'move';

    const storageSlot = storageKey
      ? `hud-panel-pos:${storageKey}`
      : `hud-panel-pos:${panel.id || panel.className || 'panel'}`;
    this.restorePanelPosition(panel, storageSlot);

    let dragging = false;
    let offsetX = 0;
    let offsetY = 0;

    const onMove = (ev: MouseEvent): void => {
      if (!dragging) return;
      panel.style.left = `${Math.max(0, ev.clientX - offsetX)}px`;
      panel.style.top = `${Math.max(0, ev.clientY - offsetY)}px`;
    };
    const onUp = (): void => {
      dragging = false;
      window.removeEventListener('mousemove', onMove);
      window.removeEventListener('mouseup', onUp);
      this.persistPanelPosition(panel, storageSlot);
    };

    handle.addEventListener('mousedown', (ev: MouseEvent) => {
      const target = ev.target as HTMLElement | null;
      if (target?.closest('button, input, textarea, select, label, a')) return;

      const rect = panel.getBoundingClientRect();
      panel.style.position = 'fixed';
      panel.style.left = `${rect.left}px`;
      panel.style.top = `${rect.top}px`;
      panel.style.right = 'auto';
      panel.style.bottom = 'auto';
      panel.style.margin = '0';

      dragging = true;
      offsetX = ev.clientX - rect.left;
      offsetY = ev.clientY - rect.top;
      window.addEventListener('mousemove', onMove);
      window.addEventListener('mouseup', onUp);
      ev.preventDefault();
    });
  }

  private restorePanelPosition(panel: HTMLElement, storageSlot: string): void {
    try {
      const raw = localStorage.getItem(storageSlot);
      if (!raw) return;
      const parsed = JSON.parse(raw) as { left?: number; top?: number };
      if (typeof parsed.left !== 'number' || typeof parsed.top !== 'number') return;
      panel.style.position = 'fixed';
      panel.style.left = `${Math.max(0, parsed.left)}px`;
      panel.style.top = `${Math.max(0, parsed.top)}px`;
      panel.style.right = 'auto';
      panel.style.bottom = 'auto';
      panel.style.margin = '0';
    } catch {
      // Ignore invalid localStorage value.
    }
  }

  private persistPanelPosition(panel: HTMLElement, storageSlot: string): void {
    const rect = panel.getBoundingClientRect();
    const payload = { left: Math.max(0, rect.left), top: Math.max(0, rect.top) };
    try {
      localStorage.setItem(storageSlot, JSON.stringify(payload));
    } catch {
      // Ignore storage write errors (e.g. quota/private mode).
    }
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
    // Rebuild picked-vehicle debug geometry every frame to avoid stale markers
    // that otherwise only refresh on map interactions (zoom/pan).
    if (this.pickedVehicleDebugOverlayVisible) {
      this.redrawSelectedRoute();
      this.redrawPickDebugHud();
    }
    if (this.trafficMotionDebugEnabled) {
      this.redrawTrafficLeaderDebug();
    }
    // Keep the dedicated pick-debug canvas in lockstep with moving vehicles.
    this.overlay.renderPickDebug();

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

  private hideSimulationHud(): void {
    const hideById = (id: string) => {
      const el = document.getElementById(id);
      if (el) el.style.display = 'none';
    };
    hideById('clock-display');
    hideById('control-panel');
    hideById('score-display');
    hideById('satisfaction-bar');
    hideById('idm-debug-panel');
    hideById('vehicle-telemetry-panel');
    hideById('light-control-panel');
  }

  // ─── Cleanup ───────────────────────────────────────────────────────────────

  destroy(): void {
    this.unlistenCongestion?.();
    this.unlistenLights?.();
    this.unlistenGameOver?.();
    this.bboxPicker?.destroy();
    this.unlistenIdmDebug?.();
    this.unlistenLeaderDebug?.();
    this.sandboxUI?.destroy();
    this.overlay.destroyPickDebugApp();
    this.debugRouteGfx = null;
    this.pickDebugHud = null;
    this.laneLinesGfx?.destroy();
    this.turnConnectorGfx?.destroy();
    this.conflictGfx?.destroy();
    this.trafficDebugGfx?.destroy();
    this.trafficDebugLabels?.destroy();
    this.mapScenarioEditorUI?.destroy();
    this.edgeEditorPanel?.remove();
    this.toolSwitchPanel?.remove();
    this.edgeEditorPanel = null;
    this.toolSwitchPanel = null;
    this.buildingRenderer.destroy();
    this.roadRenderer.destroy();
    this.vehicleRenderer.destroy();
    this.infraRenderer.destroy();
    this.trafficLightRenderer.destroy();
    this.congestionRenderer.destroy();
  }
}
