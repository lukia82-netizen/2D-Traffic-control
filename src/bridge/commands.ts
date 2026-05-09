import { invoke, Channel } from '@tauri-apps/api/core';

// ─── Domain types ────────────────────────────────────────────────────────────

export interface BBox {
  west: number;
  south: number;
  east: number;
  north: number;
}

export interface NodeData {
  id: number;
  lat: number;
  lng: number;
  intersectionType: string;
}

export interface EdgeData {
  from: number;
  to: number;
  lanes: number;
  maxSpeed: number;
  oneway: boolean;
  infraType: string;
  layer: number;
  lengthM: number;
  roadType: string;
  /** Per-lane turn directions: "left" | "straight" | "right" | "uturn" */
  laneDirections: string[];
}

export interface BuildingData {
  id: number;
  /** Ordered list of [lng, lat] vertices (GeoJSON convention). */
  polygon: [number, number][];
  buildingType: 'residential' | 'commercial' | 'office' | 'other';
}

/** @deprecated Use BuildingData */
export interface BuildingPolygon {
  polygon: [number, number][];
}

export interface TurnRestriction {
  fromWayId: number;
  viaNodeId: number;
  toWayId: number;
  kind:
    | 'no_left_turn'
    | 'no_right_turn'
    | 'no_straight_on'
    | 'no_u_turn'
    | 'only_left_turn'
    | 'only_right_turn'
    | 'only_straight_on'
    | 'no_entry';
}

export interface TramStop {
  id: number;
  lat: number;
  lng: number;
  dwellS: number;
}

export interface TramEdge {
  fromOsmId: number;
  toOsmId: number;
  lengthM: number;
  maxSpeed: number;
  /** 'dedicated' | 'shared_with_road' */
  trackType: string;
}

export interface LaneData {
  id: number;
  width: number;
  connections: number[];
  conflictAreas: number[];
  points: [number, number][];
  lengthM: number;
  fromNodeOsmId: number;
  toNodeOsmId: number;
  laneIndex: number;
  /** True for connector (junction-crossing arc) lanes; false for straight road lanes. */
  isConnector: boolean;
}

export interface ConflictAreaData {
  id: number;
  centerLat: number;
  centerLng: number;
  radiusM: number;
  laneIds: number[];
}

export interface MapData {
  nodes: NodeData[];
  edges: EdgeData[];
  spawnPoints: [number, number][];
  bbox: [number, number, number, number];
  buildings: BuildingData[];
  restrictions: TurnRestriction[];
  tramStops: TramStop[];
  turnConnectors: TurnConnector[];
  lanes: LaneData[];
  conflictAreas: ConflictAreaData[];
}

export interface TurnConnector {
  fromNodeId: number;
  viaNodeId: number;
  toNodeId: number;
  bezierLut: [number, number][];
}

// ─── Typed invoke wrappers ────────────────────────────────────────────────────

/**
 * Request Rust to build and return map data.
 * bbox: [west, south, east, north]
 *
 * forceSandbox: when provided, skip Overpass and build the sandbox grid.
 *   Values: 'mixed' | 'one_lane' | 'two_lane' | 'three_lane'
 *   Pass null/undefined to use the real OSM map.
 * laneWidthM: physical lane width used for lane offset generation.
 */
export async function loadMap(
  bbox: [number, number, number, number],
  forceSandbox?: string | null,
  laneWidthM?: number,
): Promise<MapData> {
  return invoke<MapData>('load_map', {
    bbox: {
      west: bbox[0],
      south: bbox[1],
      east: bbox[2],
      north: bbox[3],
    },
    forceSandbox: forceSandbox ?? null,
    laneWidthM: laneWidthM ?? null,
  });
}

/**
 * Start the simulation loop. Rust will push binary vehicle frames via the
 * Channel every ~16 ms.
 */
export async function startSimulation(
  channel: Channel<string>,
): Promise<void> {
  return invoke<void>('start_simulation', { onVehicleFrame: channel });
}

export async function pauseSimulation(): Promise<void> {
  return invoke<void>('pause_simulation');
}

export async function resumeSimulation(): Promise<void> {
  return invoke<void>('resume_simulation');
}

/**
 * Set the time-scale multiplier (e.g. 60 = 1 real second = 1 game minute).
 */
export async function setTimeScale(scale: number): Promise<void> {
  return invoke<void>('set_time_scale', { scale });
}

/**
 * Set the maximum number of simultaneously active vehicles.
 * Takes effect on the next spawn tick.
 */
export async function setMaxVehicles(count: number): Promise<void> {
  return invoke<void>('set_max_vehicles', { count });
}

/** Select a vehicle for backend debug tracking (`null` clears selection). */
export async function setDebugVehicle(vehicleId: number | null): Promise<void> {
  return invoke<void>('set_debug_vehicle', { vehicleId });
}

/**
 * Converts PascalCase/CamelCase → snake_case for Rust traffic-light mode
 * (e.g. 'SemiAuto' → 'semi_auto').
 */
export function trafficLightModeToRust(mode: string): string {
  return mode.replace(/([a-z])([A-Z])/g, '$1_$2').toLowerCase();
}

/**
 * Change the traffic light control mode for an intersection.
 * mode: 'Manual' | 'SemiAuto' | 'Auto' | 'Adaptive'
 *
 * Converts PascalCase/CamelCase → snake_case before sending to Rust
 * (e.g. 'SemiAuto' → 'semi_auto').
 */
export async function setTrafficLightMode(
  intersectionId: number,
  mode: string,
): Promise<void> {
  const rustMode = trafficLightModeToRust(mode);
  return invoke<void>('set_traffic_light_mode', { intersectionId, mode: rustMode });
}

/** Manual “next step” for multi-phase vehicle junctions (must match Rust `TL_CMD_ADVANCE_STEP`). */
export const TRAFFIC_LIGHT_PHASE_ADVANCE_PROGRAM = 250;

/**
 * Manual traffic-light command.
 *
 * Pedestrian crossings: 0=R, 1=Y, 2=G for the car-facing lamp.
 *
 * Signalised intersections (movement programs): advance to the **next timed step**
 * (green → yellow → all-red cycle, etc.) with `phase = TRAFFIC_LIGHT_PHASE_ADVANCE_PROGRAM` (250).
 */
export async function setTrafficLightPhase(
  intersectionId: number,
  phase: number,
): Promise<void> {
  return invoke<void>('set_traffic_light_phase', { intersectionId, phase });
}

// ── Speed / compliance config ─────────────────────────────────────────────────

export interface ComplianceRange {
  base: number;
  min: number;
  max: number;
}

export interface RouteConfig {
  refSpeedMs: number;
  noiseSigma: number;
  normalAlpha: [number, number];
  sundayAlpha: [number, number];
  piratAlpha: [number, number];
  cautiousAlpha: [number, number];
}

export interface RageConfig {
  standstillThresholdS: [number, number, number, number];
  decayRateLinear: [number, number, number, number];
  recoveryRate: [number, number, number, number];
  crawlFraction: number;
  crawlThresholdS: number;
  crawlRate: [number, number, number, number];
  repeatStopBonus: [number, number, number, number];
  globalLossThreshold: number;
  globalLossDurationS: number;
  massRageFraction: number;
}

export interface SpeedConfig {
  urban1lane: number;
  urban2lane: number;
  urban3lanePlus: number;
  urbanMotorway: number;
  urbanResidential: number;
  urbanLiving: number;
  rural1lane: number;
  rural2lanePlus: number;
  ruralMotorway: number;
  complianceNormal: ComplianceRange;
  complianceSunday: ComplianceRange;
  compliancePirat: ComplianceRange;
  complianceCautious: ComplianceRange;
  noiseSigma: number;
  route: RouteConfig;
  rage: RageConfig;
}

/**
 * Update speed / compliance / rage configuration at runtime.
 * Changes take effect for newly spawned vehicles.
 */
export async function setSpeedConfig(config: SpeedConfig): Promise<void> {
  return invoke<void>('set_speed_config', { config });
}

/**
 * Set the green and red phase durations for a traffic light.
 * Effective in SemiAuto and Auto modes; has no effect in Manual / Adaptive.
 */
export async function setLightDurations(
  intersectionId: number,
  greenS: number,
  redS: number,
): Promise<void> {
  return invoke<void>('set_light_durations', { intersectionId, greenS, redS });
}

export type EditorTool = 'none' | 'move_node' | 'add_road' | 'delete' | 'select';

export async function setEditorTool(tool: EditorTool): Promise<void> {
  return invoke<void>('set_editor_tool', { tool });
}

export async function editorMoveNode(
  nodeId: number,
  lat: number,
  lng: number,
  finalCommit: boolean,
): Promise<MapData> {
  return invoke<MapData>('editor_move_node', { payload: { nodeId, lat, lng, finalCommit } });
}

export async function editorExtrude(
  fromNodeId: number,
  newNodeId: number,
  lat: number,
  lng: number,
): Promise<MapData> {
  return invoke<MapData>('editor_extrude', { payload: { fromNodeId, newNodeId, lat, lng } });
}

export async function editorConnect(fromNodeId: number, toNodeId: number): Promise<MapData> {
  return invoke<MapData>('editor_connect', { payload: { fromNodeId, toNodeId } });
}

export async function editorDeleteEdge(fromNodeId: number, toNodeId: number): Promise<MapData> {
  return invoke<MapData>('editor_delete_edge', { payload: { fromNodeId, toNodeId } });
}

export async function editorUpdateEdgeTags(
  fromNodeId: number,
  toNodeId: number,
  lanes: number,
  oneway: boolean,
  laneDirections: string[],
): Promise<MapData> {
  return invoke<MapData>('editor_update_edge_tags', {
    payload: { fromNodeId, toNodeId, lanes, oneway, laneDirections },
  });
}

export async function editorUndo(): Promise<MapData> {
  return invoke<MapData>('editor_undo');
}

export async function editorRedo(): Promise<MapData> {
  return invoke<MapData>('editor_redo');
}

export async function saveMapOverrides(): Promise<void> {
  return invoke<void>('save_map_overrides');
}
