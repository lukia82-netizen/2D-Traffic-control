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

export interface MapData {
  nodes: NodeData[];
  edges: EdgeData[];
  spawnPoints: [number, number][];
  bbox: [number, number, number, number];
  buildings: BuildingData[];
  restrictions: TurnRestriction[];
  tramStops: TramStop[];
}

// ─── Typed invoke wrappers ────────────────────────────────────────────────────

/**
 * Request Rust to build and return map data.
 * bbox: [west, south, east, north]
 *
 * forceSandbox: when provided, skip Overpass and build the sandbox grid.
 *   Values: 'mixed' | 'one_lane' | 'two_lane' | 'three_lane'
 *   Pass null/undefined to use the real OSM map.
 */
export async function loadMap(
  bbox: [number, number, number, number],
  forceSandbox?: string | null,
): Promise<MapData> {
  return invoke<MapData>('load_map', {
    bbox: {
      west: bbox[0],
      south: bbox[1],
      east: bbox[2],
      north: bbox[3],
    },
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
  // Convert PascalCase → snake_case: insert '_' before each uppercase letter
  // that follows a lowercase letter, then lowercase everything.
  const rustMode = mode
    .replace(/([a-z])([A-Z])/g, '$1_$2')
    .toLowerCase();
  return invoke<void>('set_traffic_light_mode', { intersectionId, mode: rustMode });
}

/**
 * Force a specific phase for a traffic light (used in Manual mode).
 * phase: 0 = Red, 1 = Yellow, 2 = Green
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
