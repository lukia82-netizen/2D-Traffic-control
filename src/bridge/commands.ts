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
}

export interface MapData {
  nodes: NodeData[];
  edges: EdgeData[];
  spawnPoints: [number, number][];
  bbox: [number, number, number, number];
}

// ─── Typed invoke wrappers ────────────────────────────────────────────────────

/**
 * Request Rust to parse the PBF file for the given bbox and return graph data.
 * bbox: [west, south, east, north]
 */
export async function loadMap(
  bbox: [number, number, number, number],
): Promise<MapData> {
  // Pass bbox as [west, south, east, north] array — Rust receives [f64; 4]
  return invoke<MapData>('load_map', { bbox });
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
 * Change the traffic light control mode for an intersection.
 * mode: 'Manual' | 'SemiAuto' | 'Auto' | 'Adaptive'
 */
export async function setTrafficLightMode(
  intersectionId: number,
  mode: string,
): Promise<void> {
  return invoke<void>('set_traffic_light_mode', { intersectionId, mode });
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
