import { listen } from '@tauri-apps/api/event';

// ─── Event payload types ──────────────────────────────────────────────────────

export interface VehicleState {
  id: number;
  lat: number;
  lng: number;
  angle: number;
  speed: number;
  vehicleType: number;   // 0=Car, 1=Van, 2=Bus, 3=Truck, 4=Tram
  driverProfile: number;
  tripKind: number;      // 0=local_od, 1=transit, 2=ext_in, 3=ext_out
  /** Lane index within the current edge's direction (0 = innermost / closest to centreline). */
  currentLane: number;
  /** True when vehicle currently follows a Bezier turn connector. */
  onTurnConnector: boolean;
  /** Smooth lateral position in lane-index units (0.0 = lane-0 centre, 1.0 = lane-1, …).
   *  Interpolated by Rust at ~0.35 lanes/s for GTA-style glide. */
  lateralOffset: number;
  /** Driver frustration 0 (calm) … 100 (rage). */
  frustration: number;
}

export interface CongestionData {
  edgeId: number;
  level: number;
  lat: number;
  lng: number;
}

export interface LightStateUpdate {
  intersectionId: number;
  phase: number;
  timeRemaining: number;
  /** Number of vehicles queued at this intersection (Adaptive mode sensor). */
  queueCount: number;
  /** Current control mode: "manual" | "semi_auto" | "auto" | "adaptive" */
  mode: string;
  /** Main through / straight green duration (seconds). */
  greenDuration: number;
  /** Secondary protected-left green duration (seconds). */
  redDuration: number;
  /**
   * Per inbound-arm bulb colour (phase 0=R, 1=Y, 2=G), clockwise sorted like the simulation.
   * Omitted at pedestrian crossings.
   */
  junctionArmPhases?: number[];
}

// ─── Binary frame parser ──────────────────────────────────────────────────────

/**
 * Decode a base64-encoded binary vehicle frame.
 *
 * Packet layout (40 bytes):
 *   [0..3]   id:            u32  LE
 *   [4..11]  lat:           f64  LE  ← double precision, eliminates f32 quantisation jitter
 *   [12..19] lng:           f64  LE  ← double precision
 *   [20..23] angle:         f32  LE
 *   [24..27] speed:         f32  LE
 *   [28]     vehicleType:   u8   (0=Car, 1=Van, 2=Bus, 3=Truck, 4=Tram)
 *   [29]     driverProfile: u8
 *   [30]     tripKind:      u8   (0=local_od, 1=transit, 2=ext_in, 3=ext_out)
 *   [31]     laneFlags:     u8   (bits 0..6: currentLane, bit7: onTurnConnector)
 *   [32..35] frustration:   f32  LE  (0=calm, 100=rage)
 *   [36..39] lateralOffset: f32  LE  (smooth: 0.0=lane-0 centre, 1.0=lane-1 …)
 */
export function parseVehicleFrame(base64Data: string): VehicleState[] {
  const binaryStr = atob(base64Data);
  const len = binaryStr.length;
  const bytes = new Uint8Array(len);
  for (let i = 0; i < len; i++) {
    bytes[i] = binaryStr.charCodeAt(i);
  }

  const PACKET_SIZE = 40;
  const count = Math.floor(bytes.byteLength / PACKET_SIZE);
  const view = new DataView(bytes.buffer);
  const vehicles: VehicleState[] = new Array(count);

  for (let i = 0; i < count; i++) {
    const base = i * PACKET_SIZE;
    const laneFlags = view.getUint8(base + 31);
    vehicles[i] = {
      id:              view.getUint32 (base,      true),
      lat:             view.getFloat64(base + 4,  true),
      lng:             view.getFloat64(base + 12, true),
      angle:           view.getFloat32(base + 20, true),
      speed:           view.getFloat32(base + 24, true),
      vehicleType:     view.getUint8  (base + 28),
      driverProfile:   view.getUint8  (base + 29),
      tripKind:        view.getUint8  (base + 30),
      currentLane:     laneFlags & 0x7f,
      onTurnConnector: (laneFlags & 0x80) !== 0,
      frustration:     view.getFloat32(base + 32, true),
      lateralOffset:   view.getFloat32(base + 36, true),
    };
  }

  return vehicles;
}

// ─── Event listeners ──────────────────────────────────────────────────────────

/**
 * Subscribe to congestion_update events (fired every ~500 ms real time).
 * Returns an unsubscribe function.
 */
export async function listenCongestionUpdates(
  cb: (data: CongestionData[]) => void,
): Promise<() => void> {
  const unlisten = await listen<CongestionData[]>('congestion_update', (event) => {
    cb(event.payload);
  });
  return unlisten;
}

/**
 * Subscribe to light_state_change events (fired on every phase transition).
 * Returns an unsubscribe function.
 */
export async function listenLightStateChanges(
  cb: (data: LightStateUpdate[]) => void,
): Promise<() => void> {
  const unlisten = await listen<LightStateUpdate[]>('light_state_change', (event) => {
    cb(event.payload);
  });
  return unlisten;
}

// ─── Game Over ────────────────────────────────────────────────────────────────

export interface GameOverPayload {
  /** "avg_frustration" or "mass_rage" */
  reason: string;
  /** The triggering value (frustration % or rage fraction %). */
  value: number;
  /** In-game timestamp in seconds (e.g. 6*3600 = 06:00). */
  timestampGame: number;
}

export interface IdmDebugPayload {
  vehicleId: number;
  speed: number;
  gap: number;
  deltaV: number;
  distToStopLine: number;
  redBlocking: boolean;
  onCurve: boolean;
  turnT: number;
  routePoints: [number, number][];
}

/**
 * Subscribe to the one-shot game_over event.
 * Returns an unsubscribe function.
 */
export async function listenGameOver(
  cb: (data: GameOverPayload) => void,
): Promise<() => void> {
  const unlisten = await listen<GameOverPayload>('game_over', (event) => {
    cb(event.payload);
  });
  return unlisten;
}

/**
 * Subscribe to IDM debug snapshots for one representative vehicle.
 * Returns an unsubscribe function.
 */
export async function listenIdmDebug(
  cb: (data: IdmDebugPayload) => void,
): Promise<() => void> {
  const unlisten = await listen<IdmDebugPayload>('idm_debug', (event) => {
    cb(event.payload);
  });
  return unlisten;
}
