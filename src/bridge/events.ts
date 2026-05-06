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
  /** Configured green phase duration (seconds). */
  greenDuration: number;
  /** Configured red phase duration (seconds). */
  redDuration: number;
}

// ─── Binary frame parser ──────────────────────────────────────────────────────

/**
 * Decode a base64-encoded binary vehicle frame.
 *
 * Packet layout (32 bytes, 4-byte aligned):
 *   [0..3]   id:           u32  LE
 *   [4..7]   lat:          f32  LE
 *   [8..11]  lng:          f32  LE
 *   [12..15] angle:        f32  LE
 *   [16..19] speed:        f32  LE
 *   [20]     vehicleType:  u8   (0=Car, 1=Van, 2=Bus, 3=Truck, 4=Tram)
 *   [21]     driverProfile:u8
 *   [22]     tripKind:     u8   (0=local_od, 1=transit, 2=ext_in, 3=ext_out)
 *   [23]     currentLane:   u8   (lane index, 0=innermost)
 *   [24..27] frustration:   f32  LE  (0=calm, 100=rage)
 *   [28..31] lateralOffset: f32  LE  (smooth: 0.0=lane-0 centre, 1.0=lane-1 …)
 */
export function parseVehicleFrame(base64Data: string): VehicleState[] {
  const binaryStr = atob(base64Data);
  const len = binaryStr.length;
  const bytes = new Uint8Array(len);
  for (let i = 0; i < len; i++) {
    bytes[i] = binaryStr.charCodeAt(i);
  }

  const PACKET_SIZE = 32;
  const count = Math.floor(bytes.byteLength / PACKET_SIZE);
  const view = new DataView(bytes.buffer);
  const vehicles: VehicleState[] = new Array(count);

  for (let i = 0; i < count; i++) {
    const base = i * PACKET_SIZE;
    vehicles[i] = {
      id:            view.getUint32 (base,      true),
      lat:           view.getFloat32(base + 4,  true),
      lng:           view.getFloat32(base + 8,  true),
      angle:         view.getFloat32(base + 12, true),
      speed:         view.getFloat32(base + 16, true),
      vehicleType:   view.getUint8  (base + 20),
      driverProfile: view.getUint8  (base + 21),
      tripKind:      view.getUint8  (base + 22),
      currentLane:   view.getUint8  (base + 23),
      frustration:   view.getFloat32(base + 24, true),
      lateralOffset: view.getFloat32(base + 28, true),
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
