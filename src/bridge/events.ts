import { listen } from '@tauri-apps/api/event';

// ─── Event payload types ──────────────────────────────────────────────────────

export interface VehicleState {
  id: number;
  lat: number;
  lng: number;
  angle: number;
  speed: number;
  vehicleType: number;
  driverProfile: number;
  satisfaction: number;
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
}

// ─── Binary frame parser ──────────────────────────────────────────────────────

/**
 * Decode a base64-encoded binary vehicle frame.
 *
 * Packet layout (28 bytes, 4-byte aligned):
 *   [0..3]   id:          u32  LE
 *   [4..7]   lat:         f32  LE
 *   [8..11]  lng:         f32  LE
 *   [12..15] angle:       f32  LE
 *   [16..19] speed:       f32  LE
 *   [20]     type:        u8
 *   [21]     profile:     u8
 *   [22..23] padding:     u16  (ignored)
 *   [24..27] satisfaction:f32  LE
 */
export function parseVehicleFrame(base64Data: string): VehicleState[] {
  // Decode base64 → Uint8Array
  const binaryStr = atob(base64Data);
  const len = binaryStr.length;
  const bytes = new Uint8Array(len);
  for (let i = 0; i < len; i++) {
    bytes[i] = binaryStr.charCodeAt(i);
  }

  const PACKET_SIZE = 28;
  const count = Math.floor(bytes.byteLength / PACKET_SIZE);
  const view = new DataView(bytes.buffer);
  const vehicles: VehicleState[] = new Array(count);

  for (let i = 0; i < count; i++) {
    const base = i * PACKET_SIZE;
    vehicles[i] = {
      id: view.getUint32(base, true),
      lat: view.getFloat32(base + 4, true),
      lng: view.getFloat32(base + 8, true),
      angle: view.getFloat32(base + 12, true),
      speed: view.getFloat32(base + 16, true),
      vehicleType: view.getUint8(base + 20),
      driverProfile: view.getUint8(base + 21),
      satisfaction: view.getFloat32(base + 24, true),
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
  const unlisten = await listen<CongestionData[]>(
    'congestion_update',
    (event) => {
      cb(event.payload);
    },
  );
  return unlisten;
}

/**
 * Subscribe to light_state_change events (fired on every phase transition).
 * Returns an unsubscribe function.
 */
export async function listenLightStateChanges(
  cb: (data: LightStateUpdate[]) => void,
): Promise<() => void> {
  const unlisten = await listen<LightStateUpdate[]>(
    'light_state_change',
    (event) => {
      cb(event.payload);
    },
  );
  return unlisten;
}
