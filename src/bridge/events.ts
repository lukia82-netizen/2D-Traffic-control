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
 *   [23]     padding:      u8
 *   [24..27] frustration:  f32  LE  (0=calm, 100=rage)
 *   [28..31] padding2:     u32  LE  (reserved)
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
      frustration:   view.getFloat32(base + 24, true),
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
