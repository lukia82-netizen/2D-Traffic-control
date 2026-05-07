import { describe, expect, it } from 'vitest';

import type { VehicleState } from './events';
import { parseVehicleFrame } from './events';

function expectVehicleState(actual: VehicleState, expected: VehicleState): void {
  expect(actual.id).toBe(expected.id);
  expect(actual.vehicleType).toBe(expected.vehicleType);
  expect(actual.driverProfile).toBe(expected.driverProfile);
  expect(actual.tripKind).toBe(expected.tripKind);
  expect(actual.currentLane).toBe(expected.currentLane);
  expect(actual.onTurnConnector).toBe(expected.onTurnConnector);
  expect(actual.lat).toBeCloseTo(expected.lat, 8);
  expect(actual.lng).toBeCloseTo(expected.lng, 8);
  expect(actual.angle).toBeCloseTo(expected.angle, 5);
  expect(actual.speed).toBeCloseTo(expected.speed, 5);
  expect(actual.frustration).toBeCloseTo(expected.frustration, 5);
  expect(actual.lateralOffset).toBeCloseTo(expected.lateralOffset, 5);
}

// Packet layout v2 (40 bytes):
//   [0..3]   id:            u32 LE
//   [4..11]  lat:           f64 LE
//   [12..19] lng:           f64 LE
//   [20..23] angle:         f32 LE
//   [24..27] speed:         f32 LE
//   [28]     vehicleType:   u8
//   [29]     driverProfile: u8
//   [30]     tripKind:      u8
//   [31]     laneFlags:     u8
//   [32..35] frustration:   f32 LE
//   [36..39] lateralOffset: f32 LE
const PACKET_SIZE = 40;

function vehicleStatesToBase64(vehicles: VehicleState[]): string {
  const buf = new ArrayBuffer(vehicles.length * PACKET_SIZE);
  const view = new DataView(buf);
  for (let i = 0; i < vehicles.length; i++) {
    const v = vehicles[i]!;
    const base = i * PACKET_SIZE;
    view.setUint32(base,      v.id,            true);
    view.setFloat64(base + 4, v.lat,           true);
    view.setFloat64(base + 12, v.lng,          true);
    view.setFloat32(base + 20, v.angle,        true);
    view.setFloat32(base + 24, v.speed,        true);
    view.setUint8(base + 28, v.vehicleType);
    view.setUint8(base + 29, v.driverProfile);
    view.setUint8(base + 30, v.tripKind);
    const laneFlags = (v.currentLane & 0x7f) | (v.onTurnConnector ? 0x80 : 0);
    view.setUint8(base + 31, laneFlags);
    view.setFloat32(base + 32, v.frustration,   true);
    view.setFloat32(base + 36, v.lateralOffset, true);
  }
  const bytes = new Uint8Array(buf);
  let binary = '';
  for (let i = 0; i < bytes.length; i++) {
    binary += String.fromCharCode(bytes[i]!);
  }
  return btoa(binary);
}

describe('parseVehicleFrame', () => {
  it('returns an empty array for empty payload', () => {
    expect(parseVehicleFrame('')).toEqual([]);
  });

  it('decodes a single 40-byte vehicle packet', () => {
    const expected: VehicleState = {
      id: 4242,
      lat: 52.2297,
      lng: 21.0122,
      angle: 1.57,
      speed: 12.5,
      vehicleType: 2,
      driverProfile: 1,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      frustration: 3.5,
      lateralOffset: 0.25,
    };
    const decoded = parseVehicleFrame(vehicleStatesToBase64([expected]));
    expect(decoded).toHaveLength(1);
    expectVehicleState(decoded[0]!, expected);
  });

  it('decodes multiple vehicles in order', () => {
    const a: VehicleState = {
      id: 1,
      lat: 0,
      lng: 0,
      angle: 0,
      speed: 0,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      frustration: 0,
      lateralOffset: 0,
    };
    const b: VehicleState = {
      ...a,
      id: 2,
      lat: 1,
      lng: 2,
      speed: 5,
      vehicleType: 4,
      currentLane: 2,
      onTurnConnector: true,
      frustration: 99.9,
      lateralOffset: 1.75,
    };
    const decoded = parseVehicleFrame(vehicleStatesToBase64([a, b]));
    expect(decoded).toHaveLength(2);
    expectVehicleState(decoded[0]!, a);
    expectVehicleState(decoded[1]!, b);
  });

  it('ignores trailing bytes that do not form a full packet', () => {
    const full: VehicleState = {
      id: 7,
      lat: 51.8,
      lng: 16.8,
      angle: 0,
      speed: 1,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      onTurnConnector: false,
      frustration: 0,
      lateralOffset: 0,
    };
    const buf = new ArrayBuffer(PACKET_SIZE + 10);
    const view = new DataView(buf);
    view.setUint32(0,  full.id,            true);
    view.setFloat64(4, full.lat,           true);
    view.setFloat64(12, full.lng,          true);
    view.setFloat32(20, full.angle,        true);
    view.setFloat32(24, full.speed,        true);
    view.setUint8(28, full.vehicleType);
    view.setUint8(29, full.driverProfile);
    view.setUint8(30, full.tripKind);
    const laneFlags = (full.currentLane & 0x7f) | (full.onTurnConnector ? 0x80 : 0);
    view.setUint8(31, laneFlags);
    view.setFloat32(32, full.frustration,   true);
    view.setFloat32(36, full.lateralOffset, true);
    const bytes = new Uint8Array(buf);
    let binary = '';
    for (let i = 0; i < bytes.length; i++) {
      binary += String.fromCharCode(bytes[i]!);
    }
    const base64 = btoa(binary);
    const decoded = parseVehicleFrame(base64);
    expect(decoded).toHaveLength(1);
    expectVehicleState(decoded[0]!, full);
  });
});
