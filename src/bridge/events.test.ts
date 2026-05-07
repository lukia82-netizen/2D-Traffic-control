import { describe, expect, it } from 'vitest';

import type { VehicleState } from './events';
import { parseVehicleFrame } from './events';

function expectVehicleState(actual: VehicleState, expected: VehicleState): void {
  expect(actual.id).toBe(expected.id);
  expect(actual.vehicleType).toBe(expected.vehicleType);
  expect(actual.driverProfile).toBe(expected.driverProfile);
  expect(actual.tripKind).toBe(expected.tripKind);
  expect(actual.currentLane).toBe(expected.currentLane);
  expect(actual.lat).toBeCloseTo(expected.lat, 5);
  expect(actual.lng).toBeCloseTo(expected.lng, 5);
  expect(actual.angle).toBeCloseTo(expected.angle, 5);
  expect(actual.speed).toBeCloseTo(expected.speed, 5);
  expect(actual.frustration).toBeCloseTo(expected.frustration, 5);
  expect(actual.lateralOffset).toBeCloseTo(expected.lateralOffset, 5);
}

const PACKET_SIZE = 32;

function vehicleStatesToBase64(vehicles: VehicleState[]): string {
  const buf = new ArrayBuffer(vehicles.length * PACKET_SIZE);
  const view = new DataView(buf);
  for (let i = 0; i < vehicles.length; i++) {
    const v = vehicles[i]!;
    const base = i * PACKET_SIZE;
    view.setUint32(base, v.id, true);
    view.setFloat32(base + 4, v.lat, true);
    view.setFloat32(base + 8, v.lng, true);
    view.setFloat32(base + 12, v.angle, true);
    view.setFloat32(base + 16, v.speed, true);
    view.setUint8(base + 20, v.vehicleType);
    view.setUint8(base + 21, v.driverProfile);
    view.setUint8(base + 22, v.tripKind);
    view.setUint8(base + 23, v.currentLane);
    view.setFloat32(base + 24, v.frustration, true);
    view.setFloat32(base + 28, v.lateralOffset, true);
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

  it('decodes a single 32-byte vehicle packet', () => {
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
      lat: 10,
      lng: 20,
      angle: 0,
      speed: 1,
      vehicleType: 0,
      driverProfile: 0,
      tripKind: 0,
      currentLane: 0,
      frustration: 0,
      lateralOffset: 0,
    };
    const buf = new ArrayBuffer(PACKET_SIZE + 10);
    const view = new DataView(buf);
    view.setUint32(0, full.id, true);
    view.setFloat32(4, full.lat, true);
    view.setFloat32(8, full.lng, true);
    view.setFloat32(12, full.angle, true);
    view.setFloat32(16, full.speed, true);
    view.setUint8(20, full.vehicleType);
    view.setUint8(21, full.driverProfile);
    view.setUint8(22, full.tripKind);
    view.setUint8(23, full.currentLane);
    view.setFloat32(24, full.frustration, true);
    view.setFloat32(28, full.lateralOffset, true);
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
