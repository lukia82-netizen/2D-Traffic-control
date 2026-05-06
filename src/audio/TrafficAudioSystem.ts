import type { VehicleState } from '../bridge/events';

const MAX_ENGINE_VEHICLES = 60;
const HORN_COOLDOWN_MS = 1800;
const HORN_FRUSTRATION_THRESHOLD = 82;

export class TrafficAudioSystem {
  private ctx: AudioContext | null = null;
  private masterGain: GainNode | null = null;
  private engineOsc: OscillatorNode | null = null;
  private engineGain: GainNode | null = null;
  private noiseNode: AudioBufferSourceNode | null = null;
  private noiseGain: GainNode | null = null;
  private lastHornAt = 0;
  private unlocked = false;

  init(): void {
    if (this.ctx) return;
    this.ctx = new AudioContext();
    this.masterGain = this.ctx.createGain();
    this.masterGain.gain.value = 0;
    this.masterGain.connect(this.ctx.destination);

    this.engineOsc = this.ctx.createOscillator();
    this.engineOsc.type = 'sawtooth';
    this.engineGain = this.ctx.createGain();
    this.engineGain.gain.value = 0;
    this.engineOsc.connect(this.engineGain);
    this.engineGain.connect(this.masterGain);
    this.engineOsc.start();

    const noiseBuffer = this.ctx.createBuffer(1, this.ctx.sampleRate * 2, this.ctx.sampleRate);
    const data = noiseBuffer.getChannelData(0);
    for (let i = 0; i < data.length; i++) data[i] = Math.random() * 2 - 1;
    this.noiseNode = this.ctx.createBufferSource();
    this.noiseNode.buffer = noiseBuffer;
    this.noiseNode.loop = true;
    this.noiseGain = this.ctx.createGain();
    this.noiseGain.gain.value = 0;
    this.noiseNode.connect(this.noiseGain);
    this.noiseGain.connect(this.masterGain);
    this.noiseNode.start();

    const unlock = (): void => {
      if (!this.ctx) return;
      void this.ctx.resume();
      this.masterGain!.gain.setTargetAtTime(0.16, this.ctx.currentTime, 0.4);
      this.unlocked = true;
      window.removeEventListener('pointerdown', unlock);
      window.removeEventListener('keydown', unlock);
    };
    window.addEventListener('pointerdown', unlock, { once: true });
    window.addEventListener('keydown', unlock, { once: true });
  }

  update(vehicles: Map<number, VehicleState>, muted: boolean): void {
    if (!this.ctx || !this.engineOsc || !this.engineGain || !this.noiseGain || !this.unlocked) return;
    if (muted) {
      this.engineGain.gain.setTargetAtTime(0.001, this.ctx.currentTime, 0.1);
      this.noiseGain.gain.setTargetAtTime(0.0001, this.ctx.currentTime, 0.1);
      return;
    }

    const vehicleCount = vehicles.size;
    let frustrationSum = 0;
    for (const v of vehicles.values()) frustrationSum += v.frustration;
    const avgFrustration = vehicleCount > 0 ? frustrationSum / vehicleCount : 0;

    const density = Math.min(1, vehicleCount / MAX_ENGINE_VEHICLES);
    const engineFreq = 45 + density * 85 + avgFrustration * 0.18;
    const engineVol = 0.01 + density * 0.08;
    const hissVol = 0.003 + density * 0.015;

    this.engineOsc.frequency.setTargetAtTime(engineFreq, this.ctx.currentTime, 0.1);
    this.engineGain.gain.setTargetAtTime(engineVol, this.ctx.currentTime, 0.15);
    this.noiseGain.gain.setTargetAtTime(hissVol, this.ctx.currentTime, 0.2);

    if (avgFrustration >= HORN_FRUSTRATION_THRESHOLD) {
      const now = performance.now();
      if (now - this.lastHornAt > HORN_COOLDOWN_MS) {
        this.playHorn(now);
      }
    }
  }

  private playHorn(nowMs: number): void {
    if (!this.ctx || !this.masterGain) return;
    this.lastHornAt = nowMs;
    const osc = this.ctx.createOscillator();
    const gain = this.ctx.createGain();
    osc.type = 'square';
    osc.frequency.setValueAtTime(420, this.ctx.currentTime);
    osc.frequency.linearRampToValueAtTime(360, this.ctx.currentTime + 0.12);
    gain.gain.setValueAtTime(0.0001, this.ctx.currentTime);
    gain.gain.exponentialRampToValueAtTime(0.08, this.ctx.currentTime + 0.02);
    gain.gain.exponentialRampToValueAtTime(0.0001, this.ctx.currentTime + 0.16);
    osc.connect(gain);
    gain.connect(this.masterGain);
    osc.start();
    osc.stop(this.ctx.currentTime + 0.18);
  }

  destroy(): void {
    this.engineOsc?.stop();
    this.noiseNode?.stop();
    this.engineOsc?.disconnect();
    this.engineGain?.disconnect();
    this.noiseNode?.disconnect();
    this.noiseGain?.disconnect();
    this.masterGain?.disconnect();
    this.ctx?.close().catch(() => undefined);
    this.ctx = null;
    this.unlocked = false;
  }
}
