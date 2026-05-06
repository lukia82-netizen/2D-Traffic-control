import maplibregl from 'maplibre-gl';
import type { LightStateUpdate } from '../bridge/events';
import type { NodeData } from '../bridge/commands';
import {
  setTrafficLightMode,
  setTrafficLightPhase,
  setLightDurations,
} from '../bridge/commands';

type LightMode = 'Auto' | 'SemiAuto' | 'Manual' | 'Adaptive';

// Phase index constants
const PHASE_RED = 0;
const PHASE_YELLOW = 1;
const PHASE_GREEN = 2;

// ─── TrafficLightUI ───────────────────────────────────────────────────────────

/**
 * Manages the traffic light control panel in the HUD.
 *
 * Supports four modes:
 *   Auto      – fixed-time automatic cycling
 *   SemiAuto  – player sets green/red durations, automaton executes
 *   Manual    – player manually forces each phase
 *   Adaptive  – green duration scales with queue count (sensor loop)
 */
export class TrafficLightUI {
  private readonly map: maplibregl.Map;

  private readonly panel: HTMLElement;
  private readonly title: HTMLElement;
  private readonly timerEl: HTMLElement;
  private readonly modeSelect: HTMLSelectElement;
  private readonly manualControls: HTMLElement;
  private readonly semiAutoControls: HTMLElement;
  private readonly adaptiveInfo: HTMLElement;
  private readonly greenDurationInput: HTMLInputElement;
  private readonly redDurationInput: HTMLInputElement;
  private readonly applyDurationsBtn: HTMLButtonElement;
  private readonly adaptiveQueueCount: HTMLElement;
  private readonly adaptiveEffectiveGreen: HTMLElement;
  private readonly phaseDots: HTMLElement[];
  private readonly closeBtn: HTMLElement;

  private selectedIntersectionId: number | null = null;
  private readonly lightStates: Map<number, LightStateUpdate> = new Map();
  private intersectionNodes: NodeData[] = [];

  // Click-detect radius in pixels
  private static readonly CLICK_RADIUS_PX = 20;

  constructor(map: maplibregl.Map) {
    this.map = map;

    this.panel = this.require('light-control-panel');
    this.title = this.require('light-panel-title');
    this.timerEl = this.require('light-timer');
    this.modeSelect = this.require('light-mode') as HTMLSelectElement;
    this.manualControls = this.require('light-manual-controls');
    this.semiAutoControls = this.require('light-semiauto-controls');
    this.adaptiveInfo = this.require('light-adaptive-info');
    this.greenDurationInput = this.require('green-duration') as HTMLInputElement;
    this.redDurationInput = this.require('red-duration') as HTMLInputElement;
    this.applyDurationsBtn = this.require('apply-durations-btn') as HTMLButtonElement;
    this.adaptiveQueueCount = this.require('adaptive-queue-count');
    this.adaptiveEffectiveGreen = this.require('adaptive-effective-green');
    this.closeBtn = this.require('light-panel-close');

    this.phaseDots = [
      this.require('phase-red'),
      this.require('phase-yellow'),
      this.require('phase-green'),
    ];
  }

  // ─── Lifecycle ─────────────────────────────────────────────────────────────

  init(nodes: NodeData[]): void {
    this.intersectionNodes = nodes.filter(
      (n) => n.intersectionType === 'traffic_light',
    );

    // Map click → check proximity to intersection nodes
    this.map.on('click', (e) => {
      const clickPx = this.map.project([e.lngLat.lng, e.lngLat.lat]);
      const closest = this.findClosestIntersection(clickPx.x, clickPx.y);

      if (closest !== null) {
        this.selectIntersection(closest);
      } else {
        this.hidePanel();
      }
    });

    // Mode change
    this.modeSelect.addEventListener('change', () => {
      if (this.selectedIntersectionId === null) return;
      const mode = this.modeSelect.value as LightMode;
      setTrafficLightMode(this.selectedIntersectionId, mode).catch(console.error);
      this.updateModeControls(mode);
    });

    // SemiAuto: apply durations button
    this.applyDurationsBtn.addEventListener('click', () => {
      if (this.selectedIntersectionId === null) return;
      const greenS = parseFloat(this.greenDurationInput.value) || 30;
      const redS = parseFloat(this.redDurationInput.value) || 30;
      setLightDurations(this.selectedIntersectionId, greenS, redS).catch(console.error);
    });

    // Manual phase buttons
    this.manualControls.querySelectorAll<HTMLButtonElement>('.phase-btn').forEach(
      (btn) => {
        btn.addEventListener('click', () => {
          if (this.selectedIntersectionId === null) return;
          const phase = Number(btn.dataset.phase);
          setTrafficLightPhase(this.selectedIntersectionId, phase).catch(console.error);
        });
      },
    );

    // Close button
    this.closeBtn.addEventListener('click', () => this.hidePanel());
  }

  // ─── Selection ─────────────────────────────────────────────────────────────

  selectIntersection(id: number): void {
    this.selectedIntersectionId = id;
    this.showPanel(id);
  }

  showPanel(id: number): void {
    this.title.textContent = `Intersection #${id}`;
    this.panel.classList.remove('hidden');

    // Restore saved state if available
    const state = this.lightStates.get(id);
    if (state) {
      this.applyPhaseVisual(state.phase);
      this.timerEl.textContent = `${Math.round(state.timeRemaining)}s`;
      // Sync the mode selector
      const modeValue = this.rustModeToSelectValue(state.mode);
      this.modeSelect.value = modeValue;
      this.updateModeControls(modeValue as LightMode, state);
    } else {
      this.resetPhaseVisual();
      this.timerEl.textContent = '--s';
      this.updateModeControls('Auto');
    }
  }

  hidePanel(): void {
    this.panel.classList.add('hidden');
    this.selectedIntersectionId = null;
  }

  // ─── State updates ─────────────────────────────────────────────────────────

  updateLightState(updates: LightStateUpdate[]): void {
    for (const upd of updates) {
      this.lightStates.set(upd.intersectionId, upd);

      if (upd.intersectionId === this.selectedIntersectionId) {
        this.applyPhaseVisual(upd.phase);
        this.timerEl.textContent = `${Math.round(upd.timeRemaining)}s`;

        // Update adaptive readouts
        if (upd.mode === 'adaptive') {
          this.adaptiveQueueCount.textContent = String(upd.queueCount);
          // Effective green: base 20s + up to 40s based on queue (mirror of Rust logic)
          const effectiveGreen = Math.round(
            Math.min(60, 20 + (Math.min(upd.queueCount, 20) / 20) * 40),
          );
          this.adaptiveEffectiveGreen.textContent = String(effectiveGreen);
        }

        // Sync mode selector if it changed externally
        const modeValue = this.rustModeToSelectValue(upd.mode);
        if (this.modeSelect.value !== modeValue) {
          this.modeSelect.value = modeValue;
          this.updateModeControls(modeValue as LightMode, upd);
        }
      }
    }
  }

  // ─── Helpers ───────────────────────────────────────────────────────────────

  private updateModeControls(mode: LightMode, state?: LightStateUpdate): void {
    this.manualControls.classList.toggle('hidden', mode !== 'Manual');
    this.semiAutoControls.classList.toggle('hidden', mode !== 'SemiAuto');
    this.adaptiveInfo.classList.toggle('hidden', mode !== 'Adaptive');

    // Pre-fill SemiAuto inputs from current state
    if (mode === 'SemiAuto' && state) {
      this.greenDurationInput.value = String(Math.round(state.greenDuration));
      this.redDurationInput.value = String(Math.round(state.redDuration));
    }
  }

  /** Convert Rust snake_case mode string to the <select> option value. */
  private rustModeToSelectValue(rustMode: string): string {
    switch (rustMode) {
      case 'semi_auto': return 'SemiAuto';
      case 'manual':    return 'Manual';
      case 'adaptive':  return 'Adaptive';
      default:          return 'Auto';
    }
  }

  private findClosestIntersection(px: number, py: number): number | null {
    let bestId: number | null = null;
    let bestDist = TrafficLightUI.CLICK_RADIUS_PX;

    for (const node of this.intersectionNodes) {
      const nodePx = this.map.project([node.lng, node.lat]);
      const dist = Math.hypot(nodePx.x - px, nodePx.y - py);
      if (dist < bestDist) {
        bestDist = dist;
        bestId = node.id;
      }
    }

    return bestId;
  }

  private applyPhaseVisual(phase: number): void {
    for (let i = 0; i < this.phaseDots.length; i++) {
      this.phaseDots[i].classList.toggle('active', i === phase);
    }
  }

  private resetPhaseVisual(): void {
    for (const dot of this.phaseDots) dot.classList.remove('active');
  }

  private require(id: string): HTMLElement {
    const el = document.getElementById(id);
    if (!el) throw new Error(`TrafficLightUI: element #${id} not found`);
    return el;
  }
}

// Re-export phase constants for external use
export { PHASE_RED, PHASE_YELLOW, PHASE_GREEN };
