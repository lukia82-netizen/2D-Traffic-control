import maplibregl from 'maplibre-gl';
import type { LightStateUpdate } from '../bridge/events';
import type { NodeData } from '../bridge/commands';
import {
  setTrafficLightMode,
  setTrafficLightPhase,
  setLightDurations,
  TRAFFIC_LIGHT_PHASE_ADVANCE_PROGRAM,
} from '../bridge/commands';

type LightMode = 'Auto' | 'SemiAuto' | 'Manual' | 'Adaptive';

const MAP_CLICK_ADVANCE_STORAGE = 'tl-map-click-advance';

/** ~obszar skrzyżowania na mapie (zgodnie z szerokością węzła w RoadRenderer). */
const INTERSECTION_HIT_RADIUS_M = 34;

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
  private readonly mapClickAdvanceChk: HTMLInputElement;

  private selectedIntersectionId: number | null = null;
  private readonly lightStates: Map<number, LightStateUpdate> = new Map();
  /** Traffic lights + pedestrian crossings (map hit-test & panel). */
  private signalizedNodes: NodeData[] = [];
  private hiddenNodeIds: Set<number> = new Set();

  /** Click-detect radius in pixels when „map click → phase” is off. */
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
    this.mapClickAdvanceChk = this.require('light-map-click-advance') as HTMLInputElement;
    try {
      const saved = localStorage.getItem(MAP_CLICK_ADVANCE_STORAGE);
      if (saved === '0') this.mapClickAdvanceChk.checked = false;
      if (saved === '1') this.mapClickAdvanceChk.checked = true;
    } catch {
      /* ignore */
    }
    this.mapClickAdvanceChk.addEventListener('change', () => {
      try {
        localStorage.setItem(MAP_CLICK_ADVANCE_STORAGE, this.mapClickAdvanceChk.checked ? '1' : '0');
      } catch {
        /* ignore */
      }
    });
  }

  // ─── Lifecycle ─────────────────────────────────────────────────────────────

  init(nodes: NodeData[]): void {
    this.signalizedNodes = nodes.filter(
      (n) =>
        n.intersectionType === 'traffic_light' ||
        n.intersectionType === 'pedestrian_crossing',
    );

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

  /**
   * Map click: wybór skrzyżowania / opcjonalnie następna faza w obszarze (metry).
   * Zwraca `true`, gdy klik został obsłużony — wtedy nie wybieraj pojazdu pod spodem.
   */
  handleMapClick(
    clickLng: number,
    clickLat: number,
    clickPxX: number,
    clickPxY: number,
    tauriAvailable: boolean,
  ): boolean {
    const useMeters = this.mapClickAdvanceChk.checked;
    const hit = useMeters
      ? this.findClosestSignalizedMeters(clickLng, clickLat)
      : this.findClosestSignalizedPixels(clickPxX, clickPxY);

    if (hit === null) {
      this.hidePanel();
      return false;
    }

    this.selectIntersection(hit.id);

    if (tauriAvailable && useMeters) {
      this.advancePhaseFromMapClick(hit);
    }

    return true;
  }

  /**
   * Update the set of node IDs whose traffic lights are hidden (road groups
   * invisible). Clicking on hidden nodes is silently ignored; if the currently
   * open panel belongs to a now-hidden node, close it.
   */
  setHiddenNodeIds(ids: Set<number>): void {
    this.hiddenNodeIds = ids;
    if (this.selectedIntersectionId !== null && ids.has(this.selectedIntersectionId)) {
      this.hidePanel();
    }
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

  private findClosestSignalizedPixels(px: number, py: number): NodeData | null {
    let best: NodeData | null = null;
    let bestDist = TrafficLightUI.CLICK_RADIUS_PX;

    for (const node of this.signalizedNodes) {
      if (this.hiddenNodeIds.has(node.id)) continue;
      const nodePx = this.map.project([node.lng, node.lat]);
      const dist = Math.hypot(nodePx.x - px, nodePx.y - py);
      if (dist < bestDist) {
        bestDist = dist;
        best = node;
      }
    }

    return best;
  }

  private findClosestSignalizedMeters(clickLng: number, clickLat: number): NodeData | null {
    let best: NodeData | null = null;
    let bestM = INTERSECTION_HIT_RADIUS_M;

    for (const node of this.signalizedNodes) {
      if (this.hiddenNodeIds.has(node.id)) continue;
      const m = planarDistanceM(clickLat, clickLng, clickLat, node.lng, node.lat);
      if (m < bestM) {
        bestM = m;
        best = node;
      }
    }

    return best;
  }

  /** Ustaw tryb Manual i przejdź do następnej fazy (program TL lub R→Y→G na przejściu). */
  private advancePhaseFromMapClick(node: NodeData): void {
    const id = node.id;
    const run = async (): Promise<void> => {
      try {
        await setTrafficLightMode(id, 'Manual');
        if (node.intersectionType === 'pedestrian_crossing') {
          const cur = this.lightStates.get(id)?.phase ?? PHASE_RED;
          const next = (cur + 1) % 3;
          await setTrafficLightPhase(id, next);
        } else {
          await setTrafficLightPhase(id, TRAFFIC_LIGHT_PHASE_ADVANCE_PROGRAM);
        }
      } catch (err) {
        console.error(err);
      }
    };
    void run();
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

function planarDistanceM(
  refLat: number,
  lng1: number,
  lat1: number,
  lng2: number,
  lat2: number,
): number {
  const dLat = (lat2 - lat1) * 111_320;
  const dLng = (lng2 - lng1) * 111_320 * Math.cos((refLat * Math.PI) / 180);
  return Math.hypot(dLat, dLng);
}

// Re-export phase constants for external use
export { PHASE_RED, PHASE_YELLOW, PHASE_GREEN };
