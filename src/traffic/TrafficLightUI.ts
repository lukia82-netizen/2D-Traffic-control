import maplibregl from 'maplibre-gl';
import type { LightStateUpdate } from '../bridge/events';
import type { NodeData } from '../bridge/commands';
import {
  setTrafficLightMode,
  setTrafficLightPhase,
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
 * On map click near an intersection → shows panel with:
 *   - Phase indicator (R/Y/G dots)
 *   - Remaining time
 *   - Mode selector (Auto / SemiAuto / Manual / Adaptive)
 *   - Phase buttons (Manual mode only)
 */
export class TrafficLightUI {
  private readonly map: maplibregl.Map;

  private readonly panel: HTMLElement;
  private readonly title: HTMLElement;
  private readonly timerEl: HTMLElement;
  private readonly modeSelect: HTMLSelectElement;
  private readonly manualControls: HTMLElement;
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
      this.manualControls.classList.toggle('hidden', mode !== 'Manual');
    });

    // Manual phase buttons
    this.manualControls.querySelectorAll<HTMLButtonElement>('.phase-btn').forEach(
      (btn) => {
        btn.addEventListener('click', () => {
          if (this.selectedIntersectionId === null) return;
          const phase = Number(btn.dataset.phase);
          setTrafficLightPhase(this.selectedIntersectionId, phase).catch(
            console.error,
          );
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
    } else {
      this.resetPhaseVisual();
      this.timerEl.textContent = '--s';
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
      }
    }
  }

  // ─── Helpers ───────────────────────────────────────────────────────────────

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
