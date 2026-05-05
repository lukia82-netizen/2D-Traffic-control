import { pauseSimulation, resumeSimulation, setTimeScale } from '../bridge/commands';

const SECONDS_PER_DAY = 86400; // 24 * 60 * 60

// ─── GameClockUI ──────────────────────────────────────────────────────────────

/**
 * Manages the game clock HUD elements:
 *   - #clock-time:         HH:MM display (game time)
 *   - #clock-day-progress: CSS bar showing progress through the 24h day
 *   - #pause-btn:          Toggle pause/resume
 *   - #speed-slider:       Time-scale multiplier
 *   - #speed-value:        Numeric readout next to slider
 */
export class GameClockUI {
  private readonly clockTime: HTMLElement;
  private readonly dayProgress: HTMLElement;
  private readonly pauseBtn: HTMLButtonElement;
  private readonly speedSlider: HTMLInputElement;
  private readonly speedValue: HTMLElement;

  private isPaused = false;
  private currentTimeScale = 60; // 1 real second = 1 game minute

  constructor() {
    this.clockTime = this.require('clock-time');
    this.dayProgress = this.require('clock-day-progress');
    this.pauseBtn = this.require('pause-btn') as HTMLButtonElement;
    this.speedSlider = this.require('speed-slider') as HTMLInputElement;
    this.speedValue = this.require('speed-value');
  }

  // ─── Lifecycle ─────────────────────────────────────────────────────────────

  init(): void {
    // Pause / resume toggle
    this.pauseBtn.addEventListener('click', () => {
      this.isPaused = !this.isPaused;
      this.pauseBtn.textContent = this.isPaused ? '▶' : '⏸';
      this.pauseBtn.classList.toggle('paused', this.isPaused);

      if (this.isPaused) {
        pauseSimulation().catch(console.error);
      } else {
        resumeSimulation().catch(console.error);
      }
    });

    // Speed slider
    this.speedSlider.value = String(this.currentTimeScale);
    this.speedValue.textContent = String(this.currentTimeScale);

    this.speedSlider.addEventListener('input', () => {
      const scale = Number(this.speedSlider.value);
      this.currentTimeScale = scale;
      this.speedValue.textContent = String(scale);
      setTimeScale(scale).catch(console.error);
    });
  }

  // ─── Frame update ──────────────────────────────────────────────────────────

  /**
   * Update the clock display.
   * @param gameTimeS  Elapsed game seconds (e.g. 0 = midnight, 21600 = 06:00)
   */
  updateClock(gameTimeS: number): void {
    const normalised = ((gameTimeS % SECONDS_PER_DAY) + SECONDS_PER_DAY) % SECONDS_PER_DAY;
    const hours = Math.floor(normalised / 3600);
    const minutes = Math.floor((normalised % 3600) / 60);

    this.clockTime.textContent =
      `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}`;

    const progress = (normalised / SECONDS_PER_DAY) * 100;
    this.dayProgress.style.width = `${progress.toFixed(2)}%`;
  }

  // ─── Accessors ─────────────────────────────────────────────────────────────

  get paused(): boolean {
    return this.isPaused;
  }

  get timeScale(): number {
    return this.currentTimeScale;
  }

  // ─── Helpers ───────────────────────────────────────────────────────────────

  private require(id: string): HTMLElement {
    const el = document.getElementById(id);
    if (!el) throw new Error(`GameClockUI: element #${id} not found`);
    return el;
  }
}
