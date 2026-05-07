type NotificationType = 'info' | 'warning' | 'error';

const NOTIFICATION_DURATION_MS = 3500;
const NOTIFICATION_FADE_MS = 300;

// ─── UIRenderer ──────────────────────────────────────────────────────────────

/**
 * Manages the HTML/CSS HUD elements:
 *   - Frustration (Rage Meter) progress bar (top-right)
 *   - Score display (top-right, below frustration bar)
 *   - Vehicle count (bottom-left panel)
 *   - Notification toasts (top-center)
 *   - Game-over overlay
 */
export class UIRenderer {
  private readonly satisfactionFill: HTMLElement;
  private readonly satisfactionValue: HTMLElement;
  private readonly vehicleCountEl: HTMLElement;
  private readonly notificationArea: HTMLElement;
  private readonly scoreValueEl: HTMLElement;
  private readonly idmDebugLine: HTMLElement;
  private readonly idmDebugLine2: HTMLElement;
  private readonly idmDebugLine3: HTMLElement;
  private readonly idmDebugLine4: HTMLElement;

  // Game-over overlay elements
  private readonly gameOverOverlay: HTMLElement;
  private readonly gameOverReason: HTMLElement;
  private readonly finalScore: HTMLElement;
  private readonly finalFrustration: HTMLElement;
  private readonly finalGameTime: HTMLElement;
  private readonly restartBtn: HTMLElement;

  private gameOverVisible = false;

  constructor() {
    this.satisfactionFill = this.require('satisfaction-fill');
    this.satisfactionValue = this.require('satisfaction-value');
    this.vehicleCountEl = this.require('vehicle-count');
    this.notificationArea = this.require('notification-area');
    this.scoreValueEl = this.require('score-value');
    this.idmDebugLine = this.require('idm-debug-line');
    this.idmDebugLine2 = this.require('idm-debug-line2');
    this.idmDebugLine3 = this.require('idm-debug-line3');
    this.idmDebugLine4 = this.require('idm-debug-line4');

    this.gameOverOverlay = this.require('game-over-overlay');
    this.gameOverReason = this.require('game-over-reason');
    this.finalScore = this.require('final-score');
    this.finalFrustration = this.require('final-frustration');
    this.finalGameTime = this.require('final-game-time');
    this.restartBtn = this.require('game-over-restart');

    // Restart reloads the page for simplicity
    this.restartBtn.addEventListener('click', () => {
      window.location.reload();
    });
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  /**
   * Update the Traffic Rage Meter (frustration bar).
   * @param avgFrustration 0 (calm) – 100 (full rage)
   */
  updateSatisfaction(avgFrustration: number): void {
    const clamped = Math.max(0, Math.min(100, avgFrustration));
    const pct = clamped.toFixed(0);

    this.satisfactionFill.style.width = `${pct}%`;
    this.satisfactionValue.textContent = `${pct}%`;

    // Colour: green (calm) → yellow → orange → red (rage)
    if (clamped < 40) {
      this.satisfactionFill.style.background =
        'linear-gradient(90deg, #00d4ff, #00ff88)';
      this.satisfactionValue.style.color = '#00ff88';
    } else if (clamped < 65) {
      this.satisfactionFill.style.background =
        'linear-gradient(90deg, #ffd040, #ff8c00)';
      this.satisfactionValue.style.color = '#ffd040';
    } else {
      this.satisfactionFill.style.background =
        'linear-gradient(90deg, #ff6b35, #ff3b3b)';
      this.satisfactionValue.style.color = '#ff3b3b';
    }
  }

  updateVehicleCount(count: number): void {
    this.vehicleCountEl.textContent = `${count} vehicle${count !== 1 ? 's' : ''}`;
  }

  updateScore(score: number): void {
    this.scoreValueEl.textContent = Math.floor(score).toLocaleString();
  }

  updateIdmDebug(data: {
    vehicleId: number;
    speed: number;
    gap: number;
    deltaV: number;
    distToStopLine: number;
    redBlocking: boolean;
    onCurve: boolean;
    turnT: number;
  }): void {
    this.idmDebugLine.textContent = `vehicle: ${data.vehicleId}`;
    this.idmDebugLine2.textContent =
      `v: ${data.speed.toFixed(1)} m/s | gap: ${data.gap.toFixed(1)} m`;
    this.idmDebugLine3.textContent =
      `dv: ${data.deltaV.toFixed(1)} m/s | stop: ${data.distToStopLine.toFixed(1)} m`;
    this.idmDebugLine4.textContent =
      `red: ${data.redBlocking ? 'YES' : 'NO'} | curve: ${data.onCurve ? 'YES' : 'NO'} | t: ${data.turnT.toFixed(3)}`;
    this.idmDebugLine4.style.color = data.redBlocking ? '#ff6b6b' : (data.onCurve ? '#67e8f9' : '#7fffb0');
  }

  /**
   * Display a temporary toast notification.
   */
  showNotification(message: string, type: NotificationType = 'info'): void {
    const el = document.createElement('div');
    el.className = `notification ${type}`;
    el.textContent = message;
    this.notificationArea.appendChild(el);

    setTimeout(() => {
      el.classList.add('fade-out');
      setTimeout(() => el.remove(), NOTIFICATION_FADE_MS);
    }, NOTIFICATION_DURATION_MS);
  }

  /**
   * Show the game-over overlay.
   */
  showGameOver(
    reason: string,
    frustration: number,
    score: number,
    gameTimeS: number,
  ): void {
    if (this.gameOverVisible) return;
    this.gameOverVisible = true;

    const reasonText =
      reason === 'avg_frustration'
        ? `Average frustration exceeded ${Math.round(frustration)}% for 30 seconds`
        : `${Math.round(frustration)}% of vehicles reached max frustration`;

    this.gameOverReason.textContent = reasonText;
    this.finalScore.textContent = Math.floor(score).toLocaleString();
    this.finalFrustration.textContent = `${Math.round(frustration)}%`;

    const h = Math.floor(gameTimeS / 3600) % 24;
    const m = Math.floor((gameTimeS % 3600) / 60);
    this.finalGameTime.textContent =
      `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}`;

    this.gameOverOverlay.classList.remove('hidden');
  }

  // ─── Private helpers ────────────────────────────────────────────────────────

  private require(id: string): HTMLElement {
    const el = document.getElementById(id);
    if (!el) throw new Error(`UIRenderer: element #${id} not found`);
    return el;
  }
}
