type NotificationType = 'info' | 'warning' | 'error';

const NOTIFICATION_DURATION_MS = 3500;
const NOTIFICATION_FADE_MS = 300;

// ─── UIRenderer ──────────────────────────────────────────────────────────────

/**
 * Manages the HTML/CSS HUD elements:
 *   - Satisfaction progress bar (top-right)
 *   - Vehicle count (bottom-left panel)
 *   - Notification toasts (top-center)
 */
export class UIRenderer {
  private readonly satisfactionFill: HTMLElement;
  private readonly satisfactionValue: HTMLElement;
  private readonly vehicleCountEl: HTMLElement;
  private readonly notificationArea: HTMLElement;

  constructor() {
    this.satisfactionFill = this.require('satisfaction-fill');
    this.satisfactionValue = this.require('satisfaction-value');
    this.vehicleCountEl = this.require('vehicle-count');
    this.notificationArea = this.require('notification-area');
  }

  // ─── Public API ────────────────────────────────────────────────────────────

  /**
   * Update the satisfaction progress bar.
   * @param avgSatisfaction 0–100
   */
  updateSatisfaction(avgSatisfaction: number): void {
    const clamped = Math.max(0, Math.min(100, avgSatisfaction));
    const pct = clamped.toFixed(0);

    this.satisfactionFill.style.width = `${pct}%`;
    this.satisfactionValue.textContent = `${pct}%`;

    // Colour feedback: green > 70, yellow 40–70, red < 40
    if (clamped >= 70) {
      this.satisfactionFill.style.background =
        'linear-gradient(90deg, #00d4ff, #00ff88)';
      this.satisfactionValue.style.color = '#00ff88';
    } else if (clamped >= 40) {
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

  // ─── Private helpers ────────────────────────────────────────────────────────

  private require(id: string): HTMLElement {
    const el = document.getElementById(id);
    if (!el) throw new Error(`UIRenderer: element #${id} not found`);
    return el;
  }
}
