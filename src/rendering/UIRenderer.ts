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
  private readonly telemetrySpeed: HTMLElement;
  private readonly telemetryDesired: HTMLElement;
  private readonly telemetryAccel: HTMLElement;
  private readonly telemetryDistance: HTMLElement;
  private readonly telemetryTurnT: HTMLElement;
  private readonly telemetryLaneRoute: HTMLElement;
  private readonly telemetryBrake: HTMLElement;
  private readonly telemetryHint: HTMLElement;

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

    this.telemetrySpeed = this.require('telemetry-speed');
    this.telemetryDesired = this.require('telemetry-desired');
    this.telemetryAccel = this.require('telemetry-accel');
    this.telemetryDistance = this.require('telemetry-distance');
    this.telemetryTurnT = this.require('telemetry-turn-t');
    this.telemetryLaneRoute = this.require('telemetry-lane-route');
    this.telemetryBrake = this.require('telemetry-brake');
    this.telemetryHint = this.require('vehicle-telemetry-hint');

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
    desiredSpeed: number;
    acceleration: number;
    distanceToLeader: number;
    leaderVehicleId: number | null;
    conflictReserverId: number | null;
    distToStopLine: number;
    redBlocking: boolean;
    onCurve: boolean;
    turnT: number;
    shapeLengthM: number;
    shapeWidthM: number;
    shapeRadiusM: number;
    threatKind: string;
    threatPoint: [number, number] | null;
    threatLineStyle: string;
    brakeReason?: string | null;
    laneRouteIds?: number[];
  }): void {
    this.idmDebugLine.textContent = `vehicle: ${data.vehicleId}`;
    this.idmDebugLine2.textContent =
      `v: ${data.speed.toFixed(1)} m/s | gap: ${data.gap.toFixed(1)} m | v₀: ${data.desiredSpeed.toFixed(1)}`;
    this.idmDebugLine3.textContent =
      `accel: ${data.acceleration.toFixed(2)} m/s² | dv: ${data.deltaV.toFixed(1)} | stop: ${data.distToStopLine.toFixed(1)} m`;
    const telem = `dist leader: ${data.distanceToLeader.toFixed(1)} m | style: ${data.threatLineStyle}`;
    const brake = data.brakeReason ? ` | ⚠ ${data.brakeReason}` : '';
    const lanes =
      data.laneRouteIds && data.laneRouteIds.length > 0
        ? ` | lanes: ${data.laneRouteIds.slice(0, 8).join('→')}${data.laneRouteIds.length > 8 ? '…' : ''}`
        : '';
    this.idmDebugLine4.textContent =
      `red: ${data.redBlocking ? 'YES' : 'NO'} | curve: ${data.onCurve ? 'YES' : 'NO'} | t: ${data.turnT.toFixed(3)} | L/W/R: ${data.shapeLengthM.toFixed(1)}/${data.shapeWidthM.toFixed(1)}/${data.shapeRadiusM.toFixed(1)} m | threat: ${data.threatKind}${data.leaderVehicleId != null ? ` | L#${data.leaderVehicleId}` : ''}${data.conflictReserverId != null ? ` | reserver:${data.conflictReserverId}` : ''} | ${telem}${lanes}${brake}`;
    this.idmDebugLine4.style.color = data.redBlocking ? '#ff6b6b' : (data.onCurve ? '#67e8f9' : '#7fffb0');
  }

  updateVehicleTelemetrySelected(data: {
    vehicleId: number | null;
    speed: number;
    desiredSpeed: number;
    acceleration: number;
    distanceToLeader: number;
    turnT?: number;
    onCurve?: boolean;
    laneRouteIds?: number[];
    brakeReason?: string | null;
  }): void {
    if (data.vehicleId === null) {
      this.telemetryHint.textContent =
        'Klik na mapie w pojazd (Pixi ma pointer-events:none — trafiasz w MapLibre).';
      this.telemetrySpeed.textContent = 'Current Speed: -- m/s';
      this.telemetryDesired.textContent = 'Desired Speed: -- m/s';
      this.telemetryAccel.textContent = 'Acceleration: -- m/s²';
      this.telemetryDistance.textContent = 'Distance to Leader: -- m';
      this.telemetryTurnT.textContent = 'Bezier t (connector): --';
      this.telemetryLaneRoute.textContent = 'Lane route ids: --';
      this.telemetryBrake.textContent = 'Brake: —';
      return;
    }
    this.telemetryHint.textContent = `Pojazd #${data.vehicleId}`;
    this.telemetrySpeed.textContent = `Current Speed: ${data.speed.toFixed(2)} m/s`;
    this.telemetryDesired.textContent = `Desired Speed: ${data.desiredSpeed.toFixed(2)} m/s`;
    this.telemetryAccel.textContent = `Acceleration: ${data.acceleration.toFixed(2)} m/s²`;
    this.telemetryDistance.textContent =
      `Distance to Leader: ${data.distanceToLeader.toFixed(2)} m (along path / IDM gap)`;
    const tt = data.turnT;
    const oc = data.onCurve;
    this.telemetryTurnT.textContent =
      tt !== undefined
        ? `Bezier t (connector): ${tt.toFixed(3)}${oc ? ' (on curve)' : ''}`
        : 'Bezier t (connector): --';
    const ids = data.laneRouteIds;
    this.telemetryLaneRoute.textContent =
      ids && ids.length > 0
        ? `Lane route ids: ${ids.join(' → ')}`
        : 'Lane route ids: (empty — fallback path)';
    this.telemetryBrake.textContent =
      data.brakeReason != null && data.brakeReason !== ''
        ? `Brake: ${data.brakeReason}`
        : 'Brake: —';
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
