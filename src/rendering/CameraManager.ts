import maplibregl from 'maplibre-gl';

// ─── Types ────────────────────────────────────────────────────────────────────

export type RenderMode = 'dot' | 'sprite';

export interface CarVisuals {
  /** How to draw vehicles at current zoom */
  mode: RenderMode;
  /**
   * Multiplier applied to base VEHICLE_DIMS (e.g. 2.0 at zoom 16 gives
   * a car 12×24 px instead of 6×12 px).  Only meaningful when mode='sprite'.
   */
  spriteScale: number;
  /** Dot radius in pixels.  Only meaningful when mode='dot'. */
  dotRadius: number;
}

// ─── CameraManager ────────────────────────────────────────────────────────────

/**
 * Translates MapLibre zoom into PixiJS visual parameters so the simulation
 * always looks "game-like" regardless of how much the player zooms.
 *
 * Zoom curve design (Cities: Skylines / GTA1 style):
 *
 *  MapLibre zoom │ Render mode │ Visual
 *  ──────────────┼─────────────┼────────────────────────────────────────────
 *    < 12        │ dot         │ 2 px dot  (city overview, 3000+ cars fast)
 *   12 – 13      │ dot         │ 2 – 4 px dots
 *   13 – 14      │ transition  │ small sprites fading in
 *   14 – 16      │ sprite      │ scale 1.0 – 2.0  (normal street view)
 *   16 – 17      │ sprite      │ scale 2.0 – 3.0  (zoomed in, GTA-like)
 *    > 17        │ sprite      │ scale capped at 3.5 (maximum detail)
 *
 * Road widths follow the same curve so lanes feel proportional to cars.
 */
export class CameraManager {
  private readonly map: maplibregl.Map;

  // Zoom breakpoints
  static readonly ZOOM_DOT_MAX = 13;        // below → pure dot mode
  static readonly ZOOM_SPRITE_FULL = 14;    // above → pure sprite mode
  static readonly ZOOM_REF = 16;            // reference zoom for scale = 2.0
  static readonly SPRITE_SCALE_MIN = 0.7;   // scale at ZOOM_SPRITE_FULL
  static readonly SPRITE_SCALE_REF = 2.0;   // scale at ZOOM_REF (car = 12×24 px)
  static readonly SPRITE_SCALE_MAX = 3.5;   // cap so sprites don't get enormous

  constructor(map: maplibregl.Map) {
    this.map = map;
  }

  // ─── Accessors ─────────────────────────────────────────────────────────────

  get zoom(): number {
    return this.map.getZoom();
  }

  // ─── Car visuals ───────────────────────────────────────────────────────────

  /**
   * Returns the rendering parameters for vehicles at the current zoom level.
   * Call this once per frame in VehicleRenderer.update().
   */
  getCarVisuals(): CarVisuals {
    const z = this.zoom;

    // ── Dot zone ──────────────────────────────────────────────────────────────
    if (z <= CameraManager.ZOOM_DOT_MAX) {
      // Dots grow from 2 px (zoom 10) to 4 px (zoom 13)
      const dotRadius = 2 + Math.max(0, z - 10) * (2 / 3);
      return { mode: 'dot', spriteScale: 0, dotRadius };
    }

    // ── Transition zone ───────────────────────────────────────────────────────
    if (z < CameraManager.ZOOM_SPRITE_FULL) {
      const t = (z - CameraManager.ZOOM_DOT_MAX) /
                (CameraManager.ZOOM_SPRITE_FULL - CameraManager.ZOOM_DOT_MAX);
      // Blend: at t=0 still dot, at t=1 switch to sprite mode
      if (t < 0.5) {
        const dotRadius = 4 - t * 2; // shrink dot
        return { mode: 'dot', spriteScale: 0, dotRadius };
      }
      const spriteScale = CameraManager.lerp(
        CameraManager.SPRITE_SCALE_MIN * 0.5,
        CameraManager.SPRITE_SCALE_MIN,
        (t - 0.5) * 2,
      );
      return { mode: 'sprite', spriteScale, dotRadius: 0 };
    }

    // ── Sprite zone ───────────────────────────────────────────────────────────
    // Exponential curve anchored at ZOOM_REF=16 → scale=2.0
    const rawScale = CameraManager.SPRITE_SCALE_REF *
      Math.pow(2, z - CameraManager.ZOOM_REF);
    const spriteScale = Math.min(CameraManager.SPRITE_SCALE_MAX,
      Math.max(CameraManager.SPRITE_SCALE_MIN, rawScale));
    return { mode: 'sprite', spriteScale, dotRadius: 0 };
  }

  // ─── Road-overlay widths ───────────────────────────────────────────────────

  /**
   * Returns pixel width for road infrastructure overlays (bridge shadow,
   * tunnel line) at the current zoom level.
   *
   * @param lanes   Number of lanes on the road
   */
  getRoadOverlayWidth(lanes: number): number {
    const z = this.zoom;
    const basePx = lanes * 3; // 3 px per lane at reference zoom

    if (z <= 12) return Math.max(1, basePx * 0.25);

    if (z <= CameraManager.ZOOM_REF) {
      const t = (z - 12) / (CameraManager.ZOOM_REF - 12);
      return Math.max(1, basePx * CameraManager.lerp(0.25, 1.0, t));
    }

    return Math.min(basePx * 3, basePx * Math.pow(2, z - CameraManager.ZOOM_REF));
  }

  /**
   * Returns the pixel offset from road centerline to the center of the right
   * lane at the current zoom.  Matches the halfPx=9 residential road standard
   * used in RoadRenderer so cars sit visually inside their lane.
   */
  getLaneOffset(): number {
    // halfPx=9 is the residential reference; half of that is one lane center
    return Math.max(4, 9 * Math.pow(2, this.zoom - 16));
  }

  /**
   * Returns pixel spacing between oneway arrows (larger gap at high zoom).
   */
  getArrowSpacing(): number {
    const z = this.zoom;
    if (z < 14) return 120;
    if (z < 16) return 100;
    return 80;
  }

  /**
   * Returns half-size for arrowhead glyphs in pixels.
   */
  getArrowSize(): number {
    return Math.max(4, this.getRoadOverlayWidth(2) * 0.8);
  }

  // ─── Utility ───────────────────────────────────────────────────────────────

  private static lerp(a: number, b: number, t: number): number {
    return a + (b - a) * Math.max(0, Math.min(1, t));
  }
}
