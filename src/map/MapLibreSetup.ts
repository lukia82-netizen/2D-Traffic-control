import maplibregl from 'maplibre-gl';

// ─── City presets ─────────────────────────────────────────────────────────────

/** Kraków – Rynek Główny, ~1 km × 1 km */
export const KRAKOW_CENTER: [number, number] = [19.9368, 50.0614];
export const KRAKOW_BBOX: [number, number, number, number] = [19.930, 50.057, 19.944, 50.066];

/** Leszno – centrum, ~500 m × 500 m (sandbox default) */
export const LESZNO_CENTER: [number, number] = [16.575, 51.845];
export const LESZNO_BBOX: [number, number, number, number] = [16.571, 51.843, 16.579, 51.847];

// Active defaults (sandbox starts with Leszno)
const DEFAULT_CENTER = LESZNO_CENTER;
const DEFAULT_ZOOM = 15;
// legacy alias kept for Rust commands.ts
const DEFAULT_CENTER_KRAKOW = KRAKOW_CENTER;
void DEFAULT_CENTER_KRAKOW;

// OpenFreeMap – free vector tiles, no API key required
// https://openfreemap.org
const ONLINE_STYLE_URL = 'https://tiles.openfreemap.org/styles/liberty';

// Offline fallback style — dark background, no network required
const OFFLINE_STYLE: maplibregl.StyleSpecification = {
  version: 8,
  name: 'Offline',
  sources: {},
  layers: [
    {
      id: 'background',
      type: 'background',
      paint: { 'background-color': '#1a1a2e' },
    },
  ],
};

// ─── Factory ──────────────────────────────────────────────────────────────────

/**
 * Try to load OpenFreeMap tiles.  If the style URL is unreachable (corporate
 * firewall, offline dev mode) fall back to a plain dark canvas.
 *
 * MapLibre provides pan/zoom and coordinate projection; PixiJS renders the
 * simulation overlay (vehicles, congestion, traffic-light sprites, etc.).
 */
export async function createMap(containerId: string): Promise<maplibregl.Map> {
  // First attempt – try online style, timeout after 5 s
  const onlineAvailable = await tryFetchStyle(ONLINE_STYLE_URL, 5000);
  const styleUrl: string | maplibregl.StyleSpecification = onlineAvailable
    ? ONLINE_STYLE_URL
    : OFFLINE_STYLE;

  return new Promise((resolve) => {
    let settled = false;

    const map = new maplibregl.Map({
      container: containerId,
      style: styleUrl,
      center: DEFAULT_CENTER,
      zoom: DEFAULT_ZOOM,
      maxZoom: 19,
      minZoom: 12,
      attributionControl: false,
    });

    const onLoad = (): void => {
      if (settled) return;
      settled = true;
      // Make tram rails visible if the layer exists in the online style
      try {
        if (map.getLayer('railway')) {
          map.setLayoutProperty('railway', 'visibility', 'visible');
        }
      } catch (_) {
        // Layer not present in this style variant — no-op
      }
      resolve(map);
    };

    const onError = (): void => {
      if (settled) return;
      settled = true;
      map.remove();
      const fallback = new maplibregl.Map({
        container: containerId,
        style: OFFLINE_STYLE,
        center: DEFAULT_CENTER,
        zoom: DEFAULT_ZOOM,
        maxZoom: 19,
        minZoom: 12,
        attributionControl: false,
      });
      fallback.once('load', () => resolve(fallback));
    };

    map.once('load', onLoad);
    map.once('error', onError);
  });
}

/**
 * Quick connectivity check: fetch only the HTTP HEAD of the style URL.
 * Returns `true` if the server responds within `timeoutMs`.
 */
async function tryFetchStyle(url: string, timeoutMs: number): Promise<boolean> {
  try {
    const controller = new AbortController();
    const tid = setTimeout(() => controller.abort(), timeoutMs);
    const resp = await fetch(url, { method: 'HEAD', signal: controller.signal });
    clearTimeout(tid);
    return resp.ok;
  } catch {
    return false;
  }
}

// ─── Projection helpers ───────────────────────────────────────────────────────

/**
 * Project a single [lng, lat] coordinate to screen {x, y}.
 */
export function projectPoint(
  map: maplibregl.Map,
  lng: number,
  lat: number,
): { x: number; y: number } {
  const point = map.project([lng, lat]);
  return { x: point.x, y: point.y };
}

/**
 * Batch-project an array of [lng, lat] pairs using MapLibre's optimised
 * internal projection.
 */
export function batchProject(
  map: maplibregl.Map,
  points: Array<[number, number]>,
): Array<{ x: number; y: number }> {
  const result: Array<{ x: number; y: number }> = new Array(points.length);
  for (let i = 0; i < points.length; i++) {
    const p = map.project(points[i]);
    result[i] = { x: p.x, y: p.y };
  }
  return result;
}

/**
 * Returns the underlying projection matrix from the map transform, or null.
 */
export function getProjectionMatrix(map: maplibregl.Map): Float64Array | null {
  const transform = (map as unknown as { transform: Record<string, unknown> }).transform;
  if (!transform) return null;
  const matrix = transform['_projMatrix'] ?? transform['projMatrix'] ?? null;
  if (matrix instanceof Float64Array || matrix instanceof Float32Array) {
    return new Float64Array(matrix);
  }
  return null;
}

// ─── Keyboard navigation ──────────────────────────────────────────────────────

const KEY_PAN_STEP = 100; // pixels

export function setupKeyboardNavigation(map: maplibregl.Map): void {
  const handler = (e: KeyboardEvent): void => {
    const tag = (e.target as HTMLElement).tagName.toLowerCase();
    if (tag === 'input' || tag === 'select' || tag === 'textarea') return;

    switch (e.key) {
      case 'ArrowUp':
      case 'w':
      case 'W':
        e.preventDefault();
        map.panBy([0, -KEY_PAN_STEP], { animate: true, duration: 200 });
        break;
      case 'ArrowDown':
      case 's':
      case 'S':
        e.preventDefault();
        map.panBy([0, KEY_PAN_STEP], { animate: true, duration: 200 });
        break;
      case 'ArrowLeft':
      case 'a':
      case 'A':
        e.preventDefault();
        map.panBy([-KEY_PAN_STEP, 0], { animate: true, duration: 200 });
        break;
      case 'ArrowRight':
      case 'd':
      case 'D':
        e.preventDefault();
        map.panBy([KEY_PAN_STEP, 0], { animate: true, duration: 200 });
        break;
      case '+':
      case '=':
        e.preventDefault();
        map.zoomIn({ animate: true });
        break;
      case '-':
      case '_':
        e.preventDefault();
        map.zoomOut({ animate: true });
        break;
      case ' ':
        e.preventDefault();
        map.flyTo({ center: DEFAULT_CENTER, zoom: DEFAULT_ZOOM });
        break;
      case 'f':
      case 'F':
        e.preventDefault();
        map.fitBounds(
          [
            [KRAKOW_BBOX[0], KRAKOW_BBOX[1]],
            [KRAKOW_BBOX[2], KRAKOW_BBOX[3]],
          ],
          { padding: 40, animate: true },
        );
        break;
    }
  };

  window.addEventListener('keydown', handler);
  (map as unknown as Record<string, unknown>)['_keyboardCleanup'] = () =>
    window.removeEventListener('keydown', handler);
}
