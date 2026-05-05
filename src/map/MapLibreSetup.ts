import maplibregl from 'maplibre-gl';

// Default map center: Kraków Rynek Główny
const DEFAULT_CENTER: [number, number] = [19.9368, 50.0614];
const DEFAULT_ZOOM = 16;
const KRAKOW_BBOX: [number, number, number, number] = [19.930, 50.057, 19.944, 50.066];

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

// Online tile style URL
const ONLINE_STYLE_URL = 'https://tiles.openfreemap.org/styles/bright';
// Timeout before giving up on online tiles
const STYLE_LOAD_TIMEOUT_MS = 6000;

// ─── Factory ──────────────────────────────────────────────────────────────────

/**
 * Creates a MapLibre map in "sketch" mode — pure offline dark canvas.
 * Roads and buildings are drawn by PixiJS from OSM data, so we don't need
 * map tiles at all. MapLibre is kept only for its camera (pan/zoom) and
 * coordinate projection system.
 */
export async function createMap(containerId: string): Promise<maplibregl.Map> {
  return new Promise((resolve) => {
    const map = new maplibregl.Map({
      container: containerId,
      style: OFFLINE_STYLE,   // always offline – PixiJS draws the map
      center: DEFAULT_CENTER,
      zoom: DEFAULT_ZOOM,
      maxZoom: 19,
      minZoom: 12,
      attributionControl: false,
    });
    map.once('load', () => resolve(map));
  });
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
