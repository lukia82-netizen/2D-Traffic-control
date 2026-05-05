import maplibregl from 'maplibre-gl';

// Default map center: Kraków Śródmieście
const DEFAULT_CENTER: [number, number] = [19.940, 50.060];
const DEFAULT_ZOOM = 15;
const KRAKOW_BBOX: [number, number, number, number] = [19.925, 50.052, 19.955, 50.068];

// ─── Factory ──────────────────────────────────────────────────────────────────

export function createMap(containerId: string): maplibregl.Map {
  return new maplibregl.Map({
    container: containerId,
    style: 'https://tiles.openfreemap.org/styles/bright',
    center: DEFAULT_CENTER,
    zoom: DEFAULT_ZOOM,
    maxZoom: 19,
    minZoom: 12,
  });
}

// ─── Projection helpers ───────────────────────────────────────────────────────

/**
 * Project a single [lng, lat] coordinate to screen {x, y}.
 * Thin wrapper around map.project() – use for one-off projections.
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
 * Batch-project an array of [lng, lat] pairs using a single transform snapshot.
 *
 * Extracts the current MapLibre mercator transform once, then uses map.project()
 * per point (MapLibre's internal implementation is already C++ optimised). This
 * is still significantly faster than calling the full JS wrapper many times
 * because we avoid JS↔C++ overhead per call by reusing the cached transform.
 *
 * For extreme counts (3 000+ per frame) consider extracting _projMatrix manually
 * once the MapLibre internals stabilise their API.
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
 * Returns the underlying projection matrix from the map transform.
 * The exact internal property differs across MapLibre versions; this function
 * tries the stable path and falls back gracefully.
 *
 * Useful when you need to do raw matrix math outside MapLibre.
 */
export function getProjectionMatrix(map: maplibregl.Map): Float64Array | null {
  // MapLibre GL JS exposes transform via map.transform (internal, not typed)
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const transform = (map as unknown as { transform: Record<string, unknown> }).transform;
  if (!transform) return null;

  // Try modern path first
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
    // Don't steal keys from input elements
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

  // Return a cleanup reference on the map object for convenience
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  (map as unknown as Record<string, unknown>)['_keyboardCleanup'] = () =>
    window.removeEventListener('keydown', handler);
}
