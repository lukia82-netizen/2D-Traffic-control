import 'maplibre-gl/dist/maplibre-gl.css';
import './styles/main.css';
import { createMap, setupKeyboardNavigation } from './map/MapLibreSetup';
import { PixiOverlay } from './rendering/PixiOverlay';
import { Game } from './game';

async function bootstrap(): Promise<void> {
  // ── 1. MapLibre ─────────────────────────────────────────────────────────────
  const map = createMap('map-container');

  // Wait for the map style to fully load before attaching anything
  await new Promise<void>((resolve) => {
    if (map.isStyleLoaded()) {
      resolve();
    } else {
      map.once('load', () => resolve());
    }
  });

  setupKeyboardNavigation(map);

  // ── 2. PixiJS overlay ───────────────────────────────────────────────────────
  const overlay = new PixiOverlay('pixi-container');
  await overlay.init();

  // ── 3. Game ─────────────────────────────────────────────────────────────────
  const game = new Game(map, overlay);
  await game.init();

  // Expose for debugging in dev builds
  if (import.meta.env.DEV) {
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    (window as unknown as Record<string, unknown>)['_game'] = game;
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    (window as unknown as Record<string, unknown>)['_map'] = map;
  }
}

bootstrap().catch((err: unknown) => {
  console.error('Fatal error during bootstrap:', err);
});
