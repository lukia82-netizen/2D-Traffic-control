import 'maplibre-gl/dist/maplibre-gl.css';
import './styles/main.css';
import { createMap, setupKeyboardNavigation } from './map/MapLibreSetup';
import { PixiOverlay } from './rendering/PixiOverlay';
import { Game } from './game';

async function bootstrap(): Promise<void> {
  // ── 1. MapLibre ─────────────────────────────────────────────────────────────
  // createMap handles online/offline fallback internally — always resolves.
  const map = await createMap('map-container');
  setupKeyboardNavigation(map);

  // ── 2. PixiJS overlay ───────────────────────────────────────────────────────
  const overlay = new PixiOverlay('pixi-container');
  await overlay.init();

  // ── 3. Game ─────────────────────────────────────────────────────────────────
  const game = new Game(map, overlay);
  await game.init();

  // Expose for debugging in dev builds
  if (import.meta.env.DEV) {
    (window as unknown as Record<string, unknown>)['_game'] = game;
    (window as unknown as Record<string, unknown>)['_map'] = map;
  }
}

bootstrap().catch((err: unknown) => {
  console.error('Fatal error during bootstrap:', err);
});
