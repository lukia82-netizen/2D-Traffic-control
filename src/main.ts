import 'maplibre-gl/dist/maplibre-gl.css';
import './styles/main.css';
import { createMap, setupKeyboardNavigation } from './map/MapLibreSetup';
import { PixiOverlay } from './rendering/PixiOverlay';
import { Game } from './game';

type AppMode = 'game' | 'editor' | 'sandbox';

function setupAppModeToggle(mode: AppMode): void {
  const btn = document.getElementById('app-mode-toggle') as HTMLButtonElement | null;
  if (!btn) return;
  const label = mode === 'editor' ? 'Tryb: Edytor' : mode === 'sandbox' ? 'Tryb: Sandbox' : 'Tryb: Gra';
  btn.textContent = label;
  btn.title = 'Przełącz tryb: Gra -> Edytor -> Sandbox';
  btn.addEventListener('click', () => {
    const cycle: AppMode[] = ['game', 'editor', 'sandbox'];
    const next = cycle[(cycle.indexOf(mode) + 1) % cycle.length];
    const url = new URL(window.location.href);
    if (next === 'sandbox') {
      url.searchParams.delete('app');
    } else {
      url.searchParams.set('app', next);
    }
    window.location.href = `${url.pathname}${url.search}`;
  });
}

async function bootstrap(): Promise<void> {
  const params = new URLSearchParams(window.location.search);
  const appModeRaw = (params.get('app') ?? 'sandbox').toLowerCase();
  const appModeFromQuery: AppMode =
    appModeRaw === 'editor' || appModeRaw === 'game' || appModeRaw === 'sandbox'
      ? appModeRaw
      : 'sandbox';
  const hasModeInQuery = params.has('app');
  const editorFromEnv = import.meta.env.VITE_EDITOR_ONLY === '1';
  const appMode: AppMode =
    hasModeInQuery ? appModeFromQuery : (editorFromEnv ? 'editor' : appModeFromQuery);
  const editorOnly = appMode === 'editor';
  document.body.classList.toggle('editor-only-app', editorOnly);
  setupAppModeToggle(appMode);

  // ── 1. MapLibre ─────────────────────────────────────────────────────────────
  // createMap handles online/offline fallback internally — always resolves.
  const map = await createMap('map-container');
  setupKeyboardNavigation(map);

  // ── 2. PixiJS overlay ───────────────────────────────────────────────────────
  const overlay = new PixiOverlay('pixi-container');
  await overlay.init();

  // ── 3. Game ─────────────────────────────────────────────────────────────────
  const game = new Game(map, overlay, { editorOnly, appMode });
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
