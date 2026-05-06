import { GROUP_LEGEND, ROAD_TYPE_GROUP } from '../rendering/RoadRenderer';

// ─── Layer definitions ────────────────────────────────────────────────────────

export interface LayerDef {
  id: string;
  label: string;
  gameColor: string;
  osmColor: string;
  defaultOn: boolean;
  isExtra?: boolean; // vehicles / buildings – not road groups
}

const ROAD_LAYERS: LayerDef[] = [
  { id: 'motorway',    ...GROUP_LEGEND['motorway'],    defaultOn: true  },
  { id: 'primary',     ...GROUP_LEGEND['primary'],     defaultOn: true  },
  { id: 'secondary',   ...GROUP_LEGEND['secondary'],   defaultOn: true  },
  { id: 'residential', ...GROUP_LEGEND['residential'], defaultOn: true  },
  { id: 'service',     ...GROUP_LEGEND['service'],     defaultOn: false },
];

// ─── Callbacks ────────────────────────────────────────────────────────────────

export type LayerToggleCb    = (group: string, visible: boolean) => void;
export type OsmModeCb        = (enabled: boolean) => void;
export type VehicleToggleCb  = (visible: boolean) => void;
export type BuildingToggleCb = (visible: boolean) => void;
export type MapBgToggleCb    = (visible: boolean) => void;

// ─── SandboxUI ────────────────────────────────────────────────────────────────

/**
 * Right-side HUD panel for Sandbox mode.
 *
 * Provides:
 *  - View-mode toggle: Game colours vs OSM Carto colours
 *  - Per-group road layer visibility checkboxes
 *  - Vehicles / Buildings quick toggles
 *  - Legend (colour swatches per road type)
 *  - Live stats: vehicle count, FPS
 */
export class SandboxUI {
  private readonly panel: HTMLElement;
  private osmMode = false;

  // DOM refs for live updates
  private readonly checkboxes: Map<string, HTMLInputElement> = new Map();
  private statVehicles!: HTMLElement;
  private statFps!: HTMLElement;
  private readonly swatches: Map<string, HTMLElement> = new Map();

  // Callbacks wired by game.ts
  onLayerToggle:    LayerToggleCb    = () => undefined;
  onOsmModeToggle:  OsmModeCb        = () => undefined;
  onVehicleToggle:  VehicleToggleCb  = () => undefined;
  onBuildingToggle: BuildingToggleCb = () => undefined;
  onMapBgToggle:    MapBgToggleCb    = () => undefined;

  constructor() {
    this.panel = this.buildPanel();
    document.body.appendChild(this.panel);
  }

  // ─── Live update ──────────────────────────────────────────────────────────

  update(vehicleCount: number, fps: number): void {
    this.statVehicles.textContent = String(vehicleCount);
    this.statFps.textContent = fps.toFixed(0);
  }

  destroy(): void {
    this.panel.remove();
  }

  // ─── DOM construction ─────────────────────────────────────────────────────

  private buildPanel(): HTMLElement {
    const panel = document.createElement('div');
    panel.id = 'sandbox-panel';
    panel.className = 'sandbox-panel';

    panel.appendChild(this.buildHeader());
    panel.appendChild(this.buildViewModeSection());
    panel.appendChild(this.buildLayerSection());
    panel.appendChild(this.buildLegendSection());
    panel.appendChild(this.buildStatsSection());

    return panel;
  }

  private buildHeader(): HTMLElement {
    const h = document.createElement('div');
    h.className = 'sbx-header';
    h.innerHTML = `<span class="sbx-badge">SANDBOX</span><span class="sbx-city">Leszno</span>`;
    return h;
  }

  // ── View mode (Game ↔ OSM) ───────────────────────────────────────────────

  private buildViewModeSection(): HTMLElement {
    const sec = this.makeSection('WIDOK MAP');
    const row = document.createElement('div');
    row.className = 'sbx-toggle-row';

    const makeBtn = (label: string, active: boolean, onClick: () => void): HTMLButtonElement => {
      const btn = document.createElement('button');
      btn.className = 'sbx-view-btn' + (active ? ' active' : '');
      btn.textContent = label;
      btn.addEventListener('click', () => {
        row.querySelectorAll('.sbx-view-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        onClick();
      });
      return btn;
    };

    const gameBtn = makeBtn('Gra', true, () => {
      this.osmMode = false;
      this.updateLegendSwatches();
      this.onOsmModeToggle(false);
    });
    const osmBtn = makeBtn('OSM', false, () => {
      this.osmMode = true;
      this.updateLegendSwatches();
      this.onOsmModeToggle(true);
    });

    row.appendChild(gameBtn);
    row.appendChild(osmBtn);
    sec.appendChild(row);
    return sec;
  }

  // ── Layer toggles ────────────────────────────────────────────────────────

  private buildLayerSection(): HTMLElement {
    const sec = this.makeSection('WARSTWY');

    // Road type groups
    for (const layer of ROAD_LAYERS) {
      sec.appendChild(this.buildCheckRow(
        layer.id,
        layer.label,
        layer.gameColor,
        layer.defaultOn,
        (checked) => this.onLayerToggle(layer.id, checked),
      ));
    }

    // Separator
    const sep = document.createElement('div');
    sep.className = 'sbx-sep';
    sec.appendChild(sep);

    // Map background
    sec.appendChild(this.buildCheckRow(
      'mapbg', 'Mapa w tle', '#2244aa', true,
      (checked) => this.onMapBgToggle(checked),
    ));

    // Vehicles
    sec.appendChild(this.buildCheckRow(
      'vehicles', 'Pojazdy', '#4488ff', true,
      (checked) => this.onVehicleToggle(checked),
    ));

    // Buildings (off by default – performance)
    sec.appendChild(this.buildCheckRow(
      'buildings', 'Budynki (wolniej)', '#6688aa', false,
      (checked) => this.onBuildingToggle(checked),
    ));

    return sec;
  }

  private buildCheckRow(
    id: string,
    label: string,
    color: string,
    defaultOn: boolean,
    onChange: (v: boolean) => void,
  ): HTMLElement {
    const row = document.createElement('label');
    row.className = 'sbx-check-row';

    const cb = document.createElement('input');
    cb.type = 'checkbox';
    cb.checked = defaultOn;
    cb.addEventListener('change', () => onChange(cb.checked));
    this.checkboxes.set(id, cb);

    const swatch = document.createElement('span');
    swatch.className = 'sbx-swatch';
    swatch.style.background = color;
    this.swatches.set(id, swatch);

    const lbl = document.createElement('span');
    lbl.textContent = label;

    row.appendChild(cb);
    row.appendChild(swatch);
    row.appendChild(lbl);
    return row;
  }

  // ── Legend ───────────────────────────────────────────────────────────────

  private buildLegendSection(): HTMLElement {
    const sec = this.makeSection('LEGENDA');
    for (const layer of ROAD_LAYERS) {
      const row = document.createElement('div');
      row.className = 'sbx-legend-row';

      const line = document.createElement('span');
      line.className = 'sbx-legend-line';
      line.style.background = layer.gameColor;
      this.swatches.set('legend-' + layer.id, line);

      const lbl = document.createElement('span');
      lbl.className = 'sbx-legend-label';
      lbl.textContent = layer.label;

      row.appendChild(line);
      row.appendChild(lbl);
      sec.appendChild(row);
    }
    return sec;
  }

  // ── Stats ────────────────────────────────────────────────────────────────

  private buildStatsSection(): HTMLElement {
    const sec = this.makeSection('STATYSTYKI IDM');

    const mkRow = (label: string): HTMLElement => {
      const row = document.createElement('div');
      row.className = 'sbx-stat-row';
      const lbl = document.createElement('span');
      lbl.textContent = label;
      const val = document.createElement('span');
      val.className = 'sbx-stat-val';
      val.textContent = '–';
      row.appendChild(lbl);
      row.appendChild(val);
      sec.appendChild(row);
      return val;
    };

    this.statVehicles = mkRow('Pojazdy:');
    this.statFps      = mkRow('FPS:');
    return sec;
  }

  // ── Helpers ──────────────────────────────────────────────────────────────

  private makeSection(title: string): HTMLElement {
    const sec = document.createElement('div');
    sec.className = 'sbx-section';
    const h = document.createElement('div');
    h.className = 'sbx-section-title';
    h.textContent = title;
    sec.appendChild(h);
    return sec;
  }

  private updateLegendSwatches(): void {
    for (const layer of ROAD_LAYERS) {
      const color = this.osmMode ? layer.osmColor : layer.gameColor;
      const swatch    = this.swatches.get(layer.id);
      const legendLine = this.swatches.get('legend-' + layer.id);
      if (swatch)    swatch.style.background = color;
      if (legendLine) legendLine.style.background = color;
    }
    // Vehicles / buildings swatches don't change with OSM mode
  }
}

// Re-export so game.ts doesn't need to import from RoadRenderer just for types
export { ROAD_TYPE_GROUP };
