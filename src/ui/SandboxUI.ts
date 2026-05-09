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
export type TurnConnectorsToggleCb = (visible: boolean) => void;
export type TurnConnectorsActiveOnlyToggleCb = (activeOnly: boolean) => void;
export type DebugVisualizationToggleCb = (enabled: boolean) => void;
export type LaneLinesToggleCb = (enabled: boolean) => void;
export type TrafficDebugToggleCb = (enabled: boolean) => void;
/** center = [lng, lat], sizeM = metres per side */
export type ReloadMapCb        = (center: [number, number], sizeM: number) => void;
export type MaxVehiclesCb      = (count: number) => void;
export type LaneWidthCb        = (laneWidthM: number) => void;
export type BboxPickCb         = () => void;
/**
 * Called when the user changes the map/grid mode.
 * null  = use real OSM map
 * string = sandbox grid type: 'mixed' | 'one_lane' | 'two_lane' | 'three_lane'
 */
export type MapModeCb = (forceSandbox: string | null) => void;

/** Grid type label, value, and colour hint. */
const GRID_TYPES = [
  { label: 'Skrzyżowanie', value: 'single_intersection', hint: '2 drogi bez świateł' },
  { label: 'Jedna droga',  value: 'single_road',  hint: '1 droga IDM test' },
  { label: 'Mieszana',     value: 'mixed',         hint: '1/2/3 pasy 3x3' },
  { label: '1 pas',        value: 'one_lane',      hint: 'tertiary 50' },
  { label: '2 pasy',       value: 'two_lane',      hint: 'secondary 70' },
  { label: '3 pasy',       value: 'three_lane',    hint: 'primary 70' },
] as const;
const UI_STORAGE_PREFIX = 'sandbox-ui:';

// ─── City & size presets ─────────────────────────────────────────────────────

export interface CityPreset { name: string; center: [number, number]; }

export const CITY_PRESETS: CityPreset[] = [
  { name: 'Leszno',  center: [16.575, 51.845] },
  { name: 'Kraków',  center: [19.937, 50.061] },
];

export const AREA_SIZES = [500, 1000, 2000] as const;
export type AreaSize = typeof AREA_SIZES[number];

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

  // Area selector state
  private selectedCity: CityPreset = CITY_PRESETS[0];
  private selectedSize: AreaSize   = 500;
  private areaStatusEl!: HTMLElement;
  private cityBtns: Map<string, HTMLButtonElement> = new Map();
  private sizeBtns: Map<number, HTMLButtonElement> = new Map();
  private reloadBtn!: HTMLButtonElement;

  // DOM refs for live updates
  private readonly checkboxes: Map<string, HTMLInputElement> = new Map();
  private statVehicles!: HTMLElement;
  private statFps!: HTMLElement;
  private readonly swatches: Map<string, HTMLElement> = new Map();

  // Grid / map-mode state
  private mapMode: 'osm' | 'sandbox' = 'sandbox';
  private selectedGridType = 'single_intersection';
  private gridTypeBtns: Map<string, HTMLButtonElement> = new Map();
  private gridSubsection!: HTMLElement;
  private mapModeBtns: Map<string, HTMLButtonElement> = new Map();
  private gameViewBtn: HTMLButtonElement | null = null;
  private osmViewBtn: HTMLButtonElement | null = null;
  private readonly checkboxDefaults: Map<string, boolean> = new Map();

  // Callbacks wired by game.ts
  onLayerToggle:       LayerToggleCb    = () => undefined;
  onOsmModeToggle:     OsmModeCb        = () => undefined;
  onVehicleToggle:     VehicleToggleCb  = () => undefined;
  onBuildingToggle:    BuildingToggleCb = () => undefined;
  onMapBgToggle:       MapBgToggleCb    = () => undefined;
  onTurnConnectorsToggle: TurnConnectorsToggleCb = () => undefined;
  onTurnConnectorsActiveOnlyToggle: TurnConnectorsActiveOnlyToggleCb = () => undefined;
  onDebugVisualizationToggle: DebugVisualizationToggleCb = () => undefined;
  onLaneLinesToggle: LaneLinesToggleCb = () => undefined;
  onTrafficDebugToggle: TrafficDebugToggleCb = () => undefined;
  onReloadMap:         ReloadMapCb      = () => undefined;
  onMaxVehiclesChange: MaxVehiclesCb    = () => undefined;
  onLaneWidthChange:   LaneWidthCb      = () => undefined;
  onBboxPickRequest:   BboxPickCb       = () => undefined;
  /** Fires when the user changes the map/grid mode. Triggers on selection (not on reload). */
  onMapModeChange:     MapModeCb        = () => undefined;

  constructor() {
    this.restoreModeState();
    this.panel = this.buildPanel();
    document.body.appendChild(this.panel);
  }

  // ─── Live update ──────────────────────────────────────────────────────────

  update(vehicleCount: number, fps: number): void {
    this.statVehicles.textContent = String(vehicleCount);
    this.statFps.textContent = fps.toFixed(0);
  }

  setChecked(id: string, checked: boolean): void {
    const cb = this.checkboxes.get(id);
    if (!cb) return;
    cb.checked = checked;
    this.persistCheckbox(id, checked);
  }

  /** Applies restored UI state after callbacks are wired by game.ts. */
  applyPersistedSettings(): void {
    this.onOsmModeToggle(this.osmMode);
    this.onMapModeChange(this.currentForceSandbox);
    for (const [id, cb] of this.checkboxes) {
      this.persistCheckbox(id, cb.checked);
      cb.dispatchEvent(new Event('change'));
    }
  }

  /** Call after a map reload completes to update status label and reset button. */
  setLoadingDone(cityName: string, sizeM: number): void {
    this.areaStatusEl.textContent = `${cityName} · ${sizeM >= 1000 ? sizeM / 1000 + ' km' : sizeM + ' m'}`;
    this.reloadBtn.disabled = false;
    this.reloadBtn.textContent = 'Przeładuj';
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
    panel.appendChild(this.buildMapModeSection());   // ← NEW: OSM vs Sandbox grid
    panel.appendChild(this.buildAreaSection());
    panel.appendChild(this.buildLaneWidthSection());
    panel.appendChild(this.buildMaxVehiclesSection());
    panel.appendChild(this.buildViewModeSection());
    panel.appendChild(this.buildLayerSection());
    panel.appendChild(this.buildLegendSection());
    panel.appendChild(this.buildStatsSection());

    return panel;
  }

  private buildHeader(): HTMLElement {
    const h = document.createElement('div');
    h.className = 'sbx-header';
    const badge = document.createElement('span');
    badge.className = 'sbx-badge';
    badge.textContent = 'SANDBOX';
    const resetBtn = document.createElement('button');
    resetBtn.className = 'sbx-reset-btn';
    resetBtn.textContent = 'Reset';
    resetBtn.title = 'Przywróć domyślne ustawienia panelu Sandbox';
    resetBtn.addEventListener('click', () => this.resetPersistedSettings());
    h.appendChild(badge);
    h.appendChild(resetBtn);
    return h;
  }

  // ── Map / grid mode selector ─────────────────────────────────────────────

  /** Returns the current forceSandbox value (null = OSM, string = grid type). */
  get currentForceSandbox(): string | null {
    return this.mapMode === 'sandbox' ? this.selectedGridType : null;
  }

  private buildMapModeSection(): HTMLElement {
    const sec = this.makeSection('TRYB MAPY');

    // Row 1: OSM Map vs Sandbox Grid toggle
    const modeRow = document.createElement('div');
    modeRow.className = 'sbx-toggle-row';

    const makeModeBtn = (id: string, label: string): HTMLButtonElement => {
      const btn = document.createElement('button');
      btn.className = 'sbx-view-btn' + (id === this.mapMode ? ' active' : '');
      btn.textContent = label;
      btn.addEventListener('click', () => {
        this.mapMode = id as 'osm' | 'sandbox';
        this.persistModeState();
        this.mapModeBtns.forEach((b, k) => b.classList.toggle('active', k === id));
        this.gridSubsection.style.display = id === 'sandbox' ? '' : 'none';
        this.onMapModeChange(this.currentForceSandbox);
      });
      this.mapModeBtns.set(id, btn);
      return btn;
    };

    modeRow.appendChild(makeModeBtn('osm',     'OSM Mapa'));
    modeRow.appendChild(makeModeBtn('sandbox', 'Siatka Demo'));
    sec.appendChild(modeRow);

    // Row 2: Grid type buttons (visible only in sandbox mode)
    this.gridSubsection = document.createElement('div');
    this.gridSubsection.className = 'sbx-toggle-row sbx-grid-row';
    this.gridSubsection.style.display = this.mapMode === 'sandbox' ? '' : 'none';

    for (const gt of GRID_TYPES) {
      const btn = document.createElement('button');
      btn.className = 'sbx-grid-btn' + (gt.value === this.selectedGridType ? ' active' : '');
      btn.textContent = gt.label;
      btn.title = gt.hint;
      btn.addEventListener('click', () => {
        this.selectedGridType = gt.value;
        this.persistModeState();
        this.gridTypeBtns.forEach((b, k) => b.classList.toggle('active', k === gt.value));
        this.onMapModeChange(this.currentForceSandbox);
      });
      this.gridTypeBtns.set(gt.value, btn);
      this.gridSubsection.appendChild(btn);
    }

    sec.appendChild(this.gridSubsection);
    return sec;
  }

  // ── Area / city selector ─────────────────────────────────────────────────

  private buildAreaSection(): HTMLElement {
    const sec = this.makeSection('OBSZAR MAPY');

    // City buttons
    const cityRow = document.createElement('div');
    cityRow.className = 'sbx-toggle-row';
    for (const city of CITY_PRESETS) {
      const btn = document.createElement('button');
      btn.className = 'sbx-view-btn' + (city.name === this.selectedCity.name ? ' active' : '');
      btn.textContent = city.name;
      btn.addEventListener('click', () => {
        this.selectedCity = city;
        this.cityBtns.forEach((b, n) => b.classList.toggle('active', n === city.name));
      });
      this.cityBtns.set(city.name, btn);
      cityRow.appendChild(btn);
    }
    sec.appendChild(cityRow);

    // Size buttons
    const sizeRow = document.createElement('div');
    sizeRow.className = 'sbx-toggle-row';
    for (const sz of AREA_SIZES) {
      const btn = document.createElement('button');
      btn.className = 'sbx-view-btn' + (sz === this.selectedSize ? ' active' : '');
      btn.textContent = sz >= 1000 ? `${sz / 1000} km` : `${sz} m`;
      btn.addEventListener('click', () => {
        this.selectedSize = sz as AreaSize;
        this.sizeBtns.forEach((b, s) => b.classList.toggle('active', s === sz));
      });
      this.sizeBtns.set(sz, btn);
      sizeRow.appendChild(btn);
    }
    sec.appendChild(sizeRow);

    // Status label
    this.areaStatusEl = document.createElement('div');
    this.areaStatusEl.className = 'sbx-area-status';
    this.areaStatusEl.textContent = `${this.selectedCity.name} · ${this.selectedSize} m`;
    sec.appendChild(this.areaStatusEl);

    // Reload button
    this.reloadBtn = document.createElement('button');
    this.reloadBtn.className = 'sbx-reload-btn';
    this.reloadBtn.textContent = 'Przeładuj';
    this.reloadBtn.addEventListener('click', () => {
      this.reloadBtn.disabled = true;
      this.reloadBtn.textContent = 'Ładowanie…';
      this.onReloadMap(this.selectedCity.center, this.selectedSize);
    });
    sec.appendChild(this.reloadBtn);

    const bboxBtn = document.createElement('button');
    bboxBtn.className = 'sbx-reload-btn sbx-bbox-btn';
    bboxBtn.textContent = 'Wybierz obszar (BBOX)';
    bboxBtn.addEventListener('click', () => this.onBboxPickRequest());
    sec.appendChild(bboxBtn);

    return sec;
  }

  // ── Max vehicles ─────────────────────────────────────────────────────────

  private buildLaneWidthSection(): HTMLElement {
    const sec = this.makeSection('SZEROKOŚĆ PASA');
    const row = document.createElement('div');
    row.className = 'sbx-input-row';

    const input = document.createElement('input');
    input.type = 'range';
    input.min = '2.5';
    input.max = '5.0';
    input.step = '0.1';
    input.value = '3.5';
    input.style.width = '100%';

    const value = document.createElement('div');
    value.className = 'sbx-area-status';
    value.textContent = '3.5 m  (auto 1.8m + 1.7m luzu)';

    const describeWidth = (w: number): string => {
      // clearance between two opposing cars (width 1.8 m each)
      const gap = (w - 1.8).toFixed(1);
      return `${w.toFixed(1)} m  (luzu ${gap} m)`;
    };

    const apply = () => {
      const laneWidthM = Math.max(2.5, Math.min(5.0, Number(input.value) || 3.5));
      input.value = laneWidthM.toFixed(1);
      value.textContent = describeWidth(laneWidthM);
      this.onLaneWidthChange(laneWidthM);
    };

    input.addEventListener('input', () => {
      const laneWidthM = Math.max(2.5, Math.min(5.0, Number(input.value) || 3.5));
      value.textContent = describeWidth(laneWidthM);
    });
    input.addEventListener('change', apply);

    row.appendChild(input);
    sec.appendChild(row);
    sec.appendChild(value);
    return sec;
  }

  // ── Max vehicles ─────────────────────────────────────────────────────────

  private buildMaxVehiclesSection(): HTMLElement {
    const sec = this.makeSection('MAKS. POJAZDÓW');

    const row = document.createElement('div');
    row.className = 'sbx-input-row';

    const input = document.createElement('input');
    input.type = 'number';
    input.className = 'sbx-num-input';
    input.min = '5';
    input.max = '1000';
    input.step = '5';
    input.value = '20';

    const applyBtn = document.createElement('button');
    applyBtn.className = 'sbx-apply-btn';
    applyBtn.textContent = 'Ustaw';

    const apply = () => {
      const val = Math.max(5, Math.min(1000, parseInt(input.value, 10) || 20));
      input.value = String(val);
      this.onMaxVehiclesChange(val);
    };

    applyBtn.addEventListener('click', apply);
    input.addEventListener('keydown', (e) => { if (e.key === 'Enter') apply(); });

    row.appendChild(input);
    row.appendChild(applyBtn);
    sec.appendChild(row);
    return sec;
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

    const gameBtn = makeBtn('Gra', !this.osmMode, () => {
      this.osmMode = false;
      this.persistModeState();
      this.updateLegendSwatches();
      this.onOsmModeToggle(false);
    });
    const osmBtn = makeBtn('OSM', this.osmMode, () => {
      this.osmMode = true;
      this.persistModeState();
      this.updateLegendSwatches();
      this.onOsmModeToggle(true);
    });
    this.gameViewBtn = gameBtn;
    this.osmViewBtn = osmBtn;

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

    // Debug turn connectors
    sec.appendChild(this.buildCheckRow(
      'turn-connectors', 'Debug: łuki skrętu', '#22d3ee', false,
      (checked) => this.onTurnConnectorsToggle(checked),
    ));
    sec.appendChild(this.buildCheckRow(
      'turn-connectors-active-only', 'Tylko aktywne łuki', '#f59e0b', false,
      (checked) => this.onTurnConnectorsActiveOnlyToggle(checked),
    ));
    sec.appendChild(this.buildCheckRow(
      'debug-visualization', 'Tryb debug skrzyżowań (klawisz D)', '#f43f5e', false,
      (checked) => this.onDebugVisualizationToggle(checked),
    ));
    sec.appendChild(this.buildCheckRow(
      'lane-lines', 'Linie pasów (debug)', '#facc15', true,
      (checked) => this.onLaneLinesToggle(checked),
    ));
    sec.appendChild(this.buildCheckRow(
      'traffic-debug', 'Tryb Debugowania Ruchu', '#a855f7', false,
      (checked) => this.onTrafficDebugToggle(checked),
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
    cb.checked = this.restoreCheckbox(id, defaultOn);
    cb.addEventListener('change', () => {
      this.persistCheckbox(id, cb.checked);
      onChange(cb.checked);
    });
    this.checkboxes.set(id, cb);
    this.checkboxDefaults.set(id, defaultOn);

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

  private persistCheckbox(id: string, value: boolean): void {
    localStorage.setItem(`${UI_STORAGE_PREFIX}cb:${id}`, value ? '1' : '0');
  }

  private restoreCheckbox(id: string, fallback: boolean): boolean {
    const raw = localStorage.getItem(`${UI_STORAGE_PREFIX}cb:${id}`);
    if (raw === '1') return true;
    if (raw === '0') return false;
    return fallback;
  }

  private restoreModeState(): void {
    const rawMapMode = localStorage.getItem(`${UI_STORAGE_PREFIX}mapMode`);
    this.mapMode = rawMapMode === 'osm' ? 'osm' : 'sandbox';
    const rawGrid = localStorage.getItem(`${UI_STORAGE_PREFIX}gridType`);
    this.selectedGridType = GRID_TYPES.some((g) => g.value === rawGrid) ? rawGrid as string : 'single_intersection';
    this.osmMode = localStorage.getItem(`${UI_STORAGE_PREFIX}osmMode`) === '1';
  }

  private persistModeState(): void {
    localStorage.setItem(`${UI_STORAGE_PREFIX}mapMode`, this.mapMode);
    localStorage.setItem(`${UI_STORAGE_PREFIX}gridType`, this.selectedGridType);
    localStorage.setItem(`${UI_STORAGE_PREFIX}osmMode`, this.osmMode ? '1' : '0');
  }

  private resetPersistedSettings(): void {
    // Reset mode/view to defaults.
    this.mapMode = 'sandbox';
    this.selectedGridType = 'single_intersection';
    this.osmMode = false;
    this.persistModeState();
    this.mapModeBtns.forEach((b, k) => b.classList.toggle('active', k === this.mapMode));
    this.gridTypeBtns.forEach((b, k) => b.classList.toggle('active', k === this.selectedGridType));
    this.gridSubsection.style.display = '';
    this.gameViewBtn?.classList.add('active');
    this.osmViewBtn?.classList.remove('active');

    // Reset all layer toggles to their default values.
    for (const [id, cb] of this.checkboxes) {
      const fallback = this.checkboxDefaults.get(id) ?? cb.checked;
      cb.checked = fallback;
      this.persistCheckbox(id, fallback);
    }

    // Apply immediately in running session.
    this.onOsmModeToggle(false);
    this.onMapModeChange(this.currentForceSandbox);
    for (const [, cb] of this.checkboxes) {
      cb.dispatchEvent(new Event('change'));
    }
  }
}

// Re-export so game.ts doesn't need to import from RoadRenderer just for types
export { ROAD_TYPE_GROUP };
