import type { MapData } from '../bridge/commands';

export interface ScenarioData {
  name: string;
  description: string;
  maxVehicles: number;
  timeScale: number;
  startTimeS: number;
  mapData: MapData;
}

const STORAGE_KEY = 'traffic-control-scenarios';

const EMPTY_MAP: MapData = {
  nodes: [],
  edges: [],
  spawnPoints: [],
  bbox: [19.93, 50.05, 19.95, 50.07],
  buildings: [],
  restrictions: [],
  tramStops: [],
  turnConnectors: [],
  lanes: [],
  conflictAreas: [],
};

const EDITOR_COLLAPSED_KEY = 'traffic-control-editor-collapsed';

export class MapScenarioEditorUI {
  private readonly panel: HTMLElement;
  /** Content below the compact title bar (hidden when collapsed). */
  private readonly panelBody: HTMLElement;
  private readonly collapseToggle: HTMLButtonElement;
  private readonly mapJson: HTMLTextAreaElement;
  private readonly scenarioName: HTMLInputElement;
  private readonly scenarioDescription: HTMLInputElement;
  private readonly scenarioMaxVehicles: HTMLInputElement;
  private readonly scenarioTimeScale: HTMLInputElement;
  private readonly scenarioStartTime: HTMLInputElement;
  private readonly scenarioList: HTMLSelectElement;

  onApplyMap: (mapData: MapData) => void = () => undefined;
  onApplyScenario: (scenario: ScenarioData) => void = () => undefined;

  constructor() {
    const built = this.buildPanel();
    this.panel = built.root;
    this.panelBody = built.body;
    this.collapseToggle = built.toggleBtn;
    document.body.appendChild(this.panel);

    this.mapJson = this.require('editor-map-json') as HTMLTextAreaElement;
    this.scenarioName = this.require('editor-scenario-name') as HTMLInputElement;
    this.scenarioDescription = this.require('editor-scenario-description') as HTMLInputElement;
    this.scenarioMaxVehicles = this.require('editor-scenario-max-vehicles') as HTMLInputElement;
    this.scenarioTimeScale = this.require('editor-scenario-time-scale') as HTMLInputElement;
    this.scenarioStartTime = this.require('editor-scenario-start-time') as HTMLInputElement;
    this.scenarioList = this.require('editor-scenario-list') as HTMLSelectElement;

    this.mapJson.value = JSON.stringify(EMPTY_MAP, null, 2);
    this.collapseToggle.addEventListener('click', () =>
      this.applyCollapsed(!this.panel.classList.contains('editor-panel--collapsed')),
    );
    try {
      const saved = sessionStorage.getItem(EDITOR_COLLAPSED_KEY);
      const wantCollapsed = saved === null ? true : saved === '1'; // default: zwinięty
      this.applyCollapsed(wantCollapsed);
    } catch {
      this.applyCollapsed(true);
    }
    this.bindEvents();
    this.refreshScenarioList();
  }

  /** Collapse to a small chip (default) or show full editor. */
  applyCollapsed(collapsed: boolean): void {
    this.panel.classList.toggle('editor-panel--collapsed', collapsed);
    const label = collapsed ? 'Mapa — rozwiń' : 'Mapa — zwiń';
    this.collapseToggle.setAttribute('aria-expanded', collapsed ? 'false' : 'true');
    this.collapseToggle.title = collapsed
      ? 'Rozwiń edytor mapy i scenariusza'
      : 'Zwiń edytor (zostanie wąski pasek)';
    this.collapseToggle.textContent = collapsed ? '▸' : '▾';
    this.collapseToggle.setAttribute('aria-label', label);
    try {
      sessionStorage.setItem(EDITOR_COLLAPSED_KEY, collapsed ? '1' : '0');
    } catch {
      /* ignore */
    }
    this.panelBody.setAttribute('aria-hidden', collapsed ? 'true' : 'false');
  }

  setMapData(data: MapData): void {
    this.mapJson.value = JSON.stringify(data, null, 2);
  }

  destroy(): void {
    this.panel.remove();
  }

  private bindEvents(): void {
    this.require('editor-apply-map-btn').addEventListener('click', () => {
      const parsed = this.readMapFromTextarea();
      if (!parsed) return;
      this.onApplyMap(parsed);
    });

    this.require('editor-save-scenario-btn').addEventListener('click', () => {
      const parsed = this.readMapFromTextarea();
      if (!parsed) return;
      const scenario = this.readScenario(parsed);
      if (!scenario) return;
      this.saveScenario(scenario);
      this.refreshScenarioList();
    });

    this.require('editor-load-scenario-btn').addEventListener('click', () => {
      const selected = this.scenarioList.value;
      if (!selected) return;
      const scenario = this.loadScenarioByName(selected);
      if (!scenario) return;
      this.populateScenarioForm(scenario);
      this.mapJson.value = JSON.stringify(scenario.mapData, null, 2);
      this.onApplyScenario(scenario);
    });

    this.require('editor-export-map-btn').addEventListener('click', () => {
      const blob = new Blob([this.mapJson.value], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'custom-map.json';
      a.click();
      URL.revokeObjectURL(url);
    });
  }

  private readMapFromTextarea(): MapData | null {
    try {
      const parsed = JSON.parse(this.mapJson.value) as Partial<MapData>;
      if (!Array.isArray(parsed.nodes) || !Array.isArray(parsed.edges) || !Array.isArray(parsed.spawnPoints)) {
        throw new Error('Map JSON requires nodes, edges, and spawnPoints arrays');
      }
      return {
        nodes: parsed.nodes,
        edges: parsed.edges,
        spawnPoints: parsed.spawnPoints,
        bbox: parsed.bbox ?? EMPTY_MAP.bbox,
        buildings: parsed.buildings ?? [],
        restrictions: parsed.restrictions ?? [],
        tramStops: parsed.tramStops ?? [],
        turnConnectors: parsed.turnConnectors ?? [],
        lanes: parsed.lanes ?? [],
        conflictAreas: parsed.conflictAreas ?? [],
      };
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      alert(`Niepoprawny JSON mapy: ${message}`);
      return null;
    }
  }

  private readScenario(mapData: MapData): ScenarioData | null {
    const name = this.scenarioName.value.trim();
    if (!name) {
      alert('Podaj nazwę scenariusza.');
      return null;
    }
    return {
      name,
      description: this.scenarioDescription.value.trim(),
      maxVehicles: Math.max(5, Number(this.scenarioMaxVehicles.value) || 20),
      timeScale: Math.max(1, Number(this.scenarioTimeScale.value) || 15),
      startTimeS: this.parseTimeToSeconds(this.scenarioStartTime.value),
      mapData,
    };
  }

  private parseTimeToSeconds(text: string): number {
    const [hRaw, mRaw] = text.split(':');
    const h = Math.max(0, Math.min(23, Number(hRaw) || 0));
    const m = Math.max(0, Math.min(59, Number(mRaw) || 0));
    return h * 3600 + m * 60;
  }

  private formatSecondsToTime(seconds: number): string {
    const h = Math.floor((seconds % 86400) / 3600);
    const m = Math.floor((seconds % 3600) / 60);
    return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}`;
  }

  private getScenarios(): ScenarioData[] {
    try {
      const raw = localStorage.getItem(STORAGE_KEY);
      if (!raw) return [];
      const parsed = JSON.parse(raw) as ScenarioData[];
      return Array.isArray(parsed) ? parsed : [];
    } catch {
      return [];
    }
  }

  private saveScenario(scenario: ScenarioData): void {
    const scenarios = this.getScenarios().filter((s) => s.name !== scenario.name);
    scenarios.push(scenario);
    localStorage.setItem(STORAGE_KEY, JSON.stringify(scenarios));
  }

  private loadScenarioByName(name: string): ScenarioData | null {
    return this.getScenarios().find((s) => s.name === name) ?? null;
  }

  private refreshScenarioList(): void {
    const scenarios = this.getScenarios();
    this.scenarioList.innerHTML = '';
    const empty = document.createElement('option');
    empty.value = '';
    empty.textContent = scenarios.length ? 'Wybierz scenariusz…' : 'Brak zapisanych scenariuszy';
    this.scenarioList.appendChild(empty);
    for (const scenario of scenarios) {
      const option = document.createElement('option');
      option.value = scenario.name;
      option.textContent = scenario.name;
      this.scenarioList.appendChild(option);
    }
  }

  private populateScenarioForm(scenario: ScenarioData): void {
    this.scenarioName.value = scenario.name;
    this.scenarioDescription.value = scenario.description;
    this.scenarioMaxVehicles.value = String(scenario.maxVehicles);
    this.scenarioTimeScale.value = String(scenario.timeScale);
    this.scenarioStartTime.value = this.formatSecondsToTime(scenario.startTimeS);
  }

  private buildPanel(): {
    root: HTMLElement;
    body: HTMLElement;
    toggleBtn: HTMLButtonElement;
  } {
    const panel = document.createElement('div');
    panel.className = 'editor-panel editor-panel--collapsed';
    panel.setAttribute('role', 'region');
    panel.setAttribute('aria-label', 'Edytor mapy i scenariusza');

    const titleBar = document.createElement('div');
    titleBar.className = 'editor-titlebar';

    const titleText = document.createElement('span');
    titleText.className = 'editor-titlebar-label';
    titleText.textContent = 'MAP / scenariusz';

    const toggle = document.createElement('button');
    toggle.type = 'button';
    toggle.className = 'editor-collapse-btn';

    titleBar.appendChild(titleText);
    titleBar.appendChild(toggle);

    const body = document.createElement('div');
    body.className = 'editor-panel-body';

    body.innerHTML = `
      <div class="editor-header">MAP & SCENARIO EDITOR</div>
      <label class="editor-label">Mapa (JSON)</label>
      <textarea id="editor-map-json" class="editor-textarea"></textarea>
      <div class="editor-row">
        <button id="editor-apply-map-btn" class="editor-btn">Zastosuj mapę</button>
        <button id="editor-export-map-btn" class="editor-btn">Eksport JSON</button>
      </div>
      <div class="editor-divider"></div>
      <label class="editor-label">Nazwa scenariusza</label>
      <input id="editor-scenario-name" class="editor-input" placeholder="np. Szczyt poranny" />
      <label class="editor-label">Opis</label>
      <input id="editor-scenario-description" class="editor-input" placeholder="Krótki opis" />
      <div class="editor-grid">
        <label class="editor-label">Max pojazdów</label>
        <input id="editor-scenario-max-vehicles" class="editor-input" type="number" min="5" max="1000" value="20" />
        <label class="editor-label">Skala czasu</label>
        <input id="editor-scenario-time-scale" class="editor-input" type="number" min="1" max="360" value="15" />
        <label class="editor-label">Start (HH:MM)</label>
        <input id="editor-scenario-start-time" class="editor-input" value="06:00" />
      </div>
      <div class="editor-row">
        <button id="editor-save-scenario-btn" class="editor-btn">Zapisz scenariusz</button>
      </div>
      <label class="editor-label">Zapisane scenariusze</label>
      <select id="editor-scenario-list" class="editor-input"></select>
      <div class="editor-row">
        <button id="editor-load-scenario-btn" class="editor-btn">Wczytaj i uruchom</button>
      </div>
    `;

    panel.appendChild(titleBar);
    panel.appendChild(body);
    return { root: panel, body, toggleBtn: toggle };
  }

  private require(id: string): HTMLElement {
    const el = document.getElementById(id);
    if (!el) {
      throw new Error(`MapScenarioEditorUI: missing #${id}`);
    }
    return el;
  }
}
