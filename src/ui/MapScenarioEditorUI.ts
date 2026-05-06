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
};

export class MapScenarioEditorUI {
  private readonly panel: HTMLElement;
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
    this.panel = this.buildPanel();
    document.body.appendChild(this.panel);

    this.mapJson = this.require('editor-map-json') as HTMLTextAreaElement;
    this.scenarioName = this.require('editor-scenario-name') as HTMLInputElement;
    this.scenarioDescription = this.require('editor-scenario-description') as HTMLInputElement;
    this.scenarioMaxVehicles = this.require('editor-scenario-max-vehicles') as HTMLInputElement;
    this.scenarioTimeScale = this.require('editor-scenario-time-scale') as HTMLInputElement;
    this.scenarioStartTime = this.require('editor-scenario-start-time') as HTMLInputElement;
    this.scenarioList = this.require('editor-scenario-list') as HTMLSelectElement;

    this.mapJson.value = JSON.stringify(EMPTY_MAP, null, 2);
    this.bindEvents();
    this.refreshScenarioList();
  }

  setMapData(data: MapData): void {
    this.mapJson.value = JSON.stringify(data, null, 2);
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
      const parsed = JSON.parse(this.mapJson.value) as MapData;
      if (!Array.isArray(parsed.nodes) || !Array.isArray(parsed.edges) || !Array.isArray(parsed.spawnPoints)) {
        throw new Error('Map JSON requires nodes, edges, and spawnPoints arrays');
      }
      return parsed;
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

  private buildPanel(): HTMLElement {
    const panel = document.createElement('div');
    panel.className = 'editor-panel';
    panel.innerHTML = `
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
    return panel;
  }

  private require(id: string): HTMLElement {
    const el = document.getElementById(id);
    if (!el) {
      throw new Error(`MapScenarioEditorUI: missing #${id}`);
    }
    return el;
  }
}
