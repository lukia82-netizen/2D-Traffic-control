import * as PIXI from 'pixi.js';

/**
 * Owns the PixiJS Application and the ordered layer stack.
 *
 * Layer ordering (sketch mode — PixiJS draws everything):
 *   0 – buildings        Building footprints (filled gray polygons)
 *   1 – roads            Road geometry (colored rectangles by road type)
 *   2 – tunnelOverlay    Dashed lines over tunnel roads
 *   3 – staticMarkings   Oneway arrows, lane markings
 *   4 – tunnelVehicles   Vehicles inside tunnels (α 0.25)
 *   5 – groundVehicles   Surface-level vehicles
 *   6 – bridgeVehicles   Vehicles on bridges
 *   7 – trafficLights    Traffic light state sprites
 *   8 – congestionLayer  Congestion heat overlay
 */
export class PixiOverlay {
  app!: PIXI.Application;

  buildings!: PIXI.Container;
  roads!: PIXI.Container;
  tunnelOverlay!: PIXI.Container;
  staticMarkings!: PIXI.Container;
  tunnelVehicles!: PIXI.Container;
  groundVehicles!: PIXI.Container;
  bridgeVehicles!: PIXI.Container;
  trafficLights!: PIXI.Container;
  congestionLayer!: PIXI.Container;

  private readonly containerId: string;

  constructor(containerId: string) {
    this.containerId = containerId;
  }

  async init(): Promise<void> {
    this.app = new PIXI.Application();

    await this.app.init({
      resizeTo: window,
      backgroundAlpha: 0,
      antialias: true,
      resolution: window.devicePixelRatio ?? 1,
      autoDensity: true,
    });

    // Mount canvas into the DOM container
    const container = document.getElementById(this.containerId);
    if (!container) {
      throw new Error(`PixiOverlay: container #${this.containerId} not found`);
    }
    container.appendChild(this.app.canvas);

    // Build the layer stack (order = render order, bottom to top)
    this.buildings       = new PIXI.Container();
    this.roads           = new PIXI.Container();
    this.tunnelOverlay   = new PIXI.Container();
    this.staticMarkings  = new PIXI.Container();
    this.tunnelVehicles  = new PIXI.Container();
    this.groundVehicles  = new PIXI.Container();
    this.bridgeVehicles  = new PIXI.Container();
    this.trafficLights   = new PIXI.Container();
    this.congestionLayer = new PIXI.Container();

    this.app.stage.addChild(this.buildings);
    this.app.stage.addChild(this.roads);
    this.app.stage.addChild(this.tunnelOverlay);
    this.app.stage.addChild(this.staticMarkings);
    this.app.stage.addChild(this.tunnelVehicles);
    this.app.stage.addChild(this.groundVehicles);
    this.app.stage.addChild(this.bridgeVehicles);
    this.app.stage.addChild(this.trafficLights);
    this.app.stage.addChild(this.congestionLayer);

    window.addEventListener('resize', () => this.resize());
  }

  resize(): void {
    // PixiJS resizeTo:window already handles canvas resize;
    // this hook lets subrenderers react if needed.
    this.app.renderer.resize(window.innerWidth, window.innerHeight);
  }

  get width(): number {
    return this.app.renderer.width;
  }

  get height(): number {
    return this.app.renderer.height;
  }
}
