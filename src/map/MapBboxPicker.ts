import maplibregl from 'maplibre-gl';

type Bbox = [number, number, number, number];
type CompleteCb = (bbox: Bbox) => void | Promise<void>;

export class MapBboxPicker {
  private readonly map: maplibregl.Map;
  private boxEl: HTMLDivElement | null = null;
  private overlayEl: HTMLDivElement | null = null;
  private startPx: { x: number; y: number } | null = null;
  private moveHandler: ((e: MouseEvent) => void) | null = null;
  private upHandler: ((e: MouseEvent) => void) | null = null;
  private completeCb: CompleteCb | null = null;

  constructor(map: maplibregl.Map) {
    this.map = map;
  }

  start(onComplete: CompleteCb): void {
    this.cancel();
    this.completeCb = onComplete;

    const mapCanvas = this.map.getCanvas();
    const parent = mapCanvas.parentElement;
    if (!parent) return;

    const overlay = document.createElement('div');
    overlay.style.position = 'absolute';
    overlay.style.inset = '0';
    overlay.style.cursor = 'crosshair';
    overlay.style.zIndex = '50';
    overlay.style.pointerEvents = 'auto';

    const box = document.createElement('div');
    box.style.position = 'absolute';
    box.style.border = '2px dashed #4ea5ff';
    box.style.background = 'rgba(78, 165, 255, 0.18)';
    box.style.display = 'none';
    overlay.appendChild(box);

    this.overlayEl = overlay;
    this.boxEl = box;
    parent.appendChild(overlay);

    const downHandler = (e: MouseEvent): void => {
      if (e.button !== 0) return;
      this.startPx = { x: e.clientX, y: e.clientY };
      box.style.display = 'block';
      box.style.left = `${e.clientX}px`;
      box.style.top = `${e.clientY}px`;
      box.style.width = '0px';
      box.style.height = '0px';
      e.preventDefault();
    };

    this.moveHandler = (e: MouseEvent): void => {
      if (!this.startPx || !this.boxEl) return;
      const x1 = Math.min(this.startPx.x, e.clientX);
      const y1 = Math.min(this.startPx.y, e.clientY);
      const x2 = Math.max(this.startPx.x, e.clientX);
      const y2 = Math.max(this.startPx.y, e.clientY);
      this.boxEl.style.left = `${x1}px`;
      this.boxEl.style.top = `${y1}px`;
      this.boxEl.style.width = `${x2 - x1}px`;
      this.boxEl.style.height = `${y2 - y1}px`;
    };

    this.upHandler = (e: MouseEvent): void => {
      if (!this.startPx) {
        this.cancel();
        return;
      }
      const start = this.startPx;
      const end = { x: e.clientX, y: e.clientY };
      this.cancel();

      const dx = Math.abs(end.x - start.x);
      const dy = Math.abs(end.y - start.y);
      if (dx < 10 || dy < 10) return;

      const nw = this.map.unproject([Math.min(start.x, end.x), Math.min(start.y, end.y)]);
      const se = this.map.unproject([Math.max(start.x, end.x), Math.max(start.y, end.y)]);
      const bbox: Bbox = [nw.lng, se.lat, se.lng, nw.lat];
      void this.completeCb?.(bbox);
    };

    overlay.addEventListener('mousedown', downHandler, { once: true });
    window.addEventListener('mousemove', this.moveHandler);
    window.addEventListener('mouseup', this.upHandler, { once: true });
  }

  private cancel(): void {
    if (this.moveHandler) window.removeEventListener('mousemove', this.moveHandler);
    if (this.upHandler) window.removeEventListener('mouseup', this.upHandler);
    this.moveHandler = null;
    this.upHandler = null;
    this.startPx = null;
    this.boxEl = null;
    this.overlayEl?.remove();
    this.overlayEl = null;
  }

  destroy(): void {
    this.cancel();
    this.completeCb = null;
  }
}
