import { defineConfig } from 'vite';

export default defineConfig(({ command }) => ({
  clearScreen: false,
  server: {
    port: 1420,
    strictPort: true,
    watch: {
      ignored: ['**/src-tauri/**', '**/node_modules/**'],
    },
  },
  envPrefix: ['VITE_', 'TAURI_'],
  build: {
    target: ['chrome105'],
    minify: !process.env['TAURI_DEBUG'] ? 'esbuild' : false,
    sourcemap: !!process.env['TAURI_DEBUG'],
    outDir: 'dist',
    // Use relative input path to avoid Rollup absolute-path issues on Windows with spaces
    rollupOptions: command === 'build' ? { input: { main: 'index.html' } } : {},
  },
}));
