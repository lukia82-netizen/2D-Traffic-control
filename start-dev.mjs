// Starts Vite programmatically with configFile: false to prevent config file watching
import { createServer } from 'vite';
import { resolve } from 'path';
import { fileURLToPath } from 'url';

const __dirname = fileURLToPath(new URL('.', import.meta.url));

const server = await createServer({
  configFile: false,
  root: __dirname,
  server: {
    port: 1420,
    strictPort: true,
    host: 'localhost',
  },
  build: {
    rollupOptions: {
      input: { main: resolve(__dirname, 'index.html') },
      output: { dir: resolve(__dirname, 'dist') },
    },
  },
  resolve: {
    alias: {
      '@': resolve(__dirname, 'src'),
    },
  },
  clearScreen: false,
});

await server.listen();
console.log('\n  VITE dev server ready at http://localhost:1420/\n');
server.printUrls();

process.on('SIGINT', () => {
  server.close().then(() => process.exit(0));
});
