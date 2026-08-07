import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      // Keeps the app same-origin, so the browser never treats API calls as
      // cross-origin and the Rust side needs no CORS layer. Vite forwards these
      // server-to-server, stripping the `/api` prefix the backend doesn't use.
      '/api': {
        target: 'http://localhost:3000',
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/api/, ''),
      },
    },
  },
});
