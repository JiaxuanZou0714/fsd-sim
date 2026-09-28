import { defineConfig } from "vitest/config";

export default defineConfig({
  build: {
    chunkSizeWarningLimit: 1000,
  },
  test: {
    include: ["tests/**/*.test.ts"],
  },
});
