import { defineConfig } from "cf/config";
import * as entrypoint from "./src/index.ts" with { type: "cf-worker" };

export default defineConfig({
  accountId: "7857543912cd05f6d0a5339865057d11",
  worker: {
    name: "hudsonriver-live",
    compatibilityDate: "2026-11-13",
    entrypoint,
    workersDev: false,
    domains: ["hudsonriver.live", "www.hudsonriver.live"],
    observability: {
      enabled: true,
      logs: {
        enabled: true,
        invocationLogs: true,
      },
      traces: {
        enabled: true,
      },
    },
  },
});
