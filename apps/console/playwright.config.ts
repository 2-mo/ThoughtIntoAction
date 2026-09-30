import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "./tests",
  timeout: 45_000,
  expect: { timeout: 10_000 },
  use: {
    baseURL: "http://127.0.0.1:8181",
    screenshot: "only-on-failure",
    launchOptions: {
      args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
    },
  },
  webServer: {
    command: "env PYTHONPATH=../../src python3 -B -m arx_tools.https_static_server --host 127.0.0.1 --port 8181 --directory dist --spa",
    cwd: ".",
    url: "http://127.0.0.1:8181",
    reuseExistingServer: false,
  },
});
