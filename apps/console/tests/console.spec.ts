import { expect, test } from "@playwright/test";
import { PNG } from "pngjs";


function assertRenderedCanvas(buffer: Buffer): void {
  const png = PNG.sync.read(buffer);
  const colors = new Set<number>();
  let opaquePixels = 0;
  for (let offset = 0; offset < png.data.length; offset += 4) {
    const alpha = png.data[offset + 3];
    if (alpha < 16) continue;
    opaquePixels += 1;
    colors.add(
      (png.data[offset] << 16)
      | (png.data[offset + 1] << 8)
      | png.data[offset + 2],
    );
    if (colors.size > 64) break;
  }
  expect(opaquePixels).toBeGreaterThan(64);
  expect(colors.size).toBeGreaterThan(16);
}

for (const viewport of [
  { name: "desktop", width: 1440, height: 900 },
  { name: "mobile", width: 390, height: 844 },
]) {
  test(`control console renders at ${viewport.name}`, async ({ page }) => {
    const pageErrors: string[] = [];
    page.on("pageerror", (error) => pageErrors.push(error.message));
    await page.setViewportSize(viewport);
    await page.goto("/control", { waitUntil: "domcontentloaded" });
    const canvas = page.locator("#robotCanvas");
    await expect(canvas).toBeVisible();
    await expect(page.locator("#armConnectButton")).toBeVisible();
    await expect(page.locator("#robotModelLoading")).toBeHidden({ timeout: 30_000 });
    await page.waitForTimeout(500);
    const box = await canvas.boundingBox();
    expect(box?.width ?? 0).toBeGreaterThan(200);
    expect(box?.height ?? 0).toBeGreaterThan(180);
    assertRenderedCanvas(await canvas.screenshot());
    if (viewport.name === "mobile") {
      const controlsBox = await page.locator(".robot-joint-panel").boundingBox();
      const cameraBox = await page.locator(".vision-dock").boundingBox();
      expect(cameraBox?.y ?? 0).toBeGreaterThanOrEqual(
        (controlsBox?.y ?? 0) + (controlsBox?.height ?? 0) - 1,
      );
    }
    expect(pageErrors).toEqual([]);
    await page.screenshot({
      path: test.info().outputPath(`${viewport.name}.png`),
      fullPage: true,
    });
  });
}

test("overview lists navigation without owning the control session", async ({ page }) => {
  const pageErrors: string[] = [];
  page.on("pageerror", (error) => pageErrors.push(error.message));
  await page.setViewportSize({ width: 1440, height: 900 });
  await page.goto("/", { waitUntil: "domcontentloaded" });
  await expect(page.getByRole("heading", { name: "系统总览" })).toBeVisible();
  const nav = page.locator(".shell-nav");
  await expect(nav.getByRole("link", { name: "总览" })).toHaveAttribute("aria-current", "page");
  await expect(nav.getByRole("link", { name: "控制台" })).toBeVisible();
  await expect(nav.getByRole("link", { name: "数据采集" })).toBeVisible();
  await expect(page.locator("#robotCanvas")).toHaveCount(0);
  // The static test server cannot serve the JSON status endpoint: the page must
  // report the missing backend instead of opening a control WebSocket.
  await expect(page.locator("#overviewConnection")).toHaveAttribute("data-state", "disconnected");
  expect(pageErrors).toEqual([]);
});

test("navigation keeps the pairing token", async ({ page }) => {
  await page.goto("/?token=abc", { waitUntil: "domcontentloaded" });
  const control = page.locator(".shell-nav").getByRole("link", { name: "控制台" });
  await expect(control).toHaveAttribute("href", /\/control\?token=abc$/);
});

test("policy page renders the task form", async ({ page }) => {
  const pageErrors: string[] = [];
  page.on("pageerror", (error) => pageErrors.push(error.message));
  await page.goto("/policy", { waitUntil: "domcontentloaded" });
  await expect(page.getByRole("heading", { name: "策略任务" })).toBeVisible();
  await expect(page.locator("#policyPrompt")).toBeVisible();
  await expect(page.locator("#policyStart")).toBeVisible();
  await expect(page.locator("#policyConfirm")).toBeHidden();
  expect(pageErrors).toEqual([]);
});

for (const viewport of [
  { name: "desktop", width: 1440, height: 900 },
  { name: "tablet", width: 1024, height: 768 },
  { name: "mobile", width: 390, height: 844 },
  { name: "small-phone", width: 360, height: 720 },
]) {
  test(`collection session renders at ${viewport.name}`, async ({ page }) => {
    const pageErrors: string[] = [];
    page.on("pageerror", (error) => pageErrors.push(error.message));
    await page.setViewportSize(viewport);
    await page.goto("/collection", { waitUntil: "domcontentloaded" });
    await expect(page.getByRole("heading", { name: "采集会话" })).toBeVisible();
    await expect(page.locator("#taskInput")).toBeVisible();
    await expect(page.locator("#cameraChoices")).toBeVisible();
    await expect(page.locator("#datasetCount")).toBeVisible();
    await expect(page.locator(".joint-row")).toHaveCount(6);
    await expect(page.locator("#rgbSelect")).toBeHidden();
    await page.locator(".advanced summary").click();
    await expect(page.locator("#rgbSelect")).toBeVisible();
    await expect(page.locator("#startButton")).toBeVisible();
    // No backend behind the static test server: collection cannot start.
    await expect(page.locator("#startButton")).toBeDisabled();
    await expect(page.locator("#stopButton")).toBeDisabled();
    await expect(page.locator("#recordingBadge")).toBeHidden();
    const jointCharts = page.locator(".joint-row canvas");
    await expect(jointCharts).toHaveCount(6);
    await expect(jointCharts.first()).toBeVisible();
    const chartBox = await jointCharts.first().boundingBox();
    expect(chartBox?.width ?? 0).toBeGreaterThan(20);
    const startBox = await page.locator("#startButton").boundingBox();
    expect(startBox?.height ?? 0).toBeGreaterThanOrEqual(40);
    const overflow = await page.evaluate(() => ({
      horizontal: document.documentElement.scrollWidth - window.innerWidth,
      vertical: document.documentElement.scrollHeight - window.innerHeight,
    }));
    expect(overflow.horizontal).toBeLessThanOrEqual(1);
    expect(overflow.vertical).toBeLessThanOrEqual(1);
    expect(pageErrors).toEqual([]);
    await page.screenshot({
      path: test.info().outputPath(`collection-${viewport.name}.png`),
      fullPage: false,
    });
  });
}
