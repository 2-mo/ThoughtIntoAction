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
    await page.goto("/", { waitUntil: "domcontentloaded" });
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

for (const viewport of [
  { name: "desktop", width: 1440, height: 900 },
  { name: "tablet", width: 1024, height: 768 },
  { name: "mobile", width: 390, height: 844 },
  { name: "small-phone", width: 360, height: 720 },
]) {
  test(`collection view renders at ${viewport.name}`, async ({ page }) => {
    const pageErrors: string[] = [];
    page.on("pageerror", (error) => pageErrors.push(error.message));
    await page.setViewportSize(viewport);
    await page.goto("/collection", { waitUntil: "domcontentloaded" });
    await expect(page.getByRole("heading", { name: "GR00T 示教采集" })).toBeVisible();
    await expect(page.locator("#startButton")).toBeVisible();
    await expect(page.locator("video")).toHaveCount(0);
    await expect(page.locator("#stateSamples")).toHaveText("0");
    await expect(page.locator("#cameraFrames")).toHaveText("0");
    await expect(page.locator("#startButton")).toHaveAttribute("aria-keyshortcuts", "Space");
    await expect(page.locator("#saveButton")).toHaveAttribute("aria-keyshortcuts", "S");
    await expect(page.locator("#discardButton")).toHaveAttribute("aria-keyshortcuts", "X");
    await expect(page.locator("#reviewDiscardButton")).toHaveAttribute("aria-keyshortcuts", "X");
    await expect(page.locator("#startButton kbd")).toHaveText("Space");
    await expect(page.locator("#stopButton kbd")).toHaveText("Space");
    await expect(page.locator("#saveButton kbd")).toHaveText("S");
    await expect(page.locator("#discardButton kbd")).toHaveText("X");
    const jointCharts = page.locator(".joint-card canvas");
    const healthHeatmap = page.locator("#healthHeatmap");
    await expect(jointCharts).toHaveCount(6);
    await expect(jointCharts.first()).toBeVisible();
    await expect(healthHeatmap).toBeVisible();
    assertRenderedCanvas(await jointCharts.first().screenshot());
    assertRenderedCanvas(await healthHeatmap.screenshot());
    await expect(page.getByText("ROI 未配置")).toHaveCount(0);
    await expect(page.getByText("源方块未检测")).toHaveCount(0);
    await expect(page.getByText("D455 视频中断")).toHaveCount(0);
    if (["mobile", "small-phone"].includes(viewport.name)) {
      const telemetryBox = await page.locator(".telemetry").boundingBox();
      const controlBox = await page.locator(".control-panel").boundingBox();
      const startBox = await page.locator("#startButton").boundingBox();
      expect(startBox?.height ?? 0).toBeGreaterThanOrEqual(58);
      expect((startBox?.y ?? 0) + (startBox?.height ?? 0)).toBeLessThanOrEqual(viewport.height + 1);
      expect(controlBox?.height ?? 0).toBeGreaterThan(telemetryBox?.height ?? 0);
      expect((await healthHeatmap.boundingBox())?.height ?? 0).toBeGreaterThanOrEqual(38);
    }
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

    await page.evaluate(() => {
      const review = document.querySelector<HTMLElement>("#reviewPanel");
      review?.removeAttribute("hidden");
      review?.closest(".control-panel")?.classList.add("review-mode");
      document.querySelector<HTMLElement>("#copyValidationButton")?.removeAttribute("hidden");
    });
    await expect(page.locator("#saveButton")).toBeVisible();
    await expect(page.locator("#reviewDiscardButton")).toBeVisible();
    await expect(page.locator("#copyValidationButton")).toBeVisible();
    for (const selector of ["#saveButton", "#reviewDiscardButton", "#copyValidationButton", "#message"]) {
      const control = await page.locator(selector).boundingBox();
      expect((control?.y ?? 0) + (control?.height ?? 0)).toBeLessThanOrEqual(viewport.height + 1);
    }
    const reviewOverflow = await page.evaluate(() =>
      document.documentElement.scrollHeight - window.innerHeight);
    expect(reviewOverflow).toBeLessThanOrEqual(1);
    await page.screenshot({
      path: test.info().outputPath(`collection-review-${viewport.name}.png`),
      fullPage: false,
    });
  });
}
