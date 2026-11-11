// @ts-nocheck
// RTC schedule diagram: chunk size / reinfer point / blend / inference delay.
// Mirrors the openpi real-time chunking figure so the numbers on the sliders
// have a visible meaning before a run starts.

// Chunk size comes from the checkpoint's action horizon (50) and is not a knob.
const DEFAULTS = { chunk: 50, reinfer: 30, blend: 16, delay: 14, guidance: 10, horizon: 10, fps: 30 };
let measuredDelayMs = null; // read from the run's chunk events, not a setting

const ui = {
  reinfer: document.getElementById("rtcReinfer"),
  blend: document.getElementById("rtcBlend"),
  guidance: document.getElementById("rtcGuidance"),
  reinferOut: document.getElementById("rtcReinferOut"),
  blendOut: document.getElementById("rtcBlendOut"),
  guidanceOut: document.getElementById("rtcGuidanceOut"),
  chunkOut: document.getElementById("rtcChunkOut"),
  delayOut: document.getElementById("rtcDelayOut"),
  canvas: document.getElementById("rtcSchedule"),
  summary: document.getElementById("rtcSummary"),
};

function uiSettings() {
  const delayTicks = measuredDelayMs === null
    ? DEFAULTS.delay
    : Math.max(0, Math.round((measuredDelayMs / 1000) * DEFAULTS.fps));
  return {
    chunk: DEFAULTS.chunk,
    reinfer: Number(ui.reinfer?.value || DEFAULTS.reinfer),
    blend: Number(ui.blend?.value || DEFAULTS.blend),
    guidance: Number(ui.guidance?.value || DEFAULTS.guidance),
    delay: delayTicks,
    delayMeasured: measuredDelayMs !== null,
  };
}

export function rtcSettings() {
  const settings = uiSettings();
  return {
    chunk_steps: settings.chunk,
    reinfer_remaining: settings.reinfer,
    blend: settings.blend,
    guidance: settings.guidance,
    inference_delay: settings.delay,
  };
}

/** Update the delay readout from the runner's chunk events (milliseconds). */
export function setMeasuredDelay(ms) {
  if (!Number.isFinite(ms) || ms <= 0) return;
  measuredDelayMs = ms;
  draw();
}

function schedule({ chunk, reinfer, blend, delay }) {
  // Anchor of the next chunk = request point (chunk - reinfer) + inference delay.
  const period = Math.max(chunk - reinfer + delay, 1);
  const rows = [];
  for (let index = 0; index < 5; index += 1) {
    const anchor = index * period;
    rows.push({
      anchor,
      end: anchor + chunk,
      request: anchor + chunk - reinfer,
      handoff: anchor + period,
      free: anchor + chunk - reinfer + delay,
      dry: Math.max(delay - reinfer, 0),
    });
  }
  return { period, rows, span: rows[rows.length - 1].handoff + chunk };
}

function draw() {
  const settings = uiSettings();
  const { rows, span } = schedule(settings);
  const canvas = ui.canvas;
  if (!canvas) return;
  const dpr = window.devicePixelRatio || 1;
  const width = canvas.clientWidth || 900;
  const height = Math.max(canvas.clientHeight || 220, 120);
  if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  const pad = { left: 54, right: 16, top: 18, bottom: 34 };
  const plotW = width - pad.left - pad.right;
  const rowH = 34;
  const x = (tick) => pad.left + (tick / span) * plotW;

  // axes
  ctx.strokeStyle = "rgba(255,255,255,.18)";
  ctx.fillStyle = "rgba(255,255,255,.45)";
  ctx.font = "11px ui-monospace, monospace";
  ctx.beginPath();
  ctx.moveTo(pad.left, pad.top);
  ctx.lineTo(pad.left, height - pad.bottom);
  ctx.lineTo(width - pad.right, height - pad.bottom);
  ctx.stroke();
  const step = span > 90 ? 20 : 10;
  for (let tick = 0; tick <= span; tick += step) {
    ctx.fillText(String(tick), x(tick) - 6, height - pad.bottom + 16);
    ctx.strokeStyle = "rgba(255,255,255,.06)";
    ctx.beginPath();
    ctx.moveTo(x(tick), pad.top);
    ctx.lineTo(x(tick), height - pad.bottom);
    ctx.stroke();
  }
  ctx.fillStyle = "rgba(255,255,255,.45)";
  ctx.fillText("time (ticks)", width - pad.right - 70, height - 6);
  ctx.save();
  ctx.translate(14, pad.top + 40);
  ctx.rotate(-Math.PI / 2);
  ctx.fillText("chunk", 0, 0);
  ctx.restore();

  rows.forEach((row, index) => {
    const y = pad.top + index * rowH + 14;
    // free prediction
    ctx.strokeStyle = "rgba(255,255,255,.5)";
    ctx.fillStyle = "rgba(255,255,255,.04)";
    ctx.beginPath();
    ctx.rect(x(row.anchor), y - 7, x(row.end) - x(row.anchor), 14);
    ctx.fill();
    ctx.stroke();
    // blended prefix
    if (settings.blend > 0) {
      const blendEnd = row.anchor + Math.min(settings.blend, settings.chunk);
      ctx.save();
      ctx.beginPath();
      ctx.rect(x(row.anchor), y - 7, x(blendEnd) - x(row.anchor), 14);
      ctx.clip();
      ctx.strokeStyle = "rgba(255,255,255,.35)";
      for (let px = x(row.anchor) - 14; px < x(blendEnd); px += 6) {
        ctx.beginPath();
        ctx.moveTo(px, y + 7);
        ctx.lineTo(px + 14, y - 7);
        ctx.stroke();
      }
      ctx.restore();
      ctx.strokeStyle = "rgba(255,255,255,.5)";
      ctx.beginPath();
      ctx.rect(x(row.anchor), y - 7, x(blendEnd) - x(row.anchor), 14);
      ctx.stroke();
    }
    // guidance band: the new chunk's prefix is pulled toward the old queue,
    // and the weight sets how strongly (alpha follows the slider).
    const guided = Math.max(settings.reinfer - settings.delay, 0);
    if (guided > 0 && settings.guidance > 0) {
      const alpha = 0.05 + 0.5 * Math.min(settings.guidance / 50, 1);
      ctx.fillStyle = `rgba(120, 240, 199, ${alpha.toFixed(3)})`;
      ctx.fillRect(x(row.handoff), y - 7, x(row.handoff + guided) - x(row.handoff), 14);
    }
    // executed line: from this anchor to the handoff
    ctx.strokeStyle = "#e9e4d8";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(x(row.anchor), y - 11);
    ctx.lineTo(x(row.handoff), y - 11);
    ctx.stroke();
    ctx.lineWidth = 1;
    // frozen tail: executed while inference ran
    ctx.fillStyle = "#e9e4d8";
    ctx.fillRect(x(row.request), y + 10, x(row.handoff) - x(row.request), 4);
    // handoff line
    ctx.strokeStyle = "rgba(255,255,255,.25)";
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(x(row.handoff), pad.top);
    ctx.lineTo(x(row.handoff), height - pad.bottom);
    ctx.stroke();
    ctx.setLineDash([]);
    // dry ticks
    if (row.dry > 0) {
      ctx.strokeStyle = "rgba(255,156,151,.8)";
      for (let tick = row.end; tick < row.free; tick += Math.max(1, Math.round(settings.fps / 10))) {
        const px = x(tick);
        ctx.beginPath();
        ctx.moveTo(px - 3, y + 15);
        ctx.lineTo(px + 3, y + 21);
        ctx.moveTo(px + 3, y + 15);
        ctx.lineTo(px - 3, y + 21);
        ctx.stroke();
      }
    }
  });

  if (ui.delayOut) {
    ui.delayOut.textContent = settings.delayMeasured
      ? `${settings.delay} tick（${Math.round(measuredDelayMs)} ms）`
      : "运行后读取";
  }
  const dry = Math.max(settings.delay - settings.reinfer, 0);
  if (ui.summary) {
    const delayLabel = settings.delayMeasured
      ? `${settings.delay} tick`
      : `${settings.delay} tick（预估，运行后按实测更新）`;
    ui.summary.textContent =
      `新块在队列剩 ${settings.reinfer} 步时请求，${delayLabel}后到达；`
      + `到达时队列还剩 ${Math.max(settings.reinfer - settings.delay, 0)} 步，`
      + `其中 ${Math.min(settings.blend, Math.max(settings.reinfer - settings.delay, 0))} 步会与旧块混合。`
      + (dry > 0 ? `推理期间队列会空转 ${dry} tick（机器人等待）。` : "队列不会空转。")
      + ` 新块到达时前 ${settings.delay} 步已过期会被丢弃，之后 ${DEFAULTS.horizon} 步按引导强度 ${settings.guidance} 贴近旧动作。`;
  }
}

function bind() {
  if (!ui.canvas) return;
  const sync = () => {
    const chunk = DEFAULTS.chunk;
    const reinfer = Number(ui.reinfer.value);
    const blend = Number(ui.blend.value);
    if (reinfer >= chunk) ui.reinfer.value = String(chunk - 1);
    if (blend > reinfer) ui.blend.value = String(reinfer);
    if (ui.chunkOut) ui.chunkOut.textContent = String(chunk);
    if (ui.reinferOut) ui.reinferOut.textContent = String(Math.min(Number(ui.reinfer.value), chunk - 1));
    if (ui.blendOut) ui.blendOut.textContent = ui.blend.value;
    if (ui.guidanceOut) ui.guidanceOut.textContent = ui.guidance.value;
    draw();
  };
  [ui.reinfer, ui.blend, ui.guidance].forEach((node) => node?.addEventListener("input", sync));
  window.addEventListener("resize", draw);
  sync();
}

bind();
