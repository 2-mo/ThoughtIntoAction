// @ts-nocheck
// Policy task page: read-only kernel status plus whitelisted policy runs.
import { withToken } from "../../api/control-protocol";
import { listOperations, operationLog, startOperation, stopOperation } from "../../api/operations";
import { rtcSettings, setMeasuredDelay } from "./rtc";

const STATUS_MS = 500;
const OPERATIONS_MS = 2000;

const ui = Object.fromEntries(
  [
    "policyKernel", "policyState", "policyPrompt", "policyPromptHistory", "policyKind", "policySeconds",
    "policyMode", "policyServer", "policyServerField", "policyLocalKindField", "policyRtcPanel",
    "policyRtcRow", "policyRtc", "policyDryRun",
    "policyConfirm", "policyConfirmStart", "policyCancel", "policyStart",
    "policyStop", "policyHome", "policyReady", "policyMessage", "policyRun", "policyRuntime", "policyOwner",
    "policyAge", "policyOps", "policyLog", "policyRefreshLog",
    "modelCards", "modelHint", "policyModelHint",
  ].map((id) => [id, document.getElementById(id)]),
);

const statusUrl = withToken("/api/v1/system/status");
let statusTimer = undefined;
let operationsTimer = undefined;
let current = null;

function setText(node, value) {
  if (node) node.textContent = value;
}

function setPill(node, state, label) {
  if (!node) return;
  node.dataset.state = state;
  node.textContent = label;
}

function setMessage(text, tone = "") {
  if (!ui.policyMessage) return;
  ui.policyMessage.textContent = text;
  ui.policyMessage.dataset.tone = tone;
}

function renderButtons() {
  const running = current && ["starting", "running", "stopping"].includes(current.state);
  if (ui.policyStart) ui.policyStart.disabled = Boolean(running);
  if (ui.policyStop) ui.policyStop.disabled = !running;
  if (running) {
    setPill(ui.policyState, "on", current.state === "stopping" ? "停止中" : "运行中");
  } else if (current && current.state === "failed") {
    setPill(ui.policyState, "error", "失败");
  } else {
    setPill(ui.policyState, "off", "空闲");
  }
  setText(ui.policyRun, current?.run_id || "—");
}

async function refreshOperations() {
  try {
    const list = await listOperations();
    current = list.find((operation) => operation.kind === "policy") || null;
    renderButtons();
  } catch {
    // Backend unavailable; keep the last state.
  }
}

async function refreshLog() {
  try {
    const lines = await operationLog("policy", 80);
    if (ui.policyLog) ui.policyLog.textContent = lines.join("\n") || "等待运行…";
    // Chunk events carry the measured inference latency; feed the RTC readout.
    for (let index = lines.length - 1; index >= 0; index -= 1) {
      const match = /"latency_ms"\s*:\s*([0-9.]+)/.exec(lines[index]);
      if (match) {
        setMeasuredDelay(Number(match[1]));
        break;
      }
    }
  } catch (error) {
    if (ui.policyLog) {
      const message = error instanceof Error ? error.message : String(error);
      ui.policyLog.textContent = message.includes("尚未启动") ? "尚未运行" : "日志不可用";
    }
  }
}

async function pollStatus() {
  if (document.hidden) return;
  try {
    const response = await fetch(statusUrl, { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    if (!data.connected) {
      setPill(ui.policyKernel, "off", "内核未连接");
      setText(ui.policyRuntime, "—");
      setText(ui.policyOwner, "—");
      setText(ui.policyAge, "—");
      setText(ui.policyOps, "—");
      return;
    }
    setPill(ui.policyKernel, "on", "内核已连接");
    setText(ui.policyRuntime, data.runtime_id || "—");
    setText(ui.policyOwner, data.owner
      ? [data.owner.source, data.owner.purpose].filter(Boolean).join(" · ") || "已接管"
      : "空闲");
    setText(ui.policyAge, `${Math.round(data.feedback_age_ms)} ms`);
    const operations = Array.isArray(data.operations) ? data.operations : [];
    setText(ui.policyOps, operations.length
      ? operations.map((operation) => `${operation.operation_id} · ${operation.status}`).join("；")
      : "空闲");
  } catch {
    setPill(ui.policyKernel, "error", "后端未运行");
  }
}

function syncModeFields() {
  // Only the fields for the selected mode stay on screen; the other mode's
  // controls are hidden rather than left visible but ignored.
  const remote = (ui.policyMode?.value || "remote") === "remote";
  if (ui.policyServerField) ui.policyServerField.hidden = !remote;
  if (ui.policyLocalKindField) ui.policyLocalKindField.hidden = remote;
  if (ui.policyRtcRow) ui.policyRtcRow.hidden = !remote;
  if (ui.policyRtcPanel) ui.policyRtcPanel.hidden = !remote;
}

function showConfirm(show) {
  if (ui.policyConfirm) ui.policyConfirm.hidden = !show;
}

async function runPolicy(confirm) {
  const prompt = (ui.policyPrompt?.value || "").trim();
  if (!prompt) {
    setMessage("先填写任务描述", "error");
    return;
  }
  if (!confirm) {
    showConfirm(true);
    return;
  }
  showConfirm(false);
  setMessage("正在启动策略…");
  try {
    const mode = ui.policyMode?.value || "remote";
    if (mode === "remote" && ui.policyServer?.value) {
      try {
        window.localStorage.setItem("tia.policy.server", ui.policyServer.value.trim());
      } catch {
        /* storage unavailable */
      }
    }
    const seconds = Number(ui.policySeconds?.value || 60);
    const payload = mode === "remote"
      ? {
          mode: "remote",
          prompt,
          server: (ui.policyServer?.value || "").trim(),
          seconds,
          rtc: Boolean(ui.policyRtc?.checked),
          ...rtcSettings(),
          dry_run: Boolean(ui.policyDryRun?.checked),
          confirm: true,
        }
      : { mode: "local", prompt, policy: ui.policyKind?.value || "sft", seconds, confirm: true };
    await startOperation("policy", payload);
    setMessage("已提交运行", "success");
    await refreshOperations();
    await refreshLog();
  } catch (error) {
    setMessage(error instanceof Error ? error.message : String(error), "error");
  }
}

// Both preset moves need a second click within 4 s: the arms travel a long way.
const armedMoves = new Map();

function moveTo(button, url, label, speed = 25) {
  if (current && ["starting", "running", "stopping"].includes(current.state)) {
    setMessage(`策略运行中，先停止再${label}`, "error");
    return;
  }
  const armed = armedMoves.has(url);
  if (!armed) {
    armedMoves.set(url, true);
    if (button) button.textContent = `确认${label}`;
    setMessage(`再点一次「确认${label}」：两臂会以 ${speed}% 速度移动到${label}姿态。`, "warn");
    window.setTimeout(() => {
      armedMoves.delete(url);
      if (button) button.textContent = label;
    }, 4000);
    return;
  }
  armedMoves.delete(url);
  if (button) button.textContent = label;
  setMessage(`正在${label}…`);
  void (async () => {
    try {
      const response = await fetch(withToken(url), {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify({ speed_percent: speed }),
      });
      const data = await response.json().catch(() => null);
      if (!response.ok) throw new Error(data?.detail || `HTTP ${response.status}`);
      const residual = Number(data?.residual_rad ?? 0);
      const worst = residual > 0.004 ? `，最大偏差 ${(residual * 180 / Math.PI).toFixed(1)}°` : "";
      setMessage(`已到${label}（${Number(data.seconds).toFixed(1)} s${worst}）`,
        residual > 0.004 ? "warn" : "success");
    } catch (error) {
      setMessage(`${label}失败：${error instanceof Error ? error.message : String(error)}`, "error");
    }
  })();
}

const homeRobot = () => moveTo(ui.policyHome, "/api/v1/system/home", "回参考位");
const readyRobot = () => moveTo(ui.policyReady, "/api/v1/system/ready", "准备位");

async function stopPolicy() {
  setMessage("正在停止…");
  try {
    await stopOperation("policy");
    setMessage("已停止", "success");
  } catch (error) {
    setMessage(error instanceof Error ? error.message : String(error), "error");
  }
  await refreshOperations();
  await refreshLog();
}


// Models: registry cards, mount/unmount by id, health polled while the page is open.

const MODELS_MS = 2000;
let modelTimer = undefined;
let modelList = [];
let selectedModelId = null;
let modelsPending = false;

function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>"']/g, (character) => (
    { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[character]
  ));
}

const MODEL_STATE_LABEL = { online: "在线", starting: "启动中…", offline: "离线" };

function selectedModel() {
  return modelList.find((model) => model.id === selectedModelId) || null;
}

async function refreshModels() {
  if (modelsPending) return;
  modelsPending = true;
  try {
    const response = await fetch(withToken("/api/v1/policy/models"), { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    modelList = data.models || [];
    renderModels();
    updateModelHint();
  } catch (error) {
    if (ui.modelCards) {
      ui.modelCards.innerHTML =
        `<p class="model-empty">读取模型登记表失败：${escapeHtml(error instanceof Error ? error.message : error)}</p>`;
    }
  } finally {
    modelsPending = false;
  }
}

function renderModels() {
  if (!ui.modelCards) return;
  if (!modelList.length) {
    ui.modelCards.innerHTML = '<p class="model-empty">未登记模型（configs/models.toml）</p>';
    return;
  }
  ui.modelCards.innerHTML = modelList.map((model) => {
    const state = model.state || "offline";
    const selected = model.id === selectedModelId ? " selected" : "";
    const mountable = model.mountable !== false && state === "offline";
    return `<article class="model-card${selected}" data-model="${escapeHtml(model.id)}">
      <div class="model-card-head">
        <span class="model-dot" data-state="${escapeHtml(state)}"></span>
        <strong>${escapeHtml(model.name)}</strong>
        <em>${escapeHtml(MODEL_STATE_LABEL[state] || state)}</em>
      </div>
      <small class="model-url">${escapeHtml(model.url)}</small>
      <small class="model-vram">${escapeHtml(model.vram || "")}</small>
      <div class="model-actions">
        <button type="button" data-action="mount" ${mountable ? "" : "disabled"}
          title="${escapeHtml(model.note || "执行登记表里的挂载命令")}">挂载</button>
        <button type="button" data-action="unmount" ${state === "offline" ? "disabled" : ""}>卸载</button>
      </div>
    </article>`;
  }).join("");
  for (const card of ui.modelCards.querySelectorAll(".model-card")) {
    card.addEventListener("click", (event) => {
      if (event.target.closest("button")) return;
      selectModel(card.dataset.model);
    });
  }
  for (const button of ui.modelCards.querySelectorAll("button[data-action]")) {
    button.addEventListener("click", (event) => {
      event.stopPropagation();
      const card = button.closest(".model-card");
      if (!card) return;
      if (button.dataset.action === "mount") void mountModel(card.dataset.model);
      else void unmountModel(card.dataset.model);
    });
  }
}

function selectModel(modelId) {
  const model = modelList.find((item) => item.id === modelId);
  if (!model) return;
  selectedModelId = modelId;
  // A model is always a remote service: switching the mode first keeps the
  // address field visible and stops the old local SFT path from running.
  if (ui.policyMode && ui.policyMode.value !== "remote") {
    ui.policyMode.value = "remote";
    ui.policyMode.dispatchEvent(new Event("change"));
  }
  if (ui.policyPrompt && model.prompt) ui.policyPrompt.value = model.prompt;
  if (ui.policyPromptHistory) {
    const match = [...ui.policyPromptHistory.options].find((option) => option.value === model.prompt);
    ui.policyPromptHistory.value = match ? model.prompt : "";
  }
  if (ui.policyServer) ui.policyServer.value = model.url;
  if (ui.policyStart) {
    // The button names the selected model; keep the original label in the dataset.
    if (!ui.policyStart.dataset.idleText) {
      ui.policyStart.dataset.idleText = ui.policyStart.textContent.trim() || "运行策略";
    }
    ui.policyStart.textContent = `${ui.policyStart.dataset.idleText} · ${model.name}`;
  }
  setMessage(`已选择 ${model.name}（${MODEL_STATE_LABEL[model.state] || model.state}）`,
    model.state === "online" ? "info" : "warn");
  updateModelHint();
  renderModels();
}

/** Warn next to the run button when the chosen model cannot answer. */
function updateModelHint() {
  if (!ui.policyModelHint) return;
  const model = selectedModel();
  const offline = Boolean(model) && model.state !== "online";
  ui.policyModelHint.hidden = !offline;
  ui.policyModelHint.textContent = offline ? `${model.name} 当前离线，运行会连不上` : "";
}

async function mountModel(modelId) {
  setMessage("正在挂载模型…");
  try {
    const response = await fetch(withToken(`/api/v1/policy/models/${encodeURIComponent(modelId)}/mount`),
      { method: "POST" });
    const data = await response.json().catch(() => null);
    if (!response.ok) throw new Error(data?.detail || `HTTP ${response.status}`);
    setMessage(data.state === "online" ? "模型已在线" : "已发起挂载，启动中…（首次约 2–4 分钟）", "warn");
    selectedModelId = modelId;
    await refreshModels();
  } catch (error) {
    setMessage(`挂载失败：${error instanceof Error ? error.message : String(error)}`, "error");
  }
}

async function unmountModel(modelId) {
  setMessage("正在卸载模型…");
  try {
    const response = await fetch(withToken(`/api/v1/policy/models/${encodeURIComponent(modelId)}/unmount`),
      { method: "POST" });
    const data = await response.json().catch(() => null);
    if (!response.ok) throw new Error(data?.detail || `HTTP ${response.status}`);
    setMessage("模型已卸载", "success");
    await refreshModels();
  } catch (error) {
    setMessage(`卸载失败：${error instanceof Error ? error.message : String(error)}`, "error");
  }
}

function start() {
  if (statusTimer !== undefined) return;
  void pollStatus();
  void refreshOperations();
  void refreshLog();
  statusTimer = window.setInterval(pollStatus, STATUS_MS);
  operationsTimer = window.setInterval(refreshOperations, OPERATIONS_MS);
  void refreshModels();
  modelTimer = window.setInterval(refreshModels, MODELS_MS);
}

function stop() {
  if (statusTimer !== undefined) {
    window.clearInterval(statusTimer);
    statusTimer = undefined;
  }
  if (operationsTimer !== undefined) {
    window.clearInterval(operationsTimer);
    operationsTimer = undefined;
  }
  if (modelTimer !== undefined) {
    window.clearInterval(modelTimer);
    modelTimer = undefined;
  }
}

if (ui.policyStart) ui.policyStart.addEventListener("click", () => void runPolicy(false));
if (ui.policyConfirmStart) ui.policyConfirmStart.addEventListener("click", () => void runPolicy(true));
if (ui.policyCancel) ui.policyCancel.addEventListener("click", () => showConfirm(false));
if (ui.policyStop) ui.policyStop.addEventListener("click", () => void stopPolicy());
if (ui.policyRefreshLog) ui.policyRefreshLog.addEventListener("click", () => void refreshLog());
document.addEventListener("visibilitychange", () => (document.hidden ? stop() : start()));
window.addEventListener("pagehide", stop);
start();

const SERVER_KEY = "tia.policy.server";
try {
  const saved = window.localStorage.getItem(SERVER_KEY);
  // The loopback address only works through an SSH tunnel; never restore it
  // automatically, otherwise a stale entry looks like the server is down.
  if (saved && saved !== "ws://127.0.0.1:8000" && ui.policyServer) ui.policyServer.value = saved;
} catch {
  /* storage unavailable: keep the built-in default */
}

ui.policyMode?.addEventListener("change", syncModeFields);
ui.policyHome?.addEventListener("click", homeRobot);
ui.policyReady?.addEventListener("click", readyRobot);

ui.policyPromptHistory?.addEventListener("change", () => {
  const value = ui.policyPromptHistory.value;
  if (value && ui.policyPrompt) ui.policyPrompt.value = value;
});
syncModeFields();
