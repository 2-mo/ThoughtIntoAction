// @ts-nocheck
// Policy task page: read-only kernel status plus whitelisted policy runs.
import { withToken } from "../../api/control-protocol";
import { listOperations, operationLog, startOperation, stopOperation } from "../../api/operations";

const STATUS_MS = 500;
const OPERATIONS_MS = 2000;

const ui = Object.fromEntries(
  [
    "policyKernel", "policyState", "policyPrompt", "policyKind", "policySeconds",
    "policyConfirm", "policyConfirmStart", "policyCancel", "policyStart",
    "policyStop", "policyMessage", "policyRun", "policyRuntime", "policyOwner",
    "policyAge", "policyOps", "policyLog", "policyRefreshLog",
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
    await startOperation("policy", {
      prompt,
      policy: ui.policyKind?.value || "sft",
      seconds: Number(ui.policySeconds?.value || 60),
      confirm: true,
    });
    setMessage("已提交运行", "success");
    await refreshOperations();
    await refreshLog();
  } catch (error) {
    setMessage(error instanceof Error ? error.message : String(error), "error");
  }
}

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

function start() {
  if (statusTimer !== undefined) return;
  void pollStatus();
  void refreshOperations();
  void refreshLog();
  statusTimer = window.setInterval(pollStatus, STATUS_MS);
  operationsTimer = window.setInterval(refreshOperations, OPERATIONS_MS);
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
}

if (ui.policyStart) ui.policyStart.addEventListener("click", () => void runPolicy(false));
if (ui.policyConfirmStart) ui.policyConfirmStart.addEventListener("click", () => void runPolicy(true));
if (ui.policyCancel) ui.policyCancel.addEventListener("click", () => showConfirm(false));
if (ui.policyStop) ui.policyStop.addEventListener("click", () => void stopPolicy());
if (ui.policyRefreshLog) ui.policyRefreshLog.addEventListener("click", () => void refreshLog());
document.addEventListener("visibilitychange", () => (document.hidden ? stop() : start()));
window.addEventListener("pagehide", stop);
start();
