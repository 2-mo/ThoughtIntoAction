// @ts-nocheck
export {};

const statusbar = document.querySelector(".app-statusbar");

if (statusbar) {
  const tooltip = document.createElement("div");
  tooltip.id = "statusbarTooltip";
  tooltip.className = "statusbar-tooltip";
  tooltip.setAttribute("role", "tooltip");
  tooltip.hidden = true;
  document.body.append(tooltip);

  let activeSource = null;

  function moveNativeTitle(element) {
    if (!(element instanceof HTMLElement) || !element.hasAttribute("title")) return;
    const text = element.getAttribute("title")?.trim();
    if (text) element.dataset.statusTooltip = text;
    else delete element.dataset.statusTooltip;
    element.removeAttribute("title");
    if (element === activeSource) showTooltip(element);
  }

  function tooltipSource(target) {
    if (!(target instanceof Element)) return null;
    const source = target.closest("[data-status-tooltip]");
    return source && statusbar.contains(source) ? source : null;
  }

  function positionTooltip(source) {
    const sourceRect = source.getBoundingClientRect();
    const tooltipRect = tooltip.getBoundingClientRect();
    const viewportMargin = 8;
    const gap = 7;
    const left = Math.min(
      window.innerWidth - tooltipRect.width - viewportMargin,
      Math.max(
        viewportMargin,
        sourceRect.left + (sourceRect.width - tooltipRect.width) / 2,
      ),
    );
    const preferredTop = sourceRect.top - tooltipRect.height - gap;
    const top = preferredTop >= viewportMargin
      ? preferredTop
      : sourceRect.bottom + gap;
    tooltip.style.left = `${Math.round(left)}px`;
    tooltip.style.top = `${Math.round(top)}px`;
  }

  function showTooltip(source) {
    if (source?.hasAttribute("data-status-tooltip-panel")) {
      hideTooltip();
      return;
    }
    const text = source?.dataset.statusTooltip?.trim();
    if (!text) return;
    activeSource = source;
    tooltip.textContent = text;
    tooltip.hidden = false;
    positionTooltip(source);
  }

  function hideTooltip() {
    activeSource = null;
    tooltip.hidden = true;
  }

  statusbar.querySelectorAll("[title]").forEach(moveNativeTitle);

  const titleObserver = new MutationObserver((records) => {
    records.forEach(({ target }) => moveNativeTitle(target));
  });
  titleObserver.observe(statusbar, {
    attributes: true,
    attributeFilter: ["title"],
    subtree: true,
  });

  statusbar.addEventListener("pointerover", (event) => {
    const source = tooltipSource(event.target);
    if (source && source !== tooltipSource(event.relatedTarget)) showTooltip(source);
  });
  statusbar.addEventListener("pointerout", (event) => {
    if (activeSource && tooltipSource(event.relatedTarget) !== activeSource) hideTooltip();
  });
  statusbar.addEventListener("focusin", (event) => {
    const source = tooltipSource(event.target);
    if (source) showTooltip(source);
  });
  statusbar.addEventListener("focusout", (event) => {
    if (activeSource && tooltipSource(event.relatedTarget) !== activeSource) hideTooltip();
  });
  statusbar.addEventListener("click", hideTooltip);
  statusbar.addEventListener("keydown", (event) => {
    if (event.key === "Escape") hideTooltip();
  });
  statusbar.addEventListener("pointerleave", hideTooltip);
  window.addEventListener("resize", () => {
    if (activeSource) positionTooltip(activeSource);
  });
}
