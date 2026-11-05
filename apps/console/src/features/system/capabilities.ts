// @ts-nocheck
// Read-only capability probe shared by every page: which cameras exist and whether
// the camera service is running. Pages adapt their layout from the body data attributes.
import { withToken } from "../../api/control-protocol";

async function load() {
  try {
    const response = await fetch(withToken("/api/v1/system/status"), { cache: "no-store" });
    if (!response.ok) return;
    if (!(response.headers.get("content-type") || "").includes("application/json")) return;
    const data = await response.json();
    const cameras = (Array.isArray(data.cameras) ? data.cameras : [])
      .filter((camera) => camera.enabled !== false)
      .map((camera) => camera.id);
    document.body.dataset.cameraSet = cameras.join(",");
    document.body.dataset.cameraService = data.camera_service ? "on" : "off";
    window.dispatchEvent(new CustomEvent("tia-capabilities", { detail: data }));
  } catch {
    // Static preview or backend unavailable: keep the full layout in place.
  }
}

window.__tiaCapabilitiesReady = load();
