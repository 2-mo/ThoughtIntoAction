import { defineComponent } from "vue";

import { withToken } from "../api/control-protocol";
import template from "./nav.html?raw";

const ROUTES = new Set(["/", "/control", "/collection", "/policy"]);

function normalize(path: string): string {
  const trimmed = path.replace(/\/+$/, "");
  if (trimmed === "" || trimmed === "/index.html") return "/";
  return trimmed;
}

export default defineComponent({
  name: "AppNav",
  template,
  mounted() {
    const current = normalize(window.location.pathname);
    const active = ROUTES.has(current) ? current : "/";
    const root = this.$el as Element;
    for (const element of root.querySelectorAll("a[data-route]")) {
      const link = element as HTMLAnchorElement;
      link.href = withToken(link.dataset.route ?? "/");
      if (link.hasAttribute("data-nav") && link.dataset.route === active) {
        link.setAttribute("aria-current", "page");
      }
    }
  },
});
