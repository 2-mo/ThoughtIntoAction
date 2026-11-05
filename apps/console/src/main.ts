import "@fortawesome/fontawesome-free/css/all.min.css";
import { createApp, defineComponent } from "vue";

import collectionShell from "./features/collection/shell.html?raw";
import controlShell from "./features/control/shell.html?raw";
import overviewShell from "./features/overview/shell.html?raw";
import policyShell from "./features/policy/shell.html?raw";
import AppNav from "./shell/nav";
import "./styles/shell.css";

interface PageDefinition {
  name: string;
  title: string;
  shell: string;
  styles: () => Promise<unknown>;
  runtime: () => Promise<unknown>;
}

const pages: Record<string, PageDefinition> = {
  "/": {
    name: "OverviewPage",
    title: "总览",
    shell: overviewShell,
    styles: () => import("./styles/overview.css"),
    runtime: () => import("./features/overview/runtime"),
  },
  "/control": {
    name: "ControlConsolePage",
    title: "控制台",
    shell: controlShell,
    styles: () => import("./styles/console.css"),
    async runtime() {
      await import("./features/system/statusbar-tooltip");
      await import("./features/control/runtime");
      await import("./features/scene/runtime");
      await import("./features/camera/runtime");
      await import("./features/teleoperation/runtime");
    },
  },
  "/collection": {
    name: "CollectionPage",
    title: "GR00T 数据采集",
    shell: collectionShell,
    styles: () => import("./styles/collection.css"),
    runtime: () => import("./features/collection/runtime"),
  },
  "/policy": {
    name: "PolicyPage",
    title: "策略",
    shell: policyShell,
    styles: () => import("./styles/policy.css"),
    runtime: () => import("./features/policy/runtime"),
  },
};

function currentPage(): PageDefinition {
  const trimmed = window.location.pathname.replace(/\/+$/, "");
  const path = trimmed === "" || trimmed === "/index.html" ? "/" : trimmed;
  return pages[path] ?? pages["/"];
}

const page = currentPage();
const Page = defineComponent({
  name: page.name,
  template: page.shell,
  async mounted() {
    await page.styles();
    await import("./features/system/capabilities");
    await page.runtime();
  },
});

const App = defineComponent({
  name: "AppRoot",
  components: { AppNav, Page },
  template: "<app-nav></app-nav><page></page>",
});

document.title = `知行有灵 · Thought into Action · ${page.title}`;
createApp(App).mount("#app");
