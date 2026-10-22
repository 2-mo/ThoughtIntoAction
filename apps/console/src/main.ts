import "@fortawesome/fontawesome-free/css/all.min.css";
import { createApp, defineComponent } from "vue";

import collectionShell from "./features/collection/shell.html?raw";
import controlShell from "./features/control/shell.html?raw";


const collectionPage = window.location.pathname === "/collection";
const shell = collectionPage ? collectionShell : controlShell;

const Page = defineComponent({
  name: collectionPage ? "CollectionPage" : "ControlConsolePage",
  template: shell,
  async mounted() {
    if (collectionPage) {
      await import("./styles/collection.css");
      await import("./features/collection/runtime");
      return;
    }
    await import("./styles/console.css");
    await import("./features/system/statusbar-tooltip");
    await import("./features/control/runtime");
    await import("./features/scene/runtime");
    await import("./features/camera/runtime");
    await import("./features/teleoperation/runtime");
  },
});

document.title = collectionPage
  ? "知行有灵 · Thought into Action · GR00T 数据采集"
  : "知行有灵 · Thought into Action · 控制台";
createApp(Page).mount("#app");
