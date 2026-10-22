"use strict";

(() => {
  const repo = "https://github.com/2-mo/ThoughtIntoAction/";
  const source = (path, tree = false) => `${repo}${tree ? "tree" : "blob"}/main/${path}`;
  const modules = {
    applications: { title: "应用与工作台", path: "src/embodied/applications/", description: "组织操作者的工作流：服务启停、动作程序、网页交互和手眼标定。界面处理交互，动作能力由公共接口提供。", interface: "Robot / CLI · 工作台装配 · 控制台与监看", status: "Python 示例和动作 CLI 已共用 Robot；网页控制台仍有待迁移的控制流程。", file: "src/embodied/applications/operations/robot.py" },
    inputs: { title: "输入与遥操", path: "src/embodied/devices/inputs/", description: "将人的操作转成控制意图。Quest 原生链路与手机 RPC 来源在适配端处理坐标、追踪质量与按键。", interface: "NativeDevice / NativeRouting / CommandIngress", status: "已接 Quest 与 iPhone。新输入设备需适配原生 ABI 或统一目标协议，并非登记名称即可使用。", file: "src/embodied/devices/inputs/phone.py" },
    contracts: { title: "共享接口与生命周期", path: "src/embodied/contracts/", description: "定义模块之间如何说话：控制请求、只读观测、设备所有权和组件注册。上层通过契约协作，不重复创建硬件连接。", interface: "ControlSession / RuntimeReader · Registry · DeviceLease", status: "控制与观测接口已供动作、采集、标定、策略使用；具体设备能力仍由实现决定。", file: "src/embodied/contracts/interfaces.py" },
    motion: { title: "独立运动算法", path: "src/embodied/motion/", description: "根据目标和逐轴约束计算轨迹。关节时间参数化、笛卡尔几何与路径算法独立维护，不拥有设备 SDK。", interface: "目标 + 状态 + 约束 → q / dq / ddq 与时间", status: "MoveJ 已复用 Ruckig；MoveL 仍依赖后端 IK。MoveP 只有离线几何示例，尚未接入内核执行。", file: "src/embodied/motion/offline.py" },
    data: { title: "采集与数据", path: "src/embodied/data/", description: "读取内核反馈与共享相机帧，写入可追溯的 episode，再进行对齐、检查、回放和格式导出。", interface: "RuntimeReader + SharedCamera → episode → 导出适配", status: "提供 LeRobot / GR00T 数据适配。采集不控制机械臂，Robot.record() 默认只记录无相机轨迹。", file: "src/embodied/data/capture/capture.py" },
    policy: { title: "模型策略", path: "src/embodied/policy/", description: "把状态与图像转成模型观察，在独立环境中推理，再将模型输出适配成内核动作目标。", interface: "观察 + 任务 → PolicyWorker → 动作块 → ControlSession", status: "已有 π0.5 / GR00T 适配。当前 OpenPI 内核入口提交流式 joint 目标，不经过 MoveJ 的 Ruckig 链。", file: "src/embodied/policy/kernel_runner.py" },
    core: { title: "平台基础", path: "src/embodied/core/", description: "集中维护平台配置、路径、坐标变换与遥测工具；具体型号的模型和 SDK 归设备适配器。", interface: "项目配置 / 坐标变换 / 基础遥测工具", status: "X5 的 MuJoCo 与 Pinocchio 模型由设备模块维护。公共基础设施不承担设备生命周期。", file: "src/embodied/core/project.py" },
    control: { title: "控制运行时", path: "src/embodied/control/", description: "接收统一目标，调度动作并发布反馈。公共 runtime 通过设备插件连接机械臂；SDK 和型号运动学由对应适配器维护。", interface: "commands.sock / observer.sock · RobotBackend", status: "共用 runtime 已分离设备实现，现有完整后端为 ARX X5 真机与 MuJoCo；新型号仍需 profile、模型和后端适配。", file: "src/embodied/devices/robots/arx5/plugin.py" },
    sensors: { title: "独立相机服务", path: "src/embodied/devices/sensors/", description: "相机服务统一拥有设备。采集、标定与监看消费共享帧，驱动与消费者分开，图像不经过机械臂 SDK。", interface: "Camera · camera_device 工厂 · SharedCamera", status: "可按 driver 选择 RealSense 或 synthetic。新硬件实现图像、时间戳与内参契约后接入同一服务。", file: "src/embodied/devices/sensors/factories.py" },
  };
  const flows = {
    control: { description: "输入表达意图，控制内核执行动作，设备反馈回到平台。", nodes: ["applications", "inputs", "contracts", "motion", "control"], note: "MoveJ 复用独立运动算法，MoveL 另需机器人 IK。250 Hz 是当前 X5 profile 的控制频率，不是实测端到端性能。", steps: [
      ["Robot.movej(q)", "读取关节状态，组织目标与控制会话。", "src/embodied/applications/operations/robot.py"],
      ["preview / execute", "通过 commands.sock 预览并提交计划。", "src/embodied/contracts/runtime/control.py"],
      ["Ruckig 轨迹规划", "共享算法生成逐时刻 q / dq 与约束。", "src/embodied/motion/offline.py"],
      ["worker → 后端", "控制运行时按周期下发目标。", "src/embodied/control/worker.py"],
      ["反馈到位", "检查实际位置、速度与稳定时间，完成后保持。", "src/embodied/control/commands.py"],
    ] },
    data: { description: "控制反馈与相机帧独立产生，在采集端汇成同一份 episode。", nodes: ["control", "sensors", "contracts", "data"], note: "相机与机器人是两条独立数据流。数据对齐、质量检查和导出在平台侧完成；采集不会取得机械臂控制权。", steps: [
      ["observer.sock", "RuntimeReader 读取命令、反馈、来源与时序。", "src/embodied/control/observer.py"],
      ["相机共享帧", "SharedCamera 读取图像和设备时间戳。", "src/embodied/devices/sensors/client.py"],
      ["capture 汇集", "汇集运行 tick 与相机帧，保留缺口信息。", "src/embodied/data/capture/capture.py"],
      ["episode 写盘", "保存 MCAP、图像与运行元数据。", "src/embodied/data/episode/runtime_episode.py"],
      ["质量检查与导出", "按模型要求组织 LeRobot 等训练格式。", "src/embodied/data/export/lerobot.py"],
    ] },
    policy: { description: "模型消费状态和图像，输出动作目标；执行与反馈仍由内核负责。", nodes: ["sensors", "policy", "contracts", "control"], note: "这里展示 OpenPI kernel_runner 的实际路径。策略使用流式 joint 目标，不等同于 Robot.movej；模型需匹配机器人动作和相机配置。", steps: [
      ["读取观察", "获取关节状态与相机图像，检查时间差。", "src/embodied/policy/kernel_runner.py"],
      ["独立模型推理", "PolicyWorker 根据观察与任务生成动作块。", "src/embodied/policy/pi05_zeroshot/worker_client.py"],
      ["校验与动作映射", "检查运行 ID、维数与时效，适配 X5 动作。", "src/embodied/policy/kernel_runner.py"],
      ["流式 joint 目标", "通过 ControlSession 提交关节与夹爪目标。", "src/embodied/contracts/runtime/control.py"],
      ["反馈进入下一轮", "设备反馈更新观察，策略退出时请求 hold。", "src/embodied/policy/kernel_runner.py"],
    ] },
  };
  const examples = {
    movej: { title: "让第一个关节转动 0.1 rad。", detail: "MoveJ 输入绝对关节位置；末端路径由关节运动决定。", lines: ["q = robot.q", "q[0] += 0.1", "robot.movej(q)"], file: "control/movej.py" },
    movel: { title: "移动到当前点上方 20 mm。", detail: "XYZ 是基座系绝对位置，单位 m；只给位置时保持当前朝向。", lines: ["x, y, z = robot.xyz", "robot.movel(xyz=[x, y, z + 0.02])"], file: "control/movel.py" },
    gripper: { title: "打开夹爪，再指定开口宽度。", detail: "宽度单位 m。到达开度不等于已检测到抓住物体。", lines: ["robot.gripper.open()", "robot.gripper.move(0.04)"], file: "control/gripper.py" },
    teach: { title: "记住一个点，移动后再返回。", detail: "teach() 记录当前位置；不会开启自由拖动模式。", lines: ["point = robot.teach()", "robot.jog(joint=0, delta=0.1)", "robot.return_to(point)"], file: "control/teach_return.py" },
    record: { title: "记录三秒命令与反馈。", detail: "只读轨迹采集，不启动相机或运动。多相机采集使用 collect 工作流。", lines: ["episode = robot.record(", "    duration=3, task=\"观察机器人状态\"", ")", "print(episode)"], file: "data/record.py" },
  };
  const evidence = {
    scene: { image: "assets/scene.jpg", title: "先在仿真里，看见自己的动作。", description: "同一套控制内核与动作入口，反馈显示在独立监看视图中。", alt: "项目 MuJoCo 工作台真实截图：X5 机械臂、桌面与三维坐标轴", file: "docs/guides/monitoring.md" },
    pose: { image: "assets/pose.png", title: "把输入、目标与实际姿态放在一起。", description: "对照手柄姿态、映射目标和夹爪反馈，理解坐标关系；截图不是跟踪精度测量。", alt: "项目姿态界面真实截图：手柄、遥操目标与实际夹爪姿态对照", file: "docs/guides/viewer-camera-controls.md" },
    telemetry: { image: "assets/telemetry.png", title: "看见指令，也看见实际反馈。", description: "查看关节目标、位置、速度及相关遥测，帮助定位控制链路中的问题。", alt: "项目遥测界面真实截图：关节状态与控制信息", file: "docs/guides/monitoring.md" },
  };

  const reducedMotion = matchMedia("(prefers-reduced-motion: reduce)");
  let paused = false;
  let currentFlow = "control";
  let currentExample = "movej";
  let currentBackend = "sim";
  let platformScene = null;
  const sceneContainer = document.getElementById("platform-scene");
  const pauseButton = document.getElementById("motion-toggle");
  const escape = text => text.replace(/[&<>]/g, character => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[character]));

  function selectModule(id) {
    const module = modules[id];
    if (!module) return;
    document.querySelectorAll("[data-module]").forEach(button => button.setAttribute("aria-pressed", String(button.dataset.module === id)));
    for (const [key, value] of Object.entries({ title: module.title, path: module.path, description: module.description, interface: module.interface, status: module.status })) {
      document.getElementById(`module-${key}`).textContent = value;
    }
    document.getElementById("module-index").textContent = `${String(Object.keys(modules).indexOf(id) + 1).padStart(2, "0")} / 09`;
    document.getElementById("module-source").href = source(module.file);
    sceneContainer.dataset.selectedModule = id;
  }
  document.querySelectorAll("[data-module]").forEach(button => button.addEventListener("click", () => selectModule(button.dataset.module)));

  function selectFlow(id) {
    const flow = flows[id];
    if (!flow) return;
    currentFlow = id;
    document.querySelectorAll("[data-flow]").forEach(button => button.setAttribute("aria-pressed", String(button.dataset.flow === id)));
    document.querySelectorAll("[data-module]").forEach(button => button.classList.toggle("in-flow", flow.nodes.includes(button.dataset.module)));
    document.getElementById("hero-flow-description").textContent = flow.description;
    document.getElementById("path-note").textContent = flow.note;
    const list = document.getElementById("path-steps");
    list.replaceChildren(...flow.steps.map(([title, description, path], index) => {
      const li = document.createElement("li");
      const number = document.createElement("span");
      number.textContent = String(index + 1).padStart(2, "0");
      const link = document.createElement("a");
      link.href = source(path);
      const strong = document.createElement("strong");
      strong.textContent = title;
      link.append(strong);
      const detail = document.createElement("p");
      detail.textContent = description;
      li.append(number, link, detail);
      if (!paused && !reducedMotion.matches) li.animate([{ opacity: .15, transform: "translateY(6px)" }, { opacity: 1, transform: "none" }], { duration: 350, delay: index * 45, fill: "backwards" });
      return li;
    }));
    platformScene?.setFlow(id);
  }
  document.querySelectorAll("[data-flow]").forEach(button => button.addEventListener("click", () => selectFlow(button.dataset.flow)));

  function highlight(code) {
    const expression = /("[^"\n]*"|'[^'\n]*'|#.*$|\b(?:from|import|with|as)\b|\b\d+(?:\.\d+)?\b)/gm;
    let result = "", index = 0;
    for (const match of code.matchAll(expression)) {
      result += escape(code.slice(index, match.index));
      const token = match[0];
      const kind = token.startsWith("#") ? "comment" : /^["']/.test(token) ? "string" : /^\d/.test(token) ? "number" : "keyword";
      result += `<span class="token-${kind}">${escape(token)}</span>`;
      index = match.index + token.length;
    }
    return result + escape(code.slice(index));
  }
  function updateCode() {
    const example = examples[currentExample];
    const connection = currentBackend === "sim" ? "Robot.sim(viewer=True)" : 'Robot.real("outputs/runtime/control/<REAL_RUN>")';
    const code = `from embodied import Robot\n\nwith ${connection} as robot:\n${example.lines.map(line => "    " + line).join("\n")}`;
    document.getElementById("api-code").innerHTML = highlight(code);
    document.getElementById("code-lines").innerHTML = code.split("\n").map((_, index) => String(index + 1).padStart(2, "0")).join("<br>");
    document.getElementById("example-title").textContent = example.title;
    document.getElementById("example-detail").textContent = example.detail;
    document.getElementById("example-source").href = source(`src/examples/${example.file}`);
    const note = document.getElementById("backend-note");
    note.textContent = currentBackend === "sim"
      ? "仿真：自动创建 MuJoCo，退出 with 后关闭自己创建的内核；原生窗口需要桌面 DISPLAY。"
      : "真机：把 <REAL_RUN> 换成已启动的真机运行目录。连接时检查后端类型，不负责启动或使能硬件；退出时保留外部内核。";
    document.querySelectorAll("[data-example]").forEach(button => button.setAttribute("aria-pressed", String(button.dataset.example === currentExample)));
    document.querySelectorAll("[data-backend]").forEach(button => button.setAttribute("aria-pressed", String(button.dataset.backend === currentBackend)));
  }
  document.querySelectorAll("[data-example]").forEach(button => button.addEventListener("click", () => { currentExample = button.dataset.example; updateCode(); }));
  document.querySelectorAll("[data-backend]").forEach(button => button.addEventListener("click", () => { currentBackend = button.dataset.backend; updateCode(); }));

  document.querySelectorAll("[data-evidence]").forEach(button => button.addEventListener("click", () => {
    const item = evidence[button.dataset.evidence];
    const picture = document.getElementById("evidence-image");
    picture.src = item.image;
    picture.alt = item.alt;
    document.getElementById("evidence-title").textContent = item.title;
    document.getElementById("evidence-description").textContent = item.description;
    document.getElementById("evidence-source").href = source(item.file);
    document.querySelectorAll("[data-evidence]").forEach(other => other.setAttribute("aria-pressed", String(other === button)));
  }));

  document.querySelectorAll('[role="group"]').forEach(group => {
    const buttons = [...group.querySelectorAll("button")];
    group.addEventListener("keydown", event => {
      const index = buttons.indexOf(event.target);
      if (index < 0 || !["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
      event.preventDefault();
      let next = event.key === "Home" ? 0 : event.key === "End" ? buttons.length - 1 : (index + (event.key === "ArrowRight" ? 1 : -1) + buttons.length) % buttons.length;
      buttons[next].focus();
      buttons[next].click();
    });
  });

  const copyStatus = document.getElementById("copy-status");
  document.querySelectorAll("[data-copy]").forEach(button => button.addEventListener("click", async () => {
    const code = document.getElementById(button.dataset.copy).textContent;
    const original = button.innerHTML;
    try {
      if (navigator.clipboard?.writeText) await navigator.clipboard.writeText(code);
      else {
        const input = document.createElement("textarea");
        input.value = code;
        input.style.cssText = "position:fixed;left:-9999px;top:0";
        document.body.append(input);
        input.select();
        const copied = document.execCommand("copy");
        input.remove();
        button.focus({ preventScroll: true });
        if (!copied) throw new Error("clipboard unavailable");
      }
      button.textContent = "已复制 ✓";
      copyStatus.textContent = "代码已复制到剪贴板。";
    } catch {
      button.textContent = "请手动复制";
      copyStatus.textContent = "当前环境无法访问剪贴板，请选中代码手动复制。";
    }
    setTimeout(() => { button.innerHTML = original; }, 1800);
  }));

  function updateMotion() {
    const disabled = paused || reducedMotion.matches;
    document.body.classList.toggle("motion-enabled", !disabled && !document.hidden);
    document.body.classList.toggle("motion-paused", disabled);
    pauseButton.setAttribute("aria-pressed", String(disabled));
    pauseButton.textContent = reducedMotion.matches ? "低动态模式" : paused ? "启用动效" : "暂停动效";
    pauseButton.disabled = reducedMotion.matches;
    platformScene?.setReducedMotion(reducedMotion.matches);
    platformScene?.setPaused(disabled);
  }
  pauseButton.addEventListener("click", () => { paused = !paused; updateMotion(); });
  reducedMotion.addEventListener("change", updateMotion);
  document.addEventListener("visibilitychange", updateMotion);
  document.getElementById("scene-reset").addEventListener("click", () => platformScene?.resetView());

  const sectionObserver = new IntersectionObserver(entries => entries.forEach(entry => entry.target.classList.toggle("is-visible", entry.isIntersecting)), { threshold: .1 });
  document.querySelectorAll(".section").forEach(section => sectionObserver.observe(section));
  if (!reducedMotion.matches) {
    const revealObserver = new IntersectionObserver(entries => entries.forEach(entry => {
      if (entry.isIntersecting) { entry.target.classList.remove("reveal-pending"); revealObserver.unobserve(entry.target); }
    }), { threshold: .06 });
    document.querySelectorAll(".reveal").forEach(element => { element.classList.add("reveal-pending"); revealObserver.observe(element); });
  }
  const navigation = [...document.querySelectorAll(".site-header nav a")];
  let scheduled = false;
  function updatePosition() {
    const height = document.documentElement.scrollHeight - innerHeight;
    document.querySelector(".reading-progress span").style.width = `${height ? Math.min(100, scrollY / height * 100) : 0}%`;
    let current = "";
    document.querySelectorAll("main > section").forEach(section => { if (section.getBoundingClientRect().top < innerHeight * .45) current = section.id; });
    navigation.forEach(link => { if (link.hash === `#${current}`) link.setAttribute("aria-current", "location"); else link.removeAttribute("aria-current"); });
    scheduled = false;
  }
  addEventListener("scroll", () => { if (!scheduled) { scheduled = true; requestAnimationFrame(updatePosition); } }, { passive: true });
  addEventListener("resize", updatePosition);

  updateCode();
  selectModule("control");
  selectFlow("control");
  updateMotion();
  updatePosition();
  // The timeline also animates without WebGL, so its pause control stays usable.
  document.querySelector(".scene-controls").hidden = false;
  document.getElementById("scene-reset").hidden = true;

  async function startScene() {
    try {
      // Check support quietly; Three's constructor otherwise logs a WebGL error.
      if (!window.WebGL2RenderingContext) throw new Error("WebGL 2 unavailable");
      const probe = document.createElement("canvas");
      const context = probe.getContext("webgl2");
      if (!context) throw new Error("WebGL 2 unavailable");
      context.getExtension("WEBGL_lose_context")?.loseContext();
      const { createPlatformScene } = await import("./scene.js");
      platformScene = createPlatformScene(sceneContainer, {
        reducedMotion: reducedMotion.matches,
        onSelect(id) {
          selectModule(id);
          document.getElementById("hero-flow-description").textContent = `${modules[id].title} · ${modules[id].interface}`;
        },
      });
      platformScene.setFlow(currentFlow);
      updateMotion();
      sceneContainer.dataset.sceneState = "ready";
      document.getElementById("scene-reset").hidden = false;
      document.getElementById("scene-hint").textContent = "拖动旋转 · 点击模块探索 · 架构示意，非实时数据";
    } catch {
      sceneContainer.dataset.sceneState = "fallback";
      document.getElementById("scene-hint").textContent = "静态架构视图 · 下方保留完整模块与调用路径";
    }
  }
  startScene();
  addEventListener("pagehide", event => {
    if (!event.persisted) { platformScene?.dispose(); platformScene = null; }
    else platformScene?.setPaused(true);
  });
  addEventListener("pageshow", event => { if (event.persisted) updateMotion(); });
})();
