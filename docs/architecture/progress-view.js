'use strict';

(() => {
  const data = window.PROJECT_PROGRESS;
  const $ = id => document.getElementById(id);
  if (!data) {
    $('snapshot').textContent = '未找到进展快照，请在仓库根目录运行 python3 scripts/docs/build_architecture.py';
    return;
  }
  const escape = value => String(value).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const date = value => new Intl.DateTimeFormat('zh-CN', {timeZone:'Asia/Shanghai', year:'numeric', month:'2-digit', day:'2-digit'}).format(new Date(value));
  const repo = 'https://github.com/2-mo/ThoughtIntoAction';
  const commitUrl = sha => `${repo}/commit/${sha}`;
  const sourceUrl = (path, line) => `${repo}/blob/${data.head}/${path.split('/').map(encodeURIComponent).join('/')}${line ? '#L' + line : ''}`;
  const byId = new Map(data.modules.map(module => [module.id, module]));
  const authors = new Map(data.authors.map(author => [author.id, author]));
  const design = window.LAB_DESIGN;
  const deployment = window.LAB_DEPLOYMENT;
  const designStatus = {base:{label:'已有实现基础',color:'#528c76'},partial:{label:'部分复用 / 待补齐',color:'#7396aa'},planned:{label:'待建设',color:'#c79449'}};
  const state = {view:location.hash === '#deployment' ? 'deployment' : 'design', selected:null, designSelected:null, deploymentSelected:null, deploymentFocus:null, period:'all', author:'all', search:'', zoom:null, shown:40};
  const isOverview = () => ['design','deployment'].includes(state.view);
  let timer;

  function filteredCommits() {
    const since = state.period === 'all' ? -Infinity : Date.parse(data.generatedAt) - Number(state.period) * 86400000;
    const query = state.search.toLocaleLowerCase();
    return data.commits.filter(commit => {
      if (Date.parse(commit.date) < since || (state.author !== 'all' && commit.authorId !== state.author)) return false;
      const text = [commit.subject, commit.sha, commit.author, ...commit.coauthors, ...commit.files,
        ...commit.modules.map(id => byId.get(id).title), ...commit.taskIds, ...authors.get(commit.authorId).aliases].join(' ').toLocaleLowerCase();
      return !query || text.includes(query);
    });
  }

  function notify(message) {
    clearTimeout(timer);
    $('toast').textContent = message;
    $('toast').hidden = false;
    timer = setTimeout(() => { $('toast').hidden = true; }, 3500);
  }

  function metrics(commits) {
    const closed = data.tasks.filter(task => task.closed).length;
    const values = [
      ['计划已关闭', `${closed}<small>/ ${data.tasks.length}</small>`, '按计划条目计数，未按工作量加权'],
      ['计划未关闭', data.tasks.length - closed, '包含部分实现与待验收的条目'],
      ['变更提交', commits.length, `当前筛选 · 已排除 ${data.mergeCount} 条合并记录`],
      ['Git 提交作者', new Set(commits.map(commit => commit.authorId)).size, '同邮箱归并 · 保留历史署名'],
    ];
    $('metrics').innerHTML = values.map(([label,value,note], index) => `<div class="metric"><div class="metric-label">${label}</div><div class="metric-value">${value}</div><div class="metric-note">${note}</div>${index === 0 ? `<div class="metric-track"><span style="width:${data.tasks.length ? closed / data.tasks.length * 100 : 0}%"></span></div>` : ''}</div>`).join('');
  }

  const svgStyle = `text{font-family:"Noto Sans CJK SC","Microsoft YaHei",sans-serif;fill:#343a3d} .layer{fill:#fff;stroke:#bac1c5;stroke-width:1.6} .layer-title{font-size:24px;font-weight:600;text-anchor:middle} .node{cursor:pointer;outline:none} .surface{fill:#fff;stroke:#495256;stroke-width:1.8} .node:hover .surface,.node:focus-visible .surface{fill:#f5fafb;stroke:#5a8b9f;stroke-width:3} .node.selected .surface{fill:#f4f9fb;stroke:#4b8199;stroke-width:3} .node.dim{opacity:.35} .node-title{font-size:23px;font-weight:600;text-anchor:middle} .node-sub{font-size:17px;fill:#7b858b;text-anchor:middle} .node-foot{font-size:15px;fill:#7b858b;text-anchor:middle} .edge{fill:none;stroke:#59656b;stroke-width:1.7;marker-end:url(#arrow)} .edge-label{font-size:16px;fill:#737e84;text-anchor:middle;paint-order:stroke;stroke:#fff;stroke-width:7px;stroke-linejoin:round} .hint{font-size:16px;fill:#8b959b} .node-note{font-size:18px;fill:#67767d;text-anchor:middle}`;

  function layer(x,y,w,h,title,color='#c5dde7') {
    return `<rect class="layer" x="${x}" y="${y}" width="${w}" height="${h}"/><text class="layer-title" x="${x+w/2}" y="${y+39}">${title}</text><path d="M${x+30} ${y+57}H${x+w-30}" stroke="${color}" stroke-width="4"/>`;
  }
  function edge(path, label, x, y, dependency=false) {
    return `<path class="edge" d="${path}" ${dependency ? 'stroke-dasharray="6 5"' : ''}/>${label ? `<text class="edge-label" x="${x}" y="${y}">${label}</text>` : ''}`;
  }
  function node(module, commits) {
    const {x,y,width:w,height:h,plan} = module;
    const own = commits.filter(commit => commit.modules.includes(module.id));
    const compact = h < 120;
    const color = plan.total === 0 ? '#a1a8ad' : plan.closed === plan.total ? '#528c76' : '#c79449';
    const queryActive = state.search || state.author !== 'all' || state.period !== 'all';
    const count = plan.total ? `计划 ${plan.closed} / ${plan.total} 关闭` : '未关联正式计划';
    return `<g class="node ${state.selected === module.id ? 'selected' : ''} ${queryActive && !own.length ? 'dim' : ''}" data-module="${module.id}" tabindex="0" role="button" aria-label="${escape(module.title)}，${count}，${own.length} 次提交" transform="translate(${x} ${y})"><title>${escape(module.description)}</title><rect class="surface" width="${w}" height="${h}" rx="2"/><rect x="1" y="1" width="${w-2}" height="5" fill="${color}" opacity=".65"/><text class="node-title" x="${w/2}" y="${compact ? 30 : 39}">${escape(module.title)}</text><text class="node-sub" x="${w/2}" y="${compact ? 54 : 68}">${escape(module.subtitle)}</text>${module.id === 'control' ? `<text class="node-note" x="${w/2}" y="104">RPC 工作区 · 反馈与关节约束</text><text class="node-note" x="${w/2}" y="134">原生输入仲裁 · 只读观测</text>` : ''}<text class="node-foot" x="${w/2}" y="${h-18}">${count} · ${own.length} 提交</text>${!compact && module.id !== 'control' ? `<text class="node-foot" x="${w/2}" y="${h-43}">${own.length ? `${date(own[0].date)} · ${escape(authors.get(own[0].authorId).name)}` : '当前范围无提交记录'}</text>` : ''}</g>`;
  }

  const relationColors = {control:'#8993aa',data:'#58a398',dependency:'#9ba5b5',manual:'#b89a72'};
  const tones = {violet:'#8772ae',indigo:'#696caa',teal:'#4b948b',orange:'#b78b61',neutral:'#8493a4',blue:'#688cab',gray:'#83909a'};
  const overviewStyle = `
    .layer-band{fill:#fafbfc;stroke:#e0e5ed;stroke-width:1}
    .layer-label{fill:#f0f3f7}
    .layer-band.indigo{fill:#f4f4fc;stroke:#d9dbee}
    .layer-label.indigo{fill:#eaeaf7}
    .layer-band.orange{fill:#fcfaf7}
    .layer-label.orange{fill:#f5eee6}
    .layer-band.gray{fill:#f3f5f7}
    .layer-id{font-size:14px;font-weight:650}
    .layer-name{font-size:19px;font-weight:600;fill:#344156}
    .layer-band.teal{fill:#f2f8f6;stroke:#cfe2db}
    .layer-label.teal{fill:#e5f0eb}
    .layer-purpose{font-size:13px;fill:#6c7b8e}
    .support-region{fill:#f8fafc;stroke:#e1e6ed;stroke-width:1;stroke-dasharray:4 4}
    .support-title{font-size:18px;font-weight:600;fill:#627387}
    .support-purpose{font-size:12px;fill:#758398}
    .overview-card{outline:none;cursor:pointer}
    .overview-card>.surface{fill:#fff;stroke:#e1e6ee;stroke-width:1}
    .overview-card.loop>.surface{fill:#eeeffa;stroke:#dcdef0}
    .overview-card.kernel>.surface{fill:#fffaf3;stroke:#e6d4bf;stroke-width:1.3}
    .overview-card.perception>.surface{fill:#f1f8f6;stroke:#dcebe6}
    .overview-card.driver>.surface{fill:#f5f7fa;stroke:#dce2e9}
    .overview-card.support>.surface{fill:#fff;stroke:#e5eaf0}
    .overview-card:hover>.surface,.overview-card:focus-visible>.surface,.overview-card.selected>.surface{stroke:#8a96b8;stroke-width:2;fill:#f2f4fa}
    .overview-title{font-size:19px;font-weight:600;fill:#364359}
    .overview-subtitle{font-size:13px;fill:#6d7b8e}
    .overview-scope{font-size:12px;fill:#6f7f93}
    .node-status{font-size:10px;fill:#a79eb0}
    .route{fill:none;stroke-width:1.5;stroke-linecap:round;stroke-linejoin:round}
    .route-label{font-size:12px;fill:#7c8797;text-anchor:middle;paint-order:stroke;stroke:#fff;stroke-width:4px}
    .loop-step{font-size:18px;font-weight:500;fill:#686c9b}
    .kernel-part{fill:#fff;stroke:#e9dccd;stroke-width:1}
    .kernel-text{font-size:13px;fill:#92785f;text-anchor:middle}
    .kernel-link{fill:none;stroke:#c8af92;stroke-width:1.1;marker-end:url(#arrow-manual)}
    .hardware-surface{fill:#e9edf1;stroke:none}
    .hardware-title{font-size:18px;font-weight:500;fill:#657383;text-anchor:middle}
    .diagram-muted{opacity:.36}
    .diagram-related>.surface{stroke:#8d98bb;stroke-width:1.6}
    .route-group.diagram-related .route{stroke-width:2.4}
    .overview-note{font-size:14px;fill:#758398}
  `;

  function designLayer(layer) {
    const {x,y,width:w,height:h,tone} = layer;
    return `<g class="design-layer" data-layer="${layer.id}"><rect class="layer-band ${tone}" x="${x}" y="${y}" width="${w}" height="${h}" rx="6"/><path class="layer-label ${tone}" d="M${x+6} ${y}H${x+230}V${y+h}H${x+6}Q${x} ${y+h} ${x} ${y+h-6}V${y+6}Q${x} ${y} ${x+6} ${y}"/><text class="layer-id" x="${x+16}" y="${y+h/2-3}" fill="${tones[tone]}">${layer.id}</text><text class="layer-name" x="${x+50}" y="${y+h/2-3}">${escape(layer.title)}</text><text class="layer-purpose" x="${x+50}" y="${y+h/2+20}" style="font-size:12px">${escape(layer.subtitle)}</text></g>`;
  }

  function supportRegion(region) {
    const {x,y,width:w,height:h}=region;
    return `<g><rect class="support-region" x="${x}" y="${y}" width="${w}" height="${h}" rx="6"/><text class="support-title" x="${x+18}" y="${y+24}">${escape(region.title)}</text><text class="support-purpose" x="${x+w-18}" y="${y+h-7}" text-anchor="end">${escape(region.subtitle)}</text></g>`;
  }

  function routeMarkup(route) {
    return `<g class="route-group" data-route="${route.id}"><path class="route" stroke="${relationColors[route.kind]}" d="${route.path}" marker-end="url(#arrow-${route.kind})"/>${route.label ? `<text class="route-label" x="${route.x}" y="${route.y}" ${route.rotate ? `transform="rotate(${route.rotate} ${route.x} ${route.y})"` : ''}>${escape(route.label)}</text>` : ''}</g>`;
  }

  function designNode(node) {
    const {x,y,width:w,height:h,kind}=node;
    let content='';
    if (kind === 'loop') {
      content=`<text class="overview-title" x="18" y="27">${escape(node.title)}</text>${['观测','规划','调用','复核'].map((label,i)=>`<text class="loop-step" x="${24+i*120}" y="65">${label}</text>${i<3 ? `<path d="M${75+i*120} 59h31m-5-4 5 4-5 4" fill="none" stroke="#b6bad5" stroke-width="1.2"/>` : ''}`).join('')}`;
    } else if (kind === 'kernel') {
      content=`<text class="overview-title" x="16" y="26">${escape(node.title)}</text><text x="${w-16}" y="26" text-anchor="end" font-size="11" fill="#bba389">非实时规划 + 本地周期控制</text><rect class="kernel-part" x="16" y="42" width="125" height="30" rx="3"/><text class="kernel-text" x="78.5" y="62">动作网关 / 会话</text><rect class="kernel-part" x="232" y="42" width="294" height="52" rx="3"/><text class="kernel-text" x="379" y="63" style="font-size:15px">命令调度 · 周期 worker</text><text class="kernel-text" x="379" y="83" style="font-size:12px">租约 / 来源仲裁 / 状态与约束</text><rect class="kernel-part" x="16" y="83" width="125" height="30" rx="3"/><text class="kernel-text" x="78.5" y="96" style="font-size:12px">轨迹规划</text><text class="kernel-text" x="78.5" y="109" style="font-size:9px">MoveJ / MoveL / Path</text><path class="kernel-link" d="M141 57H232"/><text class="kernel-text" x="186" y="50" style="font-size:10px">连续目标</text><path class="kernel-link" d="M78.5 72V83"/><path class="kernel-link" d="M141 98H192V84H232"/><text class="overview-subtitle" x="16" y="127" style="font-size:12px">读取状态 → 仲裁 / 执行 → 驱动读写 → RobotTick</text>`;
    } else if (kind === 'perception') {
      content=`<text class="overview-title" x="16" y="27">${escape(node.title)}</text><text class="overview-subtitle" x="16" y="50">${escape(node.subtitle)}</text><text x="16" y="84" font-size="17" fill="#638e85">采集 → 校验 → 发布</text><text class="overview-subtitle" x="16" y="119">订阅 · 时间质量 · 生命周期</text>`;
    } else if (kind === 'policy-stage') {
      content=`<text class="overview-title" x="16" y="25">${escape(node.title)}</text><text class="overview-subtitle" x="16" y="49">${escape(node.subtitle)}</text><text class="overview-scope" x="16" y="72">${escape(node.scope)}</text>`;
    } else if (kind === 'support') {
      content=`<text class="overview-title" x="16" y="26">${escape(node.title)}</text><text class="overview-subtitle" x="16" y="51" style="font-size:14px">${escape(node.subtitle)}</text><text class="overview-scope" x="16" y="${h-13}">${escape(node.scope)}</text>`;
    } else {
      const tall=h>80;
      content=`<text class="overview-title" x="16" y="${tall ? 29 : 22}" style="font-size:${kind === 'driver' ? 17 : 18}px">${escape(node.title)}</text><text class="overview-subtitle" x="16" y="${tall ? 57 : h-9}" style="font-size:12px">${escape(node.subtitle)}</text>${node.id === 'voice-interaction' ? `<text class="node-status" x="${w-12}" y="21" text-anchor="end">规划</text>` : ''}${node.id === 'gesture-interaction' ? `<text class="node-status" x="${w-12}" y="21" text-anchor="end">已有基础</text>` : ''}`;
    }
    return `<g class="node overview-card ${kind} ${state.designSelected === node.id ? 'selected' : ''}" data-design="${node.id}" tabindex="0" role="button" aria-label="${escape(node.title)}，${escape(node.layer)}，查看职责、接口与实现现状" transform="translate(${x} ${y})"><title>${escape(node.description)}</title><rect class="surface" width="${w}" height="${h}" rx="4"/>${content}</g>`;
  }

  function highlightRelations(id) {
    const connected=new Set([id]);
    design.edges.filter(e=>e.from===id||e.to===id).forEach(e=>{connected.add(e.from);connected.add(e.to);});
    $('diagram').querySelectorAll('[data-design]').forEach(element=>{
      element.classList.toggle('diagram-muted',Boolean(id)&&!connected.has(element.dataset.design));
      element.classList.toggle('diagram-related',Boolean(id)&&connected.has(element.dataset.design));
    });
    $('diagram').querySelectorAll('[data-route]').forEach(element=>{
      const route=design.routes.find(route=>route.id===element.dataset.route);
      element.classList.toggle('diagram-muted',Boolean(id)&&!route.nodes.some(node=>connected.has(node)));
      element.classList.toggle('diagram-related',Boolean(id)&&route.nodes.includes(id));
    });
  }

  function drawDesign() {
    const markers=Object.entries(relationColors).map(([kind,color])=>`<marker id="arrow-${kind}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0 0L10 5L0 10Z" fill="${color}"/></marker>`).join('');
    $('diagramMount').innerHTML=`<svg id="diagram" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${design.width} ${design.height}" role="group" aria-labelledby="diagramTitle diagramDesc"><title id="diagramTitle">${escape(design.title)}</title><desc id="diagramDesc">八层项目架构：交互呈现、应用服务、中央 Agent 智能编排、能力接入与契约、推理部署与策略、设备运行时、驱动适配、硬件设备。模型与数据按接口支撑各层，网络、安全、插件装配和运维横向提供基础设施。人工连续控制绕过 Agent，但仍通过控制会话与设备运行时。</desc><style>${svgStyle}${overviewStyle}</style><defs>${markers}</defs><rect width="${design.width}" height="${design.height}" fill="#fff"/>${design.layers.map(designLayer).join('')}${design.supportRegions.map(supportRegion).join('')}${design.routes.map(routeMarkup).join('')}${design.nodes.map(designNode).join('')}${design.hardware.map(h=>`<g class="hardware-item"><rect class="hardware-surface" x="${h.x}" y="${h.y}" width="${h.width}" height="${h.height}" rx="3"/><text class="hardware-title" x="${h.x+h.width/2}" y="${h.y+28}">${escape(h.title)}</text></g>`).join('')}${design.annotations.map(a=>`<text class="overview-note" x="${a.x}" y="${a.y}" style="font-size:${a.size}px">${escape(a.text)}</text>`).join('')}<text class="overview-note" x="1872" y="882" text-anchor="end" style="font-size:13px">目标架构 · 具体实现与验收状态见详情</text></svg>`;
    $('diagram').querySelectorAll('[data-design]').forEach(element=>{
      const select=()=>{state.designSelected=element.dataset.design;drawDesign();openDesignDetail();};
      element.addEventListener('click',select);
      element.addEventListener('keydown',event=>{if(event.key==='Enter'||event.key===' '){event.preventDefault();select();}});
      element.addEventListener('mouseenter',()=>highlightRelations(element.dataset.design));
      element.addEventListener('mouseleave',()=>highlightRelations(null));
      element.addEventListener('focus',()=>highlightRelations(element.dataset.design));
      element.addEventListener('blur',()=>highlightRelations(null));
    });
    resize();
  }

  function drawDeployment() {
    const styles = `
      .deploy-panel{fill:#fcfdff;stroke:#e0e6ee;stroke-width:1}
      .deploy-panel-title{font-size:19px;font-weight:600;fill:#41566e}
      .deploy-panel-note{font-size:13px;fill:#75869a}
      .deployment-card{cursor:pointer;outline:none}
      .deployment-card>.surface{fill:#fff;stroke:#d9e1eb;stroke-width:1}
      .deployment-card.hub>.surface{fill:#f3f2fb;stroke:#d4d5e8}
      .deployment-card.compute>.surface{fill:#eff7f4;stroke:#cee2d8}
      .deployment-card.router>.surface{fill:#eaf0f8;stroke:#c2d1e2}
      .deployment-card.service>.surface{fill:#f5f7fa;stroke:#d8e0e9;stroke-dasharray:4 3}
      .deployment-card:hover>.surface,.deployment-card:focus-visible>.surface{stroke:#7d8ba8;stroke-width:2}
      .deploy-label{font-size:11px;fill:#7b8aa0}
      .deploy-title{font-size:19px;font-weight:600;fill:#34445b}
      .deploy-subtitle{font-size:13px;fill:#6d7d90}
      .deploy-item{font-size:14px;fill:#4c627c}
      .deploy-foot{font-size:12px;fill:#738397}
      .physical-cable{fill:none;stroke:#92aac6;stroke-width:1.5;stroke-linejoin:round}
      .physical-cable.backhaul{stroke:#7193b9;stroke-width:2}
      .physical-cable.device{stroke:#b6a58a}
      .cable-label{font-size:12px;fill:#6d8299;text-anchor:middle;paint-order:stroke;stroke:#fcfdff;stroke-width:5px}
      .hardware-box{fill:#f3f0ea;stroke:#e5ded2;stroke-width:1}
      .workflow-node{cursor:pointer;outline:none}
      .workflow-node:hover .workflow-card,.workflow-node:focus-visible .workflow-card{stroke:#7d8ba8;stroke-width:2}
      .workflow-card{fill:#fff;stroke:#d9e1eb;stroke-width:1}
      .workflow-card.hub{fill:#f3f2fb;stroke:#d4d5e8}
      .workflow-card.compute{fill:#eff7f4;stroke:#cee2d8}
      .workflow-card.device{fill:#fcf6ee;stroke:#e5d7c5}
      .workflow-group{fill:#f2f8f5;stroke:#bad7c8;stroke-width:1;stroke-dasharray:5 4}
      .workflow-link{fill:none;stroke-width:1.5;stroke-linejoin:round}
      .workflow-label{font-size:12px;fill:#6e7f93;paint-order:stroke;stroke:#fcfdff;stroke-width:5px}
    `;
    const cards=deployment.nodes.map(n=>{
      let content;
      if(n.kind==='router') {
        content=`<text class="deploy-title" x="16" y="25">${escape(n.title)}</text><text class="deploy-subtitle" x="16" y="47">${escape(n.subtitle)}</text>`;
      } else if(n.kind==='service') {
        content=`<text class="deploy-title" x="16" y="23" style="font-size:16px">${escape(n.title)}</text><text class="deploy-foot" x="150" y="23">${escape(n.subtitle)}</text><text class="deploy-foot" x="16" y="44">${escape(n.footer)}</text>`;
      } else {
        content=`<text class="deploy-label" x="16" y="21">${escape(n.label)}</text><text class="deploy-title" x="16" y="47">${escape(n.title)}</text>${n.rows.map((row,i)=>`<text class="deploy-item" x="16" y="${72+i*21}">${escape(row)}</text>`).join('')}<text class="deploy-foot" x="16" y="${n.height-13}">${escape(n.footer)}</text>`;
      }
      return `<g class="node deployment-card ${n.kind}" data-deployment="${n.id}" transform="translate(${n.x} ${n.y})" tabindex="0" role="button" aria-label="${escape(n.title)}，查看部署职责与建设边界"><title>${escape(n.description)}</title><rect class="surface" width="${n.width}" height="${n.height}" rx="5"/>${content}</g>`;
    }).join('');
    const colors={model:'#7c9e93',task:'#8988ad',policy:'#b29874',target:'#b29874'};
    const markers=Object.entries(colors).map(([kind,color])=>`<marker id="deploy-${kind}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0 0L10 5L0 10Z" fill="${color}"/></marker>`).join('');
    const panels=deployment.panels.map(p=>`<rect class="deploy-panel" x="${p.x}" y="${p.y}" width="${p.width}" height="${p.height}" rx="7"/><text class="deploy-panel-title" x="${p.x+24}" y="${p.y+30}">${escape(p.title)}</text><text class="deploy-panel-note" x="${p.x+24}" y="${p.y+52}">${escape(p.subtitle)}</text>`).join('');
    const cables=deployment.cables.map(c=>`<g data-cable="${c.from}:${c.to}"><path class="physical-cable ${c.kind}" d="${c.path}"/>${c.label?`<text class="cable-label" x="${c.x}" y="${c.y}">${escape(c.label)}</text>`:''}</g>`).join('');
    const hardware=deployment.hardware.map(h=>`<g class="deployment-hardware"><rect class="hardware-box" x="${h.x}" y="${h.y}" width="${h.width}" height="${h.height}" rx="4"/><text class="deploy-title" x="${h.x+h.width/2}" y="${h.y+25}" text-anchor="middle" style="font-size:16px">${escape(h.title)}</text><text class="deploy-foot" x="${h.x+h.width/2}" y="${h.y+45}" text-anchor="middle">${escape(h.subtitle)}</text></g>`).join('');
    const workflow=deployment.workflow;
    const groups=(workflow.groups||[]).map(g=>`<rect class="workflow-group" x="${g.x}" y="${g.y}" width="${g.width}" height="${g.height}" rx="6"/><text class="deploy-title" x="${g.x+16}" y="${g.y+23}" style="font-size:17px">${escape(g.title)}</text>`).join('');
    const flowLinks=workflow.links.map(l=>`<path class="workflow-link" stroke="${colors[l.kind]}" d="${l.path}" marker-start="url(#deploy-${l.kind})" marker-end="url(#deploy-${l.kind})"/><text class="workflow-label" x="${l.x}" y="${l.y}" ${l.rotate?`text-anchor="middle" transform="rotate(${l.rotate} ${l.x} ${l.y})"`:l.center?'text-anchor="middle"':''}>${escape(l.label)}</text>`).join('');
    const flowNodes=workflow.nodes.map(n=>`<g class="node workflow-node" data-workflow="${n.id}" tabindex="0" role="button" aria-label="${escape(n.title)}，查看职责和实现边界"><rect class="workflow-card ${n.kind}" x="${n.x}" y="${n.y}" width="${n.width}" height="${n.height}" rx="5"/><text class="deploy-title" x="${n.x+18}" y="${n.y+27}" style="font-size:18px">${escape(n.title)}</text><text class="deploy-item" x="${n.x+18}" y="${n.y+49}">${escape(n.subtitle)}</text><text class="deploy-foot" x="${n.x+18}" y="${n.y+n.height-12}">${escape(n.footer)}</text></g>`).join('');
    $('diagramMount').innerHTML=`<svg id="diagram" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${deployment.width} ${deployment.height}" role="group" aria-labelledby="diagramTitle diagramDesc"><title id="diagramTitle">${escape(deployment.title)}</title><desc id="diagramDesc">目标架构：左侧是中兴 E2631 有线 Mesh 物理拓扑，主节点连接无 GPU Ubuntu 总台和两个子节点，子节点分别连接算力与设备接入机。右侧是服务协作：总台编排，推理部署层跨算力端和设备端，设备内核本地执行。RTC 与跨机接入尚待实现。</desc><style>${svgStyle}${styles}</style><defs>${markers}</defs><rect width="${deployment.width}" height="${deployment.height}" fill="#fff"/><text class="deploy-title" x="28" y="34" style="font-size:24px">一台总台，管理整个实验室</text><text class="deploy-subtitle" x="28" y="60">有线 Mesh 连接机器 · 总台编排任务 · 推理与设备控制分开部署</text><text class="deploy-foot" x="1872" y="35" text-anchor="end">目标设计 · 型号与网络方案由用户提供</text>${panels}${cables}${cards}${hardware}${groups}${flowLinks}${flowNodes}<text class="deploy-foot" x="62" y="326">三路 LAN：总台 + Mesh A + Mesh B</text><text class="deploy-foot" x="62" y="635">5090 也可兼任设备节点；</text><text class="deploy-foot" x="62" y="655">一台主机可连接多个设备。</text><text class="deploy-panel-note" x="52" y="750">同款中兴巡天 ZXHN E2631 · 原生 Mesh 模式 · 后台确认有线回程与主机互通</text><text class="deploy-foot" x="52" y="776">图示接入机走网线；移动机载电脑最后一跳可选 Wi-Fi。路由器型号不进入业务或设备插件。</text><text class="deploy-foot" x="28" y="837">实线：物理连接　箭头：服务协作　｜　节点可点击查看职责、接口与 Git 进展。</text><text class="deploy-foot" x="1872" y="862" text-anchor="end">RTC、跨机能力接入为待建设设计；现有启动仍以本机工作台为主。</text></svg>`;
    $('diagram').querySelectorAll('[data-deployment]').forEach(element=>{
      const select=()=>{state.deploymentFocus=element;state.deploymentSelected=element.dataset.deployment;openDeploymentDetail();};
      element.addEventListener('click',select);
      element.addEventListener('keydown',event=>{if(event.key==='Enter'||event.key===' '){event.preventDefault();select();}});
    });
    $('diagram').querySelectorAll('[data-workflow]').forEach(element=>{
      const select=()=>{
        state.deploymentFocus=element;
        if(element.dataset.workflow==='hub'){state.deploymentSelected='hub';openDeploymentDetail();}
        else{state.designSelected=element.dataset.workflow;openDesignDetail();}
      };
      element.addEventListener('click',select);
      element.addEventListener('keydown',event=>{if(event.key==='Enter'||event.key===' '){event.preventDefault();select();}});
    });
    resize();
  }

  function openDeploymentDetail() {
    const node=deployment.nodes.find(n=>n.id===state.deploymentSelected);
    $('designDetailContent').innerHTML=`<p class="eyebrow">目标部署 / ${escape(node.id.toUpperCase())}</p><h2 id="designDetailTitle">${escape(node.title)}</h2><p class="design-lead">${escape(node.description)}</p><h3>部署职责与通信边界</h3>${node.responsibilities.map(text=>`<p>${escape(text)}</p>`).join('')}<div class="warning-note">${escape(node.boundary)}</div><h3>相关架构模块与协作接口</h3><div class="chips">${node.modules.map(id=>`<button class="chip" data-deployment-module="${id}">${escape(design.nodes.find(n=>n.id===id).title)} →</button>`).join('')}</div><p class="detail-footnote">点击模块查看所属逻辑层、接口与 Git 贡献。<a href="lab-device-platform.md">多机部署与迁移依据 ↗</a></p>`;
    $('designDetailContent').querySelectorAll('[data-deployment-module]').forEach(button=>{
      button.onclick=()=>{state.designSelected=button.dataset.deploymentModule;openDesignDetail();};
    });
    $('designDetailDialog').showModal();
    $('designDetailDialog').scrollTop=0;
  }

  function openDesignDetail() {
    const node = design.nodes.find(node => node.id === state.designSelected);
    const layer = design.layers.find(layer=>layer.id === node.layer);
    const layerLabel = layer ? `${layer.id} · ${layer.title}` : '跨层支撑服务';
    const components = node.components.map(id => design.components.find(component => component.id === id));
    const relations = design.edges.filter(relation=>relation.from === node.id || relation.to === node.id || relation.via?.includes(node.id));
    const interfaces = relations.map(relation=>{
      const transit = relation.via?.includes(node.id);
      const other = relation.from === node.id ? relation.to : relation.from;
      const title = id=>design.nodes.find(n=>n.id === id).title;
      return `<div class="interface-row"><div><span class="interface-type ${relation.kind}">${{control:'调用',data:'数据',dependency:'支撑'}[relation.kind]}</span><button class="interface-link" data-neighbor="${other}">${transit ? `${escape(title(relation.from))} → ${escape(title(relation.to))}` : `${relation.both ? '↔' : relation.from === node.id ? '→' : '←'} ${escape(title(other))}`}</button></div><p>${escape(relation.detail)}</p></div>`;
    }).join('');
    $('designDetailContent').innerHTML = `<p class="eyebrow">${escape(layerLabel)} / ${escape(node.id.toUpperCase())}</p><h2 id="designDetailTitle">${escape(node.title)}</h2><p class="design-lead">${escape(node.description)}</p><h3>职责与分工边界</h3>${node.responsibilities.map(item => `<p>${escape(item)}</p>`).join('')}<h3>需要对接的接口</h3><div class="interface-list">${interfaces}</div><div class="warning-note">${escape(node.boundary)}</div><h3>实现细节与现状</h3>${components.map(component => `<details class="component-detail"><summary>${escape(component.title)}<span>${designStatus[component.status].label}</span></summary><p>${escape(component.description)}</p>${component.responsibilities.map(item => `<p>${escape(item)}</p>`).join('')}<p class="component-boundary">${escape(component.boundary)}</p></details>`).join('')}<h3>相关实现与 Git 贡献</h3>${!node.related.length ? '<p class="detail-footnote">尚无该模块的实现证据；不借用其他模块完成率。</p>' : ''}<div class="chips">${node.related.map(id => `<button class="chip" data-existing="${id}">${escape(byId.get(id).title)} →</button>`).join('')}</div><p class="detail-footnote">Git 署名用于追溯贡献，不代表已分配负责人。目标模块不继承旧模块完成率。<a href="lab-device-platform.md">完整设计说明 ↗</a></p>`;
    $('designDetailContent').querySelectorAll('[data-neighbor]').forEach(button=>{
      button.onclick=()=>{state.designSelected=button.dataset.neighbor;if(state.view==='design')drawDesign();openDesignDetail();};
    });
    $('designDetailContent').querySelectorAll('[data-existing]').forEach(button => {
      button.onclick = () => {
        state.selected = button.dataset.existing;
        $('designDetailDialog').close(); setView('architecture');
      };
    });
    if (!$('designDetailDialog').open) $('designDetailDialog').showModal();
    else $('closeDesignDetail').focus({preventScroll:true});
    $('designDetailDialog').scrollTop = 0;
  }

  function draw(commits) {
    $('diagramMount').innerHTML = `<svg id="diagram" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1540 1520" role="group" aria-labelledby="diagramTitle diagramDesc"><title id="diagramTitle">知行有灵 · 项目架构与进展</title><desc id="diagramDesc">按交互、应用、统一内核、设备、数据与策略分层。每个模块可查看计划关闭数和 Git 提交证据。</desc><style>${svgStyle}</style><defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0L10 5L0 10Z" fill="#59656b"/></marker></defs><rect width="1540" height="1520" fill="white"/><text x="40" y="65" font-size="34" font-weight="650">知行有灵 · 具身智能平台</text><text x="40" y="99" class="hint">整体设计 / 计划进展 / Git 贡献</text><text x="1500" y="94" class="hint" text-anchor="end">快照 ${data.head.slice(0,8)} · ${date(data.headDate)}</text>
      ${layer(40,140,300,650,'操作交互层')}
      ${layer(380,140,730,255,'应用与任务层')}
      ${layer(380,470,730,335,'统一控制内核')}
      ${layer(1160,140,340,455,'数据与资产层','#eccda5')}
      ${layer(1160,685,340,275,'模型与策略层','#eccda5')}
      ${layer(380,870,730,255,'设备适配与服务层')}
      ${layer(40,1170,1460,235,'工程与协作基础')}
      ${edge('M340 285H380','交互',360,272)}
      ${edge('M565 395V470','命令 / 任务',565,440)}
      ${edge('M925 470V395','反馈 / 结果',925,440)}
      ${edge('M340 660H380','输入',360,647)}
      ${edge('M1110 530H1160','tick',1135,517)}
      ${edge('M1330 595V685','数据导出 / 训练',1330,648)}
      ${edge('M1160 820H1140V735H1110','动作',1135,800)}
      ${edge('M1080 1010H1130V330H1160','共享帧',1130,1000)}
      ${edge('M565 805V870','控制 / 反馈',565,846)}
      <text class="hint" x="70" y="880">图中箭头：主要控制与数据流</text><text class="hint" x="70" y="916">色条：关联计划的关闭状态</text><text class="hint" x="70" y="952">节点下方：当前筛选的 Git 活动</text><text class="hint" x="70" y="1012">统一内核真机链路待完整验收</text><text class="hint" x="70" y="1048">Reference7 仅为教学仿真</text>
      ${data.modules.map(module => node(module,commits)).join('')}
      <rect x="40" y="1440" width="1460" height="56" fill="#fafbfb" stroke="#bdc5ca"/><text x="770" y="1476" text-anchor="middle" font-size="20">设备与仿真　ARX X5　 /　MuJoCo · X5 / Reference7　 /　Quest　 /　iPhone　 /　RealSense</text></svg>`;
    $('diagram').querySelectorAll('[data-module]').forEach(element => {
      const select = () => { state.selected = element.dataset.module; draw(filteredCommits()); renderPanel(filteredCommits()); if (window.innerWidth < 800) $('detailPanel').scrollIntoView({behavior:'smooth', block:'start'}); };
      element.addEventListener('click', select);
      element.addEventListener('keydown', event => { if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); select(); $('diagram').querySelector(`[data-module="${state.selected}"]`).focus({preventScroll:true}); } });
    });
    resize();
  }

  function timeline(commits, limit=5) {
    return commits.slice(0,limit).map(commit => `<div class="timeline-item"><div class="timeline-date">${date(commit.date)} · ${commit.sha.slice(0,7)}</div><a class="timeline-title" href="${commitUrl(commit.sha)}" target="_blank" rel="noopener noreferrer">${escape(commit.subject)}</a><div class="timeline-author">${escape(commit.author)}${commit.coauthors.length ? ' · 共同署名 ' + commit.coauthors.map(escape).join('、') : ''}</div></div>`).join('') || '<p class="panel-copy">当前筛选下没有提交记录。</p>';
  }

  function taskMarkup(task) {
    const related = data.commits.filter(commit => commit.taskIds.includes(task.id));
    return `<details class="task ${task.closed ? 'closed' : ''}"><summary><code>${task.id}</code> ${escape(task.title.replace(/`/g,''))} <span class="sr-only">${task.closed ? '文档标记关闭' : '未关闭'}</span></summary><pre class="task-body">${escape(task.body)}</pre><a href="${sourceUrl(task.path,task.line)}" target="_blank" rel="noopener noreferrer">查看计划原文 · 第 ${task.line} 行 ↗</a>${related.length ? `<p>明确提及 ${task.id} 的提交：${related.map(commit => `<a href="${commitUrl(commit.sha)}" target="_blank" rel="noopener noreferrer">${commit.sha.slice(0,7)}</a>`).join(' · ')}</p>` : ''}</details>`;
  }

  function renderPanel(commits) {
    const module = byId.get(state.selected);
    if (!module) {
      $('detailPanel').innerHTML = `<div class="panel-label">PROJECT ACTIVITY</div><div class="panel-heading"><h2>项目动态</h2></div><p class="panel-copy">选择左侧模块，查看它的计划条目、提交作者，以及具体改动。</p><div class="section-label">最近变更 <small>按作者时间</small></div>${timeline(commits,6)}<button class="subtle-link" id="allCommits">查看全部提交 →</button><div class="warning-note">计划来自 docs/plans 的正式任务。未关闭不等于未开始，勾选关闭也不等于已通过真机验收。</div>`;
      $('allCommits').onclick = () => setView('commits');
      return;
    }
    const own = commits.filter(commit => commit.modules.includes(module.id));
    const tasks = data.tasks.filter(task => task.module === module.id).sort((a,b) => Number(a.closed)-Number(b.closed));
    const identities = [...new Set(own.map(commit => commit.authorId))];
    const {closed,total} = module.plan;
    $('detailPanel').innerHTML = `<div class="panel-label">MODULE / ${escape(module.id.toUpperCase())}</div><div class="panel-heading"><h2>${escape(module.title)}</h2><button id="clearModule" aria-label="返回项目动态">×</button></div><p class="panel-copy">${escape(module.description)}</p><div class="module-metrics"><strong>${closed} / ${total}</strong><span>计划条目已关闭</span><div class="progress-track"><span style="width:${total ? closed/total*100 : 0}%"></span></div></div><div class="section-label">Git 提交作者 <small>当前范围 ${own.length} 次变更</small></div><div class="chips">${identities.map(id => `<button class="chip" data-author="${id}">${escape(authors.get(id).name)} · ${own.filter(commit => commit.authorId === id).length}</button>`).join('') || '<span class="chip">当前范围暂无记录</span>'}</div><div class="section-label">最近做了什么</div>${timeline(own,4)}<button class="subtle-link" id="moduleCommits">查看此模块全部提交 →</button><div class="section-label">计划记录 <small>未关闭优先</small></div>${tasks.map(taskMarkup).join('') || '<p class="panel-copy">尚未关联正式计划条目。</p>'}<div class="warning-note">${escape(module.boundary)}</div><div class="section-label">模块路径 / 历史路径</div><div class="source-links">${module.paths.map(path => `<span>${escape(path)}</span>`).join('')}</div>`;
    $('clearModule').onclick = () => { state.selected = null; render(); };
    $('moduleCommits').onclick = () => setView('commits');
    $('detailPanel').querySelectorAll('[data-author]').forEach(button => { button.onclick = () => { state.author = button.dataset.author; $('author').value = state.author; render(); }; });
  }

  function commitMarkup(commit) {
    return `<details class="commit-row"><summary><span class="commit-date">${date(commit.date)}<br>${commit.sha.slice(0,8)}</span><span class="commit-title">${escape(commit.subject)}<span class="chips">${commit.modules.map(id => `<span class="chip">${escape(byId.get(id).title)}</span>`).join('') || '<span class="chip">未映射模块</span>'}</span></span><span class="commit-meta">${escape(commit.author)}<br>${commit.files.length} 个文件</span></summary><div class="commit-files"><a href="${commitUrl(commit.sha)}" target="_blank" rel="noopener noreferrer">打开 GitHub 提交 ↗</a><p>作者：${escape(commit.author)} · 提交者：${escape(commit.committer)}${commit.coauthors.length ? '<br>共同署名：' + commit.coauthors.map(escape).join('、') : ''}</p><ul>${commit.files.map(path => `<li>${escape(path)}</li>`).join('')}</ul></div></details>`;
  }

  function renderCommits(commits) {
    const own = state.selected ? commits.filter(commit => commit.modules.includes(state.selected)) : commits;
    $('commitsView').innerHTML = `<div class="list-intro"><h2>${state.selected ? escape(byId.get(state.selected).title) + ' · ' : ''}提交记录 <small>(${own.length})</small></h2><p>作者时间 · 排除合并提交${state.selected ? ' · <button class="subtle-link" id="clearCommitModule">取消模块筛选</button>' : ''}</p></div>${own.slice(0,state.shown).map(commitMarkup).join('') || '<div class="empty">当前筛选下没有提交。<br>试试全部历史、全部作者或清空搜索。</div>'}${own.length > state.shown ? '<button id="loadMore">再显示 40 条</button>' : ''}`;
    if ($('loadMore')) $('loadMore').onclick = () => { state.shown += 40; renderCommits(filteredCommits()); };
    if ($('clearCommitModule')) $('clearCommitModule').onclick = () => { state.selected = null; render(); };
  }

  function renderAuthors(commits) {
    const visible = data.authors.map(author => ({...author, own:commits.filter(commit => commit.authorId === author.id)})).filter(author => author.own.length);
    $('contributorsView').innerHTML = `<div class="list-intro"><h2>谁做了什么</h2><p>根据提交作者署名归并，不推定负责人或工作量。</p></div><div class="authors-grid">${visible.map(author => {
      const modules = [...new Set(author.own.flatMap(commit => commit.modules))];
      return `<article class="author-card"><div class="avatar">${escape(author.name.slice(0,2))}</div><h3>${escape(author.name)}</h3><p>历史署名：${author.aliases.map(escape).join(' / ')}</p><strong>${author.own.length}</strong> <span class="metric-label">次变更提交</span><p>最近活动 ${date(author.own[0].date)} · 涉及 ${modules.length} 个模块</p><div class="chips">${modules.map(id => `<span class="chip">${escape(byId.get(id).title)}</span>`).join('')}</div><button data-contributor="${author.id}">查看提交记录 →</button></article>`;
    }).join('')}</div>${visible.length ? '' : '<div class="empty">当前筛选下没有作者记录。</div>'}`;
    $('contributorsView').querySelectorAll('[data-contributor]').forEach(button => { button.onclick = () => { state.author = button.dataset.contributor; state.selected = null; $('author').value = state.author; setView('commits'); }; });
  }

  function resize() {
    const svg = $('diagram');
    if (!svg || $('architectureView').hidden) return;
    const {width,height} = svg.viewBox.baseVal;
    const fitWidth = Math.max(1, $('viewport').clientWidth - 16) / width;
    const fitHeight = Math.max(1, $('viewport').clientHeight - 16) / height;
    const fit = isOverview() ? Math.min(fitWidth,fitHeight) : fitWidth;
    const scale = state.zoom ?? fit;
    svg.style.width = `${width * scale}px`;
    svg.style.height = `${height * scale}px`;
    $('fit').textContent = state.zoom === null ? (isOverview() ? '全图' : '适宽') : Math.round(scale * 100) + '%';
    $('fit').title = isOverview() ? '完整适应画布宽高' : '适应画布宽度';
  }

  function setView(view) {
    state.view = view; state.shown = 40; state.zoom = null;
    history.replaceState(null,'',location.pathname+location.search+(view === 'deployment' ? '#deployment' : ''));
    render();
  }
  function render() {
    const commits = filteredCommits();
    const isDesign = state.view === 'design', isDeployment = state.view === 'deployment';
    document.body.classList.toggle('design-view',isOverview());
    $('metrics').hidden = isOverview();
    if (!isOverview()) metrics(commits);
    $('detailPanel').hidden = isOverview();
    $('architectureView').hidden = !['design','deployment','architecture'].includes(state.view);
    for (const view of ['commits','contributors']) $(view + 'View').hidden = state.view !== view;
    document.querySelector('.filters').hidden = isOverview();
    document.querySelector('.legend').innerHTML = isDeployment
      ? '<span>目标部署 · 点击节点查看职责、通信边界与架构模块</span>'
      : isDesign
      ? '<span>点击模块，查看职责、接口与实现进展</span>'
      : '<span><i class="green"></i>计划全部关闭</span><span><i class="amber"></i>仍有未关闭项</span><span><i class="gray"></i>无关联计划</span>';
    document.querySelector('.diagram-caption').textContent = isDeployment
      ? '左侧实线说明有线 Mesh 与设备接线，右侧箭头说明服务协作。L5 推理层跨算力与设备部署，RTC 采样与动作块执行器共同衔接；L6 保留本地控制。'
      : isDesign
      ? 'L1—L8 按职责分层；策略经 L5 推理与执行器，人工和确定性控制可直接调用 L6 设备能力。机器接线见“多机部署”。点击模块查看所属层、接口与 Git 进展。'
      : '点击模块查看计划、作者与提交。时间和作者筛选只影响 Git 活动，计划计数保持完整。';
    $('coverage').textContent = isDeployment
      ? '有线 Mesh · 无 GPU 总台 · 分布式推理与 RTC · 设备本地控制'
      : isDesign
      ? '八层架构 · 推理部署与 RTC · 横向基础设施'
      : `${data.modules.length} 个已有模块 · ${data.planFiles.length} 份正式计划 · ${data.shallow ? '浅克隆，仅含本地可用历史' : 'HEAD 可达的已提交历史'}`;
    document.querySelectorAll('[data-view]').forEach(button => { const active = button.dataset.view === state.view; button.classList.toggle('active',active); button.setAttribute('aria-pressed',String(active)); });
    if (state.view === 'architecture') { draw(commits); renderPanel(commits); }
    if (isDesign) drawDesign();
    if (isDeployment) drawDeployment();
    if (state.view === 'commits') renderCommits(commits);
    if (state.view === 'contributors') renderAuthors(commits);
  }

  function exportSource() {
    const svg = $('diagram').cloneNode(true);
    svg.removeAttribute('style');
    svg.setAttribute('width',String(svg.viewBox.baseVal.width * 2));
    svg.setAttribute('height',String(svg.viewBox.baseVal.height * 2));
    svg.querySelectorAll('.node').forEach(node => { node.classList.remove('selected','diagram-muted','diagram-related'); node.removeAttribute('tabindex'); node.removeAttribute('role'); });
    svg.querySelectorAll('.diagram-muted,.diagram-related').forEach(element=>element.classList.remove('diagram-muted','diagram-related'));
    if (state.view === 'architecture') {
      const note = document.createElementNS('http://www.w3.org/2000/svg','text');
      note.setAttribute('x','40'); note.setAttribute('y',String(svg.viewBox.baseVal.height - 5)); note.setAttribute('font-size','12');
      note.textContent = `Git 筛选：${state.period === 'all' ? '全部历史' : '近 ' + state.period + ' 天'} / ${state.author === 'all' ? '全部作者' : authors.get(state.author).name} / 搜索：${state.search || '无'}；计划计数为完整快照。`;
      svg.append(note);
    }
    return new XMLSerializer().serializeToString(svg);
  }
  function download(blob, name) {
    const url = URL.createObjectURL(blob), link = document.createElement('a');
    link.href = url; link.download = name; document.body.append(link); link.click(); link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 15000);
  }
  const exportName = () => state.view === 'deployment' ? '知行有灵-实验室多机部署' : state.view === 'design' ? '知行有灵-实验室目标架构' : '知行有灵-架构进展';
  $('exportSvg').onclick = () => download(new Blob([exportSource()],{type:'image/svg+xml;charset=utf-8'}),exportName() + '.svg');
  $('exportPng').onclick = async () => {
    $('exportPng').disabled = true;
    let url;
    try {
      await document.fonts.ready;
      url = URL.createObjectURL(new Blob([exportSource()],{type:'image/svg+xml;charset=utf-8'}));
      const img = new Image();
      await new Promise((resolve,reject) => { img.onload = resolve; img.onerror = reject; img.src = url; });
      const canvas = document.createElement('canvas'); canvas.width = img.naturalWidth; canvas.height = img.naturalHeight;
      canvas.getContext('2d').drawImage(img,0,0);
      const blob = await new Promise(resolve => canvas.toBlob(resolve,'image/png'));
      if (!blob) throw new Error('无法创建 PNG');
      download(blob,exportName() + '.png'); notify(`已导出 ${canvas.width} × ${canvas.height} PNG`);
    } catch { notify('导出失败，请重试或使用 SVG 导出。'); }
    finally { if (url) URL.revokeObjectURL(url); $('exportPng').disabled = false; }
  };
  $('zoomIn').onclick = () => { state.zoom = Math.min(1.6, (state.zoom ?? $('diagram').getBoundingClientRect().width/$('diagram').viewBox.baseVal.width) + .15); resize(); };
  $('zoomOut').onclick = () => { state.zoom = Math.max(.2, (state.zoom ?? $('diagram').getBoundingClientRect().width/$('diagram').viewBox.baseVal.width) - .15); resize(); };
  $('fit').onclick = () => { state.zoom = null; resize(); $('viewport').scrollTo(0,0); };
  $('fullscreen').onclick = async () => { try { if (document.fullscreenElement) await document.exitFullscreen(); else await document.querySelector('.diagram-pane').requestFullscreen(); } catch { notify('当前浏览器不支持全屏，请使用浏览器全屏功能。'); } };
  document.querySelectorAll('[data-view]').forEach(button => { button.onclick = () => setView(button.dataset.view); });
  for (const id of ['period','author','search']) $(id).addEventListener(id === 'search' ? 'input' : 'change', () => { state[id] = $(id).value; state.shown = 40; render(); });
  $('author').insertAdjacentHTML('beforeend',data.authors.map(author => `<option value="${author.id}">${escape(author.name)}</option>`).join(''));
  $('snapshot').innerHTML = `Git 快照 <code>${data.head.slice(0,8)}</code> · ${escape(data.branch)}<br>最新提交 ${date(data.headDate)} · 生成于 ${date(data.generatedAt)}`;
  $('methodContent').innerHTML = `<p><strong>目标架构</strong>是面向实验室设备的设计说明，状态由已核对的实现边界人工维护。关联已有模块表示可复用基础，不继承它的任务完成率，也不代表新设备已接通。</p><p><strong>计划进度</strong>取自 00–09 正式计划的复选框。已勾选为“文档标记关闭”，未勾选为“未关闭”，不自动推断未开始、已实现或验收通过。设计草案与退役清单不重复计数。</p><p><strong>贡献记录</strong>只统计快照 HEAD 可达的非合并提交。按显式路径及历史路径关联模块；一条提交可涉及多个模块，全局按提交哈希去重。未映射提交仍在完整时间线中保留。</p><p><strong>作者身份</strong>按 Git 作者邮箱归并，页面不显示邮箱。保留历史署名、提交者与共同署名；这些记录不自动等于模块负责人、实际工时或工作量。</p><p><strong>快照范围</strong>计划与提交均读取 ${data.head.slice(0,8)}，不包含未提交改动。近 7 / 30 天以快照生成时间为截止点。${data.shallow ? '当前为浅克隆，历史统计不完整。' : ''}</p><p><strong>更新方式</strong>：在仓库根目录执行下列命令后刷新页面。此网页离线可用，不会自行读取 Git 或修改计划；GitHub 链接需要对应提交已推送。</p><p><code>python3 scripts/docs/build_architecture.py</code></p>`;
  $('closeDesignDetail').onclick = () => $('designDetailDialog').close();
  $('designDetailDialog').addEventListener('close', () => {
    const id = state.designSelected; state.designSelected = null;
    if (state.view === 'deployment') {
      state.deploymentSelected=null;
      const trigger=state.deploymentFocus; state.deploymentFocus=null;
      trigger?.focus({preventScroll:true});
      return;
    }
    if (state.view !== 'design') return;
    drawDesign();
    $('diagram').querySelector(`[data-design="${id}"]`)?.focus({preventScroll:true});
  });
  $('referenceContent').innerHTML = `<p>参考职责与接口设计，本轮未安装这些框架，也未新增设备或模型连接。</p>${design.references.map(ref=>`<article class="reference-item"><h3><a href="${escape(ref.url)}" target="_blank" rel="noopener noreferrer">${escape(ref.name)} ↗</a></h3><p class="reference-role">${escape(ref.role)}</p><p>${escape(ref.take)}</p><p class="reference-boundary">${escape(ref.boundary)}</p></article>`).join('')}<p><a href="lab-device-platform.md">完整架构与参考取舍 ↗</a></p>`;
  $('references').onclick=()=>$('referenceDialog').showModal();
  $('closeReferences').onclick=()=>$('referenceDialog').close();
  $('method').onclick = () => $('methodDialog').showModal();
  $('closeMethod').onclick = () => $('methodDialog').close();
  window.addEventListener('hashchange',()=>{
    const view=location.hash === '#deployment' ? 'deployment' : 'design';
    if(state.view!==view){$('designDetailDialog').close();setView(view);}
  });
  new ResizeObserver(resize).observe($('viewport'));
  render();
})();
