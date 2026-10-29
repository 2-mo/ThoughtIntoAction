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
  const state = {view:'architecture', selected:null, period:'all', author:'all', search:'', zoom:null, shown:40};
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
  function edge(path, label, x, y) {
    return `<path class="edge" d="${path}"/>${label ? `<text class="edge-label" x="${x}" y="${y}">${label}</text>` : ''}`;
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
    const fit = Math.max(320, $('viewport').clientWidth - 16) / 1540;
    const scale = state.zoom ?? fit;
    svg.style.width = `${1540 * scale}px`;
    svg.style.height = `${1520 * scale}px`;
    $('fit').textContent = state.zoom === null ? '适宽' : Math.round(scale * 100) + '%';
  }

  function setView(view) { state.view = view; state.shown = 40; render(); }
  function render() {
    const commits = filteredCommits();
    metrics(commits);
    for (const view of ['architecture','commits','contributors']) $(view + 'View').hidden = state.view !== view;
    document.querySelectorAll('[data-view]').forEach(button => { const active = button.dataset.view === state.view; button.classList.toggle('active',active); button.setAttribute('aria-pressed',String(active)); });
    if (state.view === 'architecture') { draw(commits); renderPanel(commits); }
    if (state.view === 'commits') renderCommits(commits);
    if (state.view === 'contributors') renderAuthors(commits);
  }

  function exportSource() {
    const svg = $('diagram').cloneNode(true);
    svg.removeAttribute('style');
    svg.setAttribute('width','3080'); svg.setAttribute('height','3040');
    svg.querySelectorAll('.node').forEach(node => { node.classList.remove('selected'); node.removeAttribute('tabindex'); node.removeAttribute('role'); });
    const note = document.createElementNS('http://www.w3.org/2000/svg','text');
    note.setAttribute('x','40'); note.setAttribute('y','1515'); note.setAttribute('font-size','12');
    note.textContent = `Git 筛选：${state.period === 'all' ? '全部历史' : '近 ' + state.period + ' 天'} / ${state.author === 'all' ? '全部作者' : authors.get(state.author).name} / 搜索：${state.search || '无'}；计划计数为完整快照。`;
    svg.append(note);
    return new XMLSerializer().serializeToString(svg);
  }
  function download(blob, name) {
    const url = URL.createObjectURL(blob), link = document.createElement('a');
    link.href = url; link.download = name; document.body.append(link); link.click(); link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 15000);
  }
  $('exportSvg').onclick = () => download(new Blob([exportSource()],{type:'image/svg+xml;charset=utf-8'}),'知行有灵-架构进展.svg');
  $('exportPng').onclick = async () => {
    $('exportPng').disabled = true;
    let url;
    try {
      await document.fonts.ready;
      url = URL.createObjectURL(new Blob([exportSource()],{type:'image/svg+xml;charset=utf-8'}));
      const img = new Image();
      await new Promise((resolve,reject) => { img.onload = resolve; img.onerror = reject; img.src = url; });
      const canvas = document.createElement('canvas'); canvas.width = 3080; canvas.height = 3040;
      canvas.getContext('2d').drawImage(img,0,0);
      const blob = await new Promise(resolve => canvas.toBlob(resolve,'image/png'));
      if (!blob) throw new Error('无法创建 PNG');
      download(blob,'知行有灵-架构进展.png'); notify('已导出 3080 × 3040 PNG');
    } catch { notify('导出失败，请重试或使用 SVG 导出。'); }
    finally { if (url) URL.revokeObjectURL(url); $('exportPng').disabled = false; }
  };
  $('zoomIn').onclick = () => { state.zoom = Math.min(1.6, (state.zoom ?? $('diagram').getBoundingClientRect().width/1540) + .15); resize(); };
  $('zoomOut').onclick = () => { state.zoom = Math.max(.2, (state.zoom ?? $('diagram').getBoundingClientRect().width/1540) - .15); resize(); };
  $('fit').onclick = () => { state.zoom = null; resize(); };
  $('fullscreen').onclick = async () => { try { if (document.fullscreenElement) await document.exitFullscreen(); else await document.querySelector('.diagram-pane').requestFullscreen(); } catch { notify('当前浏览器不支持全屏，请使用浏览器全屏功能。'); } };
  document.querySelectorAll('[data-view]').forEach(button => { button.onclick = () => setView(button.dataset.view); });
  for (const id of ['period','author','search']) $(id).addEventListener(id === 'search' ? 'input' : 'change', () => { state[id] = $(id).value; state.shown = 40; render(); });
  $('author').insertAdjacentHTML('beforeend',data.authors.map(author => `<option value="${author.id}">${escape(author.name)}</option>`).join(''));
  $('snapshot').innerHTML = `Git 快照 <code>${data.head.slice(0,8)}</code> · ${escape(data.branch)}<br>最新提交 ${date(data.headDate)} · 生成于 ${date(data.generatedAt)}`;
  $('coverage').textContent = `${data.modules.length} 个模块 · ${data.planFiles.length} 份正式计划 · ${data.shallow ? '浅克隆，仅含本地可用历史' : 'HEAD 可达的已提交历史'}`;
  $('methodContent').innerHTML = `<p><strong>计划进度</strong>取自 00–09 正式计划的复选框。已勾选为“文档标记关闭”，未勾选为“未关闭”，不自动推断未开始、已实现或验收通过。设计草案与退役清单不重复计数。</p><p><strong>贡献记录</strong>只统计快照 HEAD 可达的非合并提交。按显式路径及历史路径关联模块；一条提交可涉及多个模块，全局按提交哈希去重。未映射提交仍在完整时间线中保留。</p><p><strong>作者身份</strong>按 Git 作者邮箱归并，页面不显示邮箱。保留历史署名、提交者与共同署名；这些记录不自动等于模块负责人、实际工时或工作量。</p><p><strong>快照范围</strong>计划与提交均读取 ${data.head.slice(0,8)}，不包含未提交改动。近 7 / 30 天以快照生成时间为截止点。${data.shallow ? '当前为浅克隆，历史统计不完整。' : ''}</p><p><strong>更新方式</strong>：在仓库根目录执行下列命令后刷新页面。此网页离线可用，不会自行读取 Git 或修改计划；GitHub 链接需要对应提交已推送。</p><p><code>python3 scripts/docs/build_architecture.py</code></p>`;
  $('method').onclick = () => $('methodDialog').showModal();
  $('closeMethod').onclick = () => $('methodDialog').close();
  new ResizeObserver(resize).observe($('viewport'));
  render();
})();
