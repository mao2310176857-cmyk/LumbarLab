(() => {
  'use strict';

  const $ = (id) => document.getElementById(id);
  const number = (value) => typeof value === 'number' && Number.isFinite(value);
  const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
  const text = (id, value) => { $(id).textContent = value; };
  const format = (value, digits = 1) => number(value) ? value.toFixed(digits) : '—';
  const duration = (value) => {
    const seconds = Math.max(0, Math.floor(Number(value) || 0));
    return `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
  };
  const dateLabel = (value) => {
    const date = new Date(value);
    return Number.isNaN(date.getTime()) ? '实验记录' : date.toLocaleString('zh-CN', { month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
  };
  let liveState = null;
  let displayedState = null;
  let lastSuccessfulPoll = 0;
  let liveFresh = false;
  let busy = false;
  let playback = null;
  let parametersInitialized = false;
  let parametersDirty = false;
  let sourceInitialized = false;
  let sessionSignature = '';
  let sessions = [];
  let toastTimer;
  let eventSignature = '';
  let pollTimer;

  async function request(url, options = {}) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 4500);
    try {
      const response = await fetch(url, { ...options, signal: controller.signal, cache: 'no-store' });
      let result;
      try { result = await response.json(); } catch (_) { throw new Error('本地服务返回了无法读取的数据，请检查服务窗口。'); }
      if (!response.ok) throw new Error(result.error || `请求失败（${response.status}）`);
      return result;
    } catch (error) {
      if (error.name === 'AbortError') throw new Error('本地服务响应超时，请确认服务仍在运行。');
      throw error;
    } finally { clearTimeout(timer); }
  }

  function toast(message, error = false) {
    clearTimeout(toastTimer);
    $('toast').textContent = message;
    $('toast').classList.toggle('error', error);
    $('toast').hidden = false;
    toastTimer = setTimeout(() => { $('toast').hidden = true; }, error ? 5500 : 3200);
  }

  async function action(name, fields = {}, successMessage) {
    if (playback) { toast('当前为只读回放。返回实时工作台后才能操作。'); return null; }
    if (busy) return null;
    if (!liveFresh) { toast('尚未连接本地服务，请等待自动重连。', true); return null; }
    busy = true;
    updateDisabled();
    try {
      const result = await request('/api/action', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ action: name, ...fields }) });
      liveState = result;
      lastSuccessfulPoll = performance.now();
      liveFresh = true;
      render(result);
      if (successMessage) toast(successMessage);
      return result;
    } catch (error) {
      toast(error.message || '操作失败，请检查本地服务。', true);
      return null;
    } finally {
      busy = false;
      updateDisabled();
    }
  }

  function updateDisabled() {
    const blocked = busy || Boolean(playback) || !liveFresh;
    const state = liveState;
    document.querySelectorAll('.live-command, .live-input').forEach((element) => { element.disabled = blocked; });
    if (!blocked && state) {
      const outputFault = Boolean(state.actuator?.fault);
      const manualAvailable = state.source === 'simulation' ? state.actuator?.enabled !== false : state.hardware?.actuation_enabled === true;
      document.querySelectorAll('[data-scenario]').forEach((button) => { button.disabled = state.source !== 'simulation'; });
      document.querySelectorAll('[data-action="manual"]').forEach((button) => { button.disabled = !manualAvailable || outputFault || Boolean(state.calibration?.active); });
      $('calibrate-button').disabled = outputFault || Boolean(state.calibration?.active) || !state.signal_valid || !state.occupied || !state.contact;
      $('auto-toggle').disabled = state.mode !== 'auto' && (outputFault || !state.reference || Boolean(state.calibration?.active) || !state.signal_valid || !state.occupied || !state.contact);
      $('zero-button').disabled = state.source !== 'serial' || !$('empty-confirmation').checked;
      document.querySelectorAll('[data-feedback]').forEach((button) => { button.disabled = !state.recording?.active; });
    }
    document.querySelectorAll('[data-session-play]').forEach((button) => { button.disabled = busy || Boolean(playback) || !liveFresh || Boolean(liveState?.recording?.active); });
    const serialSelected = $('source-select').value === 'serial';
    $('port-select').disabled = blocked || !serialSelected;
    $('baud-input').disabled = blocked || !serialSelected;
    $('empty-confirmation').disabled = blocked;
  }

  function dot(id, good, invalid = false) {
    const element = $(id);
    element.classList.toggle('off', !good && !invalid);
    element.classList.toggle('warn', invalid);
  }

  function render(state) {
    if (!state || typeof state !== 'object') return;
    displayedState = state;
    const isPlayback = Boolean(playback);
    const online = liveFresh && state.connected;
    const valid = Boolean(state.signal_valid);
    const serial = state.source === 'serial';
    const hardwareEnabled = state.hardware?.actuation_enabled === true;
    const awaitingReference = !state.reference && state.mode === 'manual' && state.action === 'hold' && !state.calibration?.active;
    const connectionLabel = isPlayback ? '历史数据' : !liveFresh ? '服务离线' : !online ? '设备未连接' : !valid ? '信号异常' : serial ? '串口已连接' : '模拟器已连接';
    text('connection-label', connectionLabel);
    $('connection-pill').querySelector('.status-dot').className = `status-dot${!liveFresh && !isPlayback ? ' error' : !valid && !isPlayback ? ' warn' : ''}`;
    text('source-label', isPlayback ? '会话回放' : serial ? state.actuator?.fault ? '串口采集 · 驱动异常' : hardwareEnabled ? '串口采集 · 驱动已接入' : '串口采集 · 仅建议' : '本地模拟');
    $('live-indicator').lastChild.textContent = isPlayback ? ' REPLAY' : liveFresh ? ' LIVE' : ' OFFLINE';
    text('frame-number', `FRAME ${String(state.frame_seq ?? 0).padStart(5, '0')}`);
    $('stale-banner').hidden = liveFresh || isPlayback;
    // The sensors are 10-bit (0..1023). Prefer zones10 when the board sends
    // it; fall back to the 8-bit zones the algorithm uses.
    const zones10 = Array.isArray(state.zones10) ? state.zones10 : null;
    const zones8 = Array.isArray(state.zones) ? state.zones : [];
    const fullScale = zones10 ? 1023 : 255;
    for (let index = 0; index < 8; index += 1) {
      const value = zones10 ? zones10[index] : zones8[index];
      const normalized = number(value) ? clamp(value / fullScale, 0, 1) : 0;
      text(`zone-value-${index + 1}`, format(value, 0));
      $(`zone-bar-${index + 1}`).style.width = `${normalized * 100}%`;
      const sensor = $(`sensor-${index + 1}`);
      const hue = 83 + normalized * 65;
      const lightness = 69 - normalized * 43;
      sensor.querySelector('.sensor-disc').style.fill = number(value) ? `hsl(${hue}, 24%, ${lightness}%)` : '#bfcaba';
      sensor.querySelector('.sensor-halo').style.opacity = number(value) ? String(.07 + normalized * .27) : '.03';
      sensor.querySelector('.sensor-halo').style.fill = `hsl(${hue}, 33%, 46%)`;
      sensor.setAttribute('aria-label', `区域 ${index + 1}，压力 ${format(value, 0)}`);
    }
    dot('occupied-dot', state.occupied, !valid);
    dot('contact-dot', state.contact, !valid);
    text('occupied-label', !valid ? '入座状态待确认' : state.occupied ? '已检测到入座' : '当前无人入座');
    text('contact-label', !valid ? '接触状态待确认' : state.contact ? '腰部接触良好' : '腰部未接触');
    text('signal-label', number(state.frame_age_ms) ? `${Math.round(state.frame_age_ms)} ms` : '等待信号');
    text('mode-tag', awaitingReference ? '等待个人参考' : ({ manual: '手动模式', auto: serial ? hardwareEnabled ? '自动派发' : '自动建议' : '自动模拟', paused: '算法已暂停' })[state.mode] || '手动模式');
    text('position-value', format(state.position, 0));
    text('position-unit', number(state.position) ? '%' : '');
    $('position-fill').style.width = `${number(state.position) ? clamp(state.position, 0, 100) : 0}%`;
    $('position-thumb').style.left = `${number(state.position) ? clamp(state.position, 0, 100) : 0}%`;
    $('position-thumb').hidden = !number(state.position);
    const positionCaption = document.querySelector('.position-heading>span');
    positionCaption.replaceChildren(document.createTextNode('腰靠行程 '));
    const suffix = document.createElement('span'); suffix.className = 'muted'; suffix.textContent = serial ? hardwareEnabled ? '/ 未提供位置反馈' : '/ 执行器未就绪' : '/ 模拟'; positionCaption.append(suffix);
    const calibration = state.calibration || {};
    const progress = clamp(Number(calibration.progress) || 0, 0, 1) * 100;
    $('calibration-progress').hidden = !calibration.active;
    $('calibration-progress').setAttribute('aria-valuenow', String(Math.round(progress)));
    $('calibration-progress').querySelector('i').style.width = `${progress}%`;
    $('calibrate-button').querySelector('span').textContent = calibration.active ? `正在记忆 ${Math.round(progress)}%` : state.reference ? '重新记住支撑' : '记住当前支撑';
    text('reference-description', calibration.active ? '保持舒适坐姿，正在稳定采集' : state.reference ? '已保存当前舒适参考，可随时重新记忆' : '支撑合适后，稳定采集 3 秒');
    text('calibration-message', calibration.message || (state.reference ? '舒适支撑已保存' : '尚未记录舒适支撑'));
    $('auto-toggle').setAttribute('aria-checked', String(state.mode === 'auto'));
    text('auto-description', serial ? hardwareEnabled ? '偏离区间时，向已接入的驱动派发短时指令' : '仅生成调节建议；尚未启用真实执行器' : '偏离舒适区间时，小步调节模拟行程');
    const actionLabel = ({ advance: '增加支撑', retract: '减小支撑', hold: '保持', stop: '停止输出' })[state.action] || '等待判断';
    text('action-label', `算法决策 · ${actionLabel}`);
    text('reason-label', awaitingReference && valid && state.occupied && state.contact && !state.actuator?.fault ? '支撑合适时直接记住；如需调整，可先小步微调' : state.reason || '等待传感器数据');
    if (!liveFresh && !isPlayback) {
      text('action-label', '实时连接中断');
      text('reason-label', '当前显示最后收到的数据；等待本地服务恢复。');
    }
    $('control-status').classList.toggle('is-warning', !valid || !state.occupied || !state.contact || Boolean(state.actuator?.fault));
    $('action-pulse').classList.toggle('moving', state.action === 'advance' || state.action === 'retract');
    renderActuator(state, isPlayback);
    const metrics = state.metrics || {};
    text('metric-lumbar', format(metrics.lumbar));
    text('metric-target', format(metrics.target));
    text('metric-range', number(metrics.lower) && number(metrics.upper) ? `${format(metrics.lower, 0)} – ${format(metrics.upper, 0)}` : '未设置');
    document.querySelectorAll('[data-scenario]').forEach((button) => {
      const selected = button.dataset.scenario === state.scenario && !serial;
      button.classList.toggle('selected', selected);
      button.setAttribute('aria-pressed', String(selected));
    });
    $('scenario-note').lastChild.textContent = serial ? '真实设备模式 · 场景切换仅在本地模拟时可用' : '模拟数据 · 无需连接硬件即可体验完整流程';
    const recording = state.recording || {};
    const isRecording = Boolean(recording.active) && !isPlayback;
    $('recording-status').classList.toggle('active', isRecording);
    text('recording-label', isPlayback ? '正在查看历史记录' : isRecording ? `记录中 · ${recording.samples || 0} 帧` : '等待下一次实验');
    text('recording-time', duration(recording.elapsed_s));
    $('record-button').querySelector('span').textContent = isRecording ? '停止并保存' : '开始记录';
    if (isPlayback) text('feedback-hint', '只读回放 · 感受反馈可在最近事件中查看');
    else if (!isRecording) text('feedback-hint', '开始记录后，可保存此刻的支撑感受');
    else if (!$('feedback-hint').textContent.startsWith('最近反馈：')) text('feedback-hint', '反馈将作为记录中的事件保存');
    text('device-summary', serial ? state.hardware?.port || '串口传感器' : '模拟设备');
    text('hardware-message', state.hardware?.message || '等待设备信息');
    text('mapping-status', serial ? state.hardware?.mapping_ready ? '8 区映射已就绪' : '分区映射未就绪，暂不生成调节建议' : '模拟模式提供 8 区数据');
    if (!sourceInitialized && !isPlayback) {
      $('source-select').value = state.source === 'serial' ? 'serial' : 'simulation';
      sourceInitialized = true;
    }
    if (isPlayback && state.parameters) setParameterFields(state.parameters);
    else if ((!parametersInitialized || !parametersDirty) && state.parameters && !document.activeElement?.closest('#parameters-form')) {
      setParameterFields(state.parameters);
      parametersInitialized = true;
    }
    renderEvents(state.events || []);
    drawChart();
    if ($('device-section').open) drawMatrix();
    updateDisabled();
  }

  function renderActuator(state, isPlayback) {
    const output = state.actuator;
    const offline = !liveFresh && !isPlayback;
    const names = { advance: '推出', retract: '收回', hold: '保持', stop: '停止' };
    const hardware = output?.is_hardware === true;
    const simulated = Boolean(output) && !hardware && output.enabled === true && state.source === 'simulation';
    const fault = Boolean(output?.fault);
    const available = Boolean(output?.enabled);
    const command = output?.command;
    let status = '等待输出状态';
    if (!output) status = isPlayback ? '这份记录没有执行器状态' : '服务尚未提供执行器状态';
    else if (fault) status = '驱动故障 · 输出状态需确认';
    else if (offline) status = '实时状态未知';
    else if (simulated) status = output.acknowledged ? '模拟执行器已接收指令' : output.sent ? '模拟指令已派发，等待接收确认' : '模拟指令尚未派发';
    else if (!hardware || !available) status = hardware ? '真实驱动未启用' : available ? '软件适配器 · 未驱动真实电机' : '真实执行器未接入';
    else if (output.acknowledged) status = output.sent ? '设备已回执 · 动作结果未验证' : '收到回执 · 发送状态未确认';
    else if (output.sent) status = '指令已发送 · 尚无设备回执';
    else status = '指令尚未发送';
    text('actuator-state', status);
    text('actuator-kind', !output ? isPlayback ? '旧版记录' : '等待状态' : simulated ? '模拟执行' : hardware ? available ? '真实适配器' : '驱动未启用' : available ? '软件适配器' : '未接入');
    text('actuator-send-label', simulated ? '模拟派发' : '指令发送');
    text('actuator-ack-label', simulated ? '模拟接收' : output && !hardware ? '适配器接收' : '设备回执');
    text('actuator-sent', !output ? '无记录' : output.sent ? offline ? '上次已发送' : '已发送' : '未发送');
    text('actuator-ack', !output ? '无记录' : output.acknowledged ? `${offline ? '上次' : ''}${hardware ? '已回执' : '已接收'}` : '未确认');
    $('actuator-sent').classList.toggle('confirmed', Boolean(output?.sent));
    $('actuator-ack').classList.toggle('confirmed', Boolean(output?.acknowledged));
    let detail = !output ? '仅展示算法记录，不推断真实执行器动作或停止状态。' : output.message || '等待适配器提供输出状态。';
    if (fault && !detail.includes(String(output.fault))) detail += `；${String(output.fault)}`;
    if (offline) detail = `上次状态：${detail}`;
    text('actuator-message', detail);
    const instruction = command ? `${names[command.action] || '未知'}${number(command.duration_s) && command.duration_s > 0 ? ` · ${format(command.duration_s, 2)} 秒` : ''}` : '无';
    text('actuator-command', `请求 ${names[output?.requested_action] || '—'} · 最近命令 ${instruction}`);
    text('actuator-seq', number(command?.seq) ? `#${command.seq}` : '');
    text('actuator-note', !output ? isPlayback ? '旧记录兼容显示，未补写执行器结果。' : '缺少执行器状态，无法判断实际输出。' : simulated ? '仅模拟行程变化，未驱动真实电机。' : '回执仅确认接收指令，不代表物理动作或停止已完成。');
    $('actuator-status').classList.toggle('is-fault', fault);
    $('actuator-status').classList.toggle('is-unavailable', !output || !available || offline);
  }

  function setParameterFields(parameters) {
    for (const name of ['tolerance', 'persistence_s', 'speed', 'settle_s']) {
      if (number(parameters[name])) $('parameters-form').elements[name].value = name === 'tolerance' ? String(Math.round(parameters[name] * 100)) : String(parameters[name]);
    }
  }

  function renderEvents(events) {
    const latest = Array.isArray(events) ? events.slice(-30).reverse() : [];
    const signature = JSON.stringify(latest);
    if (signature === eventSignature) return;
    eventSignature = signature;
    const list = $('events-list');
    list.replaceChildren();
    if (!latest.length) {
      const item = document.createElement('li'); item.className = 'empty-event'; item.textContent = '等待实验开始'; list.append(item); return;
    }
    latest.forEach((event) => {
      const item = document.createElement('li');
      const time = document.createElement('time'); time.textContent = duration(event.t);
      const detail = document.createElement('span'); detail.textContent = typeof event.detail === 'string' ? event.detail : event.detail ? JSON.stringify(event.detail) : event.type || '事件';
      item.append(time, detail); list.append(item);
    });
  }

  function canvasContext(canvas) {
    const rectangle = canvas.getBoundingClientRect();
    if (!rectangle.width || !rectangle.height) return null;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.round(rectangle.width * ratio);
    const height = Math.round(rectangle.height * ratio);
    if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
    const context = canvas.getContext('2d');
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, rectangle.width, rectangle.height);
    return { context, width: rectangle.width, height: rectangle.height };
  }

  function historyForChart() {
    if (!displayedState) return [];
    if (!playback) return Array.isArray(displayedState.history) ? displayedState.history.filter((point) => number(point.t)) : [];
    const end = displayedState.time;
    return playback.frames.slice(Math.max(0, playback.index - 1800), playback.index + 1).filter((frame) => number(frame.time) && frame.time >= end - 60).map((frame) => ({ t: frame.time, lumbar: frame.metrics?.lumbar, z5: frame.zones?.[4], z6: frame.zones?.[5], z7: frame.zones?.[6], lower: frame.metrics?.lower, upper: frame.metrics?.upper }));
  }

  function drawChart() {
    const setup = canvasContext($('pressure-chart'));
    if (!setup) return;
    const { context: context, width, height } = setup;
    const points = historyForChart();
    $('chart-empty').hidden = points.length > 1;
    const padding = { left: 31, right: 8, top: 12, bottom: 27 };
    const plotWidth = width - padding.left - padding.right;
    const plotHeight = height - padding.top - padding.bottom;
    const latest = points.length ? points[points.length - 1].t : 60;
    const start = latest - 60;
    const visible = points.filter((point) => point.t >= start);
    const metrics = displayedState?.metrics || {};
    let maxValue = 30;
    for (const point of visible) for (const name of ['lumbar', 'z5', 'z6', 'z7', 'upper']) if (number(point[name])) maxValue = Math.max(maxValue, point[name] * 1.15);
    if (number(metrics.upper)) maxValue = Math.max(maxValue, metrics.upper * 1.15);
    const step = maxValue <= 60 ? 10 : maxValue <= 150 ? 25 : 50;
    const max = Math.ceil(maxValue / step) * step;
    const x = (time) => padding.left + clamp((time - start) / 60, 0, 1) * plotWidth;
    const y = (value) => padding.top + (1 - clamp(value / max, 0, 1)) * plotHeight;
    context.font = '8px "Segoe UI", "Microsoft YaHei", sans-serif';
    context.textAlign = 'right';
    context.textBaseline = 'middle';
    for (let index = 0; index <= 3; index += 1) {
      const value = max * index / 3;
      const ordinate = y(value);
      context.beginPath(); context.moveTo(padding.left, ordinate); context.lineTo(width - padding.right, ordinate); context.strokeStyle = '#edf1e6'; context.lineWidth = 1; context.stroke();
      context.fillStyle = '#adbaa0'; context.fillText(String(Math.round(value)), padding.left - 8, ordinate);
    }
    // Use each sample's own reference, preserving changes and periods without one.
    const referenceRuns = [];
    let referenceRun = [];
    for (const point of visible) {
      if (number(point.lower) && number(point.upper)) referenceRun.push(point);
      else if (referenceRun.length) { referenceRuns.push(referenceRun); referenceRun = []; }
    }
    if (referenceRun.length) referenceRuns.push(referenceRun);
    context.fillStyle = '#eaf1e18c';
    context.strokeStyle = '#c9d9b8'; context.lineWidth = .8;
    for (const run of referenceRuns) {
      if (run.length < 2) continue;
      const lastPoint = run[run.length - 1];
      context.beginPath(); context.moveTo(x(run[0].t), y(run[0].upper));
      for (let index = 1; index < run.length; index += 1) {
        context.lineTo(x(run[index].t), y(run[index - 1].upper));
        context.lineTo(x(run[index].t), y(run[index].upper));
      }
      context.lineTo(x(lastPoint.t), y(lastPoint.lower));
      for (let index = run.length - 2; index >= 0; index -= 1) {
        context.lineTo(x(run[index + 1].t), y(run[index].lower));
        context.lineTo(x(run[index].t), y(run[index].lower));
      }
      context.closePath(); context.fill();
      context.setLineDash([3, 4]);
      for (const bound of ['lower', 'upper']) {
        context.beginPath(); context.moveTo(x(run[0].t), y(run[0][bound]));
        for (let index = 1; index < run.length; index += 1) {
          context.lineTo(x(run[index].t), y(run[index - 1][bound]));
          context.lineTo(x(run[index].t), y(run[index][bound]));
        }
        context.stroke();
      }
      context.setLineDash([]);
    }
    context.textAlign = 'center'; context.fillStyle = '#a8b699';
    for (let index = 0; index <= 4; index += 1) context.fillText(index === 4 ? (playback ? '当前帧' : '现在') : `${60 - index * 15}s`, padding.left + plotWidth * index / 4, height - 9);
    context.save(); context.beginPath(); context.rect(padding.left, padding.top, plotWidth, plotHeight); context.clip();
    for (const [name, color, lineWidth] of [['z5', '#b4c892', 1], ['z6', '#90ad99', 1], ['z7', '#c6b18b', 1], ['lumbar', '#1f6552', 2]]) {
      context.beginPath(); let drawing = false;
      for (const point of visible) {
        if (!number(point[name])) { drawing = false; continue; }
        if (!drawing) { context.moveTo(x(point.t), y(point[name])); drawing = true; } else context.lineTo(x(point.t), y(point[name]));
      }
      context.strokeStyle = color; context.lineWidth = lineWidth; context.lineJoin = 'round'; context.stroke();
    }
    context.restore();
    const last = visible[visible.length - 1];
    if (last && number(last.lumbar)) {
      context.beginPath(); context.arc(x(last.t), y(last.lumbar), 3, 0, Math.PI * 2); context.fillStyle = '#1f6552'; context.fill(); context.strokeStyle = '#fff'; context.lineWidth = 1.5; context.stroke();
    }
  }

  function drawMatrix() {
    const setup = canvasContext($('matrix-canvas'));
    if (!setup) return;
    const { context, width, height } = setup;
    const matrix = displayedState?.matrix;
    const valid = Array.isArray(matrix) && matrix.length === 256;
    $('matrix-empty').hidden = valid;
    for (let index = 0; index < 256; index += 1) {
      const value = valid && number(matrix[index]) ? clamp(matrix[index] / 255, 0, 1) : 0;
      context.fillStyle = valid ? `hsl(${82 + value * 70}, ${25 + value * 10}%, ${94 - value * 66}%)` : '#e8eee0';
      context.fillRect((index % 16) * width / 16 + .5, Math.floor(index / 16) * height / 16 + .5, width / 16 - 1, height / 16 - 1);
    }
  }

  async function poll() {
    try {
      const state = await request('/api/state');
      if (!state || !number(state.time) || !state.parameters) throw new Error('状态数据不完整');
      liveState = state;
      lastSuccessfulPoll = performance.now();
      liveFresh = true;
      if (!playback) render(state);
      else updateDisabled();
    } catch (_) {
      if (!lastSuccessfulPoll || performance.now() - lastSuccessfulPoll > 1600) {
        liveFresh = false;
        if (!playback) {
          $('stale-banner').hidden = false;
          text('connection-label', '服务离线');
          $('connection-pill').querySelector('.status-dot').className = 'status-dot error';
          $('live-indicator').lastChild.textContent = ' OFFLINE';
          text('action-label', '实时连接中断');
          text('reason-label', '当前显示最后收到的数据；等待本地服务恢复。');
        }
        updateDisabled();
      }
    } finally { pollTimer = setTimeout(poll, liveFresh ? 200 : 1000); }
  }

  async function refreshPorts(showMessage = false) {
    $('refresh-ports').disabled = true;
    try {
      const result = await request('/api/ports');
      const selected = $('port-select').value;
      $('port-select').replaceChildren(new Option('选择串口', ''));
      (result.ports || []).forEach((port) => $('port-select').add(new Option(`${port.device} · ${port.description || '串口设备'}`, port.device)));
      if (selected) $('port-select').value = selected;
      if (!$('port-select').value && liveState?.hardware?.port) $('port-select').value = liveState.hardware.port;
      if (!$('port-select').value && result.ports?.length === 1) $('port-select').value = result.ports[0].device;
      text('ports-message', result.available ? result.ports?.length ? `${result.ports.length} 个可用串口` : '未检测到串口' : '串口依赖未安装，模拟功能可正常使用');
      if (showMessage && result.available && !result.ports?.length) toast('未检测到串口，请连接设备后重新扫描。');
    } catch (error) { text('ports-message', '暂时无法获取串口列表'); if (showMessage) toast(error.message, true); }
    finally { $('refresh-ports').disabled = false; updateDisabled(); }
  }

  async function refreshSessions(showMessage = false) {
    try {
      const result = await request('/api/sessions');
      sessions = Array.isArray(result.sessions) ? result.sessions : [];
      const signature = JSON.stringify(sessions);
      if (signature !== sessionSignature) { sessionSignature = signature; renderSessions(); }
      if (showMessage) toast('实验列表已更新。');
    } catch (error) { if (showMessage) toast(error.message, true); }
  }

  function renderSessions() {
    text('sessions-count', sessions.length);
    const list = $('sessions-list');
    list.replaceChildren();
    if (!sessions.length) {
      const empty = document.createElement('div'); empty.className = 'empty-state';
      const arrow = document.createElement('span'); arrow.textContent = '↳';
      const copy = document.createElement('div'); const title = document.createElement('strong'); title.textContent = '你的第一份实验记录，从这里开始'; const paragraph = document.createElement('p'); paragraph.textContent = '点击“开始记录”，调整支撑后停止保存。'; copy.append(title, paragraph); empty.append(arrow, copy); list.append(empty); return;
    }
    [...sessions].sort((a, b) => String(b.started_at).localeCompare(String(a.started_at))).forEach((session) => {
      const row = document.createElement('div'); row.className = 'session-row';
      const icon = document.createElement('span'); icon.className = 'session-icon'; icon.textContent = '↳';
      const main = document.createElement('div'); main.className = 'session-main';
      const title = document.createElement('strong'); title.textContent = `支撑实验 · ${dateLabel(session.started_at)}`;
      const description = document.createElement('span'); description.textContent = `${session.samples || 0} 帧数据 · ${session.feedback_count || 0} 次感受反馈${session.ended_at ? '' : ' · 记录进行中'}`;
      main.append(title, description);
      const length = document.createElement('span'); const elapsed = (new Date(session.ended_at).getTime() - new Date(session.started_at).getTime()) / 1000; length.textContent = session.ended_at && Number.isFinite(elapsed) ? duration(elapsed) : '进行中';
      const actions = document.createElement('div'); actions.className = 'session-actions';
      const replay = document.createElement('button'); replay.type = 'button'; replay.className = 'button button-outline'; replay.textContent = '回放'; replay.dataset.sessionPlay = session.id; replay.addEventListener('click', () => enterPlayback(session.id));
      const download = document.createElement('a'); download.className = 'download-link'; download.href = `/api/sessions/${encodeURIComponent(session.id)}/download`; download.setAttribute('download', `${session.id}.json`); download.title = '下载实验 JSON'; download.setAttribute('aria-label', `下载 ${dateLabel(session.started_at)} 实验 JSON`); download.innerHTML = '<svg class="icon"><use href="#i-download"/></svg>';
      actions.append(replay, download); row.append(icon, main, length, actions); list.append(row);
    });
    updateDisabled();
  }

  async function enterPlayback(id) {
    if (playback || busy) return;
    if (liveState?.recording?.active) { toast('请先停止并保存当前记录，再进入回放。'); return; }
    if (!liveFresh) { toast('需要连接本地服务才能加载实验记录。', true); return; }
    busy = true; updateDisabled();
    try {
      const session = await request(`/api/sessions/${encodeURIComponent(id)}`);
      const frames = Array.isArray(session.frames) ? session.frames.filter((frame) => frame && number(frame.time)) : [];
      if (!frames.length) throw new Error('这份记录没有可回放的有效数据帧。');
      // This is the final write before entering read-only mode. Exiting never resumes automation.
      liveState = await request('/api/action', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ action: 'auto_pause' }) });
      lastSuccessfulPoll = performance.now(); liveFresh = true;
      if (liveState.actuator?.fault) {
        render(liveState);
        throw new Error('实时执行器存在故障，输出状态需确认。请留在实时工作台检查驱动后再回放。');
      }
      playback = { session, frames, index: 0, playing: false, startedAt: 0, startedTime: frames[0].time };
      parametersDirty = false;
      $('playback-bar').hidden = false;
      text('playback-name', `支撑实验 · ${dateLabel(session.started_at)}`);
      $('playback-seek').max = String(frames.length - 1);
      $('playback-seek').value = '0';
      text('chart-window', '回放 · 最近 60 秒');
      renderPlayback();
      $('playback-bar').scrollIntoView({ behavior: window.matchMedia('(prefers-reduced-motion: reduce)').matches ? 'auto' : 'smooth', block: 'start' });
      toast('已进入只读回放，实时自动算法已暂停。');
    } catch (error) { toast(error.message || '无法加载实验记录。', true); }
    finally { busy = false; updateDisabled(); }
  }

  function renderPlayback() {
    if (!playback) return;
    const frame = playback.frames[playback.index];
    const events = (playback.session.events || []).filter((event) => !number(event.t) || event.t <= frame.time).slice(-30);
    render({ ...frame, events, parameters: frame.parameters || playback.session.parameters, reference: frame.reference !== undefined ? frame.reference : playback.session.reference });
    $('playback-seek').value = String(playback.index);
    text('playback-time', `${duration(frame.time - playback.frames[0].time)} / ${duration(playback.frames[playback.frames.length - 1].time - playback.frames[0].time)}`);
    text('playback-toggle', playback.playing ? '暂停' : '播放');
  }

  function animatePlayback(now) {
    if (playback?.playing) {
      const target = playback.startedTime + (now - playback.startedAt) / 1000;
      let index = playback.index;
      while (index + 1 < playback.frames.length && playback.frames[index + 1].time <= target) index += 1;
      if (index !== playback.index) { playback.index = index; renderPlayback(); }
      if (index >= playback.frames.length - 1) { playback.playing = false; text('playback-toggle', '播放'); }
    }
    requestAnimationFrame(animatePlayback);
  }

  document.querySelectorAll('[data-action]').forEach((button) => button.addEventListener('click', () => action(button.dataset.action, button.dataset.direction ? { direction: button.dataset.direction } : {})));
  document.querySelectorAll('[data-scenario]').forEach((button) => button.addEventListener('click', () => action('scenario', { name: button.dataset.scenario })));
  document.querySelectorAll('[data-feedback]').forEach((button) => button.addEventListener('click', async () => {
    const result = await action('feedback', { value: button.dataset.feedback }, '已记下这次支撑感受。');
    if (result) {
      document.querySelectorAll('[data-feedback]').forEach((element) => element.classList.remove('chosen'));
      button.classList.add('chosen');
      text('feedback-hint', `最近反馈：${button.textContent.trim().replace(/^[−✓+]\s*/, '')}`);
    }
  }));
  $('auto-toggle').addEventListener('click', () => action(liveState?.mode === 'auto' ? 'auto_pause' : 'auto_start'));
  $('record-button').addEventListener('click', async () => {
    const stopping = Boolean(liveState?.recording?.active);
    const result = await action(stopping ? 'record_stop' : 'record_start', {}, stopping ? '实验已保存，可在下方回放或下载。' : '开始记录压力、行程和感受反馈。');
    if (result) await refreshSessions();
  });
  $('refresh-sessions').addEventListener('click', () => refreshSessions(true));
  $('refresh-ports').addEventListener('click', () => refreshPorts(true));
  $('source-select').addEventListener('change', updateDisabled);
  $('empty-confirmation').addEventListener('change', updateDisabled);
  $('connect-button').addEventListener('click', async () => {
    const source = $('source-select').value;
    const fields = { source };
    if (source === 'serial') {
      if (!$('port-select').value) { toast('请先选择串口；未检测到设备时可点击“扫描串口”。', true); return; }
      fields.port = $('port-select').value; fields.baud = Number($('baud-input').value);
      if (!Number.isInteger(fields.baud) || fields.baud < 1200) { toast('请输入有效波特率。', true); return; }
    }
    const result = await action('connect', fields, source === 'simulation' ? '已连接本地模拟器。' : '串口已连接，正在接收传感器数据。');
    if (result) $('empty-confirmation').checked = false;
  });
  $('zero-button').addEventListener('click', async () => {
    const result = await action('zero', { confirmed_empty: $('empty-confirmation').checked }, '电脑端基线已归零，请重新记住舒适支撑。');
    if (result) { $('empty-confirmation').checked = false; updateDisabled(); }
  });
  $('parameters-form').addEventListener('input', () => { parametersDirty = true; });
  $('parameters-form').addEventListener('submit', async (event) => {
    event.preventDefault();
    if (playback) { toast('只读回放期间不能修改调节参数。'); return; }
    const form = $('parameters-form');
    if (!form.reportValidity()) return;
    const values = {};
    for (const name of ['tolerance', 'persistence_s', 'speed', 'settle_s']) values[name] = Number(form.elements[name].value) / (name === 'tolerance' ? 100 : 1);
    const result = await action('parameters', { values }, '调节参数已应用。');
    if (result) { parametersDirty = false; setParameterFields(result.parameters); }
  });
  $('device-section').addEventListener('toggle', () => { if ($('device-section').open) { drawMatrix(); refreshPorts(); } });
  $('playback-toggle').addEventListener('click', () => {
    if (!playback) return;
    if (playback.playing) playback.playing = false;
    else {
      if (playback.index >= playback.frames.length - 1) playback.index = 0;
      playback.playing = true; playback.startedAt = performance.now(); playback.startedTime = playback.frames[playback.index].time;
    }
    renderPlayback();
  });
  $('playback-seek').addEventListener('input', () => {
    if (!playback) return;
    playback.index = clamp(Number($('playback-seek').value), 0, playback.frames.length - 1);
    playback.startedTime = playback.frames[playback.index].time; playback.startedAt = performance.now();
    renderPlayback();
  });
  $('playback-exit').addEventListener('click', () => {
    playback = null; $('playback-bar').hidden = true; text('chart-window', '最近 60 秒');
    if (liveState) render(liveState);
    updateDisabled(); toast('已返回实时工作台。自动算法保持暂停，可手动重新开启。');
  });
  window.addEventListener('resize', () => { drawChart(); if ($('device-section').open) drawMatrix(); });
  window.addEventListener('pagehide', () => clearTimeout(pollTimer));
  updateDisabled();
  poll();
  refreshSessions();
  setInterval(() => {
    if (liveFresh && lastSuccessfulPoll && performance.now() - lastSuccessfulPoll > 1800) {
      liveFresh = false;
      if (!playback && liveState) render(liveState);
      updateDisabled();
    }
  }, 300);
  setInterval(() => refreshSessions(), 15000);
  requestAnimationFrame(animatePlayback);
})();
