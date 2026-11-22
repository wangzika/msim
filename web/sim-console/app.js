'use strict';

const view = {data: null, yaw: -0.72, elevation: 0.64, zoom: 1, pointer: null};
const activeKeys = new Set();
const colors = {truth: '#e4ecf1', gps: '#65bfff', route: '#f2d26f',
  lio: '#49d6e9', loose: '#f6a86d'};
const $ = id => document.getElementById(id);
const lastOf = array => array[array.length - 1];
const isInteractive = () => !!(view.data && view.data.settings && view.data.settings.interactive);

function canvasContext(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const width = Math.max(1, canvas.clientWidth), height = Math.max(1, canvas.clientHeight);
  canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio);
  const ctx = canvas.getContext('2d'); ctx.scale(ratio, ratio);
  return {ctx, width, height};
}

function format(value, digits = 1) {
  return Number.isFinite(value) ? value.toFixed(digits) : '—';
}

function project(point, center, scale, width, height) {
  const dx = point[0] - center[0], dy = point[1] - center[1], dz = point[2] - center[2];
  const c = Math.cos(view.yaw), s = Math.sin(view.yaw);
  const horizontal = c * dx - s * dy, depth = s * dx + c * dy;
  return [width / 2 + horizontal * scale,
    height / 2 + (depth * Math.sin(view.elevation) - dz * Math.cos(view.elevation)) * scale];
}

function drawLine(ctx, points, transform, color, width = 2, dash = []) {
  ctx.beginPath(); let begun = false;
  for (const value of points) {
    if (!value || !value.every(Number.isFinite)) {begun = false; continue;}
    const p = transform(value);
    if (!begun) {ctx.moveTo(...p); begun = true;} else ctx.lineTo(...p);
  }
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash);
  ctx.stroke(); ctx.setLineDash([]);
}

function boxVertices(shape) {
  const [x, y, z] = shape.center, [sx, sy, sz] = shape.size;
  const c = Math.cos(shape.yaw), s = Math.sin(shape.yaw);
  const points = [];
  for (const zz of [-sz / 2, sz / 2]) for (const [xx, yy] of
      [[-sx / 2, -sy / 2], [sx / 2, -sy / 2], [sx / 2, sy / 2], [-sx / 2, sy / 2]]) {
    points.push([x + c * xx - s * yy, y + s * xx + c * yy, z + zz]);
  }
  return points;
}

function drawScene() {
  const canvas = $('scene'), {ctx, width, height} = canvasContext(canvas);
  ctx.clearRect(0, 0, width, height);
  const data = view.data || {}, rows = data.samples || [], shapes = data.scene || [];
  const first = rows[0], current = lastOf(rows);
  const settings = data.settings || {};
  const spawn = settings.spawn_enu_m || [0, 0, 0];
  const center = first ? [first.east_m, first.north_m, 4] : [spawn[0], spawn[1], 4];
  const scale = Math.min(width / 85, height / 55) * view.zoom;
  const at = point => project(point, center, scale, width, height);
  const origin = [Math.round(center[0] / 10) * 10, Math.round(center[1] / 10) * 10];
  for (let i = -7; i <= 7; i++) {
    drawLine(ctx, [[origin[0] + i * 10, origin[1] - 70, 0],
      [origin[0] + i * 10, origin[1] + 70, 0]], at, '#244359', 1);
    drawLine(ctx, [[origin[0] - 70, origin[1] + i * 10, 0],
      [origin[0] + 70, origin[1] + i * 10, 0]], at, '#244359', 1);
  }
  const visible = shapes.filter(shape => Math.abs(shape.center[0] - center[0]) < 105 &&
    Math.abs(shape.center[1] - center[1]) < 105);
  const c = Math.cos(view.yaw), s = Math.sin(view.yaw);
  visible.sort((a, b) => (s * (a.center[0] - b.center[0]) + c * (a.center[1] - b.center[1])));
  for (const shape of visible) {
    const v = boxVertices(shape).map(at);
    const faces = [
      {indices: [0, 1, 5, 4], fill: '#274b60'},
      {indices: [1, 2, 6, 5], fill: '#315970'},
      {indices: [2, 3, 7, 6], fill: '#2b5166'},
      {indices: [3, 0, 4, 7], fill: '#24465a'},
      {indices: [4, 5, 6, 7], fill: '#51778a'},
    ];
    for (const face of faces) {
      ctx.beginPath(); face.indices.forEach((index, i) => i ? ctx.lineTo(...v[index]) : ctx.moveTo(...v[index]));
      ctx.closePath(); ctx.fillStyle = face.fill; ctx.fill();
      ctx.strokeStyle = '#7395a4'; ctx.globalAlpha = .55; ctx.lineWidth = .7; ctx.stroke(); ctx.globalAlpha = 1;
    }
  }
  if (rows.length) {
    const route = (data.route || []).map(point => [first.east_m + point.east_m,
      first.north_m + point.north_m, settings.flight_height_m || 8]);
    drawLine(ctx, route, at, colors.route, 2.5, [7, 5]);
    drawLine(ctx, rows.map(row => [row.east_m, row.north_m, row.up_m]), at, colors.truth, 2.8);
    drawLine(ctx, rows.map(row => [row.gps_east_m, row.gps_north_m, row.gps_up_m]), at, colors.gps, 2);
    const p = at([current.east_m, current.north_m, current.up_m]);
    ctx.beginPath(); ctx.arc(p[0], p[1], 7, 0, Math.PI * 2);
    ctx.fillStyle = '#e4ecf1'; ctx.fill(); ctx.lineWidth = 3; ctx.strokeStyle = '#0b1a29'; ctx.stroke();
  } else {
    ctx.fillStyle = '#b6cbd5'; ctx.font = '16px system-ui';
    ctx.fillText('等待仿真数据…', 24, 34);
  }
  ctx.fillStyle = '#b0cbd7'; ctx.font = '14px system-ui';
  ctx.fillText(`E / N / U · ${visible.length} 个场景碰撞体`, 20, height - 20);
}

function drawMap() {
  const canvas = $('map'), {ctx, width, height} = canvasContext(canvas);
  ctx.clearRect(0, 0, width, height);
  const live = (view.data && view.data.live) || {}, cloud = live.cloud || [], lio = live.lio || [], loose = live.loose || [];
  const all = [...cloud, ...lio, ...loose];
  if (!all.length) {
    ctx.fillStyle = '#a9c3d0'; ctx.font = '16px system-ui';
    ctx.fillText('等待 LiDAR / IMU 数据；离线记录仍可查看上方场景与 GPS。', 20, 34);
    return;
  }
  const minX = Math.min(...all.map(p => p[0])), maxX = Math.max(...all.map(p => p[0]));
  const minY = Math.min(...all.map(p => p[1])), maxY = Math.max(...all.map(p => p[1]));
  const range = Math.max(10, maxX - minX, maxY - minY);
  const scale = Math.min(width - 40, height - 40) / (range * 1.15);
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
  const at = p => [width / 2 + (p[0] - cx) * scale, height / 2 - (p[1] - cy) * scale];
  ctx.fillStyle = '#6e9bb0'; ctx.globalAlpha = .6;
  for (const point of cloud) {const p = at(point); ctx.fillRect(p[0], p[1], 2, 2);}
  ctx.globalAlpha = 1;
  drawLine(ctx, lio, at, colors.lio, 2.3);
  drawLine(ctx, loose, at, colors.loose, 2.3);
  $('mapFrame').textContent = live.cloud_frame || '算法坐标系';
}

function render() {
  const data = view.data; if (!data) return;
  const rows = data.samples || [], last = lastOf(rows), teleop = data.teleop || {}, live = data.live || {};
  const settings = data.settings || {};
  $('runName').textContent = data.run;
  $('sceneName').textContent = settings.world_source ? lastOf(settings.world_source.split('/')) : '场景未就绪';
  $('simTime').textContent = last ? `仿真时间 ${(last.sim_time_ns / 1e9).toFixed(1)} s` : '仿真时间 —';
  $('flightMode').textContent = settings.interactive ? (teleop.state || '等待控制器') :
    (settings.flight_pattern || '等待仿真');
  $('gpsStatus').textContent = last ? (last.detector_status === 'alarm' ? '报警' :
    last.detector_status === 'normal' ? '正常' : '初始化') : '等待 GPS';
  $('gpsStatus').style.color = last && last.detector_status === 'alarm' ? '#f47c7f' : '#e6f0f5';
  $('position').textContent = last ? `${format(last.east_m)} / ${format(last.north_m)} / ${format(last.up_m)} m` : '—';
  $('occlusion').textContent = last && Number.isFinite(last.sky_blocked_fraction) ?
    `${(last.sky_blocked_fraction * 100).toFixed(0)} %` : '—';
  $('noiseScale').textContent = last && Number.isFinite(last.environment_noise_scale) ?
    `${last.environment_noise_scale.toFixed(2)} ×` : '—';
  $('cloudCount').textContent = String((live.cloud && live.cloud.length) || 0);
  $('controls').hidden = !settings.interactive;
  const liveStatus = !!last && !data.complete;
  $('connectionDot').className = `dot${liveStatus ? ' live' : ''}`;
  $('connection').textContent = data.complete ? '实验已完成' : liveStatus ? '实时采集' : '等待数据';
  drawScene(); drawMap();
}

async function refresh() {
  try {
    const response = await fetch('/api/state', {cache: 'no-store'});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    view.data = await response.json(); render();
  } catch (error) {
    $('connectionDot').className = 'dot error';
    $('connection').textContent = '连接中断';
    console.error('仿真状态读取失败', error);
  }
}

async function command(value) {
  try {
    const response = await fetch('/api/teleop/command', {
      method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(value),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || '指令失败');
    $('commandStatus').textContent = '';
  } catch (error) {
    $('commandStatus').textContent = error.message;
  }
}

function clearKeys(hold = false) {
  activeKeys.clear();
  document.querySelectorAll('[data-key]').forEach(button => button.classList.remove('active'));
  if (isInteractive()) command({op: hold ? 'hold' : 'keys', keys: []});
}

function setupControls() {
  document.querySelectorAll('[data-op]').forEach(button => button.addEventListener('click', () => {
    const op = button.dataset.op;
    command(op === 'takeoff' ? {op, height_m: Number($('height').value)} : {op});
  }));
  $('finish').addEventListener('click', () => {
    if (confirm('降落并结束本次采集？')) command({op: 'finish'});
  });
  $('speed').addEventListener('change', () => command({op: 'speed', value: Number($('speed').value)}));
  for (const button of document.querySelectorAll('[data-key]')) {
    const key = button.dataset.key;
    button.addEventListener('pointerdown', event => {
      button.setPointerCapture(event.pointerId); activeKeys.add(key); button.classList.add('active');
    });
    const release = () => {
      activeKeys.delete(key); button.classList.remove('active');
      command(activeKeys.size ? {op: 'keys', keys: [...activeKeys]} : {op: 'hold'});
    };
    button.addEventListener('pointerup', release);
    button.addEventListener('pointercancel', release);
  }
  document.addEventListener('keydown', event => {
    if (!isInteractive() || ['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
    const key = event.key.toLowerCase();
    if ('wasdrfqe'.includes(key) && key.length === 1) {
      event.preventDefault(); activeKeys.add(key);
      const button = document.querySelector(`[data-key="${key}"]`);
      if (button) button.classList.add('active');
    } else if (key === ' ' || key === 'escape') {event.preventDefault(); clearKeys(true);}
    else if (key === 't') command({op: 'takeoff', height_m: Number($('height').value)});
    else if (key === 'l') command({op: 'land'});
  });
  document.addEventListener('keyup', event => {
    const key = event.key.toLowerCase();
    if (activeKeys.delete(key)) {
      const button = document.querySelector(`[data-key="${key}"]`);
      if (button) button.classList.remove('active');
      command(activeKeys.size ? {op: 'keys', keys: [...activeKeys]} : {op: 'hold'});
    }
  });
  addEventListener('blur', () => clearKeys(true));
  document.addEventListener('visibilitychange', () => {if (document.hidden) clearKeys(true);});
  setInterval(() => {
    if (isInteractive() && !document.hidden && activeKeys.size)
      command({op: 'keys', keys: [...activeKeys]});
  }, 150);
}

function setupView() {
  const canvas = $('scene');
  canvas.addEventListener('pointerdown', event => {
    view.pointer = [event.clientX, event.clientY]; canvas.setPointerCapture(event.pointerId);
    canvas.classList.add('dragging');
  });
  canvas.addEventListener('pointermove', event => {
    if (!view.pointer) return;
    view.yaw += (event.clientX - view.pointer[0]) * .008;
    view.elevation = Math.max(.12, Math.min(1.35,
      view.elevation + (event.clientY - view.pointer[1]) * .006));
    view.pointer = [event.clientX, event.clientY]; drawScene();
  });
  const release = () => {view.pointer = null; canvas.classList.remove('dragging');};
  canvas.addEventListener('pointerup', release);
  canvas.addEventListener('pointercancel', release);
  canvas.addEventListener('wheel', event => {
    event.preventDefault(); view.zoom = Math.max(.3, Math.min(5, view.zoom * Math.exp(-event.deltaY * .001)));
    drawScene();
  }, {passive: false});
  const reset = () => {view.yaw = -.72; view.elevation = .64; view.zoom = 1; drawScene();};
  canvas.addEventListener('dblclick', reset); $('resetView').addEventListener('click', reset);
  addEventListener('resize', render);
}

setupControls(); setupView(); refresh(); setInterval(refresh, 1000);
