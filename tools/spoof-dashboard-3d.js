"use strict";

const spaceView = {
  rows: [], route: [], yaw: -0.68, elevation: 0.62, zoom: 1,
};

function spaceHeight(row) {
  if (Number.isFinite(row.up_m)) return row.up_m;
  // Older recorded runs contain geodetic altitude only. Their first GPS
  // sample is the display origin, so the old data remain viewable.
  return row.alt_m - spaceView.rows[0].alt_m;
}

function setSpaceData(rows, route) {
  spaceView.rows = rows;
  spaceView.route = route;
  document.getElementById('export3d').disabled = rows.length === 0;
  document.getElementById('export2d').disabled = rows.length === 0;
  drawSpace();
}

function drawSpace() {
  const rows = spaceView.rows;
  if (!rows.length) return;
  const canvas = document.getElementById('space3d');
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  const dpr = Math.max(2, window.devicePixelRatio || 1);
  canvas.width = Math.round(w * dpr);
  canvas.height = Math.round(h * dpr);
  const ctx = canvas.getContext('2d');
  ctx.scale(dpr, dpr);
  ctx.fillStyle = '#0b1a27';
  ctx.fillRect(0, 0, w, h);

  const first = rows[0];
  const baseEast = first.east_m;
  const baseNorth = first.north_m;
  const altitude = row => spaceHeight(row);
  const routePoints = spaceView.route.length ? [
    [baseEast, baseNorth, 0],
    ...spaceView.route.map(p => [baseEast + p.east_m,
                                   baseNorth + p.north_m, 8]),
  ] : [];
  const series = [
    {name: '预设航线', color: '#ffd044', points: routePoints, dash: [8, 5], width: 2.5},
    {name: '飞机真值', color: '#dbe6ed', points: rows.map(r => [r.east_m, r.north_m, altitude(r)]), width: 3},
    {name: 'PX4 GPS 测量', color: '#69baff', points: rows.map(r => [r.gps_east_m, r.gps_north_m, Number.isFinite(r.gps_up_m) ? r.gps_up_m : altitude(r)]), width: 2.5},
    {name: '注入偏移示意', color: '#d88aff', points: rows.map(r => r.attack_active ?
      [baseEast + (r.east_bias_m || 0) + (r.attack_east_m || 0),
       r.north_m + (r.north_bias_m || 0) + (r.attack_north_m || 0), altitude(r)] : null),
     dash: [7, 5], width: 2.5},
    {name: '检测器二维估计', color: '#54d6a1', points: rows.map(r => [r.estimate_east_m,
      r.estimate_north_m, altitude(r)]), width: 1.5, alpha: 0.65},
  ];
  if (!routePoints.length) series.splice(0, 1);
  const all = series.flatMap(s => s.points.filter(Boolean));
  const min = [0, 1, 2].map(i => Math.min(...all.map(p => p[i])));
  const max = [0, 1, 2].map(i => Math.max(...all.map(p => p[i])));
  min[2] = Math.min(0, min[2]);
  max[2] = Math.max(8, max[2]);
  const center = min.map((v, i) => (v + max[i]) / 2);
  const radius = Math.max(8, Math.hypot(max[0] - min[0], max[1] - min[1],
                                      (max[2] - min[2]) * 1.3) / 2);
  const scale = Math.min(w * 0.38, h * 0.37) / radius * spaceView.zoom;
  const cy = Math.cos(spaceView.yaw), sy = Math.sin(spaceView.yaw);
  const ce = Math.cos(spaceView.elevation), se = Math.sin(spaceView.elevation);
  function project(p) {
    const x = p[0] - center[0], y = p[1] - center[1], z = p[2] - center[2];
    const across = cy * x - sy * y;
    const depth = sy * x + cy * y;
    return [w / 2 + across * scale, h / 2 + (depth * se - z * ce) * scale];
  }
  function line(a, b, color, width = 1, dash = []) {
    const pa = project(a), pb = project(b);
    ctx.beginPath(); ctx.moveTo(...pa); ctx.lineTo(...pb);
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash);
    ctx.stroke(); ctx.setLineDash([]);
  }

  const axis = [baseEast, baseNorth, 0];
  const reach = Math.max(5, radius * 0.38);
  line(axis, [axis[0] + reach, axis[1], 0], '#e98d7a', 1.5);
  line(axis, [axis[0], axis[1] + reach, 0], '#86d4d4', 1.5);
  line(axis, [axis[0], axis[1], reach], '#dbe6ed', 1.5);
  ctx.font = '12px system-ui';
  for (const [end, label, color] of [
    [[axis[0] + reach, axis[1], 0], '东 E', '#e98d7a'],
    [[axis[0], axis[1] + reach, 0], '北 N', '#86d4d4'],
    [[axis[0], axis[1], reach], '高 U', '#dbe6ed'],
  ]) {
    const p = project(end); ctx.fillStyle = color; ctx.fillText(label, p[0] + 5, p[1] - 5);
  }

  ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  for (const s of series) {
    ctx.beginPath();
    let drawing = false;
    for (const value of s.points) {
      if (!value || !value.every(Number.isFinite)) { drawing = false; continue; }
      const p = project(value);
      if (!drawing) { ctx.moveTo(...p); drawing = true; }
      else ctx.lineTo(...p);
    }
    ctx.globalAlpha = s.alpha || 1;
    ctx.strokeStyle = s.color; ctx.lineWidth = s.width;
    ctx.setLineDash(s.dash || []); ctx.stroke(); ctx.setLineDash([]);
    const last = [...s.points].reverse().find(Boolean);
    if (last) {
      const p = project(last);
      ctx.beginPath(); ctx.arc(p[0], p[1], 4, 0, Math.PI * 2);
      ctx.fillStyle = s.color; ctx.fill();
    }
    ctx.globalAlpha = 1;
  }
  ctx.fillStyle = '#dfe9f2'; ctx.font = 'bold 15px system-ui';
  ctx.fillText('实时三维轨迹 · 东 / 北 / 高（米）', 18, 25);
  ctx.font = '12px system-ui';
  let x = 18, y = h - 16;
  for (const s of series) {
    ctx.fillStyle = s.color; ctx.fillRect(x, y - 7, 11, 3);
    ctx.fillStyle = '#b5c8d4'; ctx.fillText(s.name, x + 16, y);
    x += ctx.measureText(s.name).width + 32;
    if (x > w - 105) { x = 18; y -= 19; }
  }
}

function exportPlot(id, filename) {
  const canvas = document.getElementById(id);
  if (!canvas.width) return;
  canvas.toBlob(blob => {
    if (!blob) return;
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url; link.download = filename;
    document.body.appendChild(link); link.click(); link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }, 'image/png');
}

function initSpaceControls() {
  const canvas = document.getElementById('space3d');
  let pointer = null;
  canvas.addEventListener('pointerdown', e => {
    pointer = [e.clientX, e.clientY];
    canvas.setPointerCapture(e.pointerId);
    canvas.classList.add('dragging');
  });
  canvas.addEventListener('pointermove', e => {
    if (!pointer) return;
    spaceView.yaw += (e.clientX - pointer[0]) * 0.008;
    spaceView.elevation = Math.max(0.12, Math.min(1.42,
      spaceView.elevation + (e.clientY - pointer[1]) * 0.006));
    pointer = [e.clientX, e.clientY]; drawSpace();
  });
  const release = () => { pointer = null; canvas.classList.remove('dragging'); };
  canvas.addEventListener('pointerup', release);
  canvas.addEventListener('pointercancel', release);
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    spaceView.zoom = Math.max(0.45, Math.min(4, spaceView.zoom * Math.exp(-e.deltaY * 0.001)));
    drawSpace();
  }, {passive: false});
  canvas.addEventListener('dblclick', () => {
    spaceView.yaw = -0.68; spaceView.elevation = 0.62; spaceView.zoom = 1;
    drawSpace();
  });
  document.getElementById('export3d').addEventListener('click', () =>
    exportPlot('space3d', 'msim-3d-trajectory.png'));
  document.getElementById('export2d').addEventListener('click', () =>
    exportPlot('track', 'msim-2d-trajectory.png'));
  window.addEventListener('resize', drawSpace);
}
