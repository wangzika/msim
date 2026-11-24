import {SlamViewer} from '/slam-viewer.js';
import {movementKey,FlightInput,CommandQueue} from '/flight-input.js';
import {buildSceneCommand,copyCommand} from '/scene-command.js';

const $ = id => document.getElementById(id);
const lastOf = values => values?.[values.length - 1];
const format = (value, digits = 1) => Number.isFinite(value) ? value.toFixed(digits) : '—';
const count = value => Number.isFinite(value) ? value.toLocaleString('zh-CN') : '—';
const activeKeys = new Set();
const flightInput = new FlightInput();
const state = {data: null, connected: false, panel: 'map', paused: false, revisions: {}, metadata: {},
  fetching: false, fetchingCloud: false, sceneInitialized: false,sceneModelsPromise:null,sceneModelStatus:''};
let mapView, sceneView;
const currentView = () => state.panel === 'map' ? mapView : sceneView;
const interactive = () => state.connected && state.data?.control_available && !state.data?.complete;

function updateCameraUi() {
  const view = currentView();if (!view) return;
  $('followPose').checked = view.follow;
  document.querySelectorAll('[data-view]').forEach(button => button.classList.toggle('selected', button.dataset.view === view.viewMode));
}

function initializeViewers() {
  try {
    mapView = new SlamViewer($('mapViewport'), {onChange: updateCameraUi});
    sceneView = new SlamViewer($('sceneViewport'), {sceneMode: true, onChange: updateCameraUi});
  } catch (error) {
    $('mapEmpty').innerHTML = '<h2>三维渲染未能启动</h2><p>请使用支持 WebGL 2 的浏览器，并检查图形加速是否可用。</p>';
    $('dataDescription').textContent = `WebGL：${error.message}`;
    console.error(error);
  }
}

function selectPanel(panel) {
  state.panel = panel;
  $('mapPanel').hidden = panel !== 'map';$('scenePanel').hidden = panel !== 'scene';
  $('mapLegend').hidden=panel !== 'map';$('layersPanel').hidden=panel !== 'map';
  for (const name of ['map', 'scene']) {
    $(`${name}Tab`).classList.toggle('selected', name === panel);
    $(`${name}Tab`).setAttribute('aria-selected', String(name === panel));
  }
  $('mapFrame').textContent = panel === 'map' ? (state.metadata.map?.frame || 'camera_init') : 'Gazebo ENU';
  currentView()?.resize();
  if (currentView() && !currentView().fitted) {
    if (panel==='scene' && sceneView.modelBounds) sceneView.fitSceneContext(sceneView.sceneExtent==='all');else currentView().fit();
  }
  updateCameraUi();
  if (state.data) updateCoordinates(state.data);
  if (panel==='scene') ensureSceneModels();
}

async function ensureSceneModels() {
  if (!state.data || !sceneView || state.sceneModelsPromise) return;
  const runName=state.data.run;
  state.sceneModelStatus='正在加载实际场景模型与纹理…';$('sceneGeometry').textContent=state.sceneModelStatus;
  state.sceneModelsPromise=(async()=>{
    try {
      const response=await fetch('/api/scene-models');const description=await response.json();
      if (!response.ok) throw new Error(description.error || '场景资源读取失败');
      if (state.data?.run!==runName) return;
      const result=await sceneView.loadSceneModels(description);
      if (state.data?.run!==runName || !result) return;
      const size=result.bounds.getSize(result.bounds.min.clone());
      state.sceneModelStatus=`实际场景 ${result.objects} 组 · 范围 ${Math.round(size.x)} × ${Math.round(size.y)} m${description.warnings.length ? ' · 部分资源未加载' : ''}`;
    } catch(error) {state.sceneModelStatus=`场景模型加载失败：${error.message}`;console.error(error);}
    $('sceneGeometry').textContent=state.sceneModelStatus;
  })();
}

function updateCoordinates(data) {
  if (state.panel === 'scene') {
    const row=lastOf(data.samples);
    $('viewCoordinates').textContent=row ? `E ${format(row.east_m)}　N ${format(row.north_m)}　U ${format(row.up_m)} m` : 'E —　N —　U —';
  } else {
    const pose=data.live?.pose?.lio?.position || lastOf(data.live?.lio);
    $('viewCoordinates').textContent=pose ? `X ${format(pose[0])}　Y ${format(pose[1])}　Z ${format(pose[2])} m` : 'X —　Y —　Z —';
  }
}

function layerVisibility() {
  if (!mapView) return;
  document.querySelectorAll('[data-layer]').forEach(input => mapView.setVisible(input.dataset.layer, input.checked));
}

function renderGeometry(data) {
  if (!mapView || state.paused) return;
  const live = data.live || {};
  for (const key of ['lio', 'loose']) {
    const frame = live.frames?.[key] || live.cloud_frame || 'camera_init';
    mapView.setTrack(key, frame === 'camera_init' ? (live[key] || []) : []);
  }
  const pose = live.pose?.lio || (lastOf(live.lio) ? {position: lastOf(live.lio)} : null);
  if (pose) mapView.setPose(pose);
  layerVisibility();
  const rows = data.samples || [], settings = data.settings || {};
  if (!state.sceneInitialized) {sceneView.setBoxes(data.scene || []);state.sceneInitialized = true;}
  sceneView.setTrack('truth', rows.map(row => [row.east_m, row.north_m, row.up_m]));
  sceneView.setTrack('gps', rows.map(row => [row.gps_east_m, row.gps_north_m, row.gps_up_m]));
  const origin = settings.spawn_enu_m || (rows[0] ? [rows[0].east_m, rows[0].north_m, rows[0].up_m] : [0, 0, 0]);
  sceneView.setTrack('route', (data.route || []).map(p => [origin[0] + p.east_m, origin[1] + p.north_m,
    origin[2] + (settings.flight_height_m || 8)]));
  const latest = lastOf(rows);
  if (live.pose?.truth) sceneView.setPose(live.pose.truth);
  else if (latest) sceneView.setPose({position: [latest.east_m, latest.north_m, latest.up_m]});
  if (!sceneView.fitted) sceneView.fit();
  $('sceneGeometry').textContent=state.sceneModelStatus || `${(data.scene || []).length} 个碰撞体 · 切换到场景视图加载实际模型`;
  // Load static geometry during initial setup so opening the scene while flying
  // does not have to parse a large COLLADA file on the first key press.
  ensureSceneModels();
}

function updateCloudLabels() {
  const map = state.metadata.map, scan = state.metadata.current;
  const displayed = mapView?.layers.map.geometry.getAttribute('position').count || 0;
  $('mapCount').textContent = map ? `${count(displayed)} / ${count(map.total)}` : '等待';
  $('scanCount').textContent = scan ? count(scan.count) : '无数据';
  $('pointSummary').textContent = map ? `${count(displayed)} 点 · 地图共 ${count(map.total)} 点` : '等待累计点云';
  const sources = {'/msim/lio_map': '累计地图 · 实时', registered_scan_accumulation: '配准帧累计 · 实时', 'map.pcd': '已保存地图 · 记录预览'};
  $('cloudSource').textContent = sources[map?.source] || (scan ? '当前配准帧' : '等待地图');
  $('mapEmpty').hidden = !!((map?.count || 0) + (scan?.count || 0));
  if (state.panel === 'map') $('mapFrame').textContent = map?.frame || scan?.frame || 'camera_init';
}

function updateSceneCommand() {
  const key = $('sceneSelect').value;
  const scene = state.data?.scene_info?.choices?.find(value => value.key === key);
  $('sceneDescription').textContent = scene?.description || '';
  const mode=$('captureMode').value;
  const next=buildSceneCommand(state.data,key,mode);
  // Reassigning an unchanged textarea value clears a manual-copy selection.
  if ($('sceneCommand').value!==next) $('sceneCommand').value=next;
  $('copySceneCommand').disabled=!next;$('downloadSceneScript').hidden=!next;
  $('downloadSceneScript').href=`/api/launch.sh?scene=${encodeURIComponent(key)}&mode=${mode}`;
  $('commandModeHelp').textContent=mode==='teleop' ? '手动模式：在终端启动后，回到网页点击“起飞”。' : '自动航线：启动后自动起飞并执行航线；网页没有手动起飞按钮。';
}

function renderTelemetry(data) {
  const latest = lastOf(data.samples), teleop = data.teleop || {}, settings = data.settings || {};
  $('runName').textContent = data.run;
  $('sceneName').textContent = data.scene_info?.name || '自定义场景';
  $('simTime').textContent = `仿真时间 ${latest ? format(latest.sim_time_ns / 1e9) : '—'} s`;
  $('modeBadge').textContent = data.complete ? '记录预览' : settings.interactive ? '网页手动飞行' : '自动航线 · 自动起飞';
  $('flightMode').textContent = teleop.mode || settings.flight_pattern || '—';
  $('gpsStatus').textContent = latest ? ({normal: '正常', alarm: '报警'}[latest.detector_status] || '初始化') : '等待 GPS';
  $('gpsStatus').style.color = latest?.detector_status === 'alarm' ? '#ef8e86' : '#c7dae7';
  $('position').textContent = latest ? `${format(latest.east_m)} / ${format(latest.north_m)} / ${format(latest.up_m)} m` : '—';
  $('occlusion').textContent = latest && Number.isFinite(latest.sky_blocked_fraction) ? `${format(latest.sky_blocked_fraction * 100, 0)} %` : '—';
  $('noiseScale').textContent = Number.isFinite(latest?.environment_noise_scale) ? `${format(latest.environment_noise_scale, 2)} ×` : '—';
  $('controls').hidden = !data.control_available;
  $('teleopState').textContent = ({ready:'已准备',taking_off:'起飞中',flying:'飞行中',landing:'降落中',landed:'已降落'}[teleop.state] || teleop.state || '等待飞控');
  $('downloadMap').hidden = !data.map_saved;
  $('connectionDot').className = data.complete ? 'dot' : 'dot live';
  $('connection').textContent = data.complete ? '记录已加载' : '实时采集';
  $('looseAvailability').textContent = data.live?.loose?.length ? '有数据' : '无此结果';
  if (teleop.finish_requested) {$('commandStatus').textContent = '正在降落，完成后保存数据。';$('finish').disabled = true;}
  updateCoordinates(data);
}

async function refreshState() {
  if (state.fetching) return;state.fetching = true;
  try {
    const response = await fetch('/api/state', {cache: 'no-store'});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();state.connected = true;
    if (state.data?.run !== data.run) {
      clearKeys();state.revisions = {};state.metadata = {};state.sceneInitialized = false;state.sceneModelsPromise=null;state.sceneModelStatus='';
      mapView?.reset();sceneView?.reset();$('finish').disabled = false;$('exportResult').hidden=true;
      if (['simple', 'complex_urban', 'industrial_yard', 'ksql_airport'].includes(data.scene_info?.key))
        $('sceneSelect').value = data.scene_info.key;
      $('captureMode').value = data.settings?.interactive ? 'teleop' : 'route';
      $('height').value = Number.isFinite(data.settings?.teleop_takeoff_height_m) ? data.settings.teleop_takeoff_height_m : 5;
      $('speed').value = Number.isFinite(data.settings?.teleop_speed_mps) ? data.settings.teleop_speed_mps : 2;
    }
    state.data = data;renderTelemetry(data);renderGeometry(data);updateSceneCommand();
  } catch (error) {
    state.connected = false;activeKeys.clear();
    $('connectionDot').className = 'dot error';
    $('connection').textContent = state.data ? '服务已停止 · 保留最后画面' : '连接中断';
    if (state.data) $('commandStatus').textContent = '采集服务已停止。';
    console.error('仿真状态读取失败', error);
  } finally {state.fetching = false;}
}

async function refreshClouds() {
  if (!mapView || state.paused || state.fetchingCloud) return;state.fetchingCloud = true;
  try {
    await Promise.all(['map', 'current'].map(async key => {
      const response = await fetch(`/api/cloud?layer=${key}&revision=${encodeURIComponent(state.revisions[key] || '')}`, {cache: 'no-store'});
      if (response.status === 204) return;
      if (!response.ok) throw new Error(`点云 HTTP ${response.status}`);
      const buffer = await response.arrayBuffer(), number = Number(response.headers.get('X-Point-Count'));
      if (buffer.byteLength !== number * 12 || number > 90000) throw new Error('点云数据长度不匹配');
      const frame = response.headers.get('X-Cloud-Frame');
      if (frame !== 'camera_init') throw new Error(`点云坐标系不匹配：${frame}`);
      const values = new Float32Array(buffer);
      if (!values.every(Number.isFinite)) throw new Error('点云包含无效坐标');
      state.revisions[key] = response.headers.get('X-Cloud-Revision');
      state.metadata[key] = {count: number, total: Number(response.headers.get('X-Point-Total')), frame,
        source: response.headers.get('X-Cloud-Source')};
      mapView.setCloud(key, values);layerVisibility();
    }));
    updateCloudLabels();
  } catch (error) {$('cloudSource').textContent = error.message;console.error(error);}
  finally {state.fetchingCloud = false;}
}

let fetchingPoses=false;
async function refreshPoses() {
  if (fetchingPoses || state.paused || !state.connected || !state.data?.streaming) return;
  fetchingPoses=true;
  try {
    const response=await fetch('/api/poses',{cache:'no-store'});if (!response.ok) return;
    const data=await response.json();
    if (data.pose?.truth) sceneView?.setPose(data.pose.truth);
    if (data.pose?.lio && data.frames?.lio==='camera_init') mapView?.setPose(data.pose.lio);
  } catch(_) {} finally {fetchingPoses=false;}
}

async function sendCommand(value) {
  if (!interactive()) return;
  const abort=new AbortController(), timer=setTimeout(()=>abort.abort(),1200), started=performance.now();
  try {
    const response=await fetch('/api/teleop/command',{method:'POST',signal:abort.signal,
      headers:{'Content-Type':'application/json'},body:JSON.stringify(value)});
    const result=await response.json();if (!response.ok) throw new Error(result.error || '指令失败');
    $('commandLatency').textContent=`指令往返 ${Math.round(performance.now()-started)} ms`;
    $('commandStatus').textContent=value.op==='finish' ? '正在降落，完成后保存数据。' : '';
  } finally {clearTimeout(timer);}
}
const commandQueue=new CommandQueue(sendCommand,error=>{
  flightInput.clear();activeKeys.clear();syncKeys();
  $('commandStatus').textContent=`控制连接异常：${error.message}。已停止按键续发。`;
});
function command(value) {if (interactive()) return commandQueue.push(value);}
function syncKeys() {
  activeKeys.clear();for (const key of flightInput.keys()) activeKeys.add(key);
  document.querySelectorAll('[data-key]').forEach(button=>button.classList.toggle('active',activeKeys.has(button.dataset.key)));
  $('inputHint').textContent=activeKeys.size ? `正在操纵：${[...activeKeys].join(' / ').toUpperCase()}` : '↑↓←→ 或 W/A/S/D 平移，R/F 升降，Q/E 转向。';
}
function clearKeys(hold=false) {
  flightInput.clear();syncKeys();
  if (interactive()) command(hold ? {op:'hold'} : {op:'keys',keys:[]});
}
function enableKeyboard() {
  currentView()?.renderer.domElement.focus({preventScroll:true});
  $('inputHint').textContent='键盘已启用：方向键或 W/A/S/D 移动；空格悬停。';
}
function setupFlightControls() {
  $('enableKeyboard').addEventListener('click',enableKeyboard);
  document.querySelectorAll('[data-op]').forEach(button=>button.addEventListener('click',()=>{
    clearKeys();const op=button.dataset.op;enableKeyboard();
    command(op==='takeoff' ? {op,height_m:Number($('height').value)} : {op});
  }));
  $('finish').addEventListener('click',()=>{clearKeys();command({op:'finish'});});
  $('speed').addEventListener('change',()=>command({op:'speed',value:Number($('speed').value)}));
  for (const id of ['height','speed']) $(id).addEventListener('keydown',event=>{
    if (event.key==='Enter') {event.preventDefault();event.target.blur();enableKeyboard();}
  });
  for (const button of document.querySelectorAll('[data-key]')) {
    const key=button.dataset.key;
    button.addEventListener('pointerdown',event=>{
      if (!interactive()) return;
      event.preventDefault();enableKeyboard();button.setPointerCapture(event.pointerId);
      flightInput.press(`pointer:${event.pointerId}`,key);syncKeys();command({op:'keys',keys:[...activeKeys]});
    });
    const release=event=>{
      if (!flightInput.release(`pointer:${event.pointerId}`)) return;
      syncKeys();command(activeKeys.size ? {op:'keys',keys:[...activeKeys]} : {op:'hold'});
    };
    button.addEventListener('pointerup',release);button.addEventListener('pointercancel',release);
    button.addEventListener('lostpointercapture',release);
  }
  document.addEventListener('keydown',event=>{
    if (!interactive()) return;
    if (event.key==='Escape') {event.preventDefault();clearKeys(true);enableKeyboard();return;}
    if (['INPUT','TEXTAREA','SELECT'].includes(document.activeElement.tagName)) {
      if (movementKey(event.key)) $('inputHint').textContent='正在编辑设置，飞行按键暂停。按 Enter 或点击“启用键盘”。';
      return;
    }
    const key=movementKey(event.key);
    if (key) {
      event.preventDefault();const wasActive=activeKeys.has(key);
      flightInput.press(`keyboard:${event.code || event.key}`,key);syncKeys();
      if (!wasActive) command({op:'keys',keys:[...activeKeys]});
    } else if (event.key===' ') {event.preventDefault();clearKeys(true);}
    else if (event.key.toLowerCase()==='t' && !event.repeat) command({op:'takeoff',height_m:Number($('height').value)});
    else if (event.key.toLowerCase()==='l' && !event.repeat) command({op:'land'});
  });
  document.addEventListener('keyup',event=>{
    if (flightInput.release(`keyboard:${event.code || event.key}`)) {
      syncKeys();command(activeKeys.size ? {op:'keys',keys:[...activeKeys]} : {op:'hold'});
    }
  });
  // Clearing the velocity snapshot stops movement without cancelling a takeoff.
  // An explicit Hold/Esc remains available when the pilot wants to abort climbing.
  addEventListener('blur',()=>{clearKeys();$('inputHint').textContent='页面失焦，方向输入已停止；点击“启用键盘”继续操纵。';});
  document.addEventListener('visibilitychange',()=>{if (document.hidden) clearKeys();});
  document.addEventListener('focusin',event=>{
    if (['INPUT','SELECT','TEXTAREA'].includes(event.target.tagName)) {clearKeys();$('inputHint').textContent='正在编辑设置，按 Enter 或点击“启用键盘”返回操纵。';}
  });
  setInterval(()=>{if (interactive() && !document.hidden && activeKeys.size) command({op:'keys',keys:[...activeKeys]});},80);
}

function setupDisplayControls() {
  $('mapTab').addEventListener('click', () => selectPanel('map'));
  $('sceneTab').addEventListener('click', () => selectPanel('scene'));
  $('sceneLocalView').addEventListener('click',()=>sceneView?.fitSceneContext());
  $('sceneWholeView').addEventListener('click',()=>sceneView?.fitSceneContext(true));
  $('sceneShowModels').addEventListener('change',()=>{if(sceneView)sceneView.modelGroup.visible=$('sceneShowModels').checked;});
  document.querySelectorAll('[data-view]').forEach(button => button.addEventListener('click', () => currentView()?.setView(button.dataset.view)));
  $('fitView').addEventListener('click', () => currentView()?.fit());
  $('followPose').addEventListener('change', () => currentView()?.setFollow($('followPose').checked));
  $('pauseView').addEventListener('click', () => {
    state.paused = !state.paused;$('pauseBanner').hidden = !state.paused;
    $('pauseView').textContent = state.paused ? '继续显示' : '暂停显示';
    if (!state.paused) {if (state.data) renderGeometry(state.data);refreshClouds();}
  });
  $('exportImage').addEventListener('click', async () => {
    const view=currentView();if (!view) return;
    $('exportImage').disabled=true;
    try {
      const blob=await view.exportPng();
      const response=await fetch('/api/export/png',{method:'POST',headers:{'Content-Type':'image/png'},body:blob});
      const result=await response.json();if (!response.ok) throw new Error(result.error || '保存失败');
      const link=$('exportResult');link.href=result.url;link.download=`msim-${state.panel}-${state.data?.run || 'view'}.png`;
      link.textContent='PNG 已保存 · 点击下载';link.hidden=false;
    } catch(error) {$('dataDescription').textContent=`图片导出失败：${error.message}`;}
    finally {$('exportImage').disabled=false;}
  });
  $('fullscreen').addEventListener('click', async () => {
    try {if (document.fullscreenElement) await document.exitFullscreen();else await document.querySelector('.workspace-main').requestFullscreen();}
    catch (error) {$('dataDescription').textContent = `无法进入全屏：${error.message}`;}
  });
  document.querySelectorAll('[data-layer]').forEach(input => input.addEventListener('change', layerVisibility));
  $('pointSize').addEventListener('input', () => {mapView?.setPointSize(Number($('pointSize').value));$('pointSizeValue').textContent = `${Number($('pointSize').value).toFixed(1)} px`;});
  $('colorMode').addEventListener('change', () => mapView?.setColorMode($('colorMode').value));
  $('pointBudget').addEventListener('change', () => {mapView?.setBudget(Number($('pointBudget').value));updateCloudLabels();});
  $('sceneSelect').addEventListener('change', updateSceneCommand);$('captureMode').addEventListener('change', updateSceneCommand);
  $('copySceneCommand').addEventListener('click', async () => {
    const copied=await copyCommand($('sceneCommand').value,navigator.clipboard,
      ()=>{$('sceneCommand').focus();$('sceneCommand').select();},()=>document.execCommand('copy'));
    $('sceneCommandStatus').textContent=copied ? '已尝试复制，命令保持选中。请到终端粘贴并按 Enter；若粘贴为空，在此按 Ctrl+C 再粘贴，或下载启动脚本。复制本身不会启动仿真。' : '浏览器未允许复制。命令已选中，请按 Ctrl+C 再粘贴，或下载启动脚本。';
  });
}

initializeViewers();setupFlightControls();setupDisplayControls();refreshState();refreshClouds();
setInterval(refreshState, 1000);setInterval(refreshClouds, 700);
setInterval(refreshPoses,100);
setInterval(() => {if (mapView) $('fps').textContent = `${mapView.fps} FPS`;}, 1000);
