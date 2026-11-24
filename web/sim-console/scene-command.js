const quote=value=>/^[A-Za-z0-9_./-]+$/.test(value) ? value : "'"+value.replace(/'/g,"'\\''")+"'";

export function buildSceneCommand(data,sceneKey,mode) {
  const info=data && data.scene_info || {}, settings=data && data.settings || {};
  const choices=info.choices || [];
  if (!choices.some(scene=>scene.key===sceneKey)) return '';
  const models=info.sensor_models || {};
  const lidar=['vlp16','hdl32','os1-64','mid360'].includes(models.lidar || info.lidar_model) ? (models.lidar || info.lidar_model) : 'vlp16';
  const gps=['f9p-dgnss-ucr','standalone','native','ideal'].includes(models.gps) ? models.gps : 'f9p-dgnss-ucr';
  const imu=['mems-baseline','low-noise'].includes(models.imu) ? models.imu : 'mems-baseline';
  const timing=['rolling','snapshot'].includes(info.lidar_timing) ? info.lidar_timing : 'rolling';
  const filter=Number.isInteger(settings.point_filter_num) && settings.point_filter_num>0 && settings.point_filter_num<=100 ? settings.point_filter_num : 2;
  const executable=quote(info.command_path || '/home/wzb/msim/tools/px4-uav-capture.sh');
  const selected=choices.find(scene=>scene.key===sceneKey);
  const height=sceneKey===info.key && Number.isFinite(settings.teleop_takeoff_height_m) ? settings.teleop_takeoff_height_m : selected.suggested_takeoff_height_m || 8;
  const flags=mode==='teleop' ? `--web --teleop --teleop-height-m ${height}` : '--web --duration-s 300 --flight-pattern route --route-clock sim --skip-signalsim';
  return `${executable} ${flags} --scene ${sceneKey} --lidar-model ${lidar} --imu-model ${imu} --gps-model ${gps} --lidar-timing ${timing} --point-filter-num ${filter}`;
}

export async function copyCommand(text,clipboard,select,legacyCopy) {
  // Keep this synchronous part inside the trusted click gesture. Some embedded
  // browsers resolve writeText without forwarding text to the system clipboard.
  select();
  try {if (legacyCopy()) return true;} catch (_) {}
  if (clipboard && clipboard.writeText) {
    try {await clipboard.writeText(text);return true;} catch (_) {}
  }
  return false;
}
