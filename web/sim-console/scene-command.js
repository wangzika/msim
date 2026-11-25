const quote=value=>/^[A-Za-z0-9_./-]+$/.test(value) ? value : "'"+value.replace(/'/g,"'\\''")+"'";

export function buildSceneCommand(data,sceneKey,mode,mission=null) {
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
  const height=sceneKey===info.key && Number.isFinite(settings.teleop_takeoff_height_m) ? settings.teleop_takeoff_height_m : mode==='autonomy' && sceneKey==='simple' ? 5 : selected.suggested_takeoff_height_m || 8;
  let flags=mode==='autonomy' ? `--web --autonomy --teleop-height-m ${height} --return-after-s 40` : mode==='teleop' ? `--web --teleop --teleop-height-m ${height}` : '--web --duration-s 300 --flight-pattern route --route-clock sim --skip-signalsim';
  if(mode==='explore') {
    const goal=mission?.goal || settings.goal_relative_enu_m || [0,300,12], speed=mission?.speed ?? 2, radius=mission?.radius ?? 2;
    if(goal.length!==3 || !goal.every(Number.isFinite) || Math.hypot(...goal.slice(0,2))<5 || Math.hypot(...goal.slice(0,2))>1000 || goal[2]<3 || goal[2]>30 || !Number.isFinite(speed) || speed<.3 || speed>3 || !Number.isFinite(radius) || radius<.5 || radius>30 || radius>=Math.hypot(...goal.slice(0,2))/2) return '';
    flags=`--web --goal-enu ${goal.join(' ')} --mission-speed-mps ${speed} --goal-radius-m ${radius} --autonomy-timeout-s 900`;
  }
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
