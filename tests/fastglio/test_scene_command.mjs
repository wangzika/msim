import assert from 'assert';
import fs from 'fs';

async function main() {
  const source=fs.readFileSync(new URL('../../web/sim-console/scene-command.js',import.meta.url),'utf8');
  const {buildSceneCommand,copyCommand}=await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
  const data={scene_info:{key:'simple',command_path:'/home/wzb/msim/tools/px4-uav-capture.sh',
    choices:[{key:'simple',suggested_takeoff_height_m:8},{key:'ksql_airport',suggested_takeoff_height_m:12}],
    sensor_models:{lidar:'vlp16',gps:'standalone',imu:'low-noise'},lidar_timing:'rolling'},settings:{point_filter_num:2}};
  const manual=buildSceneCommand(data,'ksql_airport','teleop');
  assert(manual.startsWith('/home/wzb/msim/tools/px4-uav-capture.sh --web --teleop --teleop-height-m 12'));
  for (const flag of ['--scene ksql_airport','--gps-model standalone','--imu-model low-noise','--lidar-timing rolling','--point-filter-num 2']) assert(manual.includes(flag));
  data.scene_info.sensor_models.gps='f9p-dgnss-ucr';
  assert(buildSceneCommand(data,'ksql_airport','teleop').includes('--gps-model f9p-dgnss-ucr'));
  const automatic=buildSceneCommand(data,'ksql_airport','route');
  assert(automatic.includes('--flight-pattern route'));assert(!automatic.includes('--teleop'));
  const autonomous=buildSceneCommand(data,'simple','autonomy');
  assert(autonomous.includes('--web --autonomy --teleop-height-m 5 --return-after-s 40'));
  assert(!autonomous.includes('--teleop '));assert(!autonomous.includes('--flight-pattern'));
  const exploring=buildSceneCommand(data,'ksql_airport','explore',{goal:[-20,300,12],speed:2,radius:8});
  assert(exploring.includes('--goal-enu -20 300 12 --mission-speed-mps 2'));
  assert(!exploring.includes('--flight-pattern'));assert(exploring.includes('--goal-radius-m 8'));
  assert.strictEqual(buildSceneCommand(data,'ksql_airport','explore',{goal:[0,10,12],speed:2,radius:6}),'');
  assert.strictEqual(buildSceneCommand(data,'ksql_airport','explore',{goal:[0,300,NaN],speed:2}),'');
  assert.strictEqual(buildSceneCommand(data,'ksql_airport','explore',{goal:[0,300,12],speed:99}),'');
  assert.strictEqual(buildSceneCommand(data,'../invalid','teleop'),'');
  let written='',selected=0;
  assert(await copyCommand(manual,{writeText:async text=>{written=text;}},()=>selected++,()=>false));
  assert.strictEqual(written,manual);assert.strictEqual(selected,1);
  assert(await copyCommand(manual,{writeText:async()=>{throw new Error('denied');}},()=>selected++,()=>true));
  assert.strictEqual(selected,2);
  assert.strictEqual(await copyCommand(manual,null,()=>selected++,()=>false),false);
  console.log('Scene commands: absolute executable, modes, sensor settings, safe scene keys and clipboard fallback passed');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
