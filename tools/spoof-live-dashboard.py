#!/usr/bin/env python3
"""Local read-only dashboard for an online PX4 GNSS injection run."""

import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path


HTML = r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>msim · GNSS 欺骗检测</title>
<style>
:root{font-family:system-ui,sans-serif;color:#dfe9f2;background:#09131d}*{box-sizing:border-box}
body{margin:0;padding:28px;max-width:1400px;margin:auto}h1{font-size:28px;margin:0 0 5px}
.sub{color:#8da7b7;margin:0 0 22px}.cards{display:grid;grid-template-columns:repeat(5,1fr);gap:12px}
.card,.panel{background:#102331;border:1px solid #254051;border-radius:14px;padding:18px}
.label{color:#89a7b8;font-size:13px}.value{font-size:26px;font-weight:700;margin-top:8px}#score{font-size:21px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-top:14px}
canvas{width:100%;height:340px;display:block}h2{font-size:17px;margin:0 0 10px}
.panel-head{display:flex;align-items:center;justify-content:space-between;gap:12px}
.export{color:#dfe9f2;background:#23465e;border:1px solid #47758f;border-radius:8px;padding:8px 12px;cursor:pointer;white-space:nowrap}
.export:hover{background:#2f607d}.export:disabled{opacity:.45;cursor:default}
#space3d{height:460px;touch-action:none;cursor:grab}#space3d.dragging{cursor:grabbing}
.legend{font-size:13px;color:#a8c2d0;margin:8px 0 0}.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin:0 5px 0 16px}
.dot:first-child{margin-left:0}.note{color:#88a2b1;font-size:13px;line-height:1.5;margin-top:18px}
.alarm{color:#ff7676}.normal{color:#54d6a1}.waiting{color:#f2c66d}
.outcome{background:#2b1721;border:1px solid #a34b5a;border-radius:16px;padding:20px;margin:0 0 14px}
.outcome h2{font-size:23px;color:#ff9b9b;margin:0 0 8px}.outcome p{margin:0;color:#ffe0df;line-height:1.5}
.outcome.pending{background:#172530;border-color:#3d6377}.outcome.pending h2{color:#f2c66d}
.summary{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin-bottom:14px}
.summary strong{display:block;font-size:25px;margin-top:7px}.wide{margin:0 0 14px}
@media(max-width:900px){.cards{grid-template-columns:repeat(2,1fr)}.grid{grid-template-columns:1fr}}
@media(max-width:600px){.summary{grid-template-columns:repeat(2,1fr)}.summary strong{font-size:18px}}
@media(max-width:450px){body{padding:16px}.card,.panel{padding:13px}.value{font-size:20px}#score{font-size:16px}.outcome h2{font-size:20px}}
</style>
<h1>GNSS 欺骗检测</h1><p class="sub">PX4 在线注入 · 仿真时间 · 每秒刷新 · <span id="runInfo">等待实验</span></p>
<section id="outcome" class="outcome pending"><h2 id="headline">等待仿真数据</h2><p id="explain">启动采集后，这里会直接显示攻击与报警结果。</p></section>
<div class="summary"><div class="card"><div class="label">攻击开始</div><strong id="attackStart">—</strong></div>
<div class="card"><div class="label">首次报警</div><strong id="alarmTime">—</strong></div>
<div class="card"><div class="label">报警延迟</div><strong id="delay">—</strong></div>
<div class="card"><div class="label" id="physicalLabel">飞机真实水平位移</div><strong id="physicalMove">—</strong></div></div>
<section class="panel wide" id="space3dPanel"><div class="panel-head"><h2>实时三维空间图</h2><button class="export" id="export3d" type="button" disabled>导出 3D PNG</button></div>
<canvas id="space3d" aria-label="可旋转的东、北、高三维轨迹图"></canvas>
<p class="note">拖动旋转，滚轮缩放，双击恢复视角。三维图使用飞机真实高度；绿色检测器仅估计水平位置，其显示高度借用真值。紫色是注入偏移示意，并非 PX4 接收的位置。</p></section>
<div class="panel wide"><h2>注入位置偏移（米）</h2><canvas id="drift"></canvas>
<p class="legend"><span class="dot" style="background:#ff7676"></span>攻击开始<span class="dot" style="background:#f2c66d"></span>首次报警<span class="dot" style="background:#69baff"></span>GNSS 偏移</p></div>
<div class="cards"><div class="card"><div class="label">检测状态</div><div id="status" class="value waiting">等待数据</div></div>
<div class="card"><div class="label">最大检测量 / 阈值</div><div id="score" class="value">—</div></div>
<div class="card"><div class="label">注入偏移</div><div id="offset" class="value">—</div></div>
<div class="card"><div class="label">本次运行时间</div><div id="time" class="value">—</div></div>
<div class="card"><div class="label">LiDAR 与 GPS 时间差</div><div id="lidarAge" class="value waiting">—</div></div></div>
<p id="lidarSyncDetails" class="note">等待 LiDAR 同步数据。</p>
<div class="grid"><div class="panel"><div class="panel-head"><h2>水平轨迹 (东 / 北，米)</h2><button class="export" id="export2d" type="button" disabled>导出 2D PNG</button></div><canvas id="track"></canvas>
<p class="legend"><span class="dot" style="background:#ffd044"></span>预设航线<span class="dot" style="background:#d88aff"></span>注入偏移示意<span class="dot" style="background:#69baff"></span>PX4 接收的假 GPS<span class="dot" style="background:#54d6a1"></span>检测器估计<span class="dot" style="background:#8293a3"></span>飞机真值</p>
<p class="note">紫色虚线＝航线东向位置＋注入偏移，北向坐标沿用飞机真值；它只用于展示攻击量，不是发给 PX4 的 GPS 坐标。蓝色线才是 PX4 实际接收的假 GPS 轨迹。</p></div>
<div class="panel"><h2>检测统计量（超过 80 显示在顶部）</h2><canvas id="chi"></canvas>
<p class="legend"><span class="dot" style="background:#69baff"></span>单历元 χ²<span class="dot" style="background:#e8a86c"></span>10 历元趋势<span class="dot" style="background:#bca6ff"></span>短时惯导参考<span class="dot" style="background:#ff7676"></span>阈值</p></div></div>
<p class="note">检测器只使用 IMU 和注入的 GNSS 位置；灰色真值仅在图上评估。报警只用于记录和显示，不会接管 PX4 或停止假 GPS。10 历元趋势与短时惯导参考的统计阈值尚未实测标定。固定偏移在启动前已存在时，检测器无法仅靠自身识别。</p>
<script src="/spoof-dashboard-3d.js"></script><script>
function chart(id, series, bounds, markers=[]){
 const c=document.getElementById(id), dpr=window.devicePixelRatio||1, w=c.clientWidth,h=c.clientHeight;
 c.width=w*dpr;c.height=h*dpr;const x=c.getContext('2d');x.scale(dpr,dpr);x.clearRect(0,0,w,h);
 const p={l:48,r:18,t:16,b:32},pw=w-p.l-p.r,ph=h-p.t-p.b;
 const sx=v=>p.l+(v-bounds[0])/(bounds[1]-bounds[0]||1)*pw,sy=v=>h-p.b-(Math.max(bounds[2],Math.min(bounds[3],v))-bounds[2])/(bounds[3]-bounds[2]||1)*ph;
 x.strokeStyle='#284454';x.lineWidth=1;x.fillStyle='#8da7b7';x.font='12px system-ui';
 for(let i=0;i<=4;i++){let yy=p.t+i*ph/4;x.beginPath();x.moveTo(p.l,yy);x.lineTo(w-p.r,yy);x.stroke();x.fillText((bounds[3]-i*(bounds[3]-bounds[2])/4).toFixed(1),3,yy+4)}
 x.fillText(bounds[0].toFixed(1),p.l,h-8);x.fillText(bounds[1].toFixed(1),w-48,h-8);
 markers.forEach(m=>{if(!Number.isFinite(m.at))return;x.strokeStyle=m.color;x.setLineDash([5,4]);x.beginPath();x.moveTo(sx(m.at),p.t);x.lineTo(sx(m.at),h-p.b);x.stroke();x.setLineDash([])});
 series.forEach(s=>{x.strokeStyle=s.color;x.lineWidth=s.width||2;x.setLineDash(s.dash||[]);x.beginPath();let begun=false;s.points.forEach(v=>{if(!v||!Number.isFinite(v[0])||!Number.isFinite(v[1]))return;if(!begun){x.moveTo(sx(v[0]),sy(v[1]));begun=true}else x.lineTo(sx(v[0]),sy(v[1]))});x.stroke();x.setLineDash([])});
}
function extent(values,pad=1){const a=values.filter(Number.isFinite);if(!a.length)return[0,1];let lo=Math.min(...a),hi=Math.max(...a);if(hi-lo<1){lo-=.5;hi+=.5}return[lo-pad,hi+pad]}
async function refresh(){try{let response=await fetch('/api/samples',{cache:'no-store'});if(!response.ok)throw Error('读取失败');let complete=response.headers.get('X-Run-Complete')==='1',pattern=response.headers.get('X-Flight-Pattern'),hover=pattern==='hover',routeMode=pattern==='route';let rows=await response.json();document.getElementById('runInfo').textContent=(response.headers.get('X-Run-Name')||'等待实验')+(complete?' · 已完成':' · 实时采集');if(!rows.length)return;
 let route=routeMode?await(await fetch('/api/route',{cache:'no-store'})).json():[];
 if(routeMode){let summary=document.querySelector('.summary'),space=document.getElementById('space3dPanel');summary.after(document.querySelector('.grid'));summary.after(space)}
 let base=rows[0].sim_time_ns/1e9,attack=rows.find(v=>v.attack_active),firstAlarm=rows.find(v=>v.detector_status==='alarm');
 let r=rows[rows.length-1],physicalMove=Math.hypot(r.east_m-rows[0].east_m,r.north_m-rows[0].north_m),gpsMove=Math.hypot(r.gps_east_m-rows[0].gps_east_m,r.gps_north_m-rows[0].gps_north_m);
 let attackT=attack?attack.sim_time_ns/1e9-base:null,alarmT=firstAlarm?firstAlarm.sim_time_ns/1e9-base:null;
 document.getElementById('attackStart').textContent=attackT===null?'未开始':attackT.toFixed(1)+' s';
 document.getElementById('alarmTime').textContent=alarmT===null?'未报警':alarmT.toFixed(1)+' s';
 document.getElementById('delay').textContent=alarmT===null||attackT===null?'—':(alarmT-attackT).toFixed(1)+' s';
 let routeEast=rows[0].east_m+(route[0]?.east_m??0),routeDeviation=routeMode?Math.max(...rows.map(v=>Math.abs(v.east_m-routeEast))):0;
 document.getElementById('physicalLabel').textContent=routeMode?'最大横向航线偏离':'飞机真实水平位移';
 document.getElementById('physicalMove').textContent=(routeMode?routeDeviation:physicalMove).toFixed(1)+' m';
 let outcome=document.getElementById('outcome');outcome.className='outcome '+(firstAlarm||complete&&attack&&(hover||routeMode)?'':'pending');
 document.getElementById('headline').textContent=routeMode&&complete&&attack?`结果：${routeDeviation>=3?'飞机偏离预设航线':'航线偏离不明显'}，${firstAlarm?'已报警':'检测器漏报'}`:firstAlarm?'结果：已检出 GNSS 欺骗':complete&&attack&&hover?'结果：飞机被拖走，检测器漏报':attack?'攻击进行中，尚未报警':'等待攻击开始';
 document.getElementById('explain').textContent=routeMode&&attack?`黄色是航线，紫色虚线展示注入偏移，蓝色才是 PX4 收到的假 GPS，灰色是真实轨迹；飞机最大横向偏离 ${routeDeviation.toFixed(1)} 米。${firstAlarm?'检测器已报警。':'当前检测器尚未报警。'}`:firstAlarm?`偏移开始 ${(alarmT-attackT).toFixed(1)} 秒后首次报警，当时 GNSS 位置已偏移 ${Math.hypot(firstAlarm.attack_east_m,firstAlarm.attack_north_m).toFixed(1)} 米。`:complete&&attack&&hover?`悬停目标没有变化，飞机真实水平移动 ${physicalMove.toFixed(1)} 米，GNSS 报告的位置只移动 ${gpsMove.toFixed(1)} 米。当前检测器未识别这种缓慢拖移。`:attack?'GPS 报告的位置正在偏离仿真真值。':'检测器正在接收正常的 IMU 和 GNSS 数据。';
 let status=document.getElementById('status');status.textContent=complete&&attack&&!firstAlarm?'未报警':r.detector_status==='alarm'?'警报':r.detector_status==='normal'?'正常':'初始化';status.className='value '+(complete&&attack&&!firstAlarm?'alarm':r.detector_status==='alarm'?'alarm':r.detector_status==='normal'?'normal':'waiting');
 document.getElementById('score').textContent=Math.max(r.innovation_chi2??0,r.window_chi2??0,r.inertial_chi2??0).toFixed(2)+' / '+r.threshold.toFixed(2);
 document.getElementById('offset').textContent=Math.hypot(r.east_bias_m+r.attack_east_m,r.north_bias_m+r.attack_north_m).toFixed(1)+' m';
 document.getElementById('time').textContent=(r.sim_time_ns/1e9-base).toFixed(1)+' s';
 let lidarAge=document.getElementById('lidarAge'),lidarDetails=document.getElementById('lidarSyncDetails');
 if(r.lidar_sync_status){
  let syncRows=rows.filter(v=>v.sim_time_ns/1e9-base>=1),matched=syncRows.filter(v=>v.lidar_sync_status==='matched').length;
  lidarAge.textContent=r.lidar_sync_status==='matched'?r.lidar_age_ms.toFixed(1)+' ms':r.lidar_sync_status==='stale'?'扫描过期':'等待扫描';
  lidarAge.className='value '+(r.lidar_sync_status==='matched'?'normal':'waiting');
  lidarDetails.textContent=`LiDAR 点云与欺骗 GPS 历元按 Gazebo 仿真时间配对；当前区间 ${matched}/${syncRows.length} 个历元匹配。最近扫描 ${r.lidar_point_count??'—'} 点。LiDAR 尚未参与报警判定。`;
 }else{lidarAge.textContent='未记录';lidarAge.className='value waiting';lidarDetails.textContent='此运行未记录 LiDAR 与 GPS 的仿真时间差，请启动新实验。'}
 let times=rows.map(v=>v.sim_time_ns/1e9-base),offsets=rows.map(v=>Math.hypot(v.attack_east_m??0,v.attack_north_m??0));
 chart('drift',[{color:'#69baff',points:times.map((t,i)=>[t,offsets[i]])}],[0,Math.max(1,times.at(-1)),0,Math.max(5,Math.ceil(Math.max(...offsets)/5)*5)],[{at:attackT,color:'#ff7676'},{at:alarmT,color:'#f2c66d'}]);
 let tracks=[{color:'#8293a3',points:rows.map(v=>[v.east_m,v.north_m])},{color:'#54d6a1',points:rows.map(v=>[v.estimate_east_m,v.estimate_north_m])},{color:'#69baff',points:rows.map(v=>[v.gps_east_m,v.gps_north_m])}];
 if(route.length)tracks.unshift({color:'#ffd044',points:route.map(v=>[rows[0].east_m+v.east_m,rows[0].north_m+v.north_m])});
 if(route.length)tracks.push({color:'#d88aff',dash:[7,5],width:3,points:rows.map(v=>v.attack_active?[routeEast+(v.east_bias_m??0)+(v.attack_east_m??0),v.north_m+(v.north_bias_m??0)+(v.attack_north_m??0)]:null)});
 let xs=tracks.flatMap(v=>v.points.filter(Boolean).map(p=>p[0])),ys=tracks.flatMap(v=>v.points.filter(Boolean).map(p=>p[1])),ex=extent(xs,2),ey=extent(ys,2);chart('track',tracks,[...ex,...ey]);
 setSpaceData(rows,route);
 let t0=rows[0].sim_time_ns/1e9, ts=rows.map(v=>(v.sim_time_ns/1e9)-t0), q=rows.map(v=>v.innovation_chi2),z=rows.map(v=>v.window_chi2),a=rows.map(v=>v.inertial_chi2),max=Math.max(r.threshold*1.25,...q.filter(Number.isFinite),...z.filter(Number.isFinite),...a.filter(Number.isFinite));
 chart('chi',[{color:'#69baff',points:ts.map((t,i)=>[t,q[i]])},{color:'#e8a86c',points:ts.map((t,i)=>[t,z[i]])},{color:'#bca6ff',points:ts.map((t,i)=>[t,a[i]])},{color:'#ff7676',points:ts.map(t=>[t,r.threshold])}],[0,Math.max(1,ts.at(-1)),0,Math.min(80,max*1.1)],[{at:attackT,color:'#ff7676'},{at:alarmT,color:'#f2c66d'}]);
 }catch(e){console.error('dashboard refresh failed',e);document.getElementById('status').textContent='读取失败';document.getElementById('status').className='value waiting'}}
 initSpaceControls();refresh();setInterval(refresh,1000);addEventListener('resize',refresh);
</script></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source_args = parser.add_mutually_exclusive_group(required=True)
    source_args.add_argument('--run', type=Path, help='monitor a specific run')
    source_args.add_argument('--follow-latest', type=Path, metavar='RUNS_DIR',
                             help='automatically switch to the newest run with injected GPS data')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    space_script = Path(__file__).with_name('spoof-dashboard-3d.js')
    default_route = (Path(__file__).resolve().parents[1] /
                     'adapters/ros1_noetic/fast_glio_uav/route_plan.json')

    def current_run():
        if args.run is not None:
            return args.run
        candidates = []
        if args.follow_latest.is_dir():
            for folder in args.follow_latest.iterdir():
                source = folder / 'injected-gps.jsonl'
                if folder.is_dir() and source.is_file() and source.stat().st_size:
                    candidates.append((source.stat().st_mtime_ns, folder))
        return max(candidates, default=(0, None))[1]

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            run = current_run()
            if self.path == '/':
                payload, mime = HTML.encode(), 'text/html; charset=utf-8'
            elif self.path == '/api/samples':
                rows = deque(maxlen=1200)
                source = run / 'injected-gps.jsonl' if run else None
                if source and source.exists():
                    with source.open() as stream:
                        for line in stream:
                            try:
                                row = json.loads(line)
                            except json.JSONDecodeError:
                                continue
                            if 'detector_status' in row:
                                rows.append(row)
                payload, mime = json.dumps(list(rows)).encode(), 'application/json'
            elif self.path == '/api/route':
                route_file = run / 'route_plan.json' if run else default_route
                if not route_file.is_file():
                    route_file = default_route
                payload, mime = route_file.read_bytes(), 'application/json'
            elif self.path == '/spoof-dashboard-3d.js':
                payload, mime = space_script.read_bytes(), 'text/javascript; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            if self.path == '/api/samples':
                self.send_header('X-Run-Complete',
                                 '1' if run and (run / 'px4-flight-report.json').is_file() else '0')
                self.send_header('X-Run-Name', run.name if run else '')
                controller = run / 'controller.log' if run else None
                controller_text = controller.read_text() if controller and controller.is_file() else ''
                pattern = 'route' if 'PX4 flight pattern: route' in controller_text else (
                    'hover' if 'PX4 flight pattern: hover' in controller_text else 'patrol')
                self.send_header('X-Flight-Pattern', pattern)
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

    print(f'http://127.0.0.1:{args.port}  ←  '
          f'{args.run or args.follow_latest}', flush=True)
    ThreadingHTTPServer(('127.0.0.1', args.port), Handler).serve_forever()


if __name__ == '__main__':
    main()
