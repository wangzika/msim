"""Standalone HTML: software-projected interactive 3D, no CDN or GUI library."""
import json
from collections import defaultdict


class ReportSamples:
    def __init__(self, maximum=2000):self.maximum=maximum;self.series=defaultdict(list)
    def add(self,row):
        rows=self.series[row['stream']];rows.append(row)
        if len(rows)>self.maximum*2:rows[:]=rows[::2]+([rows[-1]] if len(rows)%2==0 else [])


def write_report(path,summary,samples):
    data=json.dumps(dict(summary=summary,series=dict(samples.series)),ensure_ascii=False,allow_nan=False).replace('<','\\u003c')
    path.write_text(HTML.replace('__DATA__',data),encoding='utf-8')


HTML='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>msim 便携回放报告</title><style>
*{box-sizing:border-box}body{margin:0;padding:24px;background:#0b1420;color:#d5e4f1;font:16px system-ui,sans-serif}main{max-width:1280px;margin:auto}h1{font-size:27px}h2{font-size:20px}.panel{background:#122335;border:1px solid #304a60;border-radius:12px;padding:18px;margin:18px 0}.muted{color:#99b5ca;line-height:1.7}button{background:#21405b;color:white;border:1px solid #517a96;border-radius:6px;padding:9px 14px;cursor:pointer}canvas{width:100%;height:450px;background:#09131f;border-radius:8px;touch-action:none}table{border-collapse:collapse;width:100%;font-size:15px}td,th{text-align:left;padding:10px;border-bottom:1px solid #355066}label{margin-right:18px}svg{width:100%;height:auto}.controls{margin:12px 0;line-height:2}#metrics{overflow:auto}details pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:13px}.note{color:#ffc780}</style>
<main><h1>msim · 便携传感器回放</h1><p class="muted">此报告可直接打开，无需 ROS、Gazebo、PX4 或网络。轨迹来自记录中的位姿；本工具不重新运行 FAST-LIO，也不控制飞机。</p>
<section class="panel"><h2>回放与评价结果</h2><p id="status"></p><div id="metrics"></div><p id="alignment" class="muted"></p></section>
<section class="panel"><h2>三维轨迹</h2><div id="layers" class="controls"></div><button id="reset">重置视角</button><p class="muted">拖动旋转，滚轮缩放。E / N / U 为评价坐标；仅显示可匹配并可对齐的轨迹，显示点经过抽样。</p><canvas id="trajectory" aria-label="可旋转三维轨迹"></canvas></section>
<section class="panel"><h2>ENU 时序误差</h2><div id="errors"></div></section>
<section class="panel"><details><summary>完整指标与数据边界</summary><pre id="raw"></pre></details></section></main>
<script>const report=__DATA__;
const names={truth:'参考真值',gnss:'输入 GPS',gnss_normal:'GPS 正常旁路',lio:'LiDAR / IMU（录制位姿）',loose:'LiDAR / IMU / GPS 松组合（录制位姿）'};
const $=id=>document.getElementById(id), colors=['#43cde4','#f5ae65','#d4a0ff','#92df9c'];
$('raw').textContent=JSON.stringify(report.summary,null,2);const stats=report.summary;
$('status').textContent=`回放 ${stats.events} 条消息；有效检测 ${stats.detector.valid_updates} 次，报警 ${stats.detector.alarm_count} 次。回放总耗时 ${stats.processing_wall_s.toFixed(3)} s。`;
const table=document.createElement('table'),header=table.insertRow();for(const name of ['轨迹','东 RMSE / m','北 RMSE / m','天 RMSE / m','三维 RMSE / m','匹配数量']){const th=document.createElement('th');th.textContent=name;header.append(th)}
for(const [name,m] of Object.entries(stats.evaluation.position)){const row=table.insertRow();for(const v of [names[name]||name,...m.rmse_enu_m.map(x=>x.toFixed(3)),m.rmse_3d_m.toFixed(3),m.matched_pairs]){const c=row.insertCell();c.textContent=v}}$('metrics').append(table);
$('alignment').textContent=Object.entries(stats.evaluation.position).map(([k,v])=>`${k}: ${v.alignment}`).join('；')||'没有可评价的匹配位姿。请检查真值、坐标系及时间范围。';
const series=Object.entries(report.series).map(([name,rows],i)=>({name,rows,color:colors[i%colors.length],show:true}));
if(series.length){const src=series[0].rows;series.unshift({name:'truth',rows:src.map(p=>({...p,east_m:p.truth_east_m,north_m:p.truth_north_m,up_m:p.truth_up_m})),color:'#e9eef4',show:true})}
for(const s of series){const label=document.createElement('label'),box=document.createElement('input');box.type='checkbox';box.checked=true;box.onchange=()=>{s.show=box.checked;draw()};label.append(box,document.createTextNode(' '+(names[s.name]||s.name)));label.style.color=s.color;$('layers').append(label)}
let yaw=-.65,pitch=.65,zoom=1,drag=null;const canvas=$('trajectory'),ctx=canvas.getContext('2d');
let all=series.flatMap(s=>s.rows),bounds=[['east_m'],['north_m'],['up_m']].map(([k])=>{let lo=Infinity,hi=-Infinity;for(const p of all){lo=Math.min(lo,p[k]);hi=Math.max(hi,p[k])}return all.length?[lo,hi]:[-1,1]});
const center=bounds.map(([a,b])=>(a+b)/2),span=Math.max(1,...bounds.map(([a,b])=>b-a));
function draw(){const ratio=Math.min(devicePixelRatio||1,2),w=canvas.clientWidth,h=450;canvas.width=w*ratio;canvas.height=h*ratio;ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);const scale=Math.min(w,h)*.7/span*zoom;
function project(v){const x=v[0]-center[0],y=v[1]-center[1],z=v[2]-center[2],u=Math.cos(yaw)*x-Math.sin(yaw)*y,a=Math.sin(yaw)*x+Math.cos(yaw)*y;return [w/2+u*scale,h/2+(Math.sin(pitch)*a-Math.cos(pitch)*z)*scale]}
ctx.lineWidth=1;const o=project(center);for(let i=0;i<3;i++){const v=[...center];v[i]+=span*.25;const b=project(v);ctx.strokeStyle=['#ee8a88','#87d49b','#83b9f1'][i];ctx.beginPath();ctx.moveTo(...o);ctx.lineTo(...b);ctx.stroke();ctx.fillStyle=ctx.strokeStyle;ctx.fillText(['E','N','U'][i],b[0]+5,b[1])}
for(const s of series){if(!s.show||!s.rows.length)continue;ctx.strokeStyle=s.color;ctx.lineWidth=s.name==='truth'?2:1.7;ctx.setLineDash(s.name==='truth'?[5,4]:[]);ctx.beginPath();s.rows.forEach((p,i)=>{const xy=project([p.east_m,p.north_m,p.up_m]);i?ctx.lineTo(...xy):ctx.moveTo(...xy)});ctx.stroke()}ctx.setLineDash([])}
canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};canvas.onpointermove=e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.008;pitch=Math.max(-1.5,Math.min(1.5,pitch+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];draw()};canvas.onpointerup=canvas.onpointercancel=()=>drag=null;canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.min(20,Math.max(.1,zoom*Math.exp(-e.deltaY*.001)));draw()},{passive:false});$('reset').onclick=()=>{yaw=-.65;pitch=.65;zoom=1;draw()};new ResizeObserver(draw).observe(canvas);draw();
const ns='http://www.w3.org/2000/svg';function element(tag,attrs={},text=null){const e=document.createElementNS(ns,tag);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==null)e.textContent=text;return e}
for(const [axis,label]of [['east','东 E'],['north','北 N'],['up','天 U']]){const rows=all;if(!rows.length)break;let first=Infinity,last=-Infinity,maximum=.1;for(const p of rows){first=Math.min(first,p.stamp_ns);last=Math.max(last,p.stamp_ns);maximum=Math.max(maximum,Math.abs(p['error_'+axis+'_m']))}const svg=element('svg',{viewBox:'0 0 1000 220','aria-label':label+'误差'});svg.append(element('text',{x:10,y:24,fill:'#d5e4f1','font-size':19},label+'误差 / m'));
for(const r of [-1,0,1]){const y=110-r*maximum/maximum*65;svg.append(element('line',{x1:85,y1:y,x2:975,y2:y,stroke:'#304a60'}),element('text',{x:8,y:y+5,fill:'#99b5ca'},(r*maximum).toFixed(2)))}
for(const s of series){if(s.name==='truth')continue;const points=s.rows.map(p=>`${85+(p.stamp_ns-first)/Math.max(1,last-first)*890},${110-p['error_'+axis+'_m']/maximum*65}`).join(' ');svg.append(element('polyline',{points,fill:'none',stroke:s.color,'stroke-width':2}))}
svg.append(element('text',{x:85,y:205,fill:'#99b5ca'},'0 s'),element('text',{x:850,y:205,fill:'#99b5ca'},((last-first)/1e9).toFixed(1)+' s'));$('errors').append(svg)}
</script></html>'''
