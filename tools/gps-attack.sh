#!/usr/bin/env bash
set -euo pipefail
fast="${FAST_GLIO_ROOT:-/home/wzb/FAST_GLIO}"
attack_args=("$@")
set --
export PATH="$fast/.pixi/envs/default/bin:$PATH"
set +u
source "$fast/devel/setup.bash"
set -u
python - "${attack_args[@]}" <<'PY'
import argparse,json,math,time
import rospy
from std_msgs.msg import String
p=argparse.ArgumentParser(description='修改正在运行的 Gazebo GPS 欺骗')
p.add_argument('--stop',action='store_true')
p.add_argument('--east-rate-mps',type=float,default=0.)
p.add_argument('--north-rate-mps',type=float,default=0.)
p.add_argument('--east-offset-m',type=float,default=0.)
p.add_argument('--north-offset-m',type=float,default=0.)
p.add_argument('--up-offset-m',type=float,default=0.)
p.add_argument('--start-delay-s',type=float,default=0.)
p.add_argument('--position-only',action='store_true')
a=p.parse_args()
values=(a.east_rate_mps,a.north_rate_mps,a.east_offset_m,a.north_offset_m,a.up_offset_m,a.start_delay_s)
if not all(math.isfinite(x) for x in values) or max(abs(x) for x in values[:2])>10 or math.hypot(*values[2:5])>1000 or a.start_delay_s<0:
    p.error('攻击参数超出范围')
rospy.init_node('msim_gps_attack_command',anonymous=True)
pub=rospy.Publisher('/msim/gps_attack',String,queue_size=1)
deadline=time.monotonic()+10
while pub.get_num_connections()==0 and time.monotonic()<deadline:time.sleep(.05)
if pub.get_num_connections()==0:raise SystemExit('没有找到运行中的 GPS 中继，请先启动仿真。')
payload=dict(enabled=not a.stop,east_rate_mps=a.east_rate_mps,north_rate_mps=a.north_rate_mps,
             east_offset_m=a.east_offset_m,north_offset_m=a.north_offset_m,up_offset_m=a.up_offset_m,
             start_delay_s=a.start_delay_s,velocity_consistent=not a.position_only)
pub.publish(String(data=json.dumps(payload)));time.sleep(.3)
print('已发送 GPS 攻击命令：'+json.dumps(payload,ensure_ascii=False))
PY
