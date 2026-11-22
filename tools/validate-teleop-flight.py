#!/usr/bin/env python3
"""Exercise the manual command path in an explicitly selected msim SITL run."""
import argparse
import json
import math
from pathlib import Path
import time

import rospy
from std_msgs.msg import String


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--resume',action='store_true')
    args=ap.parse_args();run=args.run.resolve()
    deadline=time.monotonic()+100
    while not (run/'capture-settings.json').exists() and time.monotonic()<deadline:time.sleep(.2)
    settings=json.loads((run/'capture-settings.json').read_text())
    if not settings.get('interactive') or settings.get('flight_pattern')!='teleop':
        raise ValueError('Select a running --teleop capture, not a normal flight')
    rospy.init_node('msim_teleop_integration_test',disable_signals=True)
    publisher=rospy.Publisher('/msim/teleop/command',String,queue_size=10)
    def status():
        try:return json.loads((run/'teleop-status.json').read_text())
        except (FileNotFoundError,ValueError):return {}
    def wait_for(condition, description, timeout=70):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            row=status()
            if row.get('error'):raise RuntimeError(row['error'])
            if condition(row):return row
            time.sleep(.1)
        raise RuntimeError('Timeout: '+description+'; last status='+str(status()))
    wait_for(lambda r:r.get('state')=='ready' or args.resume and r.get('state')=='flying','controller ready')
    until=time.monotonic()+10
    while publisher.get_num_connections()==0 and time.monotonic()<until:time.sleep(.1)
    if publisher.get_num_connections()==0:raise RuntimeError('No teleop command subscriber')
    def send(op,**extra):publisher.publish(String(data=json.dumps(dict(op=op,**extra))))
    results=[]
    if status()['state']=='ready':send('takeoff',height_m=3)
    row=wait_for(lambda r:r.get('state')=='flying','takeoff')
    assert row['armed'] and row['mode']=='OFFBOARD'
    results.append(dict(check='takeoff',height_m=row['relative_height_m']))
    def hold_keys(keys, seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            send('keys',keys=keys);time.sleep(.05)
        return status()
    row=hold_keys(['w'],3)
    def follows_heading(row):
        yaw=math.radians(row['yaw_deg'])
        return abs(row['velocity_ned_mps'][0]-2*math.cos(yaw))<.05 and abs(row['velocity_ned_mps'][1]-2*math.sin(yaw))<.05
    assert follows_heading(row),row
    initial_yaw=row['yaw_deg']
    results.append(dict(check='forward',velocity_ned_mps=row['velocity_ned_mps']))
    # Do not send key-up: loss of input heartbeat itself must enter position hold.
    row=wait_for(lambda r:r.get('state')=='flying' and not r.get('keys'),'watchdog hold',timeout=3)
    assert row['velocity_ned_mps']==[0.,0.,0.],row
    results.append(dict(check='watchdog_hold',pass_check=True))
    row=hold_keys(['r'],2)
    assert row['velocity_ned_mps'][2]<-.5,row
    send('hold');time.sleep(.5)
    results.append(dict(check='climb',velocity_ned_mps=row['velocity_ned_mps']))
    row=hold_keys(['q'],2)
    assert row['yaw_rate_radps']<0,row
    send('hold');time.sleep(.5)
    results.append(dict(check='turn_left',yaw_deg=row['yaw_deg']))
    row=hold_keys(['w'],3)
    assert follows_heading(row),row
    assert abs((row['yaw_deg']-initial_yaw+180)%360-180)>10,row
    send('hold');time.sleep(.5)
    results.append(dict(check='forward_after_turn',velocity_ned_mps=row['velocity_ned_mps']))
    send('finish')
    until=time.monotonic()+70
    while not (run/'capture-stop.json').exists() and time.monotonic()<until:time.sleep(.2)
    assert (run/'capture-stop.json').exists(),'Landing/finish did not complete'
    row=status();assert not row['armed'],row
    results.append(dict(check='land_and_finish',armed=row['armed']))
    report=dict(pass_all=True,run=str(run),checks=results)
    (run/'teleop-integration-report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
