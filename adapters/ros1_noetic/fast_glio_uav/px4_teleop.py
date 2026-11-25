#!/usr/bin/env python3
"""PX4 SITL keyboard panel with continuous offboard setpoints and a watchdog."""
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

from teleop_controls import CommandBuffer, KeyInput, MOVEMENT_KEYS, body_velocity


class Controller:
    def __init__(self):
        self.commands = CommandBuffer()
        self.status = dict(state='connecting', armed=False, mode='等待 PX4', error=None)
        self.running = True
        self.finished = False
        self.run = Path(os.environ['MSIM_TELEOP_RUN'])

    def command(self, value):
        self.commands.put(value)

    def loop(self):
        from pymavlink import mavutil
        import rospy
        from std_msgs.msg import String
        rospy.init_node('msim_px4_teleop', disable_signals=True)
        publisher = rospy.Publisher('/msim/teleop/status', String, queue_size=5)
        def receive(msg):
            try:
                value = json.loads(msg.data)
                if not isinstance(value, dict):
                    raise ValueError('Command must be a JSON object')
                self.command(value)
            except (ValueError, TypeError) as error:
                rospy.logwarn('Invalid teleop command: %s', error)
        subscriber = rospy.Subscriber('/msim/teleop/command', String, receive, queue_size=5, tcp_nodelay=True)
        master = mavutil.mavlink_connection('udpin:0.0.0.0:14540', source_system=255)
        keys = KeyInput()
        speed = float(os.environ.get('MSIM_TELEOP_SPEED_MPS', '2'))
        height = float(os.environ.get('MSIM_TELEOP_HEIGHT_M', '5'))
        state = 'connecting'; hold = None; origin = None
        position = None; measured_velocity = None; yaw = 0.; armed = False; mode = '等待 PX4'
        heartbeat_at = position_at = -math.inf
        arm_begin = None; next_action = 0.; finish_requested = False
        last_velocity = False; last_report = 0.
        position_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        velocity_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                         mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
        def set_hold():
            nonlocal hold, last_velocity
            if position is not None:
                hold = (*position, yaw)
            keys.clear(); last_velocity = False
        def landing():
            nonlocal state, next_action
            set_hold()
            state = 'landing'; next_action = 0.
        try:
            with (self.run/'teleop-commands.jsonl').open('w', buffering=1) as log:
                while self.running and not rospy.is_shutdown():
                    now = time.monotonic()
                    for _ in range(200):
                        msg = master.recv_match(blocking=False)
                        if msg is None:
                            break
                        kind = msg.get_type()
                        if kind == 'HEARTBEAT' and msg.get_srcComponent() == 1:
                            heartbeat_at = now
                            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                            mode = mavutil.mode_string_v10(msg)
                        elif kind == 'LOCAL_POSITION_NED':
                            position = (msg.x, msg.y, msg.z); position_at = now
                            measured_velocity=(msg.vx,msg.vy,msg.vz)
                        elif kind == 'STATUSTEXT':
                            text=msg.text.decode(errors='replace') if isinstance(msg.text,bytes) else msg.text
                            row=dict(source='PX4',severity=int(msg.severity),text=text.rstrip('\0'),sim_time_s=rospy.Time.now().to_sec())
                            with (self.run/'px4-events.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')
                        elif kind == 'ATTITUDE':
                            yaw = msg.yaw
                    connected = position is not None and now-heartbeat_at < 3 and now-position_at < 2
                    if connected and origin is None:
                        origin = position; set_hold(); state = 'ready'
                        (self.run/'teleop-ready.json').write_text(json.dumps(dict(ready=True))+'\n')
                        print('手动控制就绪：T 起飞，W/S/A/D 平移，R/F 升降，Q/E 转向，空格悬停，L 降落。', flush=True)
                    for command,received_at in self.commands.drain():
                        op = command.get('op')
                        try:
                            if op == 'keys':
                                keys.update(command.get('keys', []), received_at)
                            elif op == 'speed':
                                value = float(command['value'])
                                if not math.isfinite(value) or not .2 <= value <= 8:
                                    raise ValueError('Speed must be between 0.2 and 8 m/s')
                                speed = value
                            elif op == 'takeoff' and connected and not armed and state in ('ready', 'landed'):
                                value = float(command.get('height_m', height))
                                if not math.isfinite(value) or not .5 <= value <= 30:
                                    raise ValueError('Takeoff height must be between 0.5 and 30 m')
                                height = value; origin = position
                                hold = (position[0], position[1], position[2]-height, yaw)
                                state = 'arming'; arm_begin = now; next_action = now+1.5; keys.clear()
                            elif op == 'hold' and state in ('flying', 'taking_off'):
                                set_hold(); state = 'flying'
                            elif op == 'land' and armed:
                                landing()
                            elif op == 'finish':
                                finish_requested = True
                                if armed:
                                    landing()
                            elif op not in ('takeoff', 'hold', 'land'):
                                if op not in ('keys', 'speed', 'finish'):
                                    raise ValueError('Unknown teleop operation')
                            log.write(json.dumps(dict(wall_monotonic_s=now, sim_time_ns=rospy.Time.now().to_nsec(), command=command))+'\n')
                        except (ValueError, TypeError, KeyError) as error:
                            print(f'忽略无效控制指令：{error}', flush=True)
                    if not connected and armed:
                        keys.clear()
                    if state == 'arming':
                        if armed and mode == 'OFFBOARD':
                            state = 'taking_off'
                        elif now-arm_begin > 60:
                            set_hold(); state = 'ready'
                            print('解锁/切换 OFFBOARD 超时，请检查 PX4 日志后重试。', flush=True)
                        elif connected and now >= next_action:
                            master.set_mode('OFFBOARD'); master.arducopter_arm(); next_action = now+3
                    if state == 'taking_off' and position is not None and abs(position[2]-hold[2]) < .35:
                        set_hold(); state = 'flying'
                    if finish_requested and armed and state != 'landing':
                        landing()
                    if state == 'landing':
                        if not armed:
                            set_hold(); state = 'landed'
                        elif connected and now >= next_action:
                            master.set_mode('LAND'); next_action = now+2
                    if state == 'flying' and (not armed or mode != 'OFFBOARD'):
                        set_hold(); state = 'landed' if not armed else 'paused'
                    if finish_requested and not armed and state != 'arming':
                        self.finished = True
                        self.status = dict(self.status, state='landed' if origin else 'ready',
                                           armed=False, mode=mode, finish_requested=True,
                                           keys=[], velocity_ned_mps=[0.,0.,0.], yaw_rate_radps=0.,
                                           sim_time_ns=rospy.Time.now().to_nsec())
                        payload=json.dumps(self.status)
                        temporary=self.run/'teleop-status.json.tmp'
                        temporary.write_text(payload+'\n');temporary.replace(self.run/'teleop-status.json')
                        publisher.publish(String(data=payload))
                        Path(os.environ['MSIM_TELEOP_STOP_FILE']).write_text(json.dumps(dict(reason='teleop_finish', armed=False, sim_time_ns=rospy.Time.now().to_nsec()))+'\n')
                        break
                    active = keys.active(now) if connected and state == 'flying' else frozenset()
                    velocity = body_velocity(active, yaw, speed) if active else (0., 0., 0., 0.)
                    if active:
                        master.mav.set_position_target_local_ned_send(
                            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            velocity_mask, 0, 0, 0, *velocity[:3], 0, 0, 0, 0, velocity[3])
                        last_velocity = True
                    elif hold is not None and state != 'paused':
                        if last_velocity:
                            set_hold()
                        master.mav.set_position_target_local_ned_send(
                            0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            position_mask, *hold[:3], 0, 0, 0, 0, 0, 0, hold[3], 0)
                    self.status = dict(state=state, armed=armed, mode=mode, connected=connected,
                                       position_ned_m=position, relative_height_m=origin[2]-position[2] if origin and position else None,
                                       yaw_deg=math.degrees(yaw), speed_mps=speed,
                                       measured_velocity_ned_mps=measured_velocity,
                                       measured_speed_mps=math.sqrt(sum(v*v for v in measured_velocity)) if measured_velocity is not None else None, keys=sorted(active),
                                       velocity_ned_mps=list(velocity[:3]), yaw_rate_radps=velocity[3],
                                       finish_requested=finish_requested, sim_time_ns=rospy.Time.now().to_nsec(), error=None)
                    if now-last_report >= .2:
                        payload = json.dumps(self.status)
                        publisher.publish(String(data=payload))
                        temporary = self.run/'teleop-status.json.tmp'
                        temporary.write_text(payload+'\n'); temporary.replace(self.run/'teleop-status.json')
                        last_report = now
                    time.sleep(.02)
        except Exception as error:
            self.status = dict(self.status, error=str(error), state='error')
            print(f'手动控制失败：{error}', flush=True)
            self.running = False
            raise
        finally:
            subscriber.unregister(); master.close()
            self.running = False


def panel(controller):
    from PyQt5 import QtCore, QtWidgets
    labels = dict(connecting='等待 PX4', ready='已就绪，等待起飞', arming='正在解锁',
                  taking_off='正在起飞', flying='手动飞行 / 悬停', landing='正在降落',
                  landed='已降落', paused='飞控已退出 OFFBOARD', error='控制异常')
    class Window(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.keys = set()
            self.setWindowTitle('msim 手动飞行采集')
            self.setMinimumSize(470, 390)
            self.setStyleSheet('QWidget {font-size: 17px;} QPushButton {padding: 10px;}')
            layout = QtWidgets.QVBoxLayout(self)
            self.status = QtWidgets.QLabel('正在连接 PX4…'); layout.addWidget(self.status)
            self.details = QtWidgets.QLabel(''); layout.addWidget(self.details)
            form = QtWidgets.QFormLayout()
            self.height = QtWidgets.QDoubleSpinBox(); self.height.setRange(.5, 30); self.height.setValue(float(os.environ.get('MSIM_TELEOP_HEIGHT_M', '5'))); self.height.setSuffix(' m')
            self.speed = QtWidgets.QDoubleSpinBox(); self.speed.setRange(.2, 8); self.speed.setSingleStep(.2); self.speed.setValue(float(os.environ.get('MSIM_TELEOP_SPEED_MPS', '2'))); self.speed.setSuffix(' m/s')
            self.speed.valueChanged.connect(lambda v: controller.command(dict(op='speed', value=v)))
            form.addRow('起飞相对高度', self.height); form.addRow('水平速度', self.speed); layout.addLayout(form)
            layout.addWidget(QtWidgets.QLabel('↑↓ / W/S：前后　←→ / A/D：左右　R/F：升降\nQ/E：左转/右转　空格：悬停\n前后左右以机头朝向为准。\nT：起飞　L：降落　松键自动悬停\n请点击此窗口后操纵；切换窗口会悬停。'))
            row = QtWidgets.QHBoxLayout()
            for text, op in [('起飞 (T)', 'takeoff'), ('悬停', 'hold'), ('降落 (L)', 'land')]:
                button = QtWidgets.QPushButton(text); button.setFocusPolicy(QtCore.Qt.NoFocus)
                button.clicked.connect(lambda checked=False, op=op: self.action(op)); row.addWidget(button)
            layout.addLayout(row)
            end = QtWidgets.QPushButton('结束并保存（先降落）');end.setFocusPolicy(QtCore.Qt.NoFocus)
            end.clicked.connect(lambda: self.action('finish')); layout.addWidget(end)
            self.setFocusPolicy(QtCore.Qt.StrongFocus)
            self.timer = QtCore.QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(50)

        def action(self, op):
            self.keys.clear()
            controller.command(dict(op=op, height_m=self.height.value()))
            self.setFocus()

        def keyPressEvent(self, event):
            if event.isAutoRepeat():return
            key = {QtCore.Qt.Key_Up:'w',QtCore.Qt.Key_Down:'s',QtCore.Qt.Key_Left:'a',QtCore.Qt.Key_Right:'d'}.get(event.key(),event.text().lower())
            if key in MOVEMENT_KEYS:
                self.keys.add(key)
                controller.command(dict(op='keys',keys=sorted(self.keys)))
            elif key == 't':self.action('takeoff')
            elif key == 'l':self.action('land')
            elif event.key() in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Escape):self.action('hold')

        def keyReleaseEvent(self, event):
            if event.isAutoRepeat():return
            key = {QtCore.Qt.Key_Up:'w',QtCore.Qt.Key_Down:'s',QtCore.Qt.Key_Left:'a',QtCore.Qt.Key_Right:'d'}.get(event.key(),event.text().lower())
            if key in self.keys:
                self.keys.discard(key)
                controller.command(dict(op='keys', keys=sorted(self.keys)))

        def changeEvent(self, event):
            if event.type() == QtCore.QEvent.ActivationChange and not self.isActiveWindow():
                self.keys.clear();controller.command(dict(op='hold'))
            super().changeEvent(event)

        def focusOutEvent(self, event):
            self.keys.clear();controller.command(dict(op='hold'))
            super().focusOutEvent(event)

        def refresh(self):
            if self.keys:
                controller.command(dict(op='keys', keys=sorted(self.keys)))
            status = controller.status
            self.status.setText(labels.get(status['state'],status['state'])+' · '+status.get('mode',''))
            height = status.get('relative_height_m')
            self.details.setText(f"相对高度：{height:.1f} m　速度：{status.get('speed_mps',2):.1f} m/s" if height is not None else '等待位姿…')
            if status.get('error'):self.details.setText(status['error'])
            if not controller.running:
                self.timer.stop();self.hide();QtWidgets.QApplication.instance().quit()

        def closeEvent(self, event):
            if controller.running:
                self.action('finish');event.ignore()
            else:event.accept()

    app = QtWidgets.QApplication([])
    window = Window();window.show();window.activateWindow();window.setFocus()
    app.exec_()


def main():
    controller = Controller()
    def stop(*_):controller.running = False
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    thread = threading.Thread(target=controller.loop, daemon=True)
    thread.start()
    if os.environ.get('MSIM_TELEOP_HEADLESS') == '1':
        while controller.running:time.sleep(.1)
    else:
        panel(controller)
    controller.running = False;thread.join(3)
    if controller.status.get('error'):
        raise SystemExit(1)


if __name__ == '__main__':main()
