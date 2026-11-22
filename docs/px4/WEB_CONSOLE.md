# 网页仿真控制台（原型）

本分支增加本机网页入口，供无窗口的 Gazebo Classic、PX4 SITL、FAST_GLIO 实验使用。浏览器显示场景碰撞体、真值与 GPS 轨迹、LiDAR / IMU 定位轨迹和抽样配准点云；手动模式可通过网页向现有 PX4 OFFBOARD 控制器发送飞行指令。Gazebo、PX4、FAST_GLIO、SignalSim 与 RTKLIB 的计算链路仍在本机，浏览器承担显示和操纵界面，不在浏览器内重算物理或 GNSS。

## 自动航线

```bash
cd /home/wzb/msim
./tools/px4-uav-capture.sh --web --scene complex_urban \
  --duration-s 60 --flight-pattern route --route-clock sim --skip-signalsim
```

打开 `http://127.0.0.1:8766/`。`--web` 自动开启无窗口采集及 FAST_GLIO 实时估计，不启动 Gazebo 或 RViz 窗口。可加 `--web-port 8767` 更换端口。浏览器关闭不会停止采集。

## 网页手动飞行

```bash
./tools/px4-uav-capture.sh --web --teleop --scene complex_urban
```

页面提供起飞、悬停、降落、结束并保存、速度和高度设置。键盘 W/S/A/D 平移、R/F 升降、Q/E 转向；移动键松开、页面失焦或隐藏时发送悬停指令。PX4 控制器还有 0.35 秒输入心跳超时保护。`--duration-s` 可设置最长采集时间；不设置时持续到网页“降落并保存”或终端中断。网页只绑定本机回环地址，控制接口仅在实时手动模式启用。

## 查看已结束的实验

```bash
/usr/bin/python3 tools/web-sim-console.py --run /home/wzb/msim/runs/某次运行
```

离线查看会读取 `world.sdf` 和 `injected-gps.jsonl`，无需 ROS；实时 LIO 点云与轨迹需要 `--web` 启动时的 ROS 订阅。页面场景只绘制 SDF 中顶层模型的盒体和圆柱碰撞体；机场地形/建筑网格、导入模型纹理以及完整 RViz 图层尚未迁入。点云使用算法坐标系，场景使用 Gazebo ENU，页面将两者放在独立视图，避免把不同坐标系错误叠加。

`web-console.log` 保存在运行目录。网页没有访问互联网的依赖；采集和控制仍需要本机原有 ROS/PX4/Gazebo 环境。
