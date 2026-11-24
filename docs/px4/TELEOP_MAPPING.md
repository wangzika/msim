# 自由操纵采集与 RViz 实时建图

`--teleop` 启动 Gazebo、RViz 和独立的键盘控制窗口，持续记录传感器、真值与 LiDAR/IMU 定位结果。飞机通过 PX4 OFFBOARD 正常飞行，不直接修改 Gazebo 模型位置。起飞前会先持续发送位置目标，再解锁与切换模式，控制循环保持 50 Hz；对应 [PX4 OFFBOARD 输入要求](https://docs.px4.io/main/en/flight_modes/offboard)。

## 启动

```bash
cd /home/wzb/msim
./tools/px4-uav-capture.sh --teleop --scene ksql_airport --point-filter-num 1
```

首次使用机场场景先运行 `python3 tools/fetch-ksql-assets.py`。把 `ksql_airport` 替换为 `simple`、`complex_urban` 或 `industrial_yard` 即可切换现有场景；`--scene-profile` 和 `--world` 仍可用于自定义配置。

默认持续采集，不会在 60 或 300 秒自动结束。点击控制窗口“结束并保存”后，先执行正常降落，收到飞控解除锁定状态后结束录包、保存地图和生成图表。关闭控制窗口也会走此流程。终端 `Ctrl+C` 可直接结束仿真并保存录包；这一路径不等待降落。应至少记录几秒钟，以便生成轨迹图。

限定最长采集时间：

```bash
./tools/px4-uav-capture.sh --teleop --scene complex_urban --duration-s 300
```

达到指定仿真时长会结束采集和仿真；没有自动执行预设航线。原有 `--manual` 仍表示关闭内置控制器、交给外部 MAVLink 控制器驾驶，与带键盘界面的 `--teleop` 不同。

## 操作

先点击“msim 手动飞行采集”窗口，使其获得键盘焦点。

| 按键 / 按钮 | 功能 |
| --- | --- |
| T / 起飞 | 解锁并上升到设定相对高度，随后悬停 |
| ↑ / ↓ 或 W / S | 沿机头方向前进 / 后退 |
| ← / → 或 A / D | 相对机头向左 / 向右平移 |
| R / F | 上升 / 下降 |
| Q / E | 左转 / 右转 |
| 空格 / Esc / 悬停 | 停止移动并保持当前位置 |
| L / 降落 | 切换 LAND，正常降落并解除锁定；可再次起飞 |
| 结束并保存 | 降落并保存此次采集 |

默认水平速度为 2 m/s、升降速度 0.8 m/s、转向速度 30°/s。斜向移动的水平总速度仍限制为设定速度。松开移动键、切换窗口或超过 0.35 秒没有输入心跳后，控制器转为位置保持。窗口中的速度可以修改，起飞高度可在起飞前修改。

默认起飞相对高度为 5 m，可用 `--teleop-height-m 8` 或 `--teleop-speed-mps 1` 调整。显示高度来自 PX4 本地位置估计，不是 Gazebo 真值离地高度；当前 GPS 包含高度偏差，低空长时间悬停可能落回地面。手动模式没有自动避障或地形跟随，场景 profile 的自动航线网格检查不覆盖用户自由飞行路径。

## 建图与保存内容

RViz 显示累计地图、红色当前配准点云、青色 LiDAR/IMU 轨迹和估计位姿。建图使用纯 LiDAR/IMU，GPS 仍保留给 PX4 并同步记录；不要求飞机移动到 GPS 松组合对齐门限后才允许保存。

地图是 `/cloud_registered` 在 `camera_init` 坐标系中的体素累计点云，默认体素大小 0.4 m，最多 100 万体素。达到上限后保留已有体素，并在 `mapping-report.json` 标明 `capped=true`。这是用于查看与导出的累计点云，不包含回环或全局地图优化。地图每 30 秒自动保存一次，结束时再保存最终版本；RViz 接收有界累计地图，不依赖长时间无限叠加历史帧。

运行目录仍为 `runs/uav-px4-日期时间/`：

| 文件 | 内容 |
| --- | --- |
| `sensors.bag` | 原始 LiDAR、IMU、GPS、真值、定位、当前配准点云及控制状态 |
| `map.pcd` | 最终累计地图，二进制 PCD，XYZ 与 intensity |
| `mapping-report.json` | 地图点数、帧数、体素配置与是否达到容量上限 |
| `teleop-commands.jsonl` | 操作指令及仿真时间，便于回查操纵过程 |
| `teleop-status.json` | 最新控制状态、飞控模式、锁定状态、位置和速度目标 |
| `capture-session.json` | 结束原因及录包是否正常落盘 |
| `capture-report.json` | 根据实际录包计算的有效时长和传感器计数 |
| `teleop-trajectory.png` | 真实轨迹与 LiDAR/IMU 轨迹对照 |
| `fastglio-live-lio-error.json` | LiDAR/IMU 相对真值的 ENU 与三维误差统计 |
| `controller.log` / `mapping.log` / `fastglio-live.log` | 控制、地图保存、算法日志 |

默认跳过离线 SignalSim/RINEX 处理。传感器型号与噪声选择仍沿用 [传感器切换方法](SENSOR_SWITCHING.md)，例如 `--lidar-model mid360` 自动采用 Livox 点云入口。

## 验证入口

```bash
# 已在另一终端启动 --teleop --headless 时，测试同一个控制入口。
export PATH=/home/wzb/FAST_GLIO/.pixi/envs/default/bin:$PATH
source /home/wzb/FAST_GLIO/devel/setup.bash
python tools/validate-teleop-flight.py --run /绝对路径/本次运行目录
```

测试会实际驱动该仿真，检查起飞、机头方向平移、输入超时悬停、升降、转向以及降落保存。单元测试另覆盖坐标方向、速度限幅、输入过期、动态录包时长及 PCD 体素保存。

2026-11-22 的实际验证：

| 运行目录 | 验证内容 | 结果 |
| --- | --- | --- |
| `runs/teleop-validation-20261122/gui-03` | KSQL 场景；真实键盘 T 起飞、移动、松键与失焦悬停；升降、转向、转向后平移、LAND 降落和保存 | 通过；最终解除锁定，地图 242333 点，PCD 约 3.7 MiB |
| `runs/teleop-validation-20261122/interrupt-04` | 复杂街区；无固定时长，终端 Ctrl+C，未起飞也允许保存 | 通过；bag 正常落盘，地图与轨迹图生成，结束原因 `keyboard_interrupt` |

图形模式的证据为 [键盘窗口截图](../../runs/teleop-validation-20261122/gui-03/control-window.png)、[RViz 实时截图](../../runs/teleop-validation-20261122/gui-03/rviz-window.png)、[键盘事件验证](../../runs/teleop-validation-20261122/gui-03/teleop-ui-report.json)及[完整飞行验证](../../runs/teleop-validation-20261122/gui-03/teleop-integration-report.json)。该轮短程飞行的纯 LIO 三维 RMSE 为 0.102 m，不代表长航线精度。早期调试目录 `headless-01`、`gui-02` 保留，未作为最终通过记录。

在配置好的 ROS/Pixi 环境运行 `python -m unittest discover -s tests/fastglio -v`，34 项测试通过；普通无 ROS 的环境会跳过依赖 ROS/PX4 的测试。

## 2026-11-23 网页输入响应修正

网页和 Qt 窗口均支持方向键及 W/A/S/D。网页新增“启用键盘”，修改高度或速度后按 Enter 可把焦点交还三维画布。网页按键立即发送，保持期间每 80 ms 续发；同一方向来自键盘和鼠标的输入分开记忆，释放其中一个不会错误清除另一个。HTTP 控制指令按顺序发送，只合并等待中的连续方向状态，避免网络拥塞时旧方向覆盖松键或新方向。控制器处理循环提高到 50 Hz，ROS 小指令关闭 Nagle 延迟，并以接收时间计算 0.35 秒失联保护，排队不会给旧指令续期。

页面失焦、编辑设置和隐藏页面时清空方向输入，飞行中会进入位置保持；起飞过程继续原高度目标。显式“悬停”、空格或 Esc 仍可中断爬升。原来失焦直接发送悬停，会把 `taking_off` 切为 `flying`，可能在刚离地时就停住，这是本轮实际复现的问题。

修正后的复杂街区验证已完成：起飞中编辑速度后仍达到 7.69 m，相邻指令无乱序，0.35 秒停止续发后进入位置保持，最终正常降落解除锁定并保存。43 项 Python 测试全部通过，网页输入另有 Node 测试。详细数字见 [网页控制说明](WEB_CONSOLE.md)及 `runs/control-response-verification-20261123/report.json`。
