# KSQL 机场大范围场景接入与实测

已接入带卫星底图、真实地形形状和三维建筑网格的 KSQL / San Carlos 机场周边场景。完成无欺骗 60 秒短测、无欺骗 300 秒航线与有欺骗 60 秒 Gazebo/RViz 双窗口测试；三组均通过飞行、LiDAR 同步和在线松组合输出检查。场景接通并不表示定位误差已经消除，完整航线仍有明显高度误差。

## 范围与资源

| 项目 | 当前设置 |
| --- | --- |
| 上游 | OSRF `uav_testing/ksql_airport`，固定提交 `0f511ef628af6c53f0e646d117b9f63b2396ead6` |
| 下载量 | 约 56.22 MB；两个 COLLADA、卫星底图、原 world、包描述 |
| 完整地形覆盖 | 东约 2.87 km，北约 2.78 km；未裁剪原网格 |
| 选定测试区域 | E∈[-300,700] m，N∈[0,1000] m，1 km × 1 km |
| 碰撞几何 | 保留原始地形 21060 个、建筑 60821 个三角面 |
| 出生点 | 世界 ENU `(-210,240,-1.35)` m；下方地面为 `-1.5 m` |
| 起飞高度 | 相对 PX4 起飞参考上升 12 m，不是统一海拔或自动地形跟随高度 |
| 航线 | 建筑群西侧，11 个航点，每圈标称约 784 m，循环执行 |
| 地理原点 | `37.523640°, -122.255122°, 1.7 m`，参考 PX4 Classic KSQL world，未做测绘级配准 |

来源、许可声明和网格变换见 [资源来源](../../scenes/KSQL_SOURCE.md)。当前建筑是上游简化网格，卫星底图不会自动给树木、车辆或底图中的其他物体增加碰撞几何。

![Gazebo 实际俯视相机画面](../../runs/ksql-integration-20261122/clean-route-01/gazebo-scene-overview.png)

## 如何运行与修改

```bash
cd /home/wzb/msim
# 首次下载；已有资源会直接校验。
python3 tools/fetch-ksql-assets.py

# Gazebo 与 RViz，完整航线测试。
./tools/px4-uav-capture.sh --scene ksql_airport --duration-s 300 \
  --fastglio-live --lidar-model vlp16 --point-filter-num 1 \
  --flight-pattern route --route-clock sim --skip-signalsim
```

追加 `--spoof-start-s 20 --spoof-east-rate-mps 0.18` 可开启定时东向 GPS 漂移，原有 GPS 噪声保留。欺骗计时以原生 GPS 首个仿真采样为原点，不是电脑时钟，也不等于飞机解锁时刻；在离散 GPS 采样历元上生效。追加 `--headless` 可后台运行。运行结束会清理 Gazebo 与算法进程并导出图表。

场景配置为 [`scenes/ksql_airport.profile.json`](../../scenes/ksql_airport.profile.json)，集中管理地理原点、出生点、飞行高度、GUI 相机视角、测试区域与资源路径。航点为 [`scenes/ksql_airport.route.json`](../../scenes/ksql_airport.route.json)，`east_m/north_m` 相对起飞点，`hold_s` 是每个航点的停留时间。当前航线只覆盖区域西侧约 120 m × 340 m，不是遍历全部 1 km × 1 km。

自定义一套配置时，复制 profile 与 route，调整 profile 中的 `route_plan`，用 `--scene-profile /绝对路径/配置.json` 启动。相对路径以 profile 所在目录为基准；复制到其他目录时也要同步调整资源清单与资源目录路径。切换传感器仍用 `--lidar-model`、`--imu-model`、`--gps-model`，见 [传感器切换](SENSOR_SWITCHING.md)。

启动前会独立读取原始 COLLADA，检查出生点地面以及标称航线沿途、左右和前后 3 m 的地面/屋顶高度。本配置检查了 1015 个位置。该检查是离散垂直采样，不能替代连续碰撞检查，也不能保证受骗、带噪声或飞控切弯后的真实航迹始终安全。

## 实测结果

数据根目录：[`runs/ksql-integration-20261122`](../../runs/ksql-integration-20261122)。三组是独立在线飞行，不是同录包受控重算；下表用于验证接入与描述各次结果，不用于孤立推断欺骗造成了多少纯 LIO 误差。

| 测试目录 | 时长 | 飞行 / 同步 / 松组合输出检查 | LiDAR 同步匹配 | 纯 LiDAR/IMU 三维 RMSE |
| --- | ---: | --- | ---: | ---: |
| `clean-smoke-01` | 60 s | 全部通过 | 209/209 | 0.457 m |
| `clean-route-01` | 300 s | 全部通过 | 1001/1001 | 2.798 m |
| `spoof-gui-01` | 60 s | 全部通过，Gazebo/RViz 已并排启动 | 214/214 | 0.640 m |

300 秒运行依次执行全部 11 个航点、回到起飞区，再开始下一圈。真值累计三维路程约 967 m，离起点最远约 347 m；实测真值东坐标范围 `[-239.08,-117.10]` m，北坐标范围 `[238.36,582.13]` m，全部位于选定区域内。标称航点折线、飞控实际切弯、GPS 噪声修正和爬升会使实际路程不同；300 秒结束时已在下一圈北向航段，并非最终停在起点。

完整 300 秒的 ENU 与三维 RMSE：

| 解算曲线 | 东 E / m | 北 N / m | 天 U / m | 三维 / m |
| --- | ---: | ---: | ---: | ---: |
| LiDAR/IMU | 1.457 | 0.723 | 2.277 | 2.798 |
| LiDAR/IMU/GPS 松组合 | 0.908 | 0.476 | 3.700 | 3.839 |
| GPS | 0.810 | 0.665 | 3.303 | 3.465 |

误差采用每条曲线第一个真值匹配点的平移对齐，不拟合旋转或尺度；三维 RMSE 为三轴 RMSE 平方和开方。当前 standalone GPS 保留相关位置偏差；本组 GPS 松组合的高度误差大于纯 LIO，不能把“更新被接受”解释为“精度更好”。完整航线没有完成 LIO 的误差诊断和参数优化，短测精度也不能代表长航线。

图表：[完整航线三维与水平轨迹](../../runs/ksql-integration-20261122/clean-route-01/fastglio-live-3d.png)、[ENU 误差](../../runs/ksql-integration-20261122/clean-route-01/fastglio-enu-error-timeseries.png)、[误差原始统计](../../runs/ksql-integration-20261122/clean-route-01/fastglio-enu-error-summary.json)。

有欺骗组中，首个生效 GPS 历元为相对 GPS 原点第 20.198 秒；记录中的最大东向注入偏移为 8.496 m，`最终 GPS－正常 GPS－注入偏移` 的东向最大绝对差约 `1.22×10⁻¹³ m`。飞机最后实际向西偏离起点约 7.73 m，证明当前场景仍保留 GPS 对 PX4 闭环控制的误导作用。Gazebo 日志确认紫色偏移参考轨迹生成，实际画面见下图；紫色是偏移参考，蓝色才是实际注入 GPS。

![带欺骗测试的 Gazebo 窗口](../../runs/ksql-integration-20261122/spoof-gui-01/gazebo-window.png)

## 如何核验 LiDAR 确实照到了模型

新增 [`tools/audit-ksql-run.py`](../../tools/audit-ksql-run.py)，用录包真值在每个抽样点时刻插值，应用归档的雷达安装位姿，再对独立读取的原始网格求射线交点。运行方式：

```bash
export PATH=/home/wzb/FAST_GLIO/.pixi/envs/default/bin:$PATH
source /home/wzb/FAST_GLIO/devel/setup.bash
python tools/audit-ksql-run.py \
  --run runs/ksql-integration-20261122/clean-route-01
```

300 秒录包共 3016 帧，帧间隔中位数为 0.100 s，点数中位数约 13007。旋转扫描覆盖约 98 ms；只保留有效回波，因此少数帧中已返回点的时间跨度为 56 ms，并不意味着整帧被改成了瞬时扫描。

在均匀分布的 16 帧中抽查 1536 条射线，1281 条命中地形、255 条命中建筑，全部找到网格交点；距离残差标准差为 0.01963 m，绝对残差 95 分位约 0.03855 m，与径向噪声标准差 0.02 m 一致。这确认了模型比例、安装位姿、运动期间的射线求交与实际回波链路；抽样检查不代表逐点穷举证明。

校验过程还明确了一项采样相位差：ROS `gazebo_ros_p3d` 在 `WorldUpdateBegin` 记录真值，本项目扫描器在 `WorldUpdateEnd` 求交，二者时间戳的仿真时刻相同，但中间执行一次物理推进。校验默认从归档 world 读取物理步长，在点时刻加一个步长查询真值。本组步长为 2 ms，来自插件执行次序而非拟合参数。[ROS 插件源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_p3d.cpp)与 [Gazebo 世界更新源码](https://github.com/gazebosim/gazebo/blob/gazebo11/gazebo/physics/World.cc)可核对这一顺序。

60 秒短测不计相位时，抽样射线距离绝对差的 95 分位约 0.141 m；计入固定 2 ms 后降至 0.041 m，标准差约 0.02063 m。`--truth-offset-s 0` 可复现未计相位的诊断。此处理仅用于独立射线校验，**未改写原始 bag、算法输入时间或轨迹误差评估**，未把它宣称为 LIO 高度漂移的原因。

## 本次修改与验证边界

- 下载器固定资源提交并校验内容，生成工具保留视觉/碰撞几何，设置本机资源路径、2 ms 物理步长与独立俯视相机。
- `scene_profile.py` 为 Gazebo、PX4_HOME、在线 GNSS、轨迹导出、航线控制和计划轨迹可视化提供一致配置；默认走廊的原点、出生点与 8 m 高度保留。
- 录包归档场景 profile、资源清单、world、机体与航线。离线处理优先读取归档原点；新生成的 SignalSim 场景也采用采集原点，避免误用旧的武汉原点。
- 现有 IMU/GPS 噪声、GPS 在线注入和分批 LiDAR 扫描继续生效；未新增 GNSS 建筑遮挡、多路径、材质反射、多回波或地形跟随控制。
- 本次测试均使用 VLP-16 参数模型、滚动扫描、Velodyne 入口、`point_filter_num=1`、MEMS 基线 IMU、standalone GPS；没有在新场景重新跑完四种雷达的对照。
- 自动测试覆盖默认配置回归、新场景原点与路径解析、COLLADA 单位/变换、路径屋顶碰撞筛查及离线原点传播。完整实测与原始门禁结果位于上述三个运行目录。

最终在已配置的 ROS 1 / Pixi 环境执行 `python -m unittest discover -s tests/fastglio -v`，29 项测试全部通过；Shell 语法检查、Python 编译检查、文档本地链接检查与 `git diff --check` 通过。三组集成门禁汇总保存在 [`integration-summary.json`](../../runs/ksql-integration-20261122/integration-summary.json)。测试结束后已清理本次 PX4、Gazebo 和 RViz 进程。
