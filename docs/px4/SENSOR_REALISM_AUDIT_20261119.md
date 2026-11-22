# GPS 与 LiDAR 真实性核查（2026-11-19）

历史基线／设计记录：后续实施结果及当前运行入口见 [传感器优化与快速切换](SENSOR_SWITCHING.md)。

本次核查代码、生成的模型、已安装插件及已有录包；只更正文档，未修改传感器行为或重新生成实验图。

## 结论

| 项目 | 当前实际行为 |
| --- | --- |
| 在线 GPS 位置 | 机体真值加旋转后的天线杆臂、固定偏移和欺骗偏移，无随机位置噪声 |
| GPS 报告协方差 | EN 各 0.49 m²、U 为 4 m²，作为估计器权重信息；不生成噪声 |
| PX4 原生 GPS | 原生插件具备位置、速度噪声和相关偏差模型；在线注入模式移除该 GPS 模型，当前数据不经过此模型 |
| LiDAR 输出周期 | 10 Hz，即相邻帧约 100 ms；不是持续 100 ms 的物理扫描 |
| LiDAR 帧内采样 | 一次 ray 更新生成全部方向，同一时刻的几何快照，无旋转扫描运动畸变 |
| 逐点时间 | Velodyne 适配器统一填写 1 微秒，仅作入口适配，不是真实发射时刻 |
| LiDAR 噪声 | SDF 配置 0.02 m，但当前 block laser 插件读取底层距离，绕过 RaySensor scan 消息的噪声层；其额外 XYZ 噪声为 0 |

## 如何核查

1. `adapters/ros1_noetic/fast_glio_uav/prepare_px4_assets.py` 的 `online_gnss` 分支移除 `gps0` 及其关节。LiDAR 为 `ray`，配置 10 Hz、SDF 距离噪声 0.02 m 和 `libgazebo_ros_block_laser.so` 插件，插件 `gaussianNoise=0`。
2. `adapters/ros1_noetic/fast_glio_uav/px4_online_gnss.py` 将真值位置、杆臂与指定偏移直接写入 ROS Odometry，并转换后发送 MAVLink HIL_GPS；没有随机数采样。ROS 方差独立写入，不参与位置生成。速度来自真值差分，当前也没有速度噪声或欺骗速度。HIL_GPS 使用墙钟 UTC 字段，ROS 测量头使用仿真时间；将来增加延迟需明确采样时刻与发布时刻。
3. `tools/annotate-gazebo-lidar.py` 保留已有 XYZ，只增加 ring 与常数 `time=1e-6`，没有帧内时间跨度。
4. 对照 Gazebo Classic gazebo11 的 `RaySensor::UpdateImpl()`：一次更新整组射线，使用一个仿真时间；噪声施加于 scan 消息中的 range。ROS Noetic `GazeboRosBlockLaser::PutLaserData()` 则直接读取 `LaserShape()->GetRange()`，随后仅叠加插件自身的 GaussianKernel 噪声。已安装 `/home/wzb/FAST_GLIO/.pixi/envs/default/lib/libgazebo_ros_block_laser.so` 的反汇编也包含 `RaySensor::LaserShape()`、`MultiRayShape::GetRange()` 与 `GaussianKernel()` 调用。
5. 读取 `runs/scene-comparison-20261116-01/simple/clean/sensors.bag` 最初 8 帧 `/velodyne_points`，时间为 2.202、2.302、2.402、2.502、2.602、2.702、2.802、2.902 s。每帧 28,023 个有效点，相邻帧所有 XYZ 逐点完全相同，位移中位数和 P95 均为 0 m。这是已有数据上的静止阶段检查，结合插件读取路径支持“0.02 m 噪声未到达 ROS 点云”的判断。

## 下一步建模建议（尚未实现）

- GPS 使用明确的测量模型：`测量位置 = 天线真值 + 正常相关偏差 + 白噪声 + 欺骗偏移`。把采样误差参数与发布协方差分开定义并校准；同时给 PX4、ROS 松组合和检测器发送同一次采样的结果，固定种子并记录参数。
- 若先采用 EN 标准差 0.7 m、U 标准差 2 m 的独立白噪声，则白噪声方差对应 0.49、0.49、4 m²。这是可选实验基线，不是已标定的硬件规格。加入额外相关偏差后，应说明报告协方差是否覆盖其不确定性。
- GPS 速度噪声应单独建模，不能简单用带白噪声位置差分代替多普勒测速。欺骗位置与速度是否一致需作为可选攻击模式明确记录。
- LiDAR 先保证测距噪声实际进入输出，可在明确的径向测距层实现，不能把插件 XYZ 各轴噪声直接当作同一种径向模型。
- 要模拟 10 Hz 机械旋转雷达，应在约 100 ms 内按方位/线束的发射时刻分批射线采样，并以当时传感器位姿生成测量；输出每点相对时间及明确的帧起止参考。仅给快照添加 0～0.1 s 时间值会引入错误的去畸变。
- 基于真值位姿给快照合成运动畸变可作近似方案，但不能完整重现扫描过程中遮挡和动态物体可见性；正式扫描模型应按各采样时刻重新求交。
- 以静止点云噪声统计、帧内时间跨度、已知运动下的畸变/去畸变、GPS 样本协方差与偏差自相关做验收，再重新采集与解算。

## 上游依据

- [Gazebo Classic RaySensor.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/sensors/RaySensor.cc)
- [ROS Noetic gazebo_ros_block_laser.cpp](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_block_laser.cpp)
- [SDFormat 1.6 sensor 规范](https://sdformat.org/spec/1.6/sensor/)：`update_rate` 表示数据生成频率，不能据此推断帧内各点的采样时刻。
- 本地 PX4 原生插件：`build/deps/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_gps_plugin.cpp` 及 `models/gps/gps.sdf`。
