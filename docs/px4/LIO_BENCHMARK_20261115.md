# 纯 LiDAR/IMU 轨迹对照（2026-11-15）

这次对照使用同一份 Gazebo 飞行录包 `runs/uav-px4-fastglio-bias-unfrozen-20261115/sensors.bag`：16 线、每帧约 1.25 万点、10 Hz 雷达，约 200 Hz IMU。无人机从走廊南端向北飞行。评估只读取各算法的里程计与 `/ground_truth/state`；没有给算法输入真值或 GNSS。FAST-LIO 原在线运行的 `/Odometry` 虽与 GNSS 松耦合模块同时录制，但此话题是独立的纯 LiDAR/IMU 输出，松耦合没有反馈到 LIO。

## 结果

共同统计区间为仿真时间 5.58–34.38 秒。每条里程计与 50 ms 内最近的真值配对，按区间内第一对位置做**平移对齐**；不做旋转或尺度拟合。下表均为 ENU 位置 RMSE，单位米。

| 算法 / 输入 | 配对数 | 东 E | 北 N | 天 U | 三维 |
| --- | ---: | ---: | ---: | ---: | ---: |
| FAST-LIO，当前 MARSIM 入口，全部点 | 289 | 0.211 | 3.621 | 2.339 | 4.316 |
| FAST-LIO，Velodyne 入口，全部点 | 288 | 0.323 | 4.101 | 1.539 | 4.392 |
| FAST-LIO，Velodyne 入口，每 2 点取 1 点 | 288 | 0.150 | 1.200 | 0.354 | 1.260 |
| GLIO 纯 LIO 前端 | 79 | 0.035 | 4.595 | 4.660 | 6.544 |
| LIGO，GNSS 关闭 | 288 | 0.040 | 12.146 | 1.979 | 12.306 |
| LIO-SAM，回环关闭，OpenCV 兼容修复 | 144 | 0.030 | 0.178 | 0.214 | 0.280 |
| LIO-SAM，IMU 姿态改为角速度积分 | 144 | 0.023 | 0.164 | 1.320 | 1.330 |

完整时段的 FAST-LIO 在线结果为 E 0.227、N 3.940、U 2.410、三维 4.624 米；最后一个匹配点的 ENU 误差为 `(-0.448, -7.778, +4.108)` 米。共同区间的数值用于横向比较，完整时段的数值用于对应当前页面显示。总对照曲线见 [`lio-enu-errors.png`](../../runs/uav-px4-fastglio-bias-unfrozen-20261115/comparisons/lio-enu-errors.png)，FAST-LIO 点数控制曲线见 [`lio-filter-ablation.png`](../../runs/uav-px4-fastglio-bias-unfrozen-20261115/comparisons/lio-filter-ablation.png)，LIO-SAM 姿态控制曲线见 [`lio-orientation-ablation.png`](../../runs/uav-px4-fastglio-bias-unfrozen-20261115/comparisons/lio-orientation-ablation.png)。各算法原始 bag 和 JSON 指标在同目录下。

## 北向、天向误差的原因判断

1. 走廊由两侧平行墙构成，墙面主要约束东西方向；沿墙的北向平移更难由单帧几何辨识。每帧约 1.245 万点，约为 16×800 最大采样数的 97%，因此**不是总点数过少**；但靠近地面的约 200 点，高度几何信息相对少。这解释了为何当前 FAST-LIO 的东向好、北向和天向差，但**不是不可避免的场景极限**，因为 LIO-SAM 在同一录包上保持了低北向误差。
2. 当前 FAST-LIO 的 `preprocess/lidar_type: 4` 使用 MARSIM 入口。该入口将每个 Gazebo 点的相对时间设为 0，且不使用 `point_filter_num`。切换为 Velodyne 入口但仍保留全部点，三维误差基本不变（4.316→4.392 米）；因此，仅补 `ring/time` 不是关键改进。
3. 在 Velodyne 入口上**只改变** `point_filter_num` 为 2，每帧点数约减半，三维误差降为 1.260 米，北向从 4.101 降至 1.200 米，天向从 1.539 降至 0.354 米。这证明当前结果对采样点集敏感。2026-11-16 的[进一步受控诊断](LIO_SAMPLING_DIAGNOSIS_20261116.md)发现，奇数点和随机半采样也改善；但单独调低雷达权重、筛除近共线邻域均未复现全部收益，法向均衡指标也没有变好。因此不能把“密集墙面点权重过大”作为已证实的唯一原因，更不能把该收益直接外推到其他场景。
4. 旧版还冻结了 IMU 偏置；之前相同 bag 的对照显示取消冻结让三维 RMSE 从 6.36 降到 4.78 米。当前在线配置已经取消冻结，所以本次剩余误差不应再归因于冻结偏置。

## 外参、时间与 IMU 姿态核查

- Gazebo 模型将 LiDAR 安装在 IMU/`base_link` 上方 0.15 m，姿态一致。FAST-LIO/LIGO 的 `extrinsic_T` 是 LiDAR 坐标转 IMU 坐标所需的 `[0,0,+0.15]`；LIO-SAM 的 `extrinsicTrans` 在位姿组合中采用相反方向，配置为 `[0,0,-0.15]`。这不是三个算法使用了不同物理安装位置。外参的微小残差不能完全排除，但现有配置没有明显的符号或数量级错误。
- 316 帧雷达中，314 帧距离最近 IMU 时间戳不超过 5 ms；中位差 2 ms。另两帧分别相差 78 ms（录制起点，早于首个 IMU）和 40 ms。雷达是 Gazebo 同时生成的快照，给 Velodyne 入口补的逐点时间统一为 1 µs。故本实验没有证据表明持续的雷达/IMU 时间不同步是米级漂移主因；真实旋转雷达仍必须使用真实逐点时间。
- Gazebo IMU 消息含姿态四元数。与真值配对后，其三维姿态误差中位数约 0.36°、95 分位约 0.65°。LIO-SAM 读取该姿态用于初值及滚转/俯仰约束；FAST-LIO 与此次 LIGO 主要从加速度、角速度估计姿态，因此各算法虽然都读同一个 IMU 话题，**实际利用的信息不完全相同**。
- 为检验这一点，另制一份录包：保留首个姿态，此后只用原始角速度按时间戳积分生成 IMU 四元数，点云、加速度和角速度均不变。LIO-SAM 北向 RMSE 仍约 0.16 m，但天向从 0.21 增至 1.32 m，三维从 0.28 增至 1.33 m。**0.28 m 的高度优势明显受益于仿真 IMU 的高质量姿态；北向优势在本次消融中仍存在。** 这不是完全去除姿态输入的实验，仍需保留首帧姿态以满足 LIO-SAM 输入格式。

LIO-SAM 的优势是这次录包上的实测结果，不代表它在所有环境、真实旋转扫描或高动态飞行中必然最优。GLIO 此次只运行 `Preprocessing + LidarOdometry`，不是带其它约束的完整系统；LIGO 使用现有仿真参数，未做专门调参。FAST-LIO 的官方实现同时支持机械旋转与固态雷达；LIGO 作者公开的实机数据使用 Livox Mid-360/Avia。当前 Velodyne 仿真结果不能推出换 Mid-360 一定更好；需要用该雷达真实扫描轨迹、逐点时间及对应 IMU 配置重新实验。

## 输入适配与复现

Gazebo 当前雷达每帧是**瞬时 ray-sensor 快照**，原始 PointCloud2 只有 `x/y/z/intensity`。LIO-SAM、LIGO、FAST-LIO Velodyne 入口要求 `ring/time`。`tools/annotate-gazebo-lidar.py` 按仿真传感器 -15° 到 +15° 的 16 线角度重建 `ring`，给所有点统一赋 1 微秒相对时间，使其表示瞬时采样。它不是旋转雷达的逐点真实时间模型。GLIO 前端直接使用原始点云。LIO-SAM 的 `Horizon_SCAN` 改成仿真的 800，雷达到 IMU 的外参按模型高度 0.15 米设置，回环关闭。

当前 FAST_GLIO 工作区的 LIO-SAM 二进制与 OpenCV 4.13 不兼容：`cv::eigen` 返回列向量，原代码却按行向量索引，运行时在地图优化节点崩溃。实验用 `tools/build-lio-sam-benchmark.py` 在本仓库运行目录编译了一个仅修改两处索引的临时二进制；外部 FAST_GLIO 源码和安装二进制没有改动。原版因此没有有效轨迹，表中 LIO-SAM 明确标记为兼容修复版。

在有 ROS1 与 FAST_GLIO 构建环境的机器上复现：

```bash
cd /home/wzb/msim
RUN=runs/uav-px4-fastglio-bias-unfrozen-20261115
/home/wzb/FAST_GLIO/.pixi/envs/default/bin/python tools/annotate-gazebo-lidar.py --run "$RUN"
bash tools/compare-fastglio-lio.sh "$RUN" glio-front-end
bash tools/compare-fastglio-lio.sh "$RUN" ligo
bash tools/compare-fastglio-lio.sh "$RUN" lio-sam-patched
/home/wzb/FAST_GLIO/.pixi/envs/default/bin/python tools/gyro-orientation-lidar-bag.py --run "$RUN"
bash tools/compare-fastglio-lio.sh "$RUN" lio-sam-gyro-orientation
bash tools/compare-fastglio-lio.sh "$RUN" fast-lio-velodyne
bash tools/compare-fastglio-lio.sh "$RUN" fast-lio-velodyne-full
MPLCONFIGDIR=/tmp/msim-matplotlib /home/wzb/FAST_GLIO/.pixi/envs/default/bin/python \
  tools/plot-fastglio-lio-comparison.py --run "$RUN"
```

下一步若要改善在线仿真，应把 Gazebo 点云实时加上 `ring/time`，让 FAST-LIO 使用 Velodyne 入口并每 2 点取 1 点；或直接把 LIO-SAM 作为纯 LIO 参考轨迹接入可视化。变更前要在新的在线飞行中复核上述离线收益。

另见 [雷达安装倾角对照](LIDAR_TILT_20261115.md)：同航线 0°、5°、10°、15° 的 FAST-LIO 实测表明，5° 下倾改善天向但恶化北向，三维整体以水平安装最好。
