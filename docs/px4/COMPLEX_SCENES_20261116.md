# 复杂场景实测：PX4、LiDAR/IMU 与 GPS 松耦合（2026-11-16）

本次已在 Gazebo 中生成错落建筑街区，并从本机 PX4 仓库导入工业货架、托盘和货箱模型，完成三种场景各有／无欺骗的六次独立飞行。**本轮复杂场景没有改善纯 LIO；不能把原来的误差简单归因于场景物体太少。** 场景几何的方向分布确实改变了，但有效回波数量、邻域与匹配条件也随之改变。

## 场景与实际画面

| 场景 | 几何与来源 |
| --- | --- |
| 简单走廊 | 原 FAST_GLIO 城市走廊，5 个箱体；保留为新采集基线。 |
| 错落建筑街区 | 39 个有碰撞几何的建筑、屋顶物体、立柱、斜墙和高架桥。 |
| 工业场地 | 导入 PX4 `warehouse.world` 中的货架、托盘、货箱，共 38 个顶层静态实例；增加 33 个仓库箱体、筒柱、高架梁与台阶。 |

生成器为 [`prepare-complex-scenes.py`](../../tools/prepare-complex-scenes.py)，world 与来源清单见 [`scenes/`](../../scenes/README.md)。工业场地保留货架碰撞网格；普通欧式托盘碰撞使用包围盒，组合托盘还有嵌套模型。没有把纯视觉模型冒充激光几何，也没有下载完整城市资源包。原型工业场地的模型重复过多，已中止并归档为 `industrial_yard/clean-heavy-excluded`，不参与以下统计。

下图由 Gazebo 固定总览相机实际生成：

![错落建筑街区](../../runs/scene-comparison-20261116-01/complex_urban/clean/gazebo-scene-overview.png)

![工业场地](../../runs/scene-comparison-20261116-01/industrial_yard/clean/gazebo-scene-overview.png)

## 对照方法

- 六次均使用同一份 16 线、水平 1800 点、±15°、10 Hz 的 LiDAR 配置及有白噪声、开机零偏和慢漂的 IMU 配置。
- 机体、传感器配置、算法配置和航线文件的 SHA-256 一致；同一场景两组 world 文件也一致。有效 ROS 参数核对为 `lidar_type=2`、`point_filter_num=4`、`bias_freeze_en=false`。
- 在线前端采用 Velodyne 格式入口；ring 来自垂直线号，时间字段统一为微小值以表示瞬时射线扫描，不虚构转一圈的逐点时间。
- 出生点世界 ENU `(0,-20,0)`，目标高度 8 m；北向目标依次为相对出生点 0、8、18、28、33 m。航点停留按仿真时间推进，控制复杂场景实时因子下降造成的时序差异。
- 有欺骗组在注入源启动第 20 s 开始，以东向 `0.18 m/s` 增加 GPS 位置偏移；相同假 GPS 同时发送给 PX4 与松耦合。纯 LIO 不接收 GPS，但实际飞机运动受 PX4 的假 GPS 影响。
- 每次配置采集 60 s；GPS 源早于录包启动，因此公共横轴相对 GPS 源为 `1.20–62.09 s`。误差对各自真值计算，公共窗口首个匹配点平移归零；不拟合旋转或尺度，真值匹配上限 50 ms。
- 每个条件只飞一次，属于功能与场景对照，不构成统计显著性或单因素因果证明。

## 三维 RMSE（m）

| 场景 | 纯 LIO 无欺骗 | 纯 LIO 有欺骗 | 松耦合无欺骗 | 松耦合有欺骗 |
| --- | ---: | ---: | ---: | ---: |
| 简单走廊 | 1.934 | 3.096 | 0.042 | 3.616 |
| 错落建筑街区 | 9.620 | 8.996 | 0.118 | 3.624 |
| 工业场地 | 13.050 | 14.144 | 0.213 | 3.650 |

![三维 RMSE](../../runs/scene-comparison-20261116-01/scene-rmse-comparison.png)

无欺骗松耦合误差小，主要因为 GPS 输入直接由真值生成；有欺骗时三种场景均约 3.6 m，末端东向误差约 7.5 m。它展示了当前松耦合配置受假 GPS 牵引的行为，不能解释为已经实现欺骗抵抗。当前试验跳过了 SignalSim 后处理，未加入建筑引起的 GNSS 多路径、遮挡或射频信号。

## ENU 误差与三维轨迹

![ENU 时序误差](../../runs/scene-comparison-20261116-01/scene-enu-comparison.png)

错落建筑街区的高度误差明显，工业场地则主要在北向失去位移；不能用一个总 RMSE 概括为相同的问题。无欺骗纯 LIO 的分轴 RMSE：

| 场景 | 东 E / m | 北 N / m | 天 U / m |
| --- | ---: | ---: | ---: |
| 简单走廊 | 0.100 | 1.905 | 0.319 |
| 错落建筑街区 | 0.732 | 4.567 | 8.435 |
| 工业场地 | 0.245 | 13.037 | 0.547 |

各场景都有三维轨迹 PNG/PDF、逐时刻误差 CSV 与有／无欺骗详细报告：

- 简单走廊：[三维轨迹](../../runs/scene-comparison-20261116-01/simple/charts/lio-loose-3d-comparison.png) · [PDF](../../runs/scene-comparison-20261116-01/simple/charts/lio-loose-3d-comparison.pdf) · [完整报告](../../runs/scene-comparison-20261116-01/simple/charts/REPORT.md)。
- 错落建筑街区：[三维轨迹](../../runs/scene-comparison-20261116-01/complex_urban/charts/lio-loose-3d-comparison.png) · [PDF](../../runs/scene-comparison-20261116-01/complex_urban/charts/lio-loose-3d-comparison.pdf) · [完整报告](../../runs/scene-comparison-20261116-01/complex_urban/charts/REPORT.md)。
- 工业场地：[三维轨迹](../../runs/scene-comparison-20261116-01/industrial_yard/charts/lio-loose-3d-comparison.png) · [PDF](../../runs/scene-comparison-20261116-01/industrial_yard/charts/lio-loose-3d-comparison.pdf) · [完整报告](../../runs/scene-comparison-20261116-01/industrial_yard/charts/REPORT.md)。

总体表格统一公共窗口并重新归零；各场景详细图使用该场景两次飞行自身的公共窗口及原始首点平移，因此数值会略有差异。

## 实际点云检查

在相对 GPS 源 20、30、40、50、60 s 各取一帧原始点云，统计 0.5–60 m 的有限回波；每十点取一个查询点，以 10 近邻 PCA 估计局部法向，只统计满足平面与邻域半径筛选的邻域。借用真值姿态转至 ENU 仅用于离线诊断，不输入估计器。下表为无欺骗五帧的算术平均：

| 场景 | 有效回波点数 | 距离 >95 m 占比 | 法向主轴 E/N/U 占比 | 靠近地面占比 |
| --- | ---: | ---: | --- | ---: |
| 简单走廊 | 24519 | 10.3% | 76.7%/18.0%/5.2% | 2.0% |
| 错落建筑街区 | 20384 | 22.5% | 34.3%/33.2%/32.5% | 5.1% |
| 工业场地 | 13259 | 43.1% | 23.0%/29.0%/47.9% | 4.4% |

新增场景确实增加了北向、竖向表面的比例，但工业场地同时有更多远距离／接近最大量程回波，60 m 内有效点数约减半。上述法向仅占通过筛选的局部邻域，**不是 FAST-LIO 实际匹配法向、信息矩阵或可观性证明**，不能据此断言哪一个量直接造成漂移。低矮货架在 8 m 高度、±15° 垂直视场下也未必贡献近距离有效点。

## 飞行、同步与完整性

六组均解锁、进入 OFFBOARD、完成实际位移与爬升；PX4 GPS 与注入坐标匹配率均为 100%。每组收到 616 条纯里程计、615 条松耦合输出，接受 615 次 GNSS 更新。原始点云时间戳约 10 Hz，最大帧间隔 100–102 ms，未看到原始扫描大量缺失。

原有在线 LiDAR/GPS 检查要求：起始 1 s 后，至少 90% 的 GPS 历元能使用已到达且不晚于本历元、年龄不超过 200 ms 的扫描。结果如下，未放宽门限：

| 场景 | 无欺骗匹配率 | 有欺骗匹配率 | 完整同步门禁 |
| --- | ---: | ---: | --- |
| 简单走廊 | 0.32% | 0.00% | 两组未通过 |
| 错落建筑街区 | 100.00% | 99.68% | 两组通过 |
| 工业场地 | 75.65% | 86.69% | 两组未通过 |

简单场景的在线扫描年龄多为 206–208 ms，刚超过原门限；工业场地有一部分历元超过 200 ms。点云时间戳完整不等于在线送达足够及时，也不代表已确认物理传感器时间偏差。完整 `pass_flight_gate` 在这四组中仍为 false；保留数据是因为基本飞行、注入与融合检查通过，不能称为所有质量检查通过。运行负载与在线送达差异是本次场景对照的限制，尚未通过固定速率离线重放或多次重复彻底分离。

## 复现与输出

单次双窗口运行与六组批量测试命令见[场景说明](../../scenes/README.md)。分析工具：

```bash
cd /home/wzb/msim
PATH=/home/wzb/FAST_GLIO/.pixi/envs/default/bin:$PATH \
  python tools/analyze-scene-comparison.py --group runs/scene-comparison-20261116-01
```

- [总体 JSON 与逐帧点云统计](../../runs/scene-comparison-20261116-01/scene-results.json)。
- [ENU 逐时刻 CSV](../../runs/scene-comparison-20261116-01/scene-errors.csv)。
- [RMSE PDF](../../runs/scene-comparison-20261116-01/scene-rmse-comparison.pdf) · [ENU PDF](../../runs/scene-comparison-20261116-01/scene-enu-comparison.pdf)。
- [几何预留通道检查](../../runs/scene-comparison-20261116-01/geometry-validation.json)：只检查生成物体的 AABB，不是所有导入网格的完整碰撞证明。

所有六组均保留 `sensors.bag`、`world.sdf`、`vehicle.sdf`、`sensor_profile.json`、`algorithm.yaml`、`parameters.yaml`、飞行与融合验收 JSON，以及实际 Gazebo 截图。后续若继续追根因，应在每个已采集 bag 上固定回放节奏比较匹配与状态估计，并重复独立飞行；当前证据只支持“增加复杂场景没有自动解决问题”。
