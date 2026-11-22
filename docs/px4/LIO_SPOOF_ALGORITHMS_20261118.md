# LiDAR/IMU 算法在有／无欺骗飞行数据上的回放（2026-11-18）

使用与本轮三维展示图相同的成对源数据：`runs/scene-comparison-20261116-01/simple/{clean,spoofed}`。每组均为独立的 60 秒 PX4 飞行，16 线、水平 1800 点、10 Hz LiDAR，以及带白噪声和非零零偏的 ROS IMU。四种前端在每个条件内读取同一录包的 LiDAR/IMU，半速回放，独立 ROS master；不输入 GPS。真值仅用于录制和离线误差计算。

“有欺骗”指 PX4 飞控接收了欺骗 GPS、真实飞行轨迹发生变化，并不代表把欺骗 GPS 输入到这些纯前端。两次飞行的初始化和实际运动存在差异，不能将两图差异全部归因于欺骗或当作抗欺骗能力排名。

## 配置

- FAST-LIO：Velodyne 入口，`point_filter_num=4`，估计 IMU 偏置，GNSS 更新关闭。
- LIO-SAM：关闭回环；水平分辨率为 1800；噪声密度与零偏随机游走参数取源录包的传感器配置。使用之前仅修复 OpenCV 特征值索引兼容问题的独立二进制，未修改外部源码。它读取模拟 IMU 消息中的姿态四元数。
- LIGO：16 线 Velodyne 输入，GNSS/NMEA 关闭，使用现有 LIO 参数。
- GLIO：只运行 Preprocessing 与 LidarOdometry；快照点云不做旋转扫描去畸变。这一配置主要提供雷达前端参考，不是完整 GLIO 系统，也不等同于另外三种惯性融合估计器。
- 格式适配只补 ring 与统一 1 微秒逐点时间，表示瞬时 Gazebo ray 快照；未伪造转一圈的扫描时间。

各组保存了实际生效的 `parameters.yaml`、输入配置 `config.yaml`、算法输出 `algorithm.bag` 及原始日志。LIO-SAM 日志有内部 TF 四元数警告；检查无欺骗组源 IMU 及最终里程计四元数范数均为 1，位置输出有限。警告不等于完整算法可靠性已验证。

## 评价

估计位置与源 bag 中 50 ms 内最近的机体真值配对，以第一对位置平移归零，不做旋转、尺度拟合或曲线平滑。图使用同一坐标范围，并保留各算法实际输出覆盖；超过 1 秒的输出间隔断线显示。LIGO 的末端有效输出比源录包短约 2 秒，未外推补齐。

共同有效统计时段为相对 GPS 源启动 1.694–60.492 秒。以下三维 RMSE 按此公共窗口计算，单位米；各算法自己的完整覆盖指标另存 JSON。

| 算法 | 无欺骗 | 有欺骗飞行数据 |
| --- | ---: | ---: |
| FAST-LIO | 2.298 | 1.768 |
| LIO-SAM | 15.266 | 1.843 |
| LIGO | 27.752 | 58.313 |
| GLIO 前端参考 | 11.266 | 17.336 |

这是当前噪声模型、点云密度、场景和参数下的结果，不沿用 2026-11-15 旧低噪声录包上的 LIO-SAM 低误差曲线，也不能直接横向比较那组旧数值。

## 图片与数据

![有欺骗飞行数据](../../runs/lio-spoof-algorithms-20261118/figures/spoofed-lio-algorithms-enu.png)

![无欺骗](../../runs/lio-spoof-algorithms-20261118/figures/clean-lio-algorithms-enu.png)

[来源与指标 JSON](../../runs/lio-spoof-algorithms-20261118/figures/summary.json) · [有欺骗误差 CSV](../../runs/lio-spoof-algorithms-20261118/figures/spoofed-errors.csv) · [无欺骗误差 CSV](../../runs/lio-spoof-algorithms-20261118/figures/clean-errors.csv)

绘图：`tools/plot-lio-spoof-algorithm-errors.py --group runs/lio-spoof-algorithms-20261118`。逐算法回放：`tools/run-lio-spoof-benchmark.sh 结果目录 算法名`。

非线性纵轴版本在绘图命令后加 `--yscale symlog`：默认 ±0.5 m 内保持线性，外侧按对称对数压缩，正负误差均保留。它只改变显示间距，数据与 RMSE 不变；曲线斜率和纵向距离不能按线性坐标解释。[有欺骗非线性图](../../runs/lio-spoof-algorithms-20261118/figures/spoofed-lio-algorithms-enu-symlog.png) · [无欺骗非线性图](../../runs/lio-spoof-algorithms-20261118/figures/clean-lio-algorithms-enu-symlog.png)。
