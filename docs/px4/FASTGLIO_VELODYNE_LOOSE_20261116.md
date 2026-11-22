# 新前端的多传感器有／无 GPS 欺骗重算（2026-11-16）

本次直接回放之前两次 60 秒闭环飞行的传感器与 GPS 录包，重新运行 FASTGLIO 纯 LIO 和 GPS 松耦合，没有重新运行 PX4/Gazebo 飞行。原始 IMU、真值和 GPS 消息保持不变；LiDAR 的 XYZ、强度保持原值，补充 16 线 `ring` 和统一 1 µs 的瞬时扫描时间以使用 Velodyne 入口。算法输出不反馈给飞控或纯 LIO 前端。

## 本次配置

先按之前较好的设置测试 Velodyne 入口、每两点取一；在当前 1800 水平采样、带独立零偏和噪声的 IMU 数据上，纯 LIO 仍有较大漂移。随后采用每四点取一，使保留的水平采样密度接近旧 800 水平采样时每两点取一的设置。两套结果均保留；本报告主结果使用两组都统一为 `point_filter_num=4` 的配置，没有按场景分别选参数。

- LiDAR：`lidar_type=2`，16 线，10 Hz；每四点取一点；扫描及地图体素均为 0.5 m。
- IMU：使用原录包的新噪声与零偏模型，时间偏移保持 0，开启算法偏置估计。
- GPS：原录包中的 `/msim/spoofed_gnss_odom`，ENU 坐标；有欺骗组从 GPS 源启动后第 20 秒起向东增加 0.18 m/s 偏移。
- 松耦合：沿用当前演示配置，残差门限较宽；不向纯 LIO 反馈修正。
- 真值：仅用于轨迹误差评估；首点平移对齐，无旋转或尺度拟合。

## 结果

公共统计区间为 GPS 源启动后 1.314～62.314 秒。下表为位置 RMSE，单位米。

| 场景 | 解算 | 东 E | 北 N | 天 U | 三维 |
|---|---|---:|---:|---:|---:|
| 无欺骗 | 纯 LiDAR/IMU | 1.020 | 1.461 | 4.049 | 4.423 |
| 无欺骗 | LiDAR＋IMU＋GPS 松耦合 | 0.051 | 0.033 | 0.062 | 0.086 |
| 有 GPS 欺骗 | 纯 LiDAR/IMU | 0.239 | 1.792 | 5.933 | 6.203 |
| 有 GPS 欺骗 | LiDAR＋IMU＋GPS 松耦合 | 3.648 | 0.029 | 0.069 | 3.648 |

有欺骗组松耦合末端东向误差约 7.59 m，表明较好的 LiDAR 前端并不能自动使当前松耦合输出抵抗 GPS 欺骗。当前策略允许接受缓慢偏移的 GPS，输出会逐渐被其拉偏。

纯 LiDAR/IMU 的北向比原在线结果改善，但高度仍漂移；没有在新传感器数据上复现旧录包约 1.3 m 的三维误差。无欺骗 GPS 仍由仿真真值生成、没有真实接收机随机误差，不能把松耦合小于 0.1 m 的结果作为实物精度结论。

![三维轨迹](../../runs/fastglio-velodyne-density-matched-20261116-01/charts/lio-loose-3d-comparison.png)

![纯 LIO 与松耦合 ENU 对比](../../runs/fastglio-velodyne-density-matched-20261116-01/charts/lio-loose-enu-comparison.png)

![松耦合有无欺骗 ENU 对比](../../runs/fastglio-velodyne-density-matched-20261116-01/charts/loose-enu-comparison.png)

## 验证和原始文件

两组均完成 GPS 对齐，分别接受 612 和 611 次融合更新。所有原始输入消息计数保留，记录的 IMU、真值及 GPS 消息逐话题序列哈希与原录包一致；参数快照确认 `lidar_type=2`、全局 `point_filter_num=4`、`feedback_to_lio=false`。检查结果见 [输入验证](../../runs/fastglio-velodyne-density-matched-20261116-01/input-validation.json)。

运行目录中的 `px4-flight-report.json` 继承自原飞行，其 200 ms 在线 LiDAR/GPS 配对检查未通过；这不是本次离线重算重新测得的传输延迟。本次重算的输入时间戳、点云格式和更新检查单独保存在 `replay-manifest.json`、`parameters.yaml`、`fastglio-live-report.json` 中。

- [无欺骗重算 bag](../../runs/fastglio-velodyne-density-matched-20261116-01/clean/sensors.bag)；[有欺骗重算 bag](../../runs/fastglio-velodyne-density-matched-20261116-01/spoofed/sensors.bag)。新包包含 IMU、带 ring/time 的点云、GPS 输入、真值以及新解算输出；原始 XYZ 点云仍在原录包中。
- [逐时刻误差 CSV](../../runs/fastglio-velodyne-density-matched-20261116-01/charts/comparison-errors.csv)、[三维轨迹 CSV](../../runs/fastglio-velodyne-density-matched-20261116-01/charts/lio-loose-3d-trajectories.csv)、[结果 JSON](../../runs/fastglio-velodyne-density-matched-20261116-01/charts/comparison-summary.json)。
- [每两点取一的完整对照](../../runs/fastglio-velodyne-loose-20261116-01/charts/REPORT.md)。
- [原在线／半采样／四分之一采样比较](../../runs/fastglio-velodyne-density-matched-20261116-01/frontend-comparison/frontend-results.png)。该图另取全部六组共同时间区间、重新以区间首点平移对齐，其统计值与上表略有不同。在线与回放也存在初始化和缓冲差别，不能作为只改变采样率的严格因果消融。

## 复现

在已有 ROS 1/Pixi 环境的终端运行，输出目录必须是尚不存在的新目录：

```bash
cd /home/wzb/msim
export PATH=/home/wzb/FAST_GLIO/.pixi/envs/default/bin:$PATH
export CONDA_PREFIX=/home/wzb/FAST_GLIO/.pixi/envs/default
source /home/wzb/FAST_GLIO/devel/setup.bash
python tools/replay-fastglio-loose-comparison.py \
  --source runs/fastglio-spoof-comparison-20261116-01/clean \
  --output runs/新的结果目录/clean --stride 4
python tools/replay-fastglio-loose-comparison.py \
  --source runs/fastglio-spoof-comparison-20261116-01/spoofed \
  --output runs/新的结果目录/spoofed --stride 4
```

脚本保存输入包、参数快照与重算结果。绘图使用 `tools/compare-fastglio-spoof-runs.py` 和 `tools/plot-fastglio-3d-comparison.py`。
