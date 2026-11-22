# 地图参数修正后的四雷达在线重跑（2026-11-22）

已重新完成 VLP-16、HDL-32E、OS1-64、Mid-360 各 60 秒无欺骗的 PX4/Gazebo 在线飞行。四组实际使用全局 `/cube_side_length=1000 m`，飞行、LiDAR 时间配对、在线松组合检查全部通过；各组报告的 LiDAR 配对率均为 100%。这些检查证明数据链路与飞行条件通过，不代表估计精度自动达标。

## 设置与对比边界

沿用原城市走廊、航线及仿真时钟，MEMS 基线 IMU、standalone 带噪 GPS、分时 LiDAR 扫描、10 Hz、0.02 m 测距噪声、全部输入点保留（`point_filter_num=1`）、0.5 m 体素、0.65 s LiDAR 处理缓冲。普通多线雷达使用 Velodyne 入口，Mid-360 使用 Livox 入口。四组均未注入欺骗。

工具已检查修正前后同型号的完整传感器配置、算法 YAML 相同，world、航线、插件摘要以及共同实验设置一致；归档参数确认旧组缺少全局地图边长、新组为 1000 m。没有重新调整 GPS 组合策略。

本次和修正前是不同的闭环在线飞行，初始化、传感器实际采样与真实飞行过程可能不同，不能视为同录包单变量实验，也不能作为实物型号排名。地图参数问题的同录包因果验证见[诊断报告](EAST_ERROR_DIAGNOSIS_20261122.md)。

两批全部曲线共同评价窗口为相对各自首个原生 GPS 采样的 **2.1～62.5 s**。真值位置按估计时间戳插值，只消除首个有效点的平移，不拟合旋转或尺度，不平滑曲线。指标在共同窗口重新计算，所以修正前数值可能与旧表有少量舍入和区间差异。

## 纯 LiDAR/IMU

单位均为米，前三列为修正后各轴 RMSE。

| 雷达 | 东 E | 北 N | 天 U | 修正后三维 RMSE | 修正前三维 RMSE |
| --- | ---: | ---: | ---: | ---: | ---: |
| VLP-16 | 0.024 | 0.010 | 0.163 | 0.165 | 1.378 |
| HDL-32E | 0.069 | 0.052 | 0.088 | 0.124 | 0.320 |
| OS1-64 | 0.050 | 0.024 | 0.080 | 0.097 | 0.707 |
| Mid-360 | 0.013 | 0.006 | 0.879 | 0.879 | 1.130 |

前三种雷达三维误差明显降低，原先 VLP-16、OS1-64 的大幅东向偏移减轻。修正后东向 RMSE 不一定小于北向，剩余厘米级差异不等于之前的地图裁剪错误仍存在。

Mid-360 水平误差很小，但天向 RMSE 仍约 0.879 m，时序曲线存在较大的高度偏差。本次没有单独改变向下覆盖、量程或匹配策略，不能把剩余误差归因于唯一原因。

[修正后 ENU 时序图](../../runs/lidar-comparison-20261122-fixed01/figures/lidar-imu-enu.png) · [修正前后对照图](../../runs/lidar-comparison-20261122-fixed01/figures/lidar-imu-before-after.png)

## LiDAR/IMU/GPS 松组合

单位均为米。

| 雷达 | 东 RMSE | 北 RMSE | 天 RMSE | 三维 RMSE |
| --- | ---: | ---: | ---: | ---: |
| VLP-16 | 0.849 | 0.931 | 2.412 | 2.721 |
| HDL-32E | 0.539 | 0.733 | 2.634 | 2.787 |
| OS1-64 | 0.639 | 0.758 | 2.344 | 2.545 |
| Mid-360 | 0.672 | 1.091 | 2.118 | 2.476 |

当前松组合仍明显受带噪 GPS 影响，精度差于本次纯 LiDAR/IMU。局部地图修正没有同时改变 GPS 的组合权重和修正策略，因此不能宣称多传感器组合精度问题也已解决。需要后续单独评价组合策略。

[修正后松组合 ENU 图](../../runs/lidar-comparison-20261122-fixed01/figures/loose-enu.png) · [松组合修正前后对照图](../../runs/lidar-comparison-20261122-fixed01/figures/loose-before-after.png)

## 数据与复现

新结果目录：`/home/wzb/msim/runs/lidar-comparison-20261122-fixed01/`。原始结果保留在 `runs/lidar-comparison-20261122-01/`。各型号目录保留 bag、实际参数、传感器配置、插件摘要、world、三维轨迹图与所有检查报告。

- [当前 ENU 数据 CSV](../../runs/lidar-comparison-20261122-fixed01/figures/enu-errors.csv)
- [当前完整指标 CSV](../../runs/lidar-comparison-20261122-fixed01/figures/metrics.csv)
- [共同窗口修正前后指标 CSV](../../runs/lidar-comparison-20261122-fixed01/figures/before-after-metrics.csv)
- [修正前后来源、配置摘要、评价方法 JSON](../../runs/lidar-comparison-20261122-fixed01/figures/before-after-summary.json)
- [新运行各组检查与完整参数 JSON](../../runs/lidar-comparison-20261122-fixed01/figures/summary.json)

新的输出目录应使用未被占用的名称：

```bash
cd /home/wzb/msim
bash tools/run-lidar-comparison.sh /home/wzb/msim/runs/lidar-comparison-new 60
PATH=/home/wzb/FAST_GLIO/.pixi/envs/default/bin:$PATH \
  MPLCONFIGDIR=/home/wzb/msim/runs/lidar-comparison-new/mpl \
  python tools/plot-lidar-comparison.py --group /home/wzb/msim/runs/lidar-comparison-new
PATH=/home/wzb/FAST_GLIO/.pixi/envs/default/bin:$PATH \
  MPLCONFIGDIR=/home/wzb/msim/runs/lidar-comparison-new/mpl \
  python tools/compare-lidar-map-fix.py \
    --before /home/wzb/msim/runs/lidar-comparison-20261122-01 \
    --after /home/wzb/msim/runs/lidar-comparison-new
```
