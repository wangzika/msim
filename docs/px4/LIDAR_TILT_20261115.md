# 雷达安装倾角对照（2026-11-15）

## 目标和设置

在同一城市走廊、同一 PX4 自动航线下，分别将 Gazebo 16 线雷达设为水平、朝行进方向下倾 5°、10° 和 15°。机体在该航线中大致保持零偏航，沿世界/机体 +Y 向北行进，因此下倾对应雷达相对 `base_link` 的滚转角 0°、−5°、−10°、−15°。IMU、雷达线数与角度范围、扫描频率、噪声、飞控航线和 FAST-LIO 参数保持相同。每组的 FAST-LIO `extrinsic_R` 同步改为该滚转矩阵，平移外参仍为模型中的 `[0,0,0.15]` 米。

每组采集 42 秒仿真时长，录制 `/velodyne_points`、`/imu/data`、`/ground_truth/state`，再用同一个适配器补 `ring` 及统一 1 µs 的瞬时扫描时间，离线回放纯 FAST-LIO 的 Velodyne 入口，`point_filter_num=2`、IMU 偏置不冻结。未输入 GNSS 或真值到 LIO。四组真值水平行程均为约 34.2–34.5 米，终点北向位移均约 19.2–19.3 米；各 436–437 帧雷达、432 个匹配里程计点。PX4 ULog 验证四组均已解锁、进入 OFFBOARD、正常爬升和飞行。误差按每组首个匹配位姿做平移对齐，不做旋转或尺度拟合。

## 结果

| 前向下倾 | 低位点/帧中位数¹ | 远北侧点/帧中位数² | 北向 RMSE | 天向 RMSE | 三维 RMSE |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0° | 332 | 712 | 0.76 m | 1.18 m | 1.42 m |
| 5° | 414 | 564 | 2.30 m | **0.42 m** | 2.35 m |
| 10° | 469 | 465 | 6.51 m | 2.20 m | 7.00 m |
| 15° | 756 | 314 | 6.54 m | 1.26 m | 6.70 m |

¹ 巡航高度 7.5–8.5 米时，按真值机体位姿和对应安装旋转把点变到世界坐标，统计高度 −0.2 到 0.6 米的回波。这是“低位点”代理指标，包含地面和墙脚，不等于严格地面分割。² 同样变换后，世界 Y>30 米且 |X|<12 米的远北方向回波，用于估计北端建筑的可见程度。

倾斜确实增加低位点，同时减少远北方向的点。**5° 下倾明显改善天向，但北向变差，三维整体仍劣于水平安装；10° 和 15° 的北向及三维误差更差。** 远北方向回波减少是北向退化的一个合理解释，尚未通过匹配残差或观测矩阵消融证明它是唯一原因。对 10° 的同一录包还做了错误外参消融：若强行使用水平安装的单位旋转矩阵，三维 RMSE 从 7.00 m 进一步恶化到 24.48 m，说明保持对应旋转外参是必要的。曲线见 [`tilt-enu-errors.png`](../../runs/lidar-tilt-20261115/tilt-enu-errors.png)，逐组数据与 JSON 见 [`runs/lidar-tilt-20261115`](../../runs/lidar-tilt-20261115)。

## 复现

```bash
cd /home/wzb/msim
bash tools/capture-lidar-tilt-case.sh 0 runs/lidar-tilt-20261115 42
bash tools/capture-lidar-tilt-case.sh 5 runs/lidar-tilt-20261115 42
bash tools/capture-lidar-tilt-case.sh 10 runs/lidar-tilt-20261115 42
bash tools/capture-lidar-tilt-case.sh 15 runs/lidar-tilt-20261115 42
MPLCONFIGDIR=/tmp/msim-matplotlib /home/wzb/FAST_GLIO/.pixi/envs/default/bin/python \
  tools/analyze-lidar-tilt.py --group runs/lidar-tilt-20261115
```

这些结果来自每个角度一次确定性仿真，不代表其它速度、环境、倾斜轴或真实旋转雷达也会如此。当前 Gazebo 雷达为瞬时 ray-sensor 快照；如果换真实雷达，需要逐点时间和实测外参。若要继续优化，应先检查各角度的特征/平面残差与北向观测退化，再考虑小于 10° 的角度或扩大垂直视场。
