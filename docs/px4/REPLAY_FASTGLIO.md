# 离线回放 PX4 bag 并查看 FAST_GLIO

## 一条命令打开 RViz

在 msim 工作空间运行：

```bash
cd /home/wzb/msim
./tools/fastglio-replay-px4.sh \
  --run /home/wzb/msim/runs/uav-px4-smoke2-20261105 \
  --mode lio --rate 0.5 --rviz --hold
```

脚本启动独立的 ROS 1 master（默认端口 11312）、FAST_GLIO 的 `fast_lio/fastlio_mapping`、RViz 和真值轨迹发布器，再用 `rosbag play --clock` 播放这次的 `combined-signalsim.bag`。它只重放算法需要的 IMU、LiDAR、GNSS、码差里程计和真值；bag 内原有 `/clock` 不再重复发布。回放结束后 RViz 会保持打开，按回车关闭。运行时**不要同时启动 Gazebo/PX4 实时仿真**，避免同名传感器话题混入回放。

若 RViz 首次启动出现 ROS 1 生命周期提示，先关闭提示框。在 RViz 中保留 `Fixed Frame = camera_init`，查看已有的 `Map`、`FAST-LIO Path`、`Odometry` 显示。如果要叠加真值，点击 `Add` → `By topic` → `/ground_truth/aligned_path` → `Path`；这条真值轨迹只减去了起点平移，和算法地图共用初始坐标系。

算法输出保存在运行目录的 `replays/fastglio-lio-时间/`：`algorithm.bag` 包含 `/Odometry`、`/path`、真值等，`summary.json` 给出与真值第一帧平移对齐后的误差，`fastglio.log` 是算法日志。可以显式指定 `--output /home/wzb/msim/runs/我的回放目录`。更换 300 秒 bag 时把 `--run` 换成对应的采集目录。

本机这批 40 秒 bag 的成功 GUI 回放位于 `runs/uav-px4-smoke2-20261105/replays/fastglio-gui-verified/`：412 条算法里程计与真值配对，水平 RMSE 2.59 m、三维 RMSE 4.93 m，末端三维误差 7.00 m。这里以第一帧作平移对齐，未拟合旋转或尺度；这些是短航程的 LIO 表现，不是 RTK 精度。

## 查看 GNSS 紧耦合模式

```bash
cd /home/wzb/msim
./tools/fastglio-replay-px4.sh \
  --run /home/wzb/msim/runs/uav-px4-smoke2-20261105 \
  --mode tight --rate 0.5 --rviz --hold
```

`--mode tight` 启用 FAST_GLIO 对 `/gnss_raw` 的紧耦合处理。**要确认 GNSS 真的参与估计**，检查输出 `summary.json` 中的 `gnss_alignment_reached`、`gnss_diagnostic_status_counts`，以及 `algorithm.bag` 是否出现 `/loose_odom`。仅看到 `/gnss_raw` 或 `/Odometry` 不足以证明融合已发生。

这批 40 秒短测的 `tight` 回放收到 GNSS，但 135 条诊断均为状态 `1`（等待对齐），没有 `/loose_odom`；`tight` 和纯 `lio` 的 `/Odometry` 因而相同。短测的 `/rtklib_odom` 只有 72 条，日志显示部分相邻解间隔大于 FAST_GLIO 的 1.1 秒插值上限。需要更长且连续的有效 GNSS 解轨迹再验收紧耦合效果。

## 手工分终端运行

需要自己控制暂停时，可不用一键脚本。每个终端先运行：

```bash
export PATH=/home/wzb/FAST_GLIO/.pixi/envs/default/bin:$PATH
export CONDA_PREFIX=/home/wzb/FAST_GLIO/.pixi/envs/default
source /home/wzb/FAST_GLIO/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
```

然后在终端 1 运行 `roscore`。在终端 2 运行：

```bash
roslaunch fast_lio gnss2iekf_experiment.launch \
  config_file:=/home/wzb/msim/adapters/ros1_noetic/fast_glio_uav/px4_replay.yaml \
  mode:=lio rviz:=true lever_arm_body:='[0.05, 0.0, 0.04]'
```

等 `/laserMapping` 启动后，在终端 3 运行：

```bash
rosbag play --clock --pause -r 0.5 \
  /home/wzb/msim/runs/uav-px4-smoke2-20261105/combined-signalsim.bag \
  --topics /imu/data /velodyne_points /gnss_raw /rtklib_odom /ground_truth/state
```

按空格开始或暂停播放。需要紧耦合时把终端 2 的 `mode:=lio` 改成 `mode:=tight`，重新启动节点并从头播放 bag。
