# 录制局部地图旧参数问题的复现视频

2026-11-26。

已提供一键入口，不需要切换 Git 历史、修改已修复的启动文件或启动 Gazebo/PX4。脚本在独立 ROS master 中重新运行纯 LiDAR/IMU，默认使用修复前四雷达实验里的 VLP-16 原始传感器包。

## 先明确视频展示什么

- 默认源：`runs/lidar-comparison-20261122-01/vlp16/sensors.bag`。这份 60 秒无欺骗录包的 LiDAR/IMU 数据仍在，源参数在同目录的 `parameters.yaml`。
- 该包没有录制 `/cloud_registered`，直接 `rosbag play` 不能重现当时的点云画面。因此必须用传感器重新解算。
- `before` 显式设置全局 `/cube_side_length=200`，等效复现原先“全局参数缺失、回退到 200 m”的地图管理条件；`after` 设置全局 1000 m。两次都保留私有位置 1000 m，只有全局地图边长不同。算法的 `mapping/det_range=100 m` 不变，移动判定阈值仍是 `1.5×100=150 m`。
- 当前默认运行本机已安装 FAST-LIO 二进制。因此应把视频标注为“局部地图旧参数复现”，不要称其为当年的原始录屏。若需要沿用当时诊断二进制，可传 `--binary runs/east-error-diagnosis-20261122/build/fastlio_mapping_diagnostic`。
- 真值与 GPS、旧里程计都不输入重新运行的算法；两次使用同一份传感器记录，避免另飞一次引入新的飞行差异。

## 录制修复前

确认外置硬盘已经挂载。`runs/` 中历史 bag 链接指向外置盘，第一次读取大文件可能需要等待。

在一个终端执行：

```bash
bash /home/wzb/msim/tools/local-map-video.sh before
```

脚本会打开 RViz，回放初始处于暂停状态。此时暂时没有点云或 Fixed Frame 尚未出现是正常的，算法还没有收到传感器数据。

1. 把 RViz 放大，保留左侧图层栏；默认固定坐标系为 `camera_init`。
2. 启动录屏。Ubuntu GNOME 可按 **Ctrl+Alt+Shift+R** 打开录屏功能，选择需要的区域，再开始录制。[GNOME 官方说明](https://help.gnome.org/gnome-help/screen-shot-record.html)。
3. 回到执行脚本的终端，按 **空格** 开始回放，再切回 RViz。
4. 默认 0.5 倍速，60 秒传感器数据约播放 120 秒。关注起始静止、起飞和沿走廊飞行阶段，记录当前帧与历史显示之间的偏移、重影和估计轨迹变化。
5. 结束录屏后，回终端按 **Enter** 关闭这一组。脚本只清理自己启动的 ROS master、算法和 RViz，不停止其他端口的仿真。

想按正常速度录制：

```bash
bash /home/wzb/msim/tools/local-map-video.sh before --rate 1
```

回放过程中空格暂停/继续；`s` 是单条消息步进，并非一整帧 LiDAR 步进。重新录制应结束后重新运行脚本，不建议对同一个算法实例循环播放并让时钟倒退。

## 再录修复后

完成上一组并关闭后执行：

```bash
bash /home/wzb/msim/tools/local-map-video.sh after
```

按照相同流程录制，保持相同相机角度、缩放和播放倍率。注意 `after` 仍使用同一个旧传感器包，只修正局部地图参数，并非使用新飞行的 bag。

如想换成 OS1-64 的旧包，两组都指定同一个源目录：

```bash
bash /home/wzb/msim/tools/local-map-video.sh before \
  --source-run /home/wzb/msim/runs/lidar-comparison-20261122-01/os1-64

bash /home/wzb/msim/tools/local-map-video.sh after \
  --source-run /home/wzb/msim/runs/lidar-comparison-20261122-01/os1-64
```

当前脚本只接受归档的 Velodyne 格式输入；不能直接把 Mid-360 的 Livox 入口当作同一格式来回放。

## 图层、证据边界与日志

| 颜色 | 图层含义 |
| --- | --- |
| 红色 | `/cloud_registered` 当前配准帧 |
| 紫色 | RViz 累计保存的配准帧显示历史 |
| 青色 | `/path` 的 LiDAR/IMU 估计轨迹 |

**紫色历史显示不是 FAST-LIO 当前的 ikd-Tree 局部地图。** 此视频可以展示配准偏移、重影与轨迹异常，但没有直接显示算法内部的裁剪方框或删除点计数。讲解时应结合 [局部地图参数诊断](EAST_ERROR_DIAGNOSIS_20261122.md) 的参数和单变量数据证据，不能把画面重影单独当成“边界正在移动”的直接测量。

每次输出单独保存到 `build/local-map-video/before-时间戳/` 或 `after-时间戳/`，包含：

- `parameters.yaml`：本次实际生效参数。
- `experiment.json`：源录包目录、模式、播放倍率、算法路径与 SHA256、复现边界。
- `algorithm.log`、`roscore.log`、`rviz.log`：算法、主节点与显示日志。
- `view.rviz`：本次显示配置，可在 RViz 的 File → Save Config 保存调整后的视角，再用同一配置进行对照。

这不是自动录屏脚本，不会自行读取或保存桌面视频。视频由用户启动的系统录屏功能保存。

## 本轮检查

脚本通过 `bash -n`。在本地 30 秒参考录包上分别用 200 m 和 1000 m 配置做无窗口短回放，确认两组能够启动、完成回放并保存正确参数；修复后组只读订阅实际收到至少 5 帧 `/cloud_registered` 和 5 条 `/Odometry`。这检查的是回放链路，未伪称已经录制了桌面视频或核验了完整 60 秒旧视频画面。
