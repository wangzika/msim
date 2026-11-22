# ROS 1 无人机与 SignalSim 项目内容参考

更新时间：2026-11-04。以仓库 `README.md`、`docs/gnss/SIGNALSIM-FASTGLIO.md` 和具体运行报告为状态真源。

## 阅读顺序

1. `README.md`：当前唯一主线、框架图和执行入口。
2. `docs/gnss/SIGNALSIM-FASTGLIO.md`：配置、采集、后处理、RINEX 与 RTKLIB。
3. `runs/<运行目录>/verification-signalsim.json`：合包和同步质量。
4. `runs/<运行目录>/signalsim/manifest.json` 与 RTKLIB `summary.json`：观测来源、解算状态和真值误差。

## 当前数据链

```text
FAST_GLIO ROS 1 Gazebo 城市场景 + quadrotor 预设航线
  → /clock、/ground_truth/state、/imu/data、/velodyne_points
  → sensors.bag + trajectory.json
  → SignalSim 轨迹驱动双站四星座双频观测
  → rover.obs、base.obs、navigation.nav + GNSS 历元
  → combined-signalsim.bag + verification-signalsim.json
  → RTKLIB 离线相对定位 .pos/.pos.stat + summary.json
```

默认 GNSS 场景是 `scenarios/fastglio_signalsim.json`。`apps/fastglio/trajectory.py` 在仿真时间插值真值，`apps/fastglio/run_signalsim.py` 驱动锁定的 SignalSim 源码，`adapters/signalsim/observables.cpp` 实施五个建筑碰撞盒的射线遮挡和一次镜面反射。合包保留传感器原始时间戳；GNSS 同时记录 GPST 和相对仿真时间。

## 已验证与边界

现有 300 秒航迹包含 1500 个 5 Hz GNSS 历元。`verification-signalsim.json` 通过：GNSS 到最近 IMU 最大差 2 ms、到最近 LiDAR 最大差 12 ms。RTKLIB 四星座相对定位输出 1482 个 FIX、18 个 FLOAT；FIX 位置误差 95% 分位约 0.0154 m。需注明这是理想化仿真设置中的解算结果，不能当作现实城市环境的精度保证。

无人机当前没有相机话题。墙面反射系数未用实测材料和接收机标定；无直达路径的观测默认由真值标签剔除。bag 内 `/rtklib_odom` 是简化码差分辅助位置，RTK FIX 状态以离线 RTKLIB `.pos` 为准。RTKLIB 解算处于离线阶段，尚未形成实时定位反馈控制闭环。SignalSim 上游许可仍需另行核查；仓库不复制其源码。

参考演示文稿的旧阶段、ROS 2 车模和独立 GPS 几何后端叙述已过时，只取视觉样式。
