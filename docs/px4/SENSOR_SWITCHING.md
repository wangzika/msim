# 传感器优化与快速切换

2026-11-19 起，`px4-uav-capture.sh` 默认使用原生 Gazebo GPS＋在线欺骗中继、带径向测距噪声的分时 LiDAR、Velodyne 前端。旧录包不变。这里的型号是研究参数近似，尚不是厂商逐束标定模型。

2026-11-22 新增 `--lidar-model mid360`：使用 Livox 官方 CSV 方向序列和原生 Livox FAST-LIO 入口，默认不抽稀。详细配置、时间与测试结果见 [Mid-360 接入报告](MID360_20261122.md)。

## 一条命令运行

```bash
cd /home/wzb/msim
./tools/px4-uav-capture.sh --duration-s 60 --fastglio-live --skip-signalsim \
  --flight-pattern route --route-clock sim --point-filter-num 4 \
  --lidar-model vlp16 --imu-model mems-baseline --gps-model standalone \
  --lidar-timing rolling --spoof-start-s 25 --spoof-east-rate-mps 0.18
```

Gazebo 与 RViz 同时显示；到时自动结束并保存录包及 PNG。不要并行启动两次默认 PX4 采集，它们使用相同的本地端口。无需显示窗口时增加 `--headless`。删除两个 `--spoof-*` 参数即可运行无欺骗组，GPS 正常噪声仍存在。

## 快速换型

查看全部选项：

```bash
./tools/px4-uav-capture.sh --list-sensors
```

| 参数 | 可选项 | 含义 |
| --- | --- | --- |
| `--lidar-model` | `vlp16`（默认） | 16 线、1800 水平采样、垂直 −15°～15° |
| | `hdl32` | 32 线、1800 水平采样、垂直 −30.67°～10.67° |
| | `os1-64` | 64 线、1024 水平采样、垂直 −22.5°～22.5° |
| | `mid360` | 官方非重复扫描方向序列，360° × 名义 −7°～52°，200,000 点/s |
| `--imu-model` | `mems-baseline`（默认） | 现有 MEMS 噪声、零偏与慢漂移基线 |
| | `low-noise` | 上述噪声、零偏和随机驱动强度的 0.2 倍；不代表某台具体实物 |
| `--gps-model` | `standalone`（默认） | 原生 GPS 噪声＋接收机相关偏差与白噪声；附加位置误差随周围遮挡变化 |
| | `native` | 仅保留 PX4 原生 GPS 误差，不附加接收机噪声；仍支持欺骗 |
| | `ideal` | 关闭原生及附加噪声，用作理想控制组；仍支持欺骗 |
| `--lidar-timing` | `rolling`（默认） | 在不同物理时刻分批求交，保留各点实际采集时间 |
| | `snapshot` | 同一时刻求交，作为运动畸变对照 |

例如将上面的命令改成 `--lidar-model hdl32` 就切换为 32 线；改成 `--lidar-model os1-64 --imu-model low-noise --gps-model native` 则同时切换三项。脚本自动配置 FAST-LIO 的入口类型、线数／通道数、扫描率、点时间单位和输入话题，不需要手改 YAML。默认抽稀参数按型号选择：Mid-360 为 1（全部保留），其他型号为 2；显式 `--point-filter-num` 可覆盖。IMU 的硬件误差参数不直接替代滤波器过程噪声，后者仍由算法配置决定。

原有三种 LiDAR 均为均匀垂直角近似、360° 水平视场、10 Hz、0.3～100 m、径向高斯测距噪声标准差 0.02 m、距离量化 0.001 m。实际型号的非均匀垂直角、通道标定及逐束发射时差尚未复现。新增 Mid-360 使用官方 CSV 方向序列，在现有插件中分时求交，不使用均匀线束网格；其默认量程 0.1～40 m，四组交错通道不等于四条固定线束，仍是研究近似。

## 保存、检查和修改自定义配置

先生成可复用 JSON，不启动仿真：

```bash
python3 tools/sensor-config.py --lidar-model hdl32 --gps-model standalone \
  --output /home/wzb/msim/config/my-sensors.json
```

编辑后运行：

```bash
./tools/px4-uav-capture.sh --duration-s 60 --fastglio-live --skip-signalsim \
  --sensor-profile /home/wzb/msim/config/my-sensors.json
```

自定义文件未被显式选择器覆盖的参数保持原值；可以再加 `--gps-model native` 只替换 GPS 部分。旧 schema 1 文件可读取，缺失 LiDAR 时间模式时按快照解释。每次运行把最终有效参数存为 `runs/本次目录/sensor_profile.json`，同时保存 `vehicle.sdf`、`world.sdf`、算法 YAML 和 `capture-settings.json`，不要凭命令中的型号判断实际参数。

## GPS 数据链路与误差

`原生 GPS 真值＋原生误差与延迟 → 独立原始 Transport 话题 → msim GPS 中继 → 同一受攻击样本送入 PX4 与 ROS`

保留原生 `gps0`，只将其输出重定向到 `msim_gps_raw`。中继向原生 MAVLink 接口期望的 `gps0` 话题发布，旧 Python HIL_GPS 发送停止；`px4_online_gnss.py` 在新模式下只记录和检测，不再次注入。PX4 与组合算法使用同一次噪声实现和攻击偏移。独立插件构建在 `build/gazebo-sensors/`，不覆盖第三方源码或插件库。

`standalone` 的附加误差（每轴）：

| 参数 | 东／北 | 天向 |
| --- | ---: | ---: |
| 位置白噪声标准差 | 0.10 m | 0.15 m |
| 一阶相关位置偏差稳态标准差 | 0.70 m | 2.0 m |
| 偏差相关时间 | 120 s | 120 s |
| 速度白噪声标准差 | 0.05 m/s | 0.10 m/s |
| ROS 报告位置方差 | 1.0 m² | 5.0 m² |

附加偏差按实际样本间隔精确离散：`b(k)=exp(-dt/tau)*b(k-1)+sigma*sqrt(1-exp(-2dt/tau))*N(0,1)`，初值从稳态分布抽样。随机种子在 JSON 中可修改。原生 GPS 的误差另行保留，因此报告方差是显式实验权重，并非实物标定结果。GPS 位置白噪声不会被差分成速度噪声。

`standalone` 默认启用场景自适应附加误差：从 GPS 天线向 12 个方位、15°／35°／60° 仰角投射 36 条射线，在 `environment_range_m`（默认 80 m）内统计被碰撞几何挡住的比例 `f`。位置白噪声、相关偏差和速度白噪声在每个 GPS 历元乘以 `1 + environment_noise_gain × f`（默认增益 3）；发布给 PX4 的 `eph/epv`、ROS 位置和速度方差同步增加。`/msim/gps_samples` 记录 `sky_blocked_fraction`、`environment_noise_scale` 与报告方差，便于核对开阔区和建筑旁的变化。自定义传感器 JSON 可调整上述两个参数，或设置 `gps.adaptive_environment=false` 关闭；`native`、`ideal` 默认关闭。

这是根据局部可见天空比例构造的环境误差近似，适用于在线 PX4 测量；它不逐卫星解算遮挡、多径或卫星几何。逐卫星传播仍由离线 SignalSim 观测链路处理。建筑需有 Gazebo 碰撞几何才会参与射线测试；单纯视觉贴图不会增加噪声。

原生 GPS 名义为 5 Hz，当前上游更新门限可能使实际间隔出现约 0.2/0.4 s，不能用名义值替代实测频率。原生发布延迟实测约 118～122 ms。中继保留样本时间；攻击的 `--spoof-start-s` 以原生 GPS 首个采样的仿真时刻为零点，日志的 `gnss_origin_sim_time_ns` 记录该原点。

默认同时给位置漂移叠加相应速度偏移。若要复现只改位置的旧实验，在自定义 JSON 中设置 `gps.spoof_velocity_consistent=false`。固定 `--gnss-east-bias-m` 与 `--gnss-north-bias-m` 独立于定时漂移，持续应用。

新增记录：`/msim/normal_gnss_odom` 为相同噪声但无欺骗的正常样本；`/msim/spoofed_gnss_odom` 为真正提供给组合算法的位置；`/msim/gps_samples` 包含两者及攻击量、采样/发布时间。正常样本是同一飞行上的旁路测量，不能视为重新飞行得到的无欺骗组。

## 运行中修改欺骗

另开终端，无需手动配置 ROS 环境：

```bash
cd /home/wzb/msim
./tools/gps-attack.sh --east-rate-mps 0.18
./tools/gps-attack.sh --stop
```

第一条命令从发送时刻开始向东漂移；第二条撤销攻击。也支持 `--east-offset-m`、`--north-offset-m`、`--up-offset-m`、`--north-rate-mps`、`--start-delay-s`、`--position-only`。使用 `--help` 查看。新命令替换先前攻击，不是累加。

底层接口为向 `/msim/gps_attack` 发布 JSON。例如已配置 ROS 环境时：

```bash
export PATH=/home/wzb/FAST_GLIO/.pixi/envs/default/bin:$PATH
source /home/wzb/FAST_GLIO/devel/setup.bash
rostopic pub -1 /msim/gps_attack std_msgs/String \
  'data: "{\"enabled\":true,\"east_rate_mps\":0.18,\"velocity_consistent\":true}"'
```

字段还包括 `start_delay_s`、`east_offset_m`、`north_offset_m`、`up_offset_m`、`north_rate_mps`。新命令替换当前攻击参数；`enabled=false` 撤销可控攻击并回到带正常噪声的测量，已有固定偏移仍按启动参数保留。撤销可产生位置跳变，若需要平滑撤回，应单独定义轨迹命令。

## 分时扫描与组合处理延迟

在当前 2 ms 物理步长下，10 Hz 分时扫描由约 50 批组成，一帧点时间覆盖 0～98 ms；每批按当时飞机的真实位姿求交。点保存在各自采集时刻的雷达坐标系，供算法进行去畸变。扫描调度由物理时间推进，不按机器墙钟速度生成。快照模式所有点仍为统一微小时间，以免旧 FAST-LIO 入口自行推算旋转时间。

输出以 ring 优先排列，保留真实时间，避免全局每四点抽稀时只留下每四条线中的一条。原始点云始终直接发布到 `/velodyne_points`。

当前外部 FAST-LIO 松组合入口需要相邻 GPS 样本包围待处理时刻。为保留原生 GPS 延迟而不修改第三方估计器，适配器默认将 LiDAR **处理**延后 0.65 s，测量头和逐点时间完全不变，IMU 在算法内缓冲。RViz 因此显示约 0.65 s 前的估计；这不等于传感器采样时钟被改动。通过 `--fusion-delay-s` 调整，降到过小可能再次失去组合更新。

`/msim/lidar_scan_info` 记录每帧起止时间、点数及模式。MARSIM 入口仅允许显式快照模式；均匀线束分时扫描使用 Velodyne 入口，Mid-360 自动使用原生 Livox 入口。

## 验证记录

四种雷达同场景、同 IMU/GPS、全部保留点云的无欺骗新采集及 ENU 曲线见 [2026-11-22 雷达对比](LIDAR_COMPARISON_20261122.md)。

- 配置与既有单元检查：`tests/fastglio/`。
- 四种房间用例（16 线快照、16/32/64 线分时）验证点数、线数、解析几何测距残差、时间跨度、GPS 在线命令和正常／攻击逐样本差值：`runs/sensor-upgrade-20261119/room-check-04/validation.json`，全部通过。
- 三种线数测距噪声实测标准差分别约 0.020019、0.020001、0.020004 m；分时帧内跨度均为 0.098 s，帧间隔 0.1 s。
- 完整飞行及定位结果：`runs/sensor-upgrade-20261119/final-rolling-vlp16/`；以该目录最终报告为准。
- 最终 60 秒飞行、GPS 注入匹配与 LiDAR 时间配对检查均通过，610 次松组合更新接受。GPS 实际约 3.312 Hz，原生延迟 118～122 ms。
- 对 35 帧运动扫描、51,889 个点按逐点时间及独立真值变换，与场景解析表面比较：距离 RMSE 0.01591 m。错误地把整帧放在帧开始／结束时刻时分别为 0.20550／0.17472 m。此数值是几何表面一致性误差，不是导航轨迹 RMSE，验证了分时求交的运动信息。报告在最终目录的 `sensor-upgrade-validation.json`。
- `rolling-vlp16` 至 `rolling-vlp16-03` 是联调失败／中间用例，不作为最终展示结果。`rolling-vlp16-04` 是处理缓冲的中间验证，最终版本另修正点排列。
