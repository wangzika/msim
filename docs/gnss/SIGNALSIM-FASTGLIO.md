# FAST_GLIO 无人机航线的 SignalSim GNSS 后端

`tools/fastglio-uav-capture.sh` 用锁定的 SignalSim 源码生成基站和无人机观测。Gazebo 的真值轨迹、LiDAR、IMU 和 `/clock` 来自 FAST_GLIO；SignalSim 按同一仿真时间轴每 0.2 秒生成一次 GPS、北斗、Galileo、GLONASS 双频观测。生成器读取无人机天线的逐点 ECEF 轨迹。

## 首次准备

```bash
cd /home/wzb/msim
git clone https://github.com/globsky/SignalSim.git build/deps/SignalSim
git -C build/deps/SignalSim checkout baba9deb68ed193efb9fd9a362b431ae8c491fa4
./tools/build-signalsim-adapter.sh
```

适配器在本地编译上游源码，不修改上游仓库。由于上游许可证仍需确认，当前做法是本机研究使用，不把 SignalSim 源码复制进 msim 仓库。另需已构建的 `/home/wzb/FAST_GLIO/devel/setup.bash`。

## 采集和已有轨迹再处理

```bash
cd /home/wzb/msim
./tools/fastglio-uav-capture.sh --duration-s 300 --flight-mode patrol_s --gui
```

脚本在 `runs/uav-fastglio-时间戳/` 下写出 `sensors.bag`、`trajectory.json`、`signalsim/rover.obs`、`signalsim/base.obs`、`signalsim/navigation.nav`、`signalsim/epochs.jsonl`、`combined-signalsim.bag` 和 `verification-signalsim.json`。若已有完整的原始采集，只运行后处理：

```bash
./tools/fastglio-uav-capture.sh --postprocess-only \
  --run /home/wzb/msim/runs/你的采集目录 --duration-s 300
```

新运行的默认场景由 `scenarios/fastglio_signalsim.json` 冻结到运行目录。`--scenario` 可指定已冻结的场景 JSON；它控制 UTC 起点、地理原点、基站、五个建筑盒及墙面反射系数。不要在同一输出目录下更改这些参数后复用旧的 `signalsim/`，应使用新的采集目录。`--retain-nlos` 用于压力测试，会保留仅有单次反射而无直达信号的观测；常规解算默认剔除这类观测。已有运行若带 SignalSim manifest，脚本会按场景 SHA-256 自动找到原始场景，以便复验。

## RTKLIB 解算

已有构建的指纹位于 `docs/rtklib/build-manifest.json`。若需要从 FAST_GLIO 固定源码重新构建：

```bash
cd /home/wzb/msim
BUILD=$PWD/build/rtklib-$(date +%Y%m%d-%H%M%S)
/usr/bin/python3 tools/rtklib/build_rtklib.py \
  --source-root /home/wzb/FAST_GLIO --output-root "$BUILD" \
  --manifest docs/rtklib/build-manifest.json
```

```bash
cd /home/wzb/msim
RUN=$PWD/runs/你的采集目录
OUT=$RUN/rtklib-signalsim-$(date +%Y%m%d-%H%M%S)
/usr/bin/python3 tools/rtklib/rtklib_runner.py \
  --rinex-dir "$RUN/signalsim" --output-dir "$OUT" \
  --build-manifest docs/rtklib/build-manifest.json \
  --mode relative_fixed --navsys 45 --with-stat --timeout 300
cat "$OUT/summary.json"
```

`--navsys 45` 启用 GPS、GLONASS、Galileo 和北斗。结果看 `status_counts`：`1` 是 RTK FIX，`2` 是 FLOAT。绘图真值用：

```bash
/usr/bin/python3 tools/fastglio-export-plot-reference.py \
  --rinex-dir "$RUN/signalsim" --output "$OUT/reference.csv"
```

RTKLIB 输入是三个 RINEX 文件，ROS bag 用于 FAST_GLIO 回放。`combined-signalsim.bag` 包含原有 `/clock`、真值、IMU、LiDAR，加上四星座 `/gnss_raw` 和 `/gnss/signalsim_observations`、`/gnss/signalsim_base_observations`。`/rtklib_odom` 是供原 FAST_GLIO 接口使用的简易 GPS 差分码解，`GNSS_Info.stat=4` 仅表示该码解，**不表示 RTK FIX**。RTK FIX 以离线 RTKLIB `.pos` 为准。无人机模型当前没有相机话题。

若用 `/home/wzb/rtklib-plot-python` 显示，载入 `$OUT/relative_fixed.pos`、同名的 `.pos.stat` 和 `$OUT/reference.csv`。已生成的诊断图见 `runs/uav-fastglio-20261102-192749/signalsim-rtklib-fixed/plots/dashboard.png`。

## 时间、城市模型和已验证结果

每条 GNSS 观测同时保存 GPST 周/周内纳秒和相对采集起点的仿真纳秒。合包时映射为 `capture-report.json` 中的 Gazebo 时间，保留 IMU 和 LiDAR 的原始头时间。五个建筑碰撞盒用于射线遮挡和单次墙面/屋顶镜面反射；可见直达信号时，将反射叠加到码、载波和 C/N0。反射系数 `0.35` 未经实测标定，模型没有完整的材料、天线、接收机环路和衍射效应。默认剔除无直达信号的观测，依赖仿真真值标签，是算法对照设置，不能视为真实接收机自动识别 NLOS。

在 `runs/uav-fastglio-20261102-192749` 的同一条 300 秒航迹上，SignalSim 输出 1500 个 5 Hz 历元。`verification-signalsim.json` 通过：GNSS 到最近 IMU 最大 2 ms、到最近 LiDAR 最大 12 ms，超出 20 ms 的 IMU 历元为 0。RTKLIB 多星座相对固定解 1482/1500 历元，固定解相对真值的三维误差 95% 分位为 0.0154 m；18 个 FLOAT 历元的误差应单独看，不应合并声称全程厘米级。报告见 `signalsim-rtklib-fixed/summary.json`。此配置没有加入接收机热噪声、周跳和复杂传播误差，厘米结果只说明该理想化仿真内部一致。保留纯反射信号的压力测试版本 `signalsim-urban-v3/` 仅有 FLOAT，三维 RMSE 约 33.7 m。当前 FAST_GLIO 内的 RTKLIB 只支持北斗 PRN 1–46，因此适配器从输出中滤掉 C59、C60；此限制在 manifest 中记录。
