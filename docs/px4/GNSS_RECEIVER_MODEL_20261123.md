# GNSS 接收机位置误差基线与参数来源

更新：2026-11-23。新采集默认使用 `--gps-model f9p-dgnss-ucr`；已有录包与历史参数不修改。

## 选定的设备、模式与适用范围

选择 **u-blox ZED-F9P，单频 L1/E1/B1 码差分 DGNSS**。参数来自参考资料 [1] 附录 B §4、Table 1 的去均值位置误差拟合。原实验使用 GPS/Galileo/BeiDou，在较开阔的车载环境采集约一小时、1 Hz 的位置数据，以另一套 M8P 固定 RTK 作参考。[1]

本实现是**文献位置误差参数复现**，不是厂商标定的完整接收机，也不在 Gazebo 中实际解算 RTCM、伪距或载波相位。它不代表单点定位、RTK 固定解或建筑峡谷的真实精度。

报告分析了有限实验的均值、慢变项和白噪声。本预设只采用去均值的随机项；不施加表中有限样本均值，也不把原文中不同位置出现的均值数字混用。北向白噪声的 `0.00` 是发表时的舍入值，本实现按表值使用，不表示真实硬件没有白噪声。

## 参数与单位

源表是 NED；实现按 ENU 排序。下向改成天向后，零均值方差不变。`white_sigma` 是标准差，不能再对它开平方；`bias_variance` 才是慢变项方差。[1]

| 参数 | 东 E | 北 N | 天 U | 实现来源 |
| --- | ---: | ---: | ---: | --- |
| 白噪声标准差 σ，m | 0.03 | 0.00 | 0.02 | 源表标准差列 |
| 慢变项稳态方差 P，m² | 0.08 | 0.08 | 0.18 | 源表方差列 |
| 慢变项稳态标准差 √P，m | 0.282843 | 0.282843 | 0.424264 | 由 P 换算 |
| 相关时间 τ，s | 128.51 | 65.60 | 95.31 | 源表相关时间列 |
| 报告位置方差 R，m² | **0.0809** | **0.0800** | **0.1804** | 本实现按 P＋σ² 推导 |

原生 GPS 噪声关闭，避免在完整位置误差模型之上重复叠加原生误差。经验遮挡放大关闭，倍率固定为 1。种子默认 `20261119`，可以通过自定义 JSON 调整。

关闭环境模型时，日志中的遮挡比例 0 表示该项未评估，不能解释为实际天空完全无遮挡。

新模型以 `position_sigma_enu_m`、`bias_sigma_enu_m` 和 `bias_tau_enu_s` 三轴字段为准；旧 XY/Z 标量噪声字段仅作兼容占位，不参与此模型的误差生成。

速度暂时使用关闭原生噪声后的理想运动学速度，附加速度噪声为零；参考资料没有提供可直接迁移的速度误差标定。**本预设不能作为真实 GPS 速度或完整飞控导航鲁棒性的标定模型。** LiDAR/IMU/GPS 松组合目前使用其位置及位置方差。

原生 GPS 的发布节奏与约 120 ms 延迟保留，名义约 5 Hz，实际间隔以记录为准。源实验为 1 Hz：慢变项按真实采样间隔重新离散，白噪声按每次测量的标准差沿用。因此当前是对采样率的工程扩展，不宣称在 5 Hz 下完成了设备实测标定。若需要原实验的严格时间复现，应使用原始 1 Hz 数据。

## 误差生成与报告方差的统一

对每一轴 i，定义零均值误差：

\[
e_{i,k}=b_{i,k}+\epsilon_{i,k},\qquad \epsilon_{i,k}\sim N(0,\sigma_i^2)
\]

慢变项按实际样本时间差更新：

\[
\phi_i=e^{-\Delta t/\tau_i},\qquad
b_{i,k}=\phi_i b_{i,k-1}+\sqrt{P_i(1-\phi_i^2)}\,\eta_{i,k},\quad \eta_{i,k}\sim N(0,1)
\]

首个慢变项从稳态分布 `N(0,P_i)` 抽样。对独立的慢变项和白噪声，单历元的边缘方差为：

\[
R=\operatorname{diag}(P_E+\sigma_E^2,\ P_N+\sigma_N^2,\ P_U+\sigma_U^2)
\]

该公式是本实现对模型的统计推导，不是从真实误差反算每一帧的可信度。它也不消除时间相关性：滤波器若把多历元测量当作独立数据，仍可能低估不确定度。

代码直接从生成误差所用的 P 和 σ 计算 R，不再为此预设手动填写独立的 `1、1、5 m²` 权重。`model_marginal` 模式禁止同时启用原生噪声或经验遮挡放大；自定义文件中该模式的报告方差会按生成参数重算。修改文献预设的参数后，来源记录会标记 `user_modified_from_published_fit` 并记录 `parameter_overrides`，不继续称其为原始文献拟合值。

正常与欺骗位置分别为：

\[
p_k^{正常}=p_k^{原生无噪声}+e_k,\qquad
p_k^{欺骗}=p_k^{正常}+a_k
\]

欺骗量 a 单独记录，不自动增加 R。这样不会把攻击大小作为飞控或组合算法已知的信息；本预设没有使用真值误差“泄露”攻击强度。

## PX4、ROS 与组合算法收到什么

```mermaid
flowchart LR
    A[原生 GPS：噪声关闭，延迟保留] --> B[同一次位置误差抽样及欺骗]
    B --> C[PX4 GPS 输入]
    B --> D[ROS 位置与 ENU 方差]
    D --> E[启用时的 FAST_GLIO 松组合]
```

- `/msim/normal_gnss_odom`：同噪声、无欺骗的旁路位置。
- `/msim/spoofed_gnss_odom`：分发给松组合的最终 GPS 测量；不是 PX4 EKF 的位置输出。
- ROS 保留三个方向的方差；PX4 接口只有标量 EPH/EPV，本实现映射为 `EPH=sqrt(max(R_E,R_N))≈0.28443 m`、`EPV=sqrt(R_U)≈0.42474 m`。前者是本接口选定的水平每轴保守上界，不是水平圆误差半径。
- MAVLink 中还会有精度量化，因此 PX4 日志常见约 `0.28 / 0.42 m`；不要求与 ROS 浮点值逐位相等。
- `--teleop` 目前是 `mapping_only=true`：GPS 被记录并供 PX4 使用，FAST_GLIO 只做 LiDAR/IMU 建图。自动航线加 `--fastglio-live` 时才启用这里的 GPS 松组合。
- FAST_GLIO 的位置测量回调会读取 ROS 协方差，但它自己的有效性检查和更新门限独立于 PX4。当前不把 FAST_GLIO 结果反馈给 PX4。

真实接收机也可逐历元报告位置标准差，例如 NovAtel 的 BESTPOS/GPGST。[2][3] 这两份资料用于说明报告量的含义，不是上述 F9P 参数的来源。

## 旧遮挡公式的标记与复现

`--gps-model standalone` 保留原先参数与行为，明确标为 **未标定经验模型**：

\[
B=\frac{\text{被环境挡住的射线数}}{36},\quad
s=1+3B,\quad EPV=\sqrt5\,s
\]

它来自本项目对遮挡与误差的经验关联，**不是 PX4 公式、不是厂商公式，也没有实测拟合依据**。`1、1、5 m²`、增益 3、相关偏差 `0.7/2 m` 均属历史实验设定。它不逐卫星计算信噪比、NLOS、多路径或 DOP，也不是论文 [1] 的环境模型。

旧预设的原生噪声仍保留，因此不能把其报告方差解释成严格匹配全部误差的统计协方差。新基线不启用这套放大规律；建筑仍有真实碰撞和 LiDAR 观测，关闭 GPS 经验放大不表示环境变成空场景。

## 运行与核对

下一次手动采集明确选择新预设：

```bash
cd /home/wzb/msim
./tools/px4-uav-capture.sh --web --teleop --scene ksql_airport \
  --teleop-height-m 12 --lidar-model vlp16 --point-filter-num 2 \
  --gps-model f9p-dgnss-ucr
```

运行 GPS 松组合对照可用：

```bash
./tools/px4-uav-capture.sh --duration-s 60 --fastglio-live \
  --scene simple --flight-pattern route --route-clock sim --skip-signalsim \
  --gps-model f9p-dgnss-ucr
```

加 `--spoof-start-s 20 --spoof-east-rate-mps 0.18` 可注入欺骗；这不会修改上述正常随机误差参数。新预设不改变 PX4 的质量门限或失效保护，也不提供避障功能。

生成可编辑配置而不启动仿真：

```bash
python3 tools/sensor-config.py --gps-model f9p-dgnss-ucr \
  --output config/f9p-dgnss-ucr.json
```

核对每次运行保存的 `sensor_profile.json` 中 `models.gps`、`gps_reference` 与 `gps.reported_variance_enu_m2`。日志 `/msim/gps_samples` 额外包含 `position_error_model_id`、`environment_model`、`reported_variance_mode` 和三轴方差。原始来源、表值、单位、适用限制及 PDF SHA-256 存在 [机器可读来源记录](../../adapters/ros1_noetic/msim_sensors/config/gps_references/f9p_dgnss_ucr_2023.json)。

从旧录包网页复制的命令会保留旧录包的 GPS 型号；首次切换时应显式使用上述 `--gps-model`。旧录包不会自动变成新模型。

## 验证

使用与 Gazebo 插件相同的 C++ 随机模型内核，检查稳态初值、生成方差及时间相关性；分别按 0.2 s、1 s 间隔生成 700,000 个样本。测试不使用真实飞行的短记录方差来代替长期模型验证。

另进行 30 秒 PX4/Gazebo/FAST_GLIO 无界面无欺骗航线实测，结果目录为 `/media/wzb/Elements/msim/runs/uav-px4-20261123-171012`：

| 检查 | 结果 |
| --- | --- |
| 飞行检查 | `pass_flight_gate=true`，最大爬升 8.19 m，最大位移 12.58 m |
| LiDAR 同步 | `pass_lidar_sync_gate=true`，108/108 个历元匹配 |
| GPS 松组合链路 | `pass_live_loose_gate=true` |
| GPS 日志 | 112 个历元，标记为 `f9p-dgnss-ucr`、环境倍率 1 |
| ROS 两路一致性 | 105 对历元共享同一噪声，三轴方差与生成模型一致 |
| PX4 精度报告 | 34 个 GPS 样本符合 EPH/EPV 映射及 MAVLink 量化容差 |
| 接收机检查报告 | `gnss-receiver-verification.json`：`pass_all=true`、`errors=[]` |

统计内核验证见 [JSON 记录](validation/f9p-position-error-20261123.json)。录包链路可重复核对：

```bash
export PATH=/home/wzb/FAST_GLIO/.pixi/envs/default/bin:$PATH
source /home/wzb/FAST_GLIO/devel/setup.bash
python tools/verify-gps-receiver.py --run /media/wzb/Elements/msim/runs/uav-px4-20261123-171012
```

短程飞行通过说明接入与数据链路正常，不表示完成了真实接收机标定、长航时精度验证或复杂建筑环境验证。

本次最终检查：61 项 Python 测试全部通过，两个 Node 输入/命令测试通过，Gazebo 插件编译成功，Python/JavaScript/shell 语法与差异空白检查通过。

## 参考资料

1. Farrell, J. A.; Wu, G.; Hu, W.; Oswald, D.; Hao, P. **Lane-Level Localization and Map Matching for Advanced Connected and Automated Vehicle (CAV) Applications**. NCST-UCR-RR-23-13, 2023. [DOI: 10.7922/G25T3HSS](https://doi.org/10.7922/G25T3HSS)，[美国交通部 PDF](https://rosap.ntl.bts.gov/view/dot/67203/dot_67203_DS1.pdf)，[大学存档 PDF](https://escholarship.org/content/qt1f7661b4/qt1f7661b4.pdf)。采用附录 B §4 Table 1；两份 PDF 封面数量不同，分别在 PDF 第 57 / 59 页。
2. NovAtel. [OEM7 BESTPOS：位置解与纬度、经度、高程标准差](https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm)。
3. NovAtel. [OEM7 GPGST：位置解误差统计与单位](https://docs.novatel.com/OEM7/Content/Logs/GPGST.htm)。
4. 报告配套数据：[DOI: 10.6086/D11M43](https://doi.org/10.6086/D11M43)，[Dryad 数据页面](https://datadryad.org/dataset/doi%3A10.6086/D11M43)。本次采用报告发表的拟合参数，未重新标定这批原始数据；报告中的旧 GitHub 代码地址在本次检查时返回 404。

访问日期：2026-11-23。
