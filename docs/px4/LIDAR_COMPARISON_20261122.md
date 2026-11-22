# 四种 LiDAR 的 ENU 误差对比（2026-11-22）

**已有修正后的完整在线重跑：** 四种型号均完成新采集并通过全部检查，当前结果请使用[地图参数修正后的四雷达在线重跑](LIDAR_COMPARISON_FIXED_20261122.md)。下面保留问题修正前的历史记录。

分别重新采集 VLP-16、HDL-32E、OS1-64、Mid-360 各 60 秒的无欺骗闭环飞行，在各自真值下评价纯 LiDAR/IMU 与 LiDAR/IMU/GPS 松组合。四组飞行、GPS 接入、LiDAR 时间配对和在线组合检查均通过。

**后续诊断发现配置问题：** 本次四组启动配置的地图边长参数都没有被 FAST-LIO 正确读取，算法地图管理范围参数均为 100 m，触发异常局部地图裁剪。VLP-16、OS1-64 的同录包单变量回放已确认，修正后东向误差显著减小；HDL-32E、Mid-360 尚未做同录包单变量对照，但四组均已有修正后的新在线重跑，见开头链接。下表保留原始实验记录，不能据此比较雷达型号的固有性能。详见[东向误差诊断与修正](EAST_ERROR_DIAGNOSIS_20261122.md)。

## 对照设置

场景为同一份城市走廊 world；航线、仿真时钟、MEMS IMU 误差、standalone GPS 误差、随机种子、算法过程与测量噪声设置相同。全部使用分时扫描、10 Hz、0.02 m 径向噪声、0.001 m 距离量化，`point_filter_num=1` 保留全部有效输入。算法内仍执行原有体素降采样，传感器安装与外参保持一致。LiDAR 处理缓冲统一为 0.65 s。

| 雷达预设 | 点率上限 | 垂直视场 | 量程 | 前端入口 |
| --- | ---: | --- | --- | --- |
| VLP-16 | 288,000 点/s | −15°～15° | 0.3～100 m | Velodyne |
| HDL-32E | 576,000 点/s | −30.67°～10.67° | 0.3～100 m | Velodyne |
| OS1-64 | 655,360 点/s | −22.5°～22.5° | 0.3～100 m | Velodyne |
| Mid-360 | 200,000 点/s | 名义 −7°～52° | 0.1～40 m | Livox |

这是整套仿真预设的对比，点率、量程、扫描方式和入口格式保留型号差异，不是仅改变线数的实验，也不是实物雷达性能排名。四次独立飞行的初始化与真实运动可能存在差异；单次实验不提供重复试验的统计结论。

## 计算方法

真值位置按估计消息的采样时刻线性插值，不在真值范围外外推，拒绝超过 50 ms 的真值间隙。每条轨迹只减去首个有效匹配点的平移差，不拟合旋转或尺度，不平滑估计曲线。共同评价区间为相对各飞行首个原生 GPS 采样时刻的 **2.0～62.5 s**，每条曲线 605 个样本。

原始 bag 各自的覆盖范围略有差异，因此下表严格使用绘图共同区间；单次运行目录中按全部有效数据计算的指标可能略有不同。三维 RMSE 定义为 `sqrt(mean(E²+N²+U²))`。

## 纯 LiDAR/IMU

| 雷达 | 东向 RMSE / m | 北向 RMSE / m | 天向 RMSE / m | 三维 RMSE / m |
| --- | ---: | ---: | ---: | ---: |
| VLP-16 | 1.275 | 0.209 | 0.474 | 1.376 |
| HDL-32E | 0.087 | 0.074 | 0.299 | 0.320 |
| OS1-64 | 0.670 | 0.060 | 0.218 | 0.707 |
| Mid-360 | 0.125 | 0.188 | 1.106 | 1.129 |

[ENU 时序图](../../runs/lidar-comparison-20261122-01/figures/lidar-imu-enu.png)

## LiDAR/IMU/GPS 松组合

| 雷达 | 东向 RMSE / m | 北向 RMSE / m | 天向 RMSE / m | 三维 RMSE / m |
| --- | ---: | ---: | ---: | ---: |
| VLP-16 | 0.624 | 0.733 | 2.651 | 2.820 |
| HDL-32E | 0.542 | 0.725 | 2.649 | 2.800 |
| OS1-64 | 0.462 | 0.903 | 2.475 | 2.675 |
| Mid-360 | 0.666 | 1.134 | 2.114 | 2.490 |

[ENU 时序图](../../runs/lidar-comparison-20261122-01/figures/loose-enu.png)

本次组合仍明显跟随带噪 GPS，不能预设组合后一定优于纯 LIO；GPS 权重和融合策略需要单独评价。Mid-360 的天向误差大于其他三种预设，与其当前向下视场和量程下地面覆盖不足的风险一致，但本实验没有单变量改变地面观测，不能据此确定因果。

## 数据与复现

结果根目录：`runs/lidar-comparison-20261122-01/`。每个型号子目录保留原始 bag、有效传感器配置、world／vehicle、算法配置、插件摘要和飞行／组合报告。

- [逐样本 ENU 误差 CSV](../../runs/lidar-comparison-20261122-01/figures/enu-errors.csv)
- [完整指标 CSV](../../runs/lidar-comparison-20261122-01/figures/metrics.csv)：另含水平 RMSE、三维 P95、最大和末端误差。
- [来源、参数及门禁 JSON](../../runs/lidar-comparison-20261122-01/figures/summary.json)
- [纯 LIO 对称对数纵轴版本](../../runs/lidar-comparison-20261122-01/figures/lidar-imu-enu-symlog.png)
- [松组合对称对数纵轴版本](../../runs/lidar-comparison-20261122-01/figures/loose-enu-symlog.png)

对称对数版本仅改变显示尺度，±0.2 m 内为线性，数据和指标与线性版本一致。

重新运行并绘图：

```bash
cd /home/wzb/msim
./tools/run-lidar-comparison.sh /home/wzb/msim/runs/lidar-comparison-new 60
PATH=/home/wzb/FAST_GLIO/.pixi/envs/default/bin:$PATH \
  MPLCONFIGDIR=/home/wzb/msim/runs/lidar-comparison-new/matplotlib-cache \
  python tools/plot-lidar-comparison.py --group /home/wzb/msim/runs/lidar-comparison-new
```
