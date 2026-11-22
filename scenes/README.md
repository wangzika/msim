# Gazebo Classic 场景

本目录是在线 LiDAR / PX4 / FASTGLIO 的几何场景，与 `scenarios/` 中 SignalSim 的 GNSS 传播配置分开。

| 选择项 | 内容 | 来源 |
| --- | --- | --- |
| `simple` | 5 个箱体组成的原始城市走廊 | 本机 FAST_GLIO 的 `urban_canyon.world` |
| `complex_urban` | 39 个有碰撞几何的物体，包含错落建筑、柱体、斜墙和高架桥 | 本项目生成 |
| `industrial_yard` | 33 个新增几何物体，以及 38 个静态货架、托盘、货箱模型实例 | 本机 PX4 `warehouse.world` 与其模型目录 |
| `ksql_airport` | 带卫星底图、地形与建筑碰撞网格的 San Carlos 机场周边；完整地形约 2.87 km × 2.78 km，选定 1 km × 1 km 测试区域 | 固定版本的 OSRF `uav_testing/ksql_airport` |

工业场地保留货架碰撞网格；普通欧式托盘保留外观网格，碰撞简化为包围盒，以控制物理计算负载。组合托盘内部还包含嵌套模型，38 是顶层导入实例数，不是总网格或碰撞形状数。相关模型的授权与来源沿用 PX4 模型目录中的声明，本仓库不复制第三方网格资源。

## KSQL 大范围场景（2026-11-22）

首次下载约 56 MB 的固定版本资源：

```bash
cd /home/wzb/msim
python3 tools/fetch-ksql-assets.py
```

下载文件存入 `build/scenes/ksql-airport/`，逐文件检查大小与 Git blob SHA-1，另记录 SHA-256。已有资源无需重复下载；启动时会校验资源并重新生成本机路径的 world，不依赖 Gazebo 在线模型库。

运行 Gazebo 与 RViz，循环执行每圈约 784 米的闭合航线：

```bash
./tools/px4-uav-capture.sh --scene ksql_airport --duration-s 300 \
  --fastglio-live --lidar-model vlp16 --point-filter-num 1 \
  --flight-pattern route --route-clock sim --skip-signalsim
```

此命令无欺骗；需要在线欺骗时追加 `--spoof-start-s 20 --spoof-east-rate-mps 0.18`。首次观察可先把时长改为 `60`，这只会飞行前几个航段；300 秒测试覆盖一圈并开始下一圈。加 `--headless` 可后台采集；运行结束会关闭 Gazebo 并生成图表。运行秒数按仿真时间计，实际耗时受电脑负载影响。

场景设置集中在 [`ksql_airport.profile.json`](ksql_airport.profile.json)：地理原点、飞机出生点、相对起飞高度、观察视角、测试区域、资源路径和航线文件。用 `--scene-profile /绝对路径/配置.json` 选择自定义配置；其中相对文件路径以配置文件所在目录为基准。航点在 [`ksql_airport.route.json`](ksql_airport.route.json)，东、北坐标相对起飞点，高度目标为起飞后上升 12 m。完整模型保留，1 km × 1 km 只是选定测试范围，**未裁剪原始网格**；当前航线沿建筑群西侧飞行，未遍历整个区域。

出生点 ENU `(-210,240,-1.35)` 下方地面为 `-1.5 m`，负高度是模型坐标定义。地理原点采用 PX4 Classic KSQL 参考值，尚未做地理配准实测。修改出生点、航线或高度后，启动前的独立网格检查会筛查路径及左右、前后 3 m 的地形/屋顶高度；这是离散采样检查，不能保证 GPS 噪声、欺骗或飞控切弯时绝不碰撞。

每次运行保存 `scene-profile.json`、`scene-assets.json`、`world.sdf`、`vehicle.sdf`、`route_plan.json` 与 `gazebo-scene-overview.png`，可追溯场景与资源。详细接入方式、实测结果与限制见 [KSQL 接入报告](../docs/px4/KSQL_INTEGRATION_20261122.md)，资源来源见 [来源说明](KSQL_SOURCE.md)。

## 生成与运行

先完成 PX4 / FAST_GLIO 环境准备，再生成三个 world：

```bash
cd /home/wzb/msim
python3 tools/prepare-complex-scenes.py
```

运行复杂街区，左侧 Gazebo、右侧 RViz，20 秒后加入 GPS 东向漂移：

```bash
./tools/px4-uav-capture.sh --scene complex_urban --duration-s 60 \
  --fastglio-live --fastglio-frontend velodyne --point-filter-num 4 \
  --flight-pattern route --route-clock sim --skip-signalsim \
  --spoof-start-s 20 --spoof-east-rate-mps 0.18
```

把 `complex_urban` 换成 `industrial_yard` 即可换场景。省略两项 `--spoof-*` 参数可运行无欺骗组；加 `--headless` 仅在后台采集。`--world /绝对路径/自定义.world` 可以覆盖场景文件，自定义 world 的坐标与出生点须与航线相容。

`--route-clock sim` 使用 PX4 启动时间推进航点停留时间，避免复杂场景降低实时因子后，航线仍按墙上时钟过快推进。飞机出生点为世界 ENU `(0,-20,0)`，航线目标高度 8 m。新增场景在 E∈[-10,4]、N∈[-23,16]、U∈[0,9.5] 留有通道；原始走廊较窄，保守检查 E∈[-8,4]。这是预留空间说明，不替代实际飞行与网格碰撞检查。

完整六组测试与绘图：

```bash
bash tools/run-scene-comparison.sh /home/wzb/msim/runs/scene-comparison-new 60
PATH=/home/wzb/FAST_GLIO/.pixi/envs/default/bin:$PATH \
  python tools/analyze-scene-comparison.py --group runs/scene-comparison-new
```

脚本保留各组原始 bag、world、机体模型、传感器配置、算法有效参数、飞行验收、融合验收和 Gazebo 实际截图。已有报告的组会跳过；重新实验应使用新目录。首次手工调整场景后需要重新生成 world。

## 测试边界

增加物体只有在激光射线实际照到相应表面时才会增加约束。低矮货架在 8 m 高度和 ±15° 垂直视场下未必被有效扫描，因此场景外观复杂不保证定位误差更小。

当前新采集使用原生 Gazebo GPS 加位置误差和在线欺骗偏移；新增建筑不会自动产生 GNSS 遮挡、多路径或真实射频欺骗。`--skip-signalsim` 明确跳过离线 GNSS 观测生成。当前默认点云由独立扫描插件按物理时间分批求交，Velodyne 入口保留逐点采样时间，10 Hz 一帧内跨度约 98 ms；`--lidar-timing snapshot` 才是快照模式。可切换型号及噪声见 [传感器切换说明](../docs/px4/SENSOR_SWITCHING.md)。

2026-11-16 的旧六组实测、ENU 误差、三维轨迹和当时的同步限制见 [复杂场景实验报告](../docs/px4/COMPLEX_SCENES_20261116.md)，其中快照扫描和理想 GPS 描述仅适用于历史录包。
