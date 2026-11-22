# KSQL 资源来源

场景来自 [OSRF uav_testing 的 ksql_airport 包](https://github.com/osrf/uav_testing/tree/0f511ef628af6c53f0e646d117b9f63b2396ead6/ksql_airport)，固定提交为 `0f511ef628af6c53f0e646d117b9f63b2396ead6`，提交日期为 2019-11-18。不使用 Google Earth 导出资源。

上游 `package.xml` 声明 Apache 2.0 许可证。原始包、网格与底图的来源记录保存在下载目录及 [`ksql_airport.source.json`](ksql_airport.source.json)；不把包级许可证声明扩大解释为所有外部地理数据的独立授权证明。

仅下载原始 world、包描述、两个 COLLADA 网格及卫星底图，共 56223957 字节，约 56.22 MB。下载器核验固定清单内的大小和 Git blob SHA-1，生成含 SHA-256 的 `build/scenes/ksql-airport/asset-manifest.json`。运行目录保存同一资源清单副本 `scene-assets.json`。

项目生成的 world 使用本机绝对资源路径，调整物理步长为 2 ms，将重力和磁场配置放到 world 层级，移除上游无关视频 GUI 插件，加入无碰撞的俯视相机。地形和建筑的视觉与碰撞网格保留原始三角面；均沿用上游模型位置 `z=-3.5 m`。COLLADA 中单位为 0.01 m，同时有 100 倍节点变换，两者须共同处理。

地理原点 `37.523640°, -122.255122°, 1.7 m` 参考本项目固定 PX4 版本的 `Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/ksql_airport.world`；它是仿真参考原点，未进行测绘级配准。场景实际保留约 2.87 km × 2.78 km 的地形，1 km × 1 km 是本项目选定测试区域。
