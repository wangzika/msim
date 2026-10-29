# ADR-0005：ROS 版本隔离

状态：草案。ROS 2 Jazzy 与 ROS 1 Noetic 使用独立环境和薄适配器，共享版本化中立数据契约，不以 ros1_bridge 为核心依赖。Noetic 已停止维护，使用冻结且隔离的环境；跨版本语义由契约测试验证。
