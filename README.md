# msim

多传感器仿真平台的早期基础工程。当前仅有独立 C++17 核心的构建和测试骨架，尚不能生成仿真数据。开发状态见 [P0-01](docs/status/P0-01.md)。

## 本机构建和测试

需要 CMake 3.16 或更新版本和 C++17 编译器：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

格式与静态检查分别运行 `tools/ci/check-format.sh` 和 `tools/ci/check-tidy.sh`，需要安装 clang-format 和 clang-tidy。缺少工具时检查会失败，不会静默跳过。

实施方案中的 GNSS、RINEX、RTKLIB、Gazebo、ROS 和融合能力尚未实现或验证；SignalSim 集成、RINEX 兼容性和正确的 RTK Fixed 均待实测。

## Phase 0 进度

[任务状态](docs/status/) 和 [架构决策草案](docs/architecture/) 记录了实际验证及未完成门禁。`tools/ci/phase0.sh` 是本地容器检查入口；两个 ROS 消息包目前仅为不含消息定义的构建占位包。核心容器使用 Ubuntu 固定时间快照并校验包锁；Jazzy/Harmonic 的软件源快照仍待固定。

`tools/validation/create_manifest.py --scenario <文件> --seed <整数> --output <新文件>` 可创建初步来源记录，要求当前仓库有 Git commit；它尚不是完整 run manifest。SignalSim 默认禁用，其许可证仍待人工确认。

初步来源记录还支持 `--resolved-scenario`、可重复的 `--artifact` 和 `--dependency-lock` 文件，以及可重复的 `--image-digest name@sha256:<64位摘要>`。文件内容按 SHA-256 记录；这些参数由调用者提供，工具不会自动发现所有运行输入或保证清单完整。
