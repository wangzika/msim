# Phase 0 容器基线

基础镜像以 Dockerfile 中的 registry digest 固定；构建需要 Docker 和网络。运行 `tools/ci/phase0.sh` 进行当前构建与测试。测试容器禁用网络，ROS 1 需要继续保持隔离。

core Dockerfile 使用 Ubuntu 快照 `20261029T040000Z`，并校验完整的包版本清单；快照服务长期保留策略仍需关注。ROS 2 镜像包含 Gazebo Sim 8，但传递依赖尚未固定；完整镜像尚未发布为可拉取摘要。ROS 包为空占位包，不证明 ROS 消息或跨版本契约可用。

三个 `*.packages.lock` 清单记录本次镜像的所有已安装 Debian 包和版本；构建会比较实际清单并在漂移时失败。该检查不保证仓库永久保留这些版本，ROS 软件源快照和镜像发布仍需完成。
