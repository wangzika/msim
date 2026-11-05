# RTKLIB 来源

本机相对定位使用 `/home/wzb/FAST_GLIO/rtklib` 的 `demo5 b34c` 版本。锁定的源码提交和 Pixi 文件哈希见 `third_party/manifest.lock`，已构建二进制及读取探针的 SHA-256 见 `docs/rtklib/build-manifest.json`。构建入口为 `tools/rtklib/build_rtklib.py`，解算入口为 `tools/rtklib/rtklib_runner.py`。

上游授权范围尚未完成独立审查。本仓库不复制 RTKLIB 源码，构建脚本从指定 FAST_GLIO 提交创建本机快照。
