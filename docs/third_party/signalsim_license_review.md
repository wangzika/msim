# SignalSim 许可证核查

状态：可发布授权仍未确认。来源：[上游仓库](https://github.com/globsky/SignalSim)。上游 README 对部分功能列出商业许可；现有材料不足以确认仓库其他部分的授权范围。需核对固定提交的文件清单及版权条款，并取得明确许可结论。当前仅在本机研究环境从固定提交构建适配器，没有把上游源码复制进 msim 仓库；S0–S8 许可审查尚未通过。

2026-10-29：通过 `git ls-remote` 确认 `main` 当时指向 `baba9deb68ed193efb9fd9a362b431ae8c491fa4`，仅作为核查快照而非启用授权；根目录树未发现 LICENSE。该提交 README 第 34、241–242 行提及商业许可。仍需人工确认授权。

固定提交 `README.md` SHA-256：`e256da9d2bc8835c10c739a6b3b24025a3325fdd837c5d4f674b612c8f4a1c19`。

2026-11-04 技术验证更新：本机 `build/deps/SignalSim` 固定到上述提交；`tools/build-signalsim-adapter.sh` 编译 `adapters/signalsim/observables.cpp` 与上游源码。FAST_GLIO 无人机轨迹已生成 300 s 的双站四星座 RINEX、ROS 1 合包和 RTKLIB 结果，输入与产物哈希保存在运行 manifest 中，具体见 [SignalSim 集成指南](../gnss/SIGNALSIM-FASTGLIO.md)。技术可运行性不构成许可结论。
