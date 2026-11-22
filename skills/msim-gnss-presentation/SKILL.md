---
name: msim-gnss-presentation
description: Create or update Chinese academic presentations about the ROS 1 FAST_GLIO UAV, SignalSim GNSS, Gazebo and RTKLIB pipeline. Includes a selectable concise group-meeting style.
metadata:
  short-description: MSIM 仿真平台学术汇报
---

# MSIM 仿真平台学术汇报

为 FAST_GLIO 无人机多传感器仿真、SignalSim GNSS 观测和 RTKLIB 主题制作或更新演示文稿。根据当前仓库文档陈述项目状态，保留“已实现”“已验证”“计划中”和“未评估”的边界。

## 工作方式

1. 先确认用户要求的页数、听众、用途和交付形式。用户未说明视觉样式时，沿用随 skill 提供的可编辑参考演示文稿，并选用常规学术汇报样式。
2. 用户提到“组会”“简洁”“大字”时，切换到[组会简洁大字样式](references/group-meeting-style.md)。如果用户只说“给一个选项”，在开始制作前说明两种样式并默认常规样式；若上下文已明确组会场景，直接使用组会样式。
3. 使用最新项目事实前，检查仓库中的 `README.md`、`docs/gnss/SIGNALSIM-FASTGLIO.md` 和运行结果的 `verification-signalsim.json`、RTKLIB `summary.json`。以下项目内容摘要供快速定位，不能覆盖更新后的状态文档。
4. 按用户要求的页数和顺序安排内容。框架、数据链路、模块关系优先用可编辑示意图；不要把所有内容塞进卡片网格。
5. 清楚区分“模块存在”和“模块已经端到端接通”。当前链路以离线轨迹驱动 SignalSim，RTKLIB 离线解算；勿画成实时 GNSS 反馈控制闭环。
6. 复用参考演示文稿的校徽、标题栏和蓝色学术汇报基调时保持原图比例。外部项目事实在相关页注明仓库或文档来源，演讲备注中可补充完整引用。
7. 按目标平台渲染并逐页检查标题、字号、中文字体、文本溢出、连线和图文关系；修正后重新渲染受影响页面。

## 项目参考

- [当前项目内容与事实边界](references/project-content.md)：架构、阶段状态、GNSS 生成链、SignalSim 输入和集成位置。
- [组会简洁大字样式](references/group-meeting-style.md)：配色、字体层级、页面密度和图示规则。
- [参考演示文稿](assets/msim-framework-signalsim-reference.pptx)：仅作视觉版式参考；其旧架构和阶段内容需要按当前 ROS 1 主线重写。

## 叙事建议

- 平台介绍：ROS 1 无人机架构 → 采集与 GNSS 数据链 → 已验收结果 → 模型局限。
- GNSS 观测：时间/轨迹/星历/信号配置 → 卫星状态和测量模型 → 观测与 RINEX → RTKLIB 验收。
- SignalSim：轨迹和星历输入 → 四星座观测、RINEX 和合包 → RTKLIB 结果 → 城市反射模型与授权边界。

保持短句和具体名词。RINEX、SPP、Float、Fixed、C/N₀、观测字段、运行时钟等术语首次出现时按听众背景解释。
