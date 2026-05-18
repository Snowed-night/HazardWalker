# System Architecture

HazardWalker 面向未知楼栋仿真环境中的四足机器人危险源自主搜索与识别任务。

初始架构：

```text
Simulation / Official Platform
        |
hazardwalker_platform
        |
HazardWalker Internal Interfaces
        |
SLAM / Localization
        |
hazardwalker_nav <----> hazardwalker_decision <----> hazardwalker_perception
        |
hazardwalker_bringup
        |
Results / Metrics / Reports
```

后续所有模块接口变更需要同步更新 `docs/interface_spec.md`。

当前阶段采用“平台适配层吸收官方环境差异，算法模块只面向内部接口”的设计：

- 官方平台、自建 Gazebo、Isaac Sim 或后续实机环境都应先通过 `hazardwalker_platform` 转换为 `/hw/*` 内部话题。
- 导航、感知、决策模块不直接依赖官方平台专有话题名或 SDK。
- 第一阶段优先完成固定航点最小闭环，验收标准见 `docs/minimal_demo_acceptance.md`。
- 官方平台发布后的接入流程见 `docs/official_platform_adaptation_design.md`。
