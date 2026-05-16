# System Architecture

HazardWalker 面向未知楼栋仿真环境中的四足机器人危险源自主搜索与识别任务。

初始架构：

```text
Simulation / Official Platform
        |
hazardwalker_platform
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
