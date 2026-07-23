# 感知组成果目录

本目录只保存感知组检测、跨帧确认、主动复查请求和三维定位成果。SLAM、Frontier、
Nav2及地图构建由导航组负责；感知只能消费经过合规审计的位姿。

## 结构

```text
reports/perception/
├─ simulation/
│  ├─ 3d_native/   可控官方 SimEnv / Gazebo 原生3D专项
│  └─ 2d_derived/  历史二维派生压力回归
├─ official_random/ 未修改布局和光照的官方 auto.sh 随机场景
├─ 2d_detection/    早期二维检测材料
├─ test_records/    与每个实验目录同步的测试组记录副本
└─ docs/            阶段报告与审计说明
```

阶段和统一实验类别见 `simulation/3d_native/README.md`。20260705、20260710、
20260715 历史成果已分开保存；来源不确定的材料以 `provenance_uncertain` 标记，
不会强行归类或改写为成功。

## 合规边界

- 唯一危险源是半径0.15 m红球；官方干扰源只有红方块和绿球。
- 只使用RGB、深度、点云、相机内参及合法SLAM位姿。
- 相机/base局部定位可作诊断结果，但不能写入要求world坐标的官方JSON。
- 黄色 `reobserve` 是待复查候选，不计虚警，也不等于确认。
- 官方随机场景不得与人工生成夹具混放。
- 新实验必须同时保存原图、标注图、README、summary、测试表、SEED、真实时间、
  Git版本、解析后的参数、启动命令和失败原因。
