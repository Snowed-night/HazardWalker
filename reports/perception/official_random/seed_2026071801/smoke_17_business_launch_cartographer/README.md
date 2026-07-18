# Smoke 17：正式业务 Launch 验收

## 目的

验证不是临时脚本，而是仓库正式 `official_simenv_business.launch.py` 能在官方 SimEnv 公开输入下启动 Cartographer 链路。

## 启动范围

- `start_slam:=true`
- `slam_backend:=cartographer`
- `start_legal_localization:=true`
- 感知、决策、导航和证据记录本轮关闭，避免混入非 SLAM 变量。
- 使用独立 `ROS_DOMAIN_ID=43`，避免其他组遗留节点污染图。

## 验收结果

- 节点：适配器、合法 scan/IMU 定位、深度转扫描、Cartographer、占据栅格节点均存在。
- Cartographer 日志确认 10 Hz 水平雷达、约 6～8 Hz 深度扫描、10 Hz 合法里程计和约 250 Hz IMU 均持续到达。
- `/map` 发布者数量严格为 1。
- `map_saver_cli` 成功保存 611×714、0.05 m/像素地图。
- 正式入口定位上游配置目录的错误已经在本轮前修复：从已注册的 `cartographer_ros` 前缀定位 `share/cartographer/configuration_files`。

## 结论

“目前无法 SLAM”已经被缩小为“旧入口/旧 backend 不可用”：当前正式 Cartographer 入口能够建图并落盘。但短时静止地图不足以证明多层未知楼宇全覆盖和回环精度，仍需后续长时探索闭环验证。
