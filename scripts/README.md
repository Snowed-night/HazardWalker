# Scripts

本目录存放项目运行、构建和结果检查脚本。

## 当前文件

- `build.sh`：进入 `ros2_ws` 后执行 `colcon build --symlink-install`，用于构建 ROS 2 工作空间。
- `setup_env.sh`：主力机环境安装占位脚本，目前只说明目标环境，后续再补自动化安装步骤。
- `run_minimal_demo.sh`：启动最小 demo，自动设置 `HAZARDWALKER_ROOT`，加载 ROS 环境和工作空间后运行 `minimal_demo.launch.py`。
- `run_offline_tests.py`：不依赖 pytest 的离线测试入口，扫描 `tests/offline/` 并执行测试函数。
- `evaluate_result.py`：检查 `reports/run_results/<timestamp>_result.json` 的结构和统计字段。

## 约定

- 新增脚本先放这里，再根据用途拆分子目录。
- 脚本名称应直接表达用途，不保留已经删除或不存在的入口名。
- 如果新增仿真启动、录包或批处理脚本，README 需要同步更新。
