# HazardWalker 主力机环境介绍

更新时间：2026-06-16  
适用对象：HazardWalker 全队成员  
主力机：`hxbl`

> 本文用于统一说明主力机的系统环境、账号规划、目录结构、开发流程和使用规则。连接方式请见《HazardWalker SSH连接指南.md》。

## 1. 环境概览

主力机 `hxbl` 用于 HazardWalker 项目的集成、仿真、离线测试和需要较高算力的实验。各组使用独立 Linux 账号和独立代码目录开发，系统级依赖由环境负责人统一维护。

当前已具备：

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic / Gazebo Sim
- Nav2、SLAM Toolbox、Gazebo bridge 等机器人开发依赖
- Python 3、pip、venv、colcon、rosdep
- NVIDIA 驱动与 RTX A6000 GPU
- 各组独立 `~/HazardWalker` 仓库和 `ros2_ws` 工作空间
- 共享大文件目录，用于数据集、模型、rosbag、视频等
- zsh + oh-my-zsh 终端环境
- PyTorch CUDA 环境，主要供感知组和算法实验使用

无图形界面需求时优先使用 SSH；需要 RViz、Gazebo GUI 时再使用远程桌面。

## 2. 主机规格

```text
主机名：hxbl
系统：Ubuntu 24.04.x LTS
内核：6.17.0-35-generic
CPU：Intel Core i9-13900K，24 核 / 32 线程
内存：约 94 GiB 可见内存，Swap 8 GiB
GPU：NVIDIA RTX A6000，约 48 GB 显存
NVIDIA Driver：535.309.01
CUDA runtime 显示版本：12.2
ROS：ROS 2 Jazzy
Gazebo：Gazebo Harmonic / Gazebo Sim 8.x
Python：Python 3.12
Shell：zsh
```

常用命令路径：

```text
ros2：/opt/ros/jazzy/bin/ros2
colcon：/usr/bin/colcon
rosdep：/usr/bin/rosdep
gz：/usr/bin/gz
python3：/usr/bin/python3
pip3：/usr/bin/pip3
```

## 3. 账号与 ROS_DOMAIN_ID

各组使用自己的 Linux 账号登录，不要共用他人账号。账号初始凭据以队内最新通知为准，建议尽快配置个人 SSH 公钥。

| 组别 | Linux 账号 | ROS_DOMAIN_ID |
| --- | --- | --- |
| 平台组 | `hazard_platform` | 11 |
| 导航组 | `hazard_nav` | 12 |
| 感知组 | `hazard_perception` | 13 |
| 决策组 | `hazard_decision` | 14 |
| 测试组 | `hazard_test` | 15 |
| 文档组 | `hazard_docs` | 无 |

权限原则：

- 普通组账号默认不使用 `sudo`
- 系统级依赖由 `hazard_admin` 或环境负责人统一维护
- Docker 权限需单独申请
- GPU 使用需提前报备，集成测试优先
- 各组只在自己的 home 目录下开发

## 4. 目录结构

登录后常用目录：

```text
~/HazardWalker/              # 本组独立 Git 仓库
~/HazardWalker/ros2_ws/      # 本组独立 ROS 2 colcon 工作空间
~/venvs/pytorch              # PyTorch CUDA 环境
~/datasets                   # 共享数据集
~/models                     # 共享模型
~/rosbags                    # 共享 rosbag
~/videos                     # 共享视频
```

共享大文件目录位于 `/srv/hazardwalker`，各账号 home 下通常通过软链接访问：

```text
~/datasets -> /srv/hazardwalker/datasets
~/models   -> /srv/hazardwalker/models
~/rosbags  -> /srv/hazardwalker/rosbags
~/videos   -> /srv/hazardwalker/videos
```

不要跨用户共享或软链接以下内容：

- 各组 `~/HazardWalker` 仓库
- `ros2_ws/build/`
- `ros2_ws/install/`
- `ros2_ws/log/`
- Python venv 或 conda 环境
- 临时实验输出

这些目录与源码路径、用户权限和环境变量强相关，共享后容易污染构建结果或产生权限问题。

## 5. 自动加载环境

默认 zsh 登录后会自动加载：

- ROS 2 Jazzy
- 本组 `ROS_DOMAIN_ID`
- 已构建工作空间的 `~/HazardWalker/ros2_ws/install/setup.zsh`

首次登录后建议检查：

```bash
whoami
echo $ROS_DOMAIN_ID
ros2 --help | head -1
gz sim --version
colcon --help
python3 --version
pip3 --version
nvidia-smi
```

如果 `ros2` 找不到，可手动执行：

```bash
source /opt/ros/jazzy/setup.zsh
```

## 6. 日常开发流程

拉取代码：

```bash
cd ~/HazardWalker
git fetch origin
git pull
```

运行离线测试：

```bash
python3 scripts/run_offline_tests.py
```

预期结果：

```text
10 passed, 0 failed
```

构建 ROS 2 工作空间：

```bash
./scripts/build.sh
```

运行最小 demo：

```bash
./scripts/run_minimal_demo.sh
```

检查结果文件：

```bash
ls reports/run_results
python3 scripts/evaluate_result.py reports/run_results/<timestamp>_result.json
```

最小 demo 当前用于验证 fake platform、HSV 检测、固定航点巡检、任务状态机和结果 JSON 输出能够串成闭环，不代表最终 Gazebo 楼层仿真系统。

## 7. PyTorch / GPU 使用

进入 PyTorch 环境：

```bash
source ~/venvs/pytorch/bin/activate
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
deactivate
```

注意：

- 不要在 PyTorch venv 或 conda 环境里运行 `colcon build`
- 不要在 PyTorch venv 或 conda 环境里运行 ROS demo
- 普通离线测试优先在 CPU 或个人电脑上跑
- 使用 GPU 前先在队内报备
- 用完 GPU 后说明已释放
- 禁止长时间占用 GPU 训练、渲染或仿真但不报备

查看 GPU 状态：

```bash
nvidia-smi
```

查看占用进程：

```bash
nvidia-smi
ps -fp <PID>
```

## 8. Docker 使用规则

普通账号默认没有 Docker 权限。需要 Docker 时向环境负责人申请，由管理员统一安装和授权。

原则：

- 能用系统 ROS/Gazebo 直接运行的任务，不优先使用 Docker
- 平台组确有需要时单独申请 Docker 权限
- GPU Docker 和 NVIDIA Container Toolkit 由环境负责人统一配置
- 加入 `docker` 组等价于拥有较高系统权限，不默认开放给所有人

## 9. 大文件与磁盘规则

大文件包括：

- 数据集
- 模型权重
- rosbag
- 视频
- 点云数据
- Gazebo 大型模型资源
- 官方平台 SDK 或仿真包

这些内容不要提交进 Git，也不要放入各组仓库。统一放入共享目录或由环境负责人指定位置。

磁盘注意：

- `/srv/hazardwalker` 用于共享大文件
- `/data` 为旧数据盘或历史路径，不建议写入新文件
- 下载大型资源前先确认保存位置

## 10. 使用规范

- 使用本组账号，不共用他人账号
- 使用本组 `ROS_DOMAIN_ID`，避免多组节点串话题
- 只修改自己账号下的 `~/HazardWalker`
- 系统级 apt 依赖不要自行安装
- 大文件不进 Git
- 长时间任务使用 SSH + `tmux`
- RDP 会话适合图形调试，不适合承载长期任务
- 环境问题优先联系测试组与集成组

## 11. 各组建议起步任务

| 组别 | 建议第一步 |
| --- | --- |
| 集成组 | 跑通验收链，维护 `dev` 分支质量 |
| 平台组 | 阅读 `fake_platform_node.py`，推进 Gazebo adapter |
| 导航组 | 阅读 `waypoint_controller.py`，预研 Frontier / Nav2 |
| 感知组 | 完善红球检测，预研三维定位与 YOLO |
| 决策组 | 完善状态机纯函数与测试 |
| 测试组 | 跑通离线测试与结果评估，设计指标统计 |
| 文档组 | 维护 `docs/`，同步资料与答辩材料 |

## 12. 常见问题

**`ros2` 找不到？**  
确认使用默认 zsh 登录，或手动执行 `source /opt/ros/jazzy/setup.zsh`。

**`git push` 失败？**  
需要配置 GitHub SSH key 或 Personal Access Token。配置完成前可先拉取、构建和测试。

**构建失败？**  
先确认没有进入 PyTorch venv 或 conda，再执行 `source /opt/ros/jazzy/setup.zsh` 后重新构建。

**需要安装系统依赖？**  
联系环境负责人，不要在普通组账号下自行改系统环境。

**需要图形界面？**  
使用远程桌面；无 GUI 需求时优先 SSH。
