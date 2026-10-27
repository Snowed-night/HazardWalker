# 快速开始

构建和实验在Ubuntu执行，Windows可用于SSH连接、浏览器观看及取回结果。

## 1 连接并获取代码

本机PowerShell：

```powershell
ssh your-user@your-server
```

远端登录后切到Bash：

```bash
bash
git clone --depth 1 --branch main https://github.com/Snowed-night/HazardWalker.git
cd HazardWalker
```

## 2 首次准备和构建

先完成[环境准备](environment.md)。已有官方环境时执行：

```bash
export OFFICIAL_SIMENV_SOURCE_ROOT=/absolute/path/to/official/SimEnv
export SIMENV_IMAGE=simenv_ros1:noetic-focal
python3 scripts/reproduction/prepare_assets.py --source "$OFFICIAL_SIMENV_SOURCE_ROOT"
bash check.sh
bash build.sh
```

存在原受测镜像时可以将SIMENV_IMAGE设置为环境文档中的精确ID。新终端需要重新设置环境变量。

## 3 启动一次完整三层实验

```bash
export DOCKER_SIMENV_USER=station_cluster
export ROS_DOMAIN_ID=43
export USE_GPU=0
bash run.sh 202609102 demo_seed202609102
```

入口启动官方仿真、适配器、SLAM、导航、感知和记录器。开启第一人称与二维SLAM录像，关闭3D点云转发和录像，沿用旧版速度、阈值和时间预算。已有其它Gazebo时启动前退出，请先协调空闲时段。

运行终端保持打开。每次使用新结果名，也可以省略结果名，自动附加UTC时间：

```bash
bash run.sh 202609102
```

一次调用只运行一次。Ctrl+C中止本轮并进入清理，不自动换种子、补跑或删除历史结果。

## 4 可选观看仿真画面

等待本轮容器就绪，在第二个SSH终端执行：

```bash
bash
cd /absolute/path/to/HazardWalker/ros2_ws/src/hazardwalker_platform
export DOCKER_SIMENV_USER=station_cluster
export ROS_DOMAIN_ID=43
./auto_docker.sh gui up
```

在本机PowerShell另开终端，建立隧道并保持打开：

```powershell
ssh -N -o ServerAliveInterval=30 -o ServerAliveCountMax=3 -L 6081:127.0.0.1:6081 your-user@your-server
```

本机浏览器打开：

- 上帝视角：`http://127.0.0.1:6081/hazardwalker.html`

GUI需要对应sidecar镜像和图形依赖。不能使用GUI时，仍可检查本轮自动保存的视频。黑屏时先检查`gui status`和SSH隧道。

## 5 检查并取回结果

在仓库根目录执行：

```bash
python3 scripts/reproduction_check_result.py reports/nav/demo_seed202609102
```

检查`reproduction_acceptance.json`，再看原始匹配、轨迹和房间记录，见[结果核验](run-and-results.md)。本机PowerShell取回：

```powershell
scp -r your-user@your-server:/absolute/path/to/HazardWalker/reports/nav/demo_seed202609102 .
```

关闭观看服务时，在第二个SSH终端执行`./auto_docker.sh gui down`。
