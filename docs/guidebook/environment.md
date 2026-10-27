# 环境准备

适用宿主为Ubuntu 24.04 x86_64、ROS2 Jazzy。ROS1 Noetic、Gazebo Classic和宇树控制器运行在Docker内。

## 宿主依赖

按ROS2官方说明准备`/opt/ros/jazzy/setup.bash`，安装Docker Engine和Compose插件。在已配置ROS2 apt源的机器上执行：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-venv \
  python3-numpy python3-opencv python3-yaml python3-pil ffmpeg git \
  ros-jazzy-cartographer-ros ros-jazzy-slam-toolbox ros-jazzy-sensor-msgs-py
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src -r -y --rosdistro jazzy
```

当前账号应能执行`docker info`。已初始化的rosdep无需再次`sudo rosdep init`。桥接还需要websocket-client：

```bash
python3 -m venv --system-site-packages "$HOME/.local/share/hazardwalker-ros2-venv"
"$HOME/.local/share/hazardwalker-ros2-venv/bin/pip" install websocket-client==1.9.0
```

这是原运行器支持的路径；也兼容`$HOME/.local/share/hazardwalker/ros2_bridge_venv`。业务包仍使用系统Python。

## 仿真镜像

历史标签为`simenv_ros1:noetic-focal`，历史成功环境的镜像ID为：

```text
sha256:19dbf5f87625e68876adbb46b5135620291d87eb5bee7836ff2168b15b44c887
```

该镜像已在本机时，直接指定可避免同名标签漂移：

```bash
export SIMENV_IMAGE=sha256:19dbf5f87625e68876adbb46b5135620291d87eb5bee7836ff2168b15b44c887
```

镜像ID不能用来自动下载镜像。新机器需要导入官方镜像，或按仓库Dockerfile构建：

```bash
export SIMENV_IMAGE=hazardwalker/simenv:stable-20260910
USE_GPU=0 bash ros2_ws/src/hazardwalker_platform/docker/auto_noetic.sh image
```

镜像构建包含较大的ROS/Gazebo、CUDA工具链和LibTorch依赖，应提前完成。LibTorch使用2.0.1 cu118，路径为`/home/ros/Guoyulun/Download/libtorch`。新构建的镜像ID通常不同，需要在记录中保留实际ID。

旧配置将运动策略放在CPU、使用软件渲染。整理入口默认`USE_GPU=0`；匹配宿主驱动和Container Toolkit后可显式设置`USE_GPU=1`，并记录环境差异。

## 运行资源

Git不包含策略权重和SDK预编译库。取得官方SimEnv后执行：

```bash
export OFFICIAL_SIMENV_SOURCE_ROOT=/absolute/path/to/official/SimEnv
python3 scripts/reproduction/prepare_assets.py --source "$OFFICIAL_SIMENV_SOURCE_ROOT"
```

输入目录需要包含`src/unitree_guide`。原交付包的外置`runtime_assets`目录也可以作为source。脚本按21项SHA-256清单先校验再复制，已有内容不同时拒绝覆盖。关键资源包括两份`.pt`运动策略、amd64的`libunitree_legged_sdk.a`与`libUnitree_motor_SDK_Linux.so`；完整路径见`reproduction/runtime-assets.json`。

只有源码时仍可运行离线检查；完整仿真需要先取得上述官方资源。

## 构建

```bash
bash check.sh
bash build.sh
```

构建生成仓库自己的`build/`、`install/`、`log/`以及平台下的`.ros1_catkin_ws/`。ROS1构建使用已准备的镜像，默认两个CPU，不启动Gazebo。只构建ROS2时用`bash build.sh --ros2-only`，首次完整实验前仍需完成ROS1构建。
