HazardWalker 主力机环境文档

主力机：hxbl

本文含队内账号和密码，仅限队内使用，请勿公开转发。

1. 机器环境

系统        Ubuntu 24.04.4 LTS
内核        6.17.0-35-generic
CPU         Intel Core i9-13900K，24 核 / 32 线程
内存        约 94 GiB，Swap 8 GiB
GPU         NVIDIA RTX A6000，约 48 GB 显存
驱动        NVIDIA Driver 535.309.01
CUDA        runtime 显示版本 12.2
ROS         ROS 2 Jazzy
Gazebo      Gazebo Harmonic，Gazebo Sim 8.12.0
Python      Python 3.12.3，pip 24.0
Shell       zsh 5.9

关键命令路径：

ros2        /opt/ros/jazzy/bin/ros2
colcon      /usr/bin/colcon
rosdep      /usr/bin/rosdep
gz          /usr/bin/gz
python3     /usr/bin/python3
pip3        /usr/bin/pip3

磁盘情况：

系统盘      /dev/nvme1n1，约 1.9T，根分区 / 可用约 1.8T
额外 NVMe   /dev/nvme0n1，约 931G，当前未挂载
机械硬盘    /dev/sda，约 1.8T，当前未挂载

2. 环境原则

系统级依赖由环境负责人统一安装和维护。
普通组员默认没有 sudo，只在自己的 home 目录开发。
Docker 权限单独申请，不默认开放。
GPU 使用需要提前报备，集成测试账号优先。
大文件可以共享，代码仓库、构建产物和 Python 虚拟环境不要跨用户共享。

3. 系统统一环境

系统级环境由 hazard_admin 或环境负责人维护。普通账号不要自行安装 apt 依赖。

已配置或应统一维护的内容：

NVIDIA Driver
ROS 2 Jazzy
Gazebo Harmonic
colcon
rosdep
Python 3 / pip / venv
基础编译工具
常用终端工具
zsh / oh-my-zsh

基础验收命令：

lsb_release -a
uname -a
nvidia-smi
ros2 --help
gz sim --version
colcon --help
rosdep --help
python3 --version
pip3 --version

当前最小 demo 不需要 CUDA Toolkit。除非后续明确需要训练或 GPU Docker，不建议提前安装 CUDA Toolkit、NVIDIA Container Toolkit 或大型模型资源。

4. 账号规划

hazard_admin          管理员，有 sudo
hazard_platform       平台组
hazard_nav            导航组
hazard_perception     感知组
hazard_decision       决策组
hazard_test           测试组
hazard_docs           文档组

队内使用账号：

平台组    hazard_platform       密码 HazardWalker2026    ROS_DOMAIN_ID 11
导航组    hazard_nav            密码 HazardWalker2026    ROS_DOMAIN_ID 12
感知组    hazard_perception     密码 HazardWalker2026    ROS_DOMAIN_ID 13
决策组    hazard_decision       密码 HazardWalker2026    ROS_DOMAIN_ID 14
测试组    hazard_test           密码 HazardWalker2026    ROS_DOMAIN_ID 15
文档组    hazard_docs           密码 HazardWalker2026

除 hazard_admin 外，普通账号不默认加入 sudo 组。确实需要临时 sudo 时，由环境负责人单独处理，完成后收回。

5. 各账号开发环境

各组只在自己的 home 目录开发。

推荐目录：

/home/hazard_integration/HazardWalker
/home/hazard_platform/HazardWalker
/home/hazard_nav/HazardWalker
/home/hazard_perception/HazardWalker
/home/hazard_decision/HazardWalker
/home/hazard_test/HazardWalker
/home/hazard_docs/HazardWalker

每个账号各自 clone、切分支、构建：

cd ~
git clone git@github.com:Snowed-night/HazardWalker.git HazardWalker
cd HazardWalker
git switch dev
python3 scripts/run_offline_tests.py
./scripts/build.sh

zsh 环境加载：

echo "source /opt/ros/jazzy/setup.zsh" >> ~/.zshrc
echo "source ~/HazardWalker/ros2_ws/install/setup.zsh" >> ~/.zshrc
source ~/.zshrc

bash 环境加载：

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/HazardWalker/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

登录后检查：

whoami
echo $ROS_DOMAIN_ID
ros2 --help
gz sim --version
colcon --help
python3 --version
nvidia-smi

6. 不共享的内容

以下内容不要跨用户共享，也不要做软链共用：

各组 HazardWalker 仓库
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
各组 Python venv
各组临时实验输出

原因：

colcon 构建目录和源码、环境变量、包路径强相关，共享后容易互相污染。
Python 虚拟环境包含绝对路径和用户权限信息，跨用户使用容易出问题。
各组独立仓库更方便定位问题、切换分支和回滚。

7. 大文件共享

大型数据、模型、rosbag、视频可以放共享目录，避免重复占用空间。

推荐共享目录：

~/datasets      数据集
~/models        模型
~/rosbags       rosbag
~/videos        视频

共享目录原则：

大文件放共享目录。
代码仓库、构建结果、Python 虚拟环境不共享。
大文件不要提交进 Git。
不要向 /data 写入新文件，旧数据盘已满。

8. Docker 权限

普通账号默认没有 Docker 权限。

需要 Docker 时，向环境负责人申请。批准后由管理员统一配置。

注意：

加入 docker 组基本等价于较高系统权限。
禁用 sudo 后，不应默认给所有人 Docker 权限。
能用系统 ROS / Gazebo 直接运行的任务，不优先使用 Docker。
GPU Docker 和 NVIDIA Container Toolkit 由环境负责人统一配置。

9. GPU 使用

当前 GPU：

NVIDIA RTX A6000，约 48 GB 显存

使用规则：

普通离线测试不占用 GPU。
hazard_integration 集成测试优先使用 GPU。
其他组需要占用 GPU 前，先通知环境负责人或集成负责人。
用完 GPU 后在群里说明已释放。
禁止长期挂后台训练、渲染或仿真不报备。

检查 GPU：

nvidia-smi

查看进程：

nvidia-smi
ps -fp <PID>

10. SSH 策略

推荐最终关闭 SSH 密码登录，只允许 SSH key 登录。

实施顺序：

第一阶段：建号和发 key 阶段，临时允许密码登录。
第二阶段：确认每个成员都能用 SSH key 登录后，关闭密码登录。

公钥位置：

/home/<username>/.ssh/authorized_keys

权限要求：

chmod 700 ~/.ssh
chmod 600 ~/.ssh/authorized_keys

建议 SSH 配置：

PubkeyAuthentication yes
PasswordAuthentication no
PermitRootLogin no
KbdInteractiveAuthentication no

重载 SSH 前，必须保留一个已登录的管理员终端，避免配置错误后无法登录。

11. 不自动下载的大文件

以下内容不要让脚本自动下载，下载前先确认保存位置：

NVIDIA CUDA Toolkit
Gazebo 大型模型资源
rosbag、视频、点云数据
YOLO 权重或训练数据集
官方平台 SDK 或仿真包

12. 当前验证状态

当前 HazardWalker 最小 demo 已在主力机跑通：

python3 scripts/run_offline_tests.py
./scripts/build.sh
./scripts/run_minimal_demo.sh

检查结果：

ls reports/run_results
python3 scripts/evaluate_result.py reports/run_results/<timestamp>_result.json

当前 demo 是 fake platform 闭环验证，不是真实 Gazebo 楼栋场景。后续真正可视化仿真应由平台组继续推进 Gazebo adapter 和 gazebo_minimal.launch.py。

13. 使用入口速查

SSH 登录：

Tailscale    ssh <账号>@100.102.91.31
公网         ssh -p 6002 <账号>@47.98.141.33

远程桌面只在需要 RViz、Gazebo 图形界面时使用。

远程桌面登录：

第一层 RDP 网关     hazard_rdp / HazardWalkerRdp2026!
第二层 Linux 桌面   本组 hazard_* 账号 / HazardWalker2026

远程桌面地址：

Tailscale    100.102.91.31:3389
公网         先开 SSH 隧道，再连接 127.0.0.1:13389

公网 RDP 隧道：

ssh -p 6002 <账号>@47.98.141.33 -L 13389:127.0.0.1:3389

注意：SSH 隧道必须在自己的电脑上执行，不要先 SSH 到主力机后再执行。


