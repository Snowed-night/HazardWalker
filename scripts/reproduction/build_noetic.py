#!/usr/bin/env python3
"""用现有官方镜像构建ROS1控制器；限制编译并行度，不启动Gazebo。"""
from pathlib import Path
import os
import subprocess

ROOT = Path(__file__).resolve().parents[2]

def main():
    platform = ROOT / 'ros2_ws/src/hazardwalker_platform'
    for rel in ('results', 'results/video_capture', 'logs', 'generated_building'):
        (platform / rel).mkdir(exist_ok=True, parents=True)
    image = os.environ.get('SIMENV_IMAGE', 'simenv_ros1:noetic-focal')
    cpus = sorted(os.sched_getaffinity(0))[-2:]
    cmd = ['docker', 'run', '--rm', '--pull=never', '--network=none', '--cpus=2',
           '--cpuset-cpus=' + ','.join(map(str, cpus)), '--memory=8g', '--memory-swap=8g',
           '--entrypoint=/ros_entrypoint.sh', '--mount', f'type=bind,src={platform},dst=/home/ros/simenv_ws',
           '-w', '/home/ros/simenv_ws', '-e', f'HOST_UID={os.getuid()}', '-e', f'HOST_GID={os.getgid()}',
           '-e', 'LIBTORCH_ROOT=/home/ros/Guoyulun/Download/libtorch', image,
           'bash', '-lc', 'bash docker/build_catkin.sh']
    subprocess.run(cmd, check=True)

if __name__ == '__main__':
    main()
