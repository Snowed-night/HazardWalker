# SimEnv ROS 1 — Ubuntu 20.04 (Focal) + ROS Noetic + Gazebo Classic 11
# Host hxbl is Ubuntu 24.04; this image provides the original ROS 1 runtime.
FROM osrf/ros:noetic-desktop-full

LABEL maintainer="hazardwalker-integration"
LABEL org.opencontainers.image.base="ubuntu:20.04"
LABEL org.opencontainers.image.description="SimEnv ROS1 Noetic on Ubuntu 20.04 Focal"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update -qq \
    && apt-get install -y -qq --no-install-recommends \
      python3-catkin-tools \
      ros-noetic-gazebo-ros-pkgs \
      ros-noetic-gazebo-ros-control \
      ros-noetic-controller-manager \
      ros-noetic-joint-state-controller \
      ros-noetic-effort-controllers \
      ros-noetic-robot-state-publisher \
      ros-noetic-xacro \
      ros-noetic-move-base-msgs xvfb \
      ros-noetic-rosbridge-server \
      liblcm-dev \
      libeigen3-dev \
      libyaml-cpp-dev \
      iproute2 \
      net-tools \
      wget \
      ca-certificates \
      curl \
      unzip \
      expect \
    && rm -rf /var/lib/apt/lists/*

# unitree_guide/junior_ctrl expects this exact prefix (see CMakeLists.txt in SimEnv src).
ENV LIBTORCH_ROOT=/home/ros/Guoyulun/Download/libtorch
ENV LIBTORCH_VERSION=2.0.1
ENV LIBTORCH_CUDA=cu118
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${LIBTORCH_ROOT}/lib:${CUDA_HOME}/lib64
ENV CMAKE_PREFIX_PATH=${LIBTORCH_ROOT}

COPY install_cuda_toolkit.sh /tmp/install_cuda_toolkit.sh
RUN chmod +x /tmp/install_cuda_toolkit.sh && /tmp/install_cuda_toolkit.sh && rm -f /tmp/install_cuda_toolkit.sh

COPY install_libtorch.sh /tmp/install_libtorch.sh
RUN chmod +x /tmp/install_libtorch.sh && /tmp/install_libtorch.sh && rm -f /tmp/install_libtorch.sh

WORKDIR /home/ros/simenv_ws
