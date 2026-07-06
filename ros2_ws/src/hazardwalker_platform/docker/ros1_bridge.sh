#!/usr/bin/env bash
# ROS1 ↔ ROS2 桥接 — 把 ROS1 传感器话题翻译成 ROS2 DDS
set -eo pipefail

echo "[ros1_bridge] Loading ROS1 Noetic..."
source /opt/ros/noetic/setup.bash

echo "[ros1_bridge] Loading ROS2 Foxy..."
source /opt/ros/foxy/setup.bash

echo "[ros1_bridge] Starting dynamic bridge..."
/opt/ros/foxy/lib/ros1_bridge/dynamic_bridge --bridge-all-topics
