#!/usr/bin/env python3

from __future__ import annotations

import rospy
import random
import json
import os
from gazebo_msgs.srv import SpawnModel, GetModelState
from geometry_msgs.msg import Pose
import tf.transformations as tf

BUILDING_CONFIG_PATH = "./building_config.json"
TRUTH_PATH = "./results/danger_truth.json"


def _source_sdf(name, shape, color, radius=None, size=None):
    color_map = {
        "red": "1 0 0 1",
        "green": "0 1 0 1",
    }
    material = color_map.get(color, "1 1 1 1")
    if shape == "sphere":
        geometry = f"<sphere><radius>{float(radius or 0.15):.3f}</radius></sphere>"
    elif shape == "box":
        sx, sy, sz = size or [0.3, 0.3, 0.3]
        geometry = f"<box><size>{float(sx):.3f} {float(sy):.3f} {float(sz):.3f}</size></box>"
    else:
        raise ValueError(f"unsupported source shape: {shape}")
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision"><geometry>{geometry}</geometry></collision>
      <visual name="visual">
        <geometry>{geometry}</geometry>
        <material><ambient>{material}</ambient><diffuse>{material}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def _load_truth_sources():
    truth_path = os.path.join(os.getcwd(), TRUTH_PATH)
    if not os.path.exists(truth_path):
        return None
    with open(truth_path, 'r') as f:
        truth_data = json.load(f)
    return truth_data.get("danger_sources", []) + truth_data.get("distraction_sources", [])


def _spawn_sources_from_truth(sources):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawned = 0
    for source in sources:
        pose = Pose()
        pose.position.x = float(source["position"][0])
        pose.position.y = float(source["position"][1])
        pose.position.z = float(source["position"][2])
        pose.orientation.w = 1.0
        name = source.get("model_name") or f"source_{source.get('id', spawned)}"
        sdf = _source_sdf(
            name,
            source.get("shape", "sphere"),
            source.get("color", "red"),
            radius=source.get("radius"),
            size=source.get("size"),
        )
        try:
            resp = spawn_model(name, sdf, "", pose, "world")
            if resp.success:
                spawned += 1
            else:
                rospy.logwarn(f"跳过或生成源 {name} 失败: {resp.status_message}")
        except rospy.ServiceException as exc:
            rospy.logerr(f"服务调用失败: {exc}")
    rospy.loginfo(f"已根据真值文件补生成 {spawned} 个源模型。")

def quaternion_rotate_vector(q, v):
    return tf.quaternion_multiply(tf.quaternion_multiply(q, [v[0], v[1], v[2], 0]), tf.quaternion_conjugate(q))[:3]

def get_model_pose(model_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = get_state(model_name, 'world')
        if resp.success:
            return resp.pose
        else:
            rospy.logerr(f"无法获取模型 {model_name} 的状态: {resp.status_message}")
            return None
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return None

def load_building_config():
    config_path = os.path.join(os.path.dirname(__file__), '..', '..', BUILDING_CONFIG_PATH)
    config_path = os.path.normpath(config_path)

    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            return json.load(f)
    return None

def get_floor_heights_from_config(config):
    if config and 'floor_heights' in config:
        return config['floor_heights']
    return [0.25, 1.75, 3.15, 4.65]

def get_building_bounds_from_config(config):
    if config and 'building_width' in config and 'building_depth' in config:
        bw = config['building_width']
        bd = config['building_depth']
        return -bw/2 + 0.5, bw/2 - 0.5, -bd/2 + 0.5, bd/2 - 0.5
    return -3.2, 3.2, -3.2, 3.2

def spawn_red_sphere(name, world_pose):
    sphere_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp = spawn_model(name, sphere_sdf, "", world_pose, "world")
        if resp.success:
            pass
        else:
            rospy.logwarn(f"生成红色球体 {name} 失败: {resp.status_message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")

def spawn_green_sphere(name, world_pose):
    sphere_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp = spawn_model(name, sphere_sdf, "", world_pose, "world")
        if resp.success:
            pass
        else:
            rospy.logwarn(f"生成绿色球体 {name} 失败: {resp.status_message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")

def spawn_red_box(name, world_pose):
    box_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.3 0.3</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.3 0.3</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp = spawn_model(name, box_sdf, "", world_pose, "world")
        if resp.success:
            pass
        else:
            rospy.logwarn(f"生成红色方块 {name} 失败: {resp.status_message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")

def main():
    rospy.init_node('spawn_obstacles_inside')
    truth_sources = _load_truth_sources()
    if truth_sources is not None:
        _spawn_sources_from_truth(truth_sources)
        return

    config = load_building_config()
    num_floors = config['num_floors'] if config else 4
    floor_heights = get_floor_heights_from_config(config)
    x_min, x_max, y_min, y_max = get_building_bounds_from_config(config)

    model_name = 'HotelBuilding'
    model_pose = get_model_pose(model_name)

    if model_pose is None:
        model_name = 'MultiFloorBuilding'
        model_pose = get_model_pose(model_name)
        if model_pose is None:
            model_name = 'Buliding'
            model_pose = get_model_pose(model_name)
            if model_pose is None:
                rospy.logwarn("无法获取建筑模型位姿，使用默认位置 (0,0,0)")
            from geometry_msgs.msg import Pose
            model_pose = Pose()
            model_pose.position.x = 0
            model_pose.position.y = 0
            model_pose.position.z = 0
            model_pose.orientation.w = 1

    pos = model_pose.position
    quat = [model_pose.orientation.x, model_pose.orientation.y,
            model_pose.orientation.z, model_pose.orientation.w]

    if not quat or all(q == 0 for q in quat):
        quat = [0, 0, 0, 1]

    num_obstacles = random.randint(5, 15)

    danger_sources = []
    distraction_sources = []
    object_id = 0

    for i in range(num_obstacles):
        floor = random.randint(0, num_floors - 1)
        z_local = floor_heights[floor] if floor < len(floor_heights) else floor_heights[-1] + 0.5 * (floor - len(floor_heights) + 1)

        x_local = random.uniform(x_min, x_max)
        y_local = random.uniform(y_min, y_max)

        local_vec = [x_local, y_local, z_local]
        world_vec = quaternion_rotate_vector(quat, local_vec)
        world_x = pos.x + world_vec[0]
        world_y = pos.y + world_vec[1]
        world_z = pos.z + world_vec[2]

        obstacle_pose = Pose()
        obstacle_pose.position.x = world_x
        obstacle_pose.position.y = world_y
        obstacle_pose.position.z = world_z
        obstacle_pose.orientation.w = 1.0

        obj_type = random.randint(0, 2)
        name_suffix = f"{i}_{random.randint(1000,9999)}"

        position = [round(world_x, 2), round(world_y, 2), round(world_z, 2)]

        if obj_type == 0:
            name = f"red_sphere_{name_suffix}"
            spawn_red_sphere(name, obstacle_pose)
            danger_sources.append({
                "id": object_id,
                "position": position,
                "color": "red",
                "shape": "sphere",
                "radius": 0.15
            })
        elif obj_type == 1:
            name = f"green_sphere_{name_suffix}"
            spawn_green_sphere(name, obstacle_pose)
            distraction_sources.append({
                "id": object_id,
                "position": position,
                "color": "green",
                "shape": "sphere",
                "radius": 0.15
            })
        else:
            name = f"red_box_{name_suffix}"
            spawn_red_box(name, obstacle_pose)
            distraction_sources.append({
                "id": object_id,
                "position": position,
                "color": "red",
                "shape": "box",
                "size": [0.3, 0.3, 0.3]
            })

        object_id += 1
        rospy.sleep(0.05)

    rospy.loginfo(f"所有障碍物生成完毕。共生成 {len(danger_sources)} 个危险源，{len(distraction_sources)} 个干扰源。")

    truth_data = {
        "num_floors": num_floors,
        "floor_heights": floor_heights,
        "danger_sources": danger_sources,
        "distraction_sources": distraction_sources
    }

    os.makedirs("./results", exist_ok=True)
    json_filename = "./results/danger_truth.json"
    with open(json_filename, 'w') as f:
        json.dump(truth_data, f, indent=2)
    rospy.loginfo(f"真值数据已保存至 {json_filename}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
