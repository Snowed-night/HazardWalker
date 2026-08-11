#!/usr/bin/env python3
"""负责人修改：在不重置随机场景的前提下扶正 Gazebo 中倒地的 A1。"""

import math
import os
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import (
    GetModelState,
    GetPhysicsProperties,
    SetModelConfiguration,
    SetModelState,
)
from std_srvs.srv import Empty


MODEL_NAME = os.environ.get("SIMENV_ROBOT_MODEL", "a1_gazebo")
RECOVERY_LIFT_M = float(os.environ.get("SIMENV_RECOVERY_LIFT_M", "0.38"))
RECOVERY_SETTLE_SEC = float(os.environ.get("SIMENV_RECOVERY_SETTLE_SEC", "4.0"))
REQUEST_FILE = os.environ.get(
    "SIMENV_RECOVERY_REQUEST_FILE", "/tmp/hazardwalker-controller-recover.request"
)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wait(name: str, timeout: float = 15.0) -> None:
    rospy.wait_for_service(name, timeout=timeout)


def _upright_cosine(orientation) -> float:
    """返回机体 z 轴与世界 z 轴夹角余弦；1 表示完全直立。"""

    return 1.0 - 2.0 * (
        float(orientation.x) ** 2 + float(orientation.y) ** 2
    )


def _wait_for_controller_request(timeout_sec: float = 2.0) -> None:
    """等待 headless FSM 删除一次性请求文件，证明控制器已退回站立态。"""

    deadline = time.monotonic() + timeout_sec
    while os.path.exists(REQUEST_FILE) and time.monotonic() < deadline:
        time.sleep(0.02)
    if os.path.exists(REQUEST_FILE):
        raise RuntimeError("junior_ctrl did not consume the recovery request")


def main() -> None:
    rospy.init_node("hazardwalker_recover_a1", anonymous=True, disable_signals=True)
    for service in (
        "/gazebo/get_physics_properties",
        "/gazebo/pause_physics",
        "/gazebo/unpause_physics",
        "/gazebo/get_model_state",
        "/gazebo/set_model_state",
        "/gazebo/set_model_configuration",
    ):
        _wait(service)

    physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)()
    was_paused = bool(physics.pause)
    pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
    unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    set_configuration = rospy.ServiceProxy(
        "/gazebo/set_model_configuration", SetModelConfiguration
    )

    if RECOVERY_LIFT_M <= 0.0 or RECOVERY_SETTLE_SEC <= 0.0:
        raise ValueError("recovery lift and settle time must be positive")

    # 状态机必须先退出 RL/move_base，避免扶正后的第一周期继续执行旧步态。
    try:
        _wait_for_controller_request()
    except Exception:
        # 旧控制器不认识请求文件时必须清理，避免以后重编或切态时误触发。
        try:
            os.remove(REQUEST_FILE)
        except FileNotFoundError:
            pass
        raise
    pause()
    initial_height = None
    initial_upright = None
    try:
        current = get_state(MODEL_NAME, "world")
        if not current.success:
            raise RuntimeError(current.status_message)
        initial_height = float(current.pose.position.z)
        initial_upright = _upright_cosine(current.pose.orientation)
        yaw = _yaw_from_quaternion(
            current.pose.orientation.x,
            current.pose.orientation.y,
            current.pose.orientation.z,
            current.pose.orientation.w,
        )

        # 先恢复四条腿的标准站立关节角，再原地抬升并只保留偏航角。
        joint_names = []
        joint_positions = []
        for leg in ("FR", "FL", "RR", "RL"):
            joint_names.extend(
                [f"{leg}_hip_joint", f"{leg}_thigh_joint", f"{leg}_calf_joint"]
            )
            joint_positions.extend([0.0, 0.67, -1.3])
        configured = set_configuration(
            MODEL_NAME, "robot_description", joint_names, joint_positions
        )
        if not configured.success:
            raise RuntimeError(configured.status_message)

        target = ModelState()
        target.model_name = MODEL_NAME
        target.reference_frame = "world"
        target.pose.position.x = current.pose.position.x
        target.pose.position.y = current.pose.position.y
        target.pose.position.z = current.pose.position.z + RECOVERY_LIFT_M
        target.pose.orientation.z = math.sin(yaw / 2.0)
        target.pose.orientation.w = math.cos(yaw / 2.0)
        result = set_state(target)
        if not result.success:
            raise RuntimeError(result.status_message)

        # 即使调用前场景处于暂停状态，也要短时恢复物理仿真，才能证明固定站立
        # 控制器实际接管，而不是只把模型悬空传送到直立姿态。
        unpause()
        time.sleep(RECOVERY_SETTLE_SEC)
        recovered = get_state(MODEL_NAME, "world")
        if not recovered.success:
            raise RuntimeError(recovered.status_message)
        upright = _upright_cosine(recovered.pose.orientation)
        height_gain = float(recovered.pose.position.z) - initial_height
        if upright < 0.75 or (initial_upright < 0.75 and height_gain < 0.05):
            raise RuntimeError(
                "recovery verification failed: "
                f"upright_cosine={upright:.3f}, height_gain={height_gain:.3f}m"
            )
    finally:
        if was_paused:
            pause()
        else:
            unpause()
    print(
        "A1 recovery verified: scene and planar pose preserved; "
        "controller is in fixed stand and the body is upright."
    )


if __name__ == "__main__":
    main()
