#!/usr/bin/env python3
"""官方 SimEnv 中只依赖公开 IMU 的感知自主环视节点。

所属组：感知定位组。
文件作用：
1. 在独占官方随机场景中，通过 `/trunk_imu` 闭环完成原地 360° 环视；
2. 发布有限速 `/cmd_vel`，让 RGB-D 感知获得真实、无人为干预的多视角；
3. 记录视角覆盖、候选/确认计数和停止原因，供正式取证复盘。

安全边界：
1. 不读取 Odometry_gazebo、ground truth、场景布局、manifest 或危险源真值；
2. 默认拒绝运行，必须显式设置 `~exclusive_session:=true`；
3. 检测到其他 `/cmd_vel` 发布者立即停车，避免与导航控制仲裁；
4. 该脚本只是入口房间环视，不得把结果冒充完整楼宇探索成绩。
"""

import json
import math
import os
import sys
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from hazardwalker_perception.scan_imu_localization import (  # noqa: E402
    normalize_angle,
    quaternion_to_yaw,
)


class OfficialPerceptionSweep(object):
    """以 IMU 偏航闭环执行一次受限原地环视。"""

    def __init__(self):
        rospy.init_node('hazardwalker_official_perception_sweep', anonymous=False)
        if not bool(rospy.get_param('~exclusive_session', False)):
            raise rospy.ROSInitException(
                '正式环视必须显式设置 ~exclusive_session:=true。',
            )
        self.imu_topic = str(rospy.get_param('~imu_topic', '/trunk_imu'))
        self.detection_topic = str(rospy.get_param(
            '~detection_topic', '/hazardwalker/perception/hazard_detections',
        ))
        self.cmd_vel_topic = str(rospy.get_param('~cmd_vel_topic', '/cmd_vel'))
        self.angular_speed = min(
            0.65, max(0.25, abs(float(rospy.get_param('~angular_speed', 0.45)))),
        )
        self.direction = -1.0 if float(rospy.get_param('~direction', 1.0)) < 0.0 else 1.0
        self.target_sweep_rad = math.radians(min(
            370.0, max(30.0, float(rospy.get_param('~target_sweep_deg', 360.0))),
        ))
        self.timeout_sec = min(
            90.0, max(5.0, float(rospy.get_param('~timeout_sec', 45.0))),
        )
        self.scenario_seed = str(rospy.get_param('~scenario_seed', '')).strip()
        self.code_version = str(rospy.get_param('~code_version', '')).strip()
        if not self.scenario_seed or not self.code_version:
            raise rospy.ROSInitException(
                '必须显式设置 ~scenario_seed 和 ~code_version。',
            )
        raw_output = str(rospy.get_param('~output_path', '')).strip()
        if not raw_output:
            raise rospy.ROSInitException('必须显式设置 ~output_path。')
        self.output_path = Path(raw_output).expanduser()
        self.latest_yaw = None
        self.detection_message_count = 0
        self.candidate_frame_count = 0
        self.confirmed_track_ids = set()
        self.max_candidate_count = 0
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=2)
        rospy.Subscriber(self.imu_topic, Imu, self._on_imu, queue_size=20)
        rospy.Subscriber(self.detection_topic, String, self._on_detection, queue_size=20)
        rospy.on_shutdown(self._stop)

    def _on_imu(self, message):
        orientation = message.orientation
        self.latest_yaw = quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w,
        )

    def _on_detection(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict):
            return
        self.detection_message_count += 1
        detections = list(payload.get('detections_2d', []))
        self.max_candidate_count = max(self.max_candidate_count, len(detections))
        if detections:
            self.candidate_frame_count += 1
        for hazard in payload.get('hazards', []):
            if hazard.get('status') == 'confirmed':
                self.confirmed_track_ids.add(str(hazard.get('id', '')))

    def run(self):
        """等待 IMU、检查控制唯一性并执行闭环环视。"""
        started_wall = time.monotonic()
        wait_deadline = started_wall + 10.0
        while not rospy.is_shutdown() and self.latest_yaw is None:
            if time.monotonic() >= wait_deadline:
                return self._finish('imu_not_ready', 0.0, started_wall)
            time.sleep(0.05)
        if self._foreign_cmd_vel_publishers():
            return self._finish('foreign_cmd_vel_publisher', 0.0, started_wall)

        previous_yaw = self.latest_yaw
        swept_rad = 0.0
        deadline = time.monotonic() + self.timeout_sec
        rate = rospy.Rate(20)
        termination_reason = 'timeout'
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                current_yaw = self.latest_yaw
                if current_yaw is not None:
                    delta = normalize_angle(current_yaw - previous_yaw)
                    # 只累计指令方向的实际旋转；反向打滑和 IMU 抖动不能伪造覆盖率。
                    swept_rad += max(0.0, self.direction * delta)
                    previous_yaw = current_yaw
                if swept_rad >= self.target_sweep_rad:
                    termination_reason = 'target_sweep_reached'
                    break
                if self._foreign_cmd_vel_publishers():
                    termination_reason = 'foreign_cmd_vel_publisher'
                    break
                command = Twist()
                command.angular.z = self.direction * self.angular_speed
                self.cmd_pub.publish(command)
                rate.sleep()
        finally:
            self._stop()
        return self._finish(termination_reason, swept_rad, started_wall)

    def _foreign_cmd_vel_publishers(self):
        """ROS master 级检查，保证本节点不是与导航同时抢占控制。"""
        try:
            code, _message, state = rospy.get_master().getSystemState()
        except Exception:
            return ['ros_master_unavailable']
        if code != 1:
            return ['ros_master_unavailable']
        own_name = rospy.get_name()
        for topic, publishers in state[0]:
            if topic == self.cmd_vel_topic:
                return [name for name in publishers if name != own_name]
        return []

    def _stop(self):
        """连续发布零速，避免单次停止消息在 ROS 队列中丢失。"""
        stop = Twist()
        for _index in range(6):
            self.cmd_pub.publish(stop)
            time.sleep(0.03)

    def _finish(self, reason, swept_rad, started_wall):
        result = {
            'schema': 'hazardwalker_official_perception_sweep_v1',
            'run_scope': 'entry_room_perception_sweep',
            'scenario_seed': self.scenario_seed,
            'code_version': self.code_version,
            'official_score_eligible': False,
            'completed_full_building_exploration': False,
            'truth_or_layout_inputs_used': False,
            'termination_reason': reason,
            'target_sweep_deg': round(math.degrees(self.target_sweep_rad), 3),
            'actual_sweep_deg': round(math.degrees(float(swept_rad)), 3),
            'elapsed_wall_sec': round(time.monotonic() - started_wall, 3),
            'angular_speed_rad_s': self.direction * self.angular_speed,
            'detection_message_count': self.detection_message_count,
            'candidate_frame_count': self.candidate_frame_count,
            'max_candidate_count': self.max_candidate_count,
            'confirmed_track_ids': sorted(self.confirmed_track_ids),
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output_path.with_suffix(self.output_path.suffix + '.tmp')
        temporary.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        temporary.replace(self.output_path)
        rospy.loginfo('Perception sweep finished: %s', json.dumps(result))
        return result


def main():
    sweep = OfficialPerceptionSweep()
    sweep.run()


if __name__ == '__main__':
    main()
