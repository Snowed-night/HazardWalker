#!/usr/bin/env python3
"""官方 SimEnv ROS1 感知正式证据记录器。

所属组：感知定位组 / 测试组。
本节点只记录已经由 RGB-D 感知和自建 SLAM 产生的公开数据：时间配对 RGB-D、
候选到确认状态、主动复查建议和合法 SLAM 轨迹。它不发布控制命令，不读取场景、
布局或裁判真值文件。正式场景结束时调用方应最后停止本节点，以便归档结果 JSON。
"""

import csv
import json
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String


# 允许脚本从官方 SimEnv 工作目录直接运行，无需将 HazardWalker 安装为 catkin 包。
REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from hazardwalker_perception.active_view_policy import choose_active_view_action  # noqa: E402
from hazardwalker_perception.dynamic_detection_records import (  # noqa: E402
    build_dynamic_summary,
    build_dynamic_testing_record,
    build_perception_evidence_contract,
)


class OfficialRos1EvidenceRecorder(object):
    """将 ROS1 感知过程落盘为可供独立校验器检查的证据目录。"""

    def __init__(self):
        rospy.init_node('hazardwalker_official_evidence_recorder', anonymous=False)
        self._declare_parameters()
        self.output_dir = _required_directory('~output_dir')
        self.test_record_dir = _required_directory('~test_record_dir')
        self.image_dir = self.output_dir / 'selected_images'
        self.raw_image_dir = self.image_dir / 'raw'
        self.annotated_image_dir = self.image_dir / 'annotated'
        self.depth_dir = self.output_dir / 'selected_depth'
        if rospy.get_param('~save_images', True):
            self.raw_image_dir.mkdir(parents=True, exist_ok=True)
            self.annotated_image_dir.mkdir(parents=True, exist_ok=True)
        if rospy.get_param('~save_depth_evidence', True):
            self.depth_dir.mkdir(parents=True, exist_ok=True)

        self.evidence_contract = build_perception_evidence_contract(
            rospy.get_param('~run_mode'), rospy.get_param('~scenario_seed'),
            rospy.get_param('~code_version'), rospy.get_param('~legal_pose_topic'),
            rospy.get_param('~localization_provenance'),
        )
        self._write_manifest()
        self.records = []
        self.latest_image = None
        self.latest_image_stamp = 0.0
        self.latest_image_frame_id = ''
        self.latest_depth = None
        self.latest_depth_stamp = 0.0
        self.latest_depth_frame_id = ''
        self.latest_legal_pose = None
        self.trajectory_sample_count = 0
        self.last_pose_save_sec = float('-inf')
        self.last_image_save_sec = float('-inf')
        self.context_image_count = 0
        self.mission_completed = False
        self.mission_finished_stamp_sec = None
        self.closed = False

        # 只订阅调用方明示的合法 SLAM 位姿；禁用真值/桥接里程计时 fail closed。
        legal_pose_topic = str(rospy.get_param('~legal_pose_topic')).strip()
        if 'forbidden_pose_topic' in self.evidence_contract['contract_violations']:
            raise rospy.ROSInitException('拒绝订阅禁用位姿话题，不能记录正式感知证据。')
        if not legal_pose_topic:
            rospy.logwarn('未提供 legal_pose_topic：本轮只能作为内部回归记录。')
        else:
            rospy.Subscriber(legal_pose_topic, Odometry, self._on_pose, queue_size=20)
        rospy.Subscriber(rospy.get_param('~image_topic'), Image, self._on_image, queue_size=10)
        rospy.Subscriber(rospy.get_param('~depth_topic'), Image, self._on_depth, queue_size=10)
        rospy.Subscriber(rospy.get_param('~detection_topic'), String, self._on_detection, queue_size=20)
        rospy.Subscriber(
            rospy.get_param('~mission_state_topic'), String,
            self._on_mission_state, queue_size=10,
        )
        rospy.on_shutdown(self.close)
        rospy.loginfo('Official ROS1 evidence recorder writes to %s', self.output_dir)

    @staticmethod
    def _declare_parameters():
        """集中声明默认值，便于启动命令与 manifest 完整复现。"""
        defaults = {
            '~output_dir': '',
            '~test_record_dir': '',
            '~scenario_name': 'official_simenv_random_perception',
            '~run_mode': 'internal_regression',
            '~scenario_seed': '',
            '~code_version': '',
            '~legal_pose_topic': '',
            '~localization_provenance': 'unverified',
            '~launch_command': '',
            '~result_json_path': '',
            '~image_topic': '/real_sense/rgb/image_raw',
            '~depth_topic': '/real_sense/depth/image_raw',
            '~detection_topic': '/hazardwalker/perception/hazard_detections',
            '~mission_state_topic': '/hazardwalker/mission/state',
            '~save_images': True,
            '~save_depth_evidence': True,
            '~min_image_save_interval_sec': 0.5,
            # 正式随机楼宇即使暂时无候选，也需低频保留真实环境覆盖帧；
            # 默认每 10 秒一帧且最多 80 帧，避免 600 秒任务无限占用磁盘。
            '~save_context_frames': True,
            '~context_image_save_interval_sec': 10.0,
            '~max_context_images': 80,
            '~max_rgb_depth_evidence_delta_sec': 0.15,
            '~trajectory_sample_interval_sec': 0.5,
        }
        for name, value in defaults.items():
            if not rospy.has_param(name):
                rospy.set_param(name, value)

    def _write_manifest(self):
        _write_json(self.output_dir / 'run_manifest.json', {
            'schema': 'hazardwalker_perception_official_evidence_v2',
            'evidence_contract': self.evidence_contract,
            'input_topics': {
                'image': str(rospy.get_param('~image_topic')),
                'depth': str(rospy.get_param('~depth_topic')),
                'detections': str(rospy.get_param('~detection_topic')),
                'legal_pose': str(rospy.get_param('~legal_pose_topic')),
                'mission_state': str(rospy.get_param('~mission_state_topic')),
            },
            'mission_completion_required': True,
            'launch_command': str(rospy.get_param('~launch_command')),
            'started_at_unix_sec': round(time.time(), 3),
        })

    def _on_image(self, message):
        image = _image_to_bgr(message)
        if image is None:
            rospy.logwarn_throttle(5.0, '不支持保存的 RGB 编码：%s', message.encoding)
            return
        self.latest_image = image
        self.latest_image_stamp = _stamp_to_sec(message.header.stamp)
        self.latest_image_frame_id = message.header.frame_id

    def _on_depth(self, message):
        depth = _depth_to_meters(message)
        if depth is None:
            rospy.logwarn_throttle(5.0, '不支持保存的深度编码：%s', message.encoding)
            return
        self.latest_depth = depth
        self.latest_depth_stamp = _stamp_to_sec(message.header.stamp)
        self.latest_depth_frame_id = message.header.frame_id

    def _on_pose(self, message):
        pose = message.pose.pose
        self.latest_legal_pose = {
            'frame_id': message.header.frame_id,
            'child_frame_id': message.child_frame_id,
            'position': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z},
            'orientation': {'x': pose.orientation.x, 'y': pose.orientation.y,
                            'z': pose.orientation.z, 'w': pose.orientation.w},
            'stamp_sec': _stamp_to_sec(message.header.stamp),
        }
        interval = float(rospy.get_param('~trajectory_sample_interval_sec'))
        if self.latest_legal_pose['stamp_sec'] - self.last_pose_save_sec >= interval:
            _append_json_line(self.output_dir / 'trajectory.jsonl', self.latest_legal_pose)
            self.last_pose_save_sec = self.latest_legal_pose['stamp_sec']
            self.trajectory_sample_count += 1

    def _on_detection(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError) as error:
            rospy.logwarn_throttle(5.0, '忽略无法解析的感知 JSON：%s', error)
            return
        if not isinstance(payload, dict):
            rospy.logwarn_throttle(5.0, '忽略非对象感知 JSON。')
            return
        detections = list(payload.get('detections_2d', []))
        hazards = list(payload.get('hazards', []))
        height = int(self.latest_image.shape[0]) if self.latest_image is not None else 0
        width = int(self.latest_image.shape[1]) if self.latest_image is not None else 0
        recommendation = choose_active_view_action(detections, width, height).to_dict()
        stamp_sec = _payload_stamp(payload, detections, time.time())
        raw_image_path, image_path, depth_path = self._save_evidence(
            stamp_sec, detections, hazards, recommendation,
        )
        record = {
            'timestamp_sec': stamp_sec,
            'image_frame_id': self.latest_image_frame_id,
            'robot_pose': self.latest_legal_pose,
            'localization_provenance': str(payload.get(
                'localization_provenance', rospy.get_param('~localization_provenance'))),
            'detections_2d': detections,
            'hazards': hazards,
            'localization_ready': bool(payload.get('localization_ready', False)),
            'view_recommendation': recommendation,
            'evidence_raw_image': raw_image_path,
            'evidence_image': image_path,
            'evidence_depth': depth_path,
        }
        self.records.append(record)
        _append_json_line(self.output_dir / 'frames.jsonl', record)

    def _on_mission_state(self, message):
        """只有导航/任务层真实发布 FINISHED 才记录完整场景完成。"""
        if str(message.data).strip().upper() == 'FINISHED':
            self.mission_completed = True
            self.mission_finished_stamp_sec = round(time.time(), 3)

    def _save_evidence(self, stamp_sec, detections, hazards, recommendation):
        """保存候选/确认帧，并拒绝未时间配对的深度作为正式证据。"""
        confirmed = any(item.get('status') == 'confirmed' for item in hazards)
        event_frame = bool(detections) or confirmed
        if not event_frame:
            if not rospy.get_param('~save_context_frames'):
                return '', '', ''
            if self.context_image_count >= int(rospy.get_param('~max_context_images')):
                return '', '', ''
        if not rospy.get_param('~save_images') or self.latest_image is None:
            return '', '', ''
        interval_parameter = (
            '~min_image_save_interval_sec'
            if event_frame else '~context_image_save_interval_sec'
        )
        if stamp_sec - self.last_image_save_sec < float(rospy.get_param(interval_parameter)):
            return '', '', ''
        prefix = 'event' if event_frame else 'context'
        filename = '%s_%06d_%.3f.png' % (
            prefix, len(self.records) + 1, stamp_sec,
        )
        annotated = self.latest_image.copy()
        for detection in detections:
            bbox = detection.get('bbox', {})
            x0, y0 = int(bbox.get('x_min', 0)), int(bbox.get('y_min', 0))
            x1, y1 = int(bbox.get('x_max', x0)), int(bbox.get('y_max', y0))
            reobserve = bool(detection.get('requires_reobservation', False))
            color = (0, 165, 255) if reobserve else (0, 0, 255)
            label = '#%s %.2f %s' % (detection.get('id', '?'),
                                     float(detection.get('confidence', 0.0)),
                                     'reobserve' if reobserve else '2d_candidate')
            cv2.rectangle(annotated, (x0, y0), (x1, y1), color, 2)
            cv2.putText(annotated, label, (x0, max(18, y0 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        cv2.putText(annotated, 'view: ' + str(recommendation.get('action', 'unknown')),
                    (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        if not event_frame:
            cv2.putText(
                annotated, 'official random scene context (no candidate)',
                (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 0), 1,
            )
        stem = Path(filename).stem
        raw_image_path = self.raw_image_dir / (stem + '_raw.png')
        image_path = self.annotated_image_dir / (stem + '_annotated.png')
        if not cv2.imwrite(str(raw_image_path), self.latest_image):
            rospy.logwarn('原始 RGB 证据写入失败：%s', raw_image_path)
            return '', '', ''
        if not cv2.imwrite(str(image_path), annotated):
            rospy.logwarn('标注 RGB 证据写入失败：%s', image_path)
            raw_image_path.unlink(missing_ok=True)
            return '', '', ''
        depth_relative = ''
        if (rospy.get_param('~save_depth_evidence') and self.latest_depth is not None
                and abs(self.latest_depth_stamp - self.latest_image_stamp)
                <= float(rospy.get_param('~max_rgb_depth_evidence_delta_sec'))):
            depth_path = self.depth_dir / (Path(filename).stem + '.npy')
            np.save(str(depth_path), self.latest_depth)
            depth_relative = str(depth_path.relative_to(self.output_dir).as_posix())
        self.last_image_save_sec = stamp_sec
        if not event_frame:
            self.context_image_count += 1
        return (
            str(raw_image_path.relative_to(self.output_dir).as_posix()),
            str(image_path.relative_to(self.output_dir).as_posix()),
            depth_relative,
        )

    def close(self):
        """停止时写汇总、失败原因、测试表和可选的最终结果副本。"""
        if self.closed:
            return
        self.closed = True
        summary = build_dynamic_summary(self.records)
        failure_reasons = _failure_reasons(
            self.records, self.trajectory_sample_count, self.evidence_contract,
            self.mission_completed,
        )
        result_path = Path(str(rospy.get_param('~result_json_path')).strip()).expanduser()
        if str(result_path) and result_path.is_file():
            shutil.copy2(str(result_path), str(self.output_dir / 'detected_danger.json'))
        elif self.evidence_contract.get('formal_evidence_eligible', False):
            failure_reasons.append('official_result_json_not_available_at_recorder_shutdown')
        _write_json(self.output_dir / 'failure_reasons.json', {
            'observed_failure_reasons': sorted(set(failure_reasons)),
            'generated_at_unix_sec': round(time.time(), 3),
            'note': '只记录本次采集可观察到的失败信号；不根据真值推测漏检或虚警。',
        })
        summary.update({
            'scenario': str(rospy.get_param('~scenario_name')),
            'generated_at_unix_sec': round(time.time(), 3),
            'record_file': 'frames.jsonl',
            'trajectory_file': 'trajectory.jsonl' if self.trajectory_sample_count else '',
            'trajectory_sample_count': self.trajectory_sample_count,
            'run_manifest_file': 'run_manifest.json',
            'failure_reasons_file': 'failure_reasons.json',
            'mission_completed': self.mission_completed,
            'mission_finished_stamp_sec': self.mission_finished_stamp_sec,
            'evidence_contract': self.evidence_contract,
        })
        _write_json(self.output_dir / 'summary.json', summary)
        row = build_dynamic_testing_record(summary, str(rospy.get_param('~scenario_name')))
        _write_json(self.test_record_dir / 'testing_record_perception.json', row)
        _write_csv(self.test_record_dir / 'testing_record_perception.csv', row)


def _required_directory(parameter):
    raw = str(rospy.get_param(parameter, '')).strip()
    if not raw:
        raise rospy.ROSInitException('%s 必须显式指定，避免写入未知目录。' % parameter)
    path = Path(raw).expanduser().resolve()
    path.mkdir(parents=True, exist_ok=True)
    return path


def _image_to_bgr(message):
    if message.encoding.lower() not in ('rgb8', 'bgr8') or message.step < message.width * 3:
        return None
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < message.step * message.height:
        return None
    image = raw[:message.step * message.height].reshape((message.height, message.step))
    image = image[:, :message.width * 3].reshape((message.height, message.width, 3)).copy()
    return image[:, :, ::-1] if message.encoding.lower() == 'rgb8' else image


def _depth_to_meters(message):
    if message.encoding.upper() == '16UC1':
        dtype, scale = np.dtype('<u2'), 0.001
    elif message.encoding.upper() == '32FC1':
        dtype, scale = np.dtype('<f4'), 1.0
    else:
        return None
    if message.step < message.width * dtype.itemsize:
        return None
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < message.step * message.height:
        return None
    rows = raw[:message.step * message.height].reshape((message.height, message.step))
    packed = rows[:, :message.width * dtype.itemsize].copy()
    return packed.reshape((message.height, message.width * dtype.itemsize)).view(dtype).reshape(
        (message.height, message.width)).astype(np.float32, copy=True) * scale


def _stamp_to_sec(stamp):
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def _payload_stamp(payload, detections, fallback):
    if isinstance(payload, dict) and 'stamp_sec' in payload:
        try:
            return float(payload['stamp_sec'])
        except (TypeError, ValueError):
            pass
    if not detections:
        return float(fallback)
    first = detections[0]
    if 'stamp_sec' in first:
        return float(first['stamp_sec'])
    stamp = first.get('stamp', {})
    if isinstance(stamp, dict) and 'sec' in stamp:
        return float(stamp.get('sec', 0.0)) + float(stamp.get('nanosec', 0.0)) * 1e-9
    return float(fallback)


def _failure_reasons(records, trajectory_count, contract, mission_completed):
    reasons = []
    if not records:
        reasons.append('no_detection_messages_recorded')
    if not trajectory_count:
        reasons.append('no_legal_slam_pose_samples_recorded')
    if not contract.get('formal_evidence_eligible', False):
        reasons.extend(contract.get('contract_violations', []))
    if not mission_completed:
        reasons.append('mission_not_completed')
    if records and not any(item.get('status') == 'confirmed'
                           for record in records for item in record.get('hazards', [])):
        reasons.append('no_confirmed_red_ball_recorded')
    return reasons


def _write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def _append_json_line(path, value):
    with path.open('a', encoding='utf-8') as handle:
        handle.write(json.dumps(value, ensure_ascii=False) + '\n')


def _write_csv(path, value):
    row = dict(value)
    row['view_action_counts'] = json.dumps(row.get('view_action_counts', {}), ensure_ascii=False)
    with path.open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
        writer.writeheader()
        writer.writerow(row)


def main():
    recorder = OfficialRos1EvidenceRecorder()
    rospy.spin()
    recorder.close()


if __name__ == '__main__':
    main()
