"""用户确认触发的红球辅助对准 ROS2 执行节点。

节点订阅感知结果，只在用户调用启动服务后发布短时原地转向命令。它通过统一
控制仲裁器取得和归还控制权，不执行靠近、横移或避障；这些动作继续由导航层
消费感知复查建议后完成。
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .assist_alignment import (
    AlignmentConfig,
    AlignmentDecision,
    compute_alignment_decision,
    evaluate_control_takeover,
    validate_alignment_config,
)


class AssistAlignmentNode(Node):
    """辅助对准状态机：人工触发、限时转向、稳定居中后归还控制。"""

    def __init__(self) -> None:
        super().__init__('hazardwalker_assist_alignment')
        self.declare_parameter(
            'detection_topic', '/hw/perception/hazard_detections')
        self.declare_parameter('command_topic', '/hw/control/assist_cmd_vel')
        self.declare_parameter('mode_request_topic', '/hw/control/mode_request')
        self.declare_parameter('control_status_topic', '/hw/control/status')
        self.declare_parameter('status_topic', '/hw/control/assist_status')
        self.declare_parameter('fallback_mode', 'keyboard')
        self.declare_parameter('center_tolerance_ratio', 0.08)
        self.declare_parameter('angular_kp', 1.2)
        self.declare_parameter('min_angular_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.80)
        self.declare_parameter('perception_timeout_sec', 0.50)
        self.declare_parameter('alignment_timeout_sec', 8.0)
        self.declare_parameter('control_takeover_timeout_sec', 1.5)
        self.declare_parameter('mode_request_retry_sec', 0.20)
        self.declare_parameter('centered_stable_frames', 3)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('status_heartbeat_sec', 1.0)

        self.config = AlignmentConfig(
            center_tolerance_ratio=float(
                self.get_parameter('center_tolerance_ratio').value),
            angular_kp=float(self.get_parameter('angular_kp').value),
            min_angular_speed=float(
                self.get_parameter('min_angular_speed').value),
            max_angular_speed=float(
                self.get_parameter('max_angular_speed').value),
        )
        validate_alignment_config(self.config)
        # 在接入控制话题前就拒绝非有限或无效参数，避免节点带着
        # 永不结束的超时或非法角速度进入已运行状态。
        self._validate_runtime_parameters()
        self.latest_payload = None
        self.latest_payload_monotonic_sec = 0.0
        self.active = False
        self.started_monotonic_sec = 0.0
        self.last_mode_request_monotonic_sec = 0.0
        self.centered_frames = 0
        self.last_target_id = ''
        self.current_control_mode = ''
        self.fallback_mode = str(
            self.get_parameter('fallback_mode').value).strip().lower()
        if self.fallback_mode not in ('keyboard', 'navigation', 'stopped'):
            raise ValueError(
                'fallback_mode 必须为 keyboard、navigation 或 stopped')
        self.resume_mode = self.fallback_mode

        self.command_pub = self.create_publisher(
            Twist, str(self.get_parameter('command_topic').value), 10)
        self.mode_pub = self.create_publisher(
            String, str(self.get_parameter('mode_request_topic').value), 10)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.detection_sub = self.create_subscription(
            String,
            str(self.get_parameter('detection_topic').value),
            self.on_detection,
            10,
        )
        self.control_status_sub = self.create_subscription(
            String,
            str(self.get_parameter('control_status_topic').value),
            self.on_control_status,
            10,
        )
        self.start_service = self.create_service(
            Trigger, '/hw/control/assist_align/start', self.on_start)
        self.cancel_service = self.create_service(
            Trigger, '/hw/control/assist_align/cancel', self.on_cancel)
        rate = float(self.get_parameter('control_rate_hz').value)
        self.status_heartbeat_sec = float(
            self.get_parameter('status_heartbeat_sec').value)
        self._last_status_payload = None
        self._last_status_publish_monotonic_sec = 0.0
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self._publish_status('idle', 'waiting_for_user_confirmation')

    def on_detection(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warning(
                '忽略损坏的感知 JSON。', throttle_duration_sec=2.0)
            return
        if not isinstance(payload, dict):
            return
        self.latest_payload = payload
        self.latest_payload_monotonic_sec = time.monotonic()

    def on_control_status(self, message: String) -> None:
        """记录辅助控制接管前的模式，完成后归还给原控制源。"""

        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        mode = str(payload.get('mode', '')).strip()
        if mode in ('keyboard', 'navigation', 'assist', 'stopped'):
            self.current_control_mode = mode

    def on_start(self, _request, response):
        if self.active:
            response.success = False
            response.message = f'辅助对准已在执行目标 {self.last_target_id}'
            return response
        now = time.monotonic()
        decision = self._current_decision(now)
        if not decision.valid:
            self._publish_status('idle', decision.reason, decision)
            response.success = False
            response.message = f'当前没有可对准候选：{decision.reason}'
            return response
        self.active = True
        self.started_monotonic_sec = now
        self.centered_frames = 0
        self.last_target_id = decision.target_id
        self.resume_mode = (
            self.current_control_mode
            if self.current_control_mode in ('keyboard', 'navigation')
            else self.fallback_mode
        )
        self._request_mode('assist')
        self.last_mode_request_monotonic_sec = now
        self._publish_status(
            'acquiring_control', 'waiting_for_control_takeover', decision)
        response.success = True
        response.message = f'开始辅助对准目标 {decision.target_id}'
        return response

    def on_cancel(self, _request, response):
        was_active = self.active
        if was_active:
            self._finish('cancelled_by_user')
        else:
            # 未接管控制时不能因一次无效取消请求改变当前键盘/导航模式。
            self._publish_status('idle', 'assist_not_running')
        response.success = True
        response.message = '辅助对准已取消' if was_active else '辅助对准未运行'
        return response

    def on_timer(self) -> None:
        if not self.active:
            self._republish_status_if_due()
            return
        now = time.monotonic()
        if now - self.started_monotonic_sec > float(
                self.get_parameter('alignment_timeout_sec').value):
            self._finish('alignment_timeout')
            return
        decision = self._current_decision(now)
        if not decision.valid:
            self._finish(decision.reason)
            return
        self.last_target_id = decision.target_id
        takeover = evaluate_control_takeover(
            self.current_control_mode,
            elapsed_sec=now - self.started_monotonic_sec,
            since_request_sec=now - self.last_mode_request_monotonic_sec,
            timeout_sec=float(
                self.get_parameter('control_takeover_timeout_sec').value),
            retry_sec=float(
                self.get_parameter('mode_request_retry_sec').value),
        )
        if takeover.failed:
            self._finish(takeover.reason)
            return
        if not takeover.ready:
            # 接管确认前不发布任何非零速度。可靠传输异常或订阅者刚启动时，
            # 周期重发显式模式请求，最终仍由短超时保护失败停车。
            if takeover.should_retry:
                self._request_mode('assist')
                self.last_mode_request_monotonic_sec = now
            self._publish_status(
                'acquiring_control', takeover.reason, decision)
            return
        if decision.centered:
            self.centered_frames += 1
            self.command_pub.publish(Twist())
            if self.centered_frames >= int(
                    self.get_parameter('centered_stable_frames').value):
                self._finish('target_centered')
            else:
                self._publish_status('settling', decision.reason, decision)
            return
        self.centered_frames = 0
        command = Twist()
        command.angular.z = decision.angular_z
        self.command_pub.publish(command)
        self._publish_status('aligning', decision.reason, decision)

    def _current_decision(self, now_monotonic_sec):
        if self.latest_payload is None:
            return compute_alignment_decision({}, self.config)
        if now_monotonic_sec - self.latest_payload_monotonic_sec > float(
                self.get_parameter('perception_timeout_sec').value):
            return AlignmentDecision(
                valid=False,
                centered=False,
                target_id=self.last_target_id,
                center_error_ratio=None,
                angular_z=0.0,
                reason='perception_timeout',
            )
        return compute_alignment_decision(
            self.latest_payload,
            self.config,
            target_id_override=(self.last_target_id if self.active else ''),
        )

    def _validate_runtime_parameters(self) -> None:
        """校验不属于 AlignmentConfig 的状态机参数。"""

        for name in (
                'perception_timeout_sec', 'alignment_timeout_sec',
                'control_takeover_timeout_sec', 'mode_request_retry_sec',
                'control_rate_hz', 'status_heartbeat_sec'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} 必须是有限正数')
        stable_frames = int(self.get_parameter('centered_stable_frames').value)
        if stable_frames <= 0:
            raise ValueError('centered_stable_frames 必须为正整数')

    def _finish(self, reason: str) -> None:
        self.command_pub.publish(Twist())
        self.active = False
        self.centered_frames = 0
        self._request_mode(self.resume_mode)
        self._publish_status('idle', reason)

    def _request_mode(self, mode: str) -> None:
        message = String()
        message.data = mode
        self.mode_pub.publish(message)

    def _publish_status(self, state, reason, decision=None) -> None:
        payload = {
            'state': state,
            'reason': reason,
            'active': self.active,
            'target_id': (
                decision.target_id if decision is not None
                else self.last_target_id
            ),
            'center_error_ratio': (
                decision.center_error_ratio if decision is not None else None
            ),
            'resume_mode': self.resume_mode,
        }
        self._last_status_payload = payload
        self._emit_status(payload)

    def _republish_status_if_due(self) -> None:
        """周期重发当前状态，保证晚加入的 GUI 和 rosbag 获得完整状态。"""

        if self._last_status_payload is None:
            return
        if (
            time.monotonic() - self._last_status_publish_monotonic_sec
            >= self.status_heartbeat_sec
        ):
            self._emit_status(self._last_status_payload)

    def _emit_status(self, payload) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(message)
        self._last_status_publish_monotonic_sec = time.monotonic()

    def publish_stop(self) -> None:
        if rclpy.ok(context=self.context):
            self.command_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AssistAlignmentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
