import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionStateMachineNode(Node):
    """Minimal mission state collector and result writer."""

    def __init__(self):
        super().__init__('mission_state_machine_node')
        self.declare_parameter('result_dir', 'reports/run_results')
        self.declare_parameter('mission_id', 'minimal_demo')

        self.nav_state = 'IDLE'
        self.hazards = {}
        self.finished = False
        self.start_time = self.get_clock().now()

        self.nav_sub = self.create_subscription(String, '/hw/nav/state', self.on_nav_state, 10)
        self.hazard_sub = self.create_subscription(
            String, '/hw/perception/hazard_detections', self.on_hazards, 10
        )
        self.state_pub = self.create_publisher(String, '/hw/mission/state', 10)
        self.result_pub = self.create_publisher(String, '/hw/mission/result', 10)
        self.timer = self.create_timer(0.5, self.on_timer)
        self.get_logger().info('Mission state machine started.')

    def on_nav_state(self, msg: String):
        self.nav_state = msg.data

    def on_hazards(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Received invalid hazard JSON.', throttle_duration_sec=5.0)
            return

        for hazard in payload.get('hazards', []):
            hazard_id = str(hazard.get('id', len(self.hazards) + 1))
            self.hazards[hazard_id] = hazard

    def on_timer(self):
        state = String()
        state.data = self.nav_state
        self.state_pub.publish(state)

        if self.nav_state == 'FINISHED' and not self.finished:
            self.finished = True
            result = self.build_result()
            self.write_result(result)
            msg = String()
            msg.data = json.dumps(result, ensure_ascii=False)
            self.result_pub.publish(msg)
            self.get_logger().info('Mission result written.')

    def build_result(self):
        now = self.get_clock().now()
        duration = (now - self.start_time).nanoseconds / 1e9
        hazards = list(self.hazards.values())
        for hazard in hazards:
            hazard['status'] = 'confirmed'

        return {
            'mission_id': self.get_parameter('mission_id').value,
            'status': 'FINISHED',
            'hazards': hazards,
            'metrics': {
                'duration_sec': duration,
                'return_success': True,
                'num_confirmed_hazards': len(hazards),
            },
        }

    def write_result(self, result):
        repo_root = os.environ.get('HAZARDWALKER_ROOT', os.getcwd())
        result_dir = self.get_parameter('result_dir').value
        output_dir = result_dir if os.path.isabs(result_dir) else os.path.join(repo_root, result_dir)
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(output_dir, f'{timestamp}_result.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(result, f, ensure_ascii=False, indent=2)


def main():
    rclpy.init()
    node = MissionStateMachineNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

