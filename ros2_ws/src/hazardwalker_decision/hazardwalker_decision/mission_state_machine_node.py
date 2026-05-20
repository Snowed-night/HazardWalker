import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hazardwalker_decision.result_builder import build_mission_result


class MissionStateMachineNode(Node):
    """第一阶段任务状态机和结果写入节点。

    当前实现是“最小闭环版”，主要负责：
    1. 订阅导航状态 `/hw/nav/state`；
    2. 收集感知模块发布的危险源候选；
    3. 在导航结束时生成 result.json；
    4. 对外发布 `/hw/mission/state` 和 `/hw/mission/result`。

    后续真正的决策逻辑会在这里扩展，包括 EXPLORING、REOBSERVING、
    REPLANNING、RETURNING、FAILED 等状态转移。
    """

    def __init__(self):
        super().__init__('mission_state_machine_node')
        self.declare_parameter('result_dir', 'reports/run_results')
        self.declare_parameter('mission_id', 'minimal_demo')

        # nav_state 保存导航组当前状态；hazards 用字典按 id 去重保存候选危险源。
        # 当前版本只做简单覆盖，后续要替换为多帧确认和状态管理。
        self.nav_state = 'IDLE'
        self.hazards = {}
        self.finished = False
        self.start_time = self.get_clock().now()

        # 导航状态来自导航组；危险源 JSON 来自感知组。
        self.nav_sub = self.create_subscription(String, '/hw/nav/state', self.on_nav_state, 10)
        self.hazard_sub = self.create_subscription(
            String, '/hw/perception/hazard_detections', self.on_hazards, 10
        )
        # mission/state 给其他模块和测试组观察；mission/result 用于发布最终结果 JSON。
        self.state_pub = self.create_publisher(String, '/hw/mission/state', 10)
        self.result_pub = self.create_publisher(String, '/hw/mission/result', 10)
        # 2Hz 状态循环。正式版本可以改成事件驱动或更完整的 FSM。
        self.timer = self.create_timer(0.5, self.on_timer)
        self.get_logger().info('Mission state machine started.')

    def on_nav_state(self, msg: String):
        # 当前直接信任导航模块发布的状态。后续可以在这里加入合法状态检查。
        self.nav_state = msg.data

    def on_hazards(self, msg: String):
        # 第一阶段感知结果用 String(JSON) 传递，方便快速集成。
        # 稳定后建议迁移到 hazardwalker_msgs/HazardArray。
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Received invalid hazard JSON.', throttle_duration_sec=5.0)
            return

        for hazard in payload.get('hazards', []):
            # 用 id 作为 key 做最简单的去重。后续应根据空间距离和观测次数融合。
            hazard_id = str(hazard.get('id', len(self.hazards) + 1))
            self.hazards[hazard_id] = hazard

    def on_timer(self):
        # 将导航状态转发为任务状态。当前最小版暂时没有独立决策逻辑。
        state = String()
        state.data = self.nav_state
        self.state_pub.publish(state)

        if self.nav_state == 'FINISHED' and not self.finished:
            # 防止重复写文件：第一次看到 FINISHED 时生成一次结果。
            self.finished = True
            result = self.build_result()
            self.write_result(result)
            msg = String()
            msg.data = json.dumps(result, ensure_ascii=False)
            self.result_pub.publish(msg)
            self.get_logger().info('Mission result written.')

    def build_result(self):
        # 生成与 docs/interface_spec.md 对齐的结果结构。
        now = self.get_clock().now()
        duration = (now - self.start_time).nanoseconds / 1e9
        return build_mission_result(
            mission_id=self.get_parameter('mission_id').value,
            status='FINISHED',
            hazards=list(self.hazards.values()),
            duration_sec=duration,
            return_success=True,
        )

    def write_result(self, result):
        # HAZARDWALKER_ROOT 由 scripts/run_minimal_demo.sh 设置。
        # 如果没有设置，就退回到当前工作目录，方便手动 ros2 run 调试。
        repo_root = os.environ.get('HAZARDWALKER_ROOT', os.getcwd())
        result_dir = self.get_parameter('result_dir').value
        output_dir = result_dir if os.path.isabs(result_dir) else os.path.join(repo_root, result_dir)
        os.makedirs(output_dir, exist_ok=True)

        # 每次运行生成一个带时间戳的 JSON，避免覆盖历史实验结果。
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
