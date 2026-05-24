"""任务状态机与结果写入 ROS 节点。

所属组：决策组。
文件作用：
- 订阅导航状态和危险源候选。
- 在任务结束时调用结果构建函数并写出 JSON。
- 对外发布任务状态和最终结果话题。

当前职责：
- 只做最小闭环的状态转发和结果聚合。
- 把重复危险源按 id 做最基础去重。
- 在收到 `FINISHED` 时生成结果文件。

后续扩展方式：
- 将来若补完整 FSM，可在这里增加 `EXPLORING`、`REOBSERVING`、`REPLANNING`、`RETURNING` 等状态转移。
- 若需要更可靠去重，应把 `self.hazards` 从简单字典改成基于空间距离、时间戳和观测次数的 track 管理。
- 若结果结构变更，优先改 `result_builder.py`，这里只负责调用。

验证方式：
- 先用 fake nav 和 perception 输出验证能生成结果文件。
- 再在最小 demo 中检查 `/hw/mission/state`、`/hw/mission/result` 和 JSON 文件是否同步。
"""
import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hazardwalker_decision.result_builder import build_mission_result


class MissionStateMachineNode(Node):
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
        # 生成与当前文档约定一致的结果结构。
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
