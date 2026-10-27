"""手工输入航点控制运行记录。

用法：从仓库根目录运行
    python ros2_ws/src/hazardwalker_nav/scripts/manual_waypoint_test.py
输出：markdown 格式的运行记录表，写入 reports/nav/。
"""

import math
import os
import sys

# 脚本位置: ros2_ws/src/hazardwalker_nav/scripts/
# 往上 4 层到仓库根
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
# 把 hazardwalker_nav 包所在目录加入 path
sys.path.insert(0, os.path.join(REPO_ROOT, 'ros2_ws', 'src', 'hazardwalker_nav'))

from hazardwalker_nav.waypoint_controller import compute_waypoint_command


TEST_CASES = [
    # (label, x, y, yaw, waypoints, goal_index, completed, description)
    (
        "原点朝向目标，前进",
        0.0, 0.0, 0.0,
        [(1.0, 0.0), (0.0, 0.0)], 0, False,
        "机器人在原点(0,0)，朝向 x 轴正方向，目标(1,0)在前方 1m",
    ),
    (
        "原点背对目标，先转向",
        0.0, 0.0, math.pi,
        [(1.0, 0.0), (0.0, 0.0)], 0, False,
        "机器人在原点，朝向 x 轴负方向，目标在前方需要先转 180°",
    ),
    (
        "原点侧对目标，需转向",
        0.0, 0.0, math.pi / 2,
        [(1.0, 0.0), (0.0, 0.0)], 0, False,
        "机器人在原点，朝 y 正方向，目标在右侧需转 90°",
    ),
    (
        "接近最后航点，FINISHED",
        0.0, 0.0, 0.0,
        [(0.0, 0.0)], 0, False,
        "机器人已经在最后一个航点(0,0)上，距离=0 → FINISHED",
    ),
    (
        "接近多个航点中最后一个",
        -0.05, 0.02, 0.1,
        [(1.0, 0.0), (2.0, 0.0), (0.0, 0.0)], 2, False,
        "最后航点是(0,0)，机器人几乎在目标上，距离 < 0.5m → FINISHED",
    ),
    (
        "已到达航点，自动切换",
        1.0, 0.0, 0.0,
        [(1.0, 0.0), (2.0, 0.0), (0.0, 0.0)], 0, False,
        "已到达第一个航点(1,0)，应自动切换到 goal_index=1 → (2,0)",
    ),
    (
        "RETURNING 状态",
        2.0, 0.0, 0.0,
        [(1.0, 0.0), (2.0, 0.0), (0.0, 0.0)], 1, False,
        "到达第二个航点(2,0)，应切换到 goal_index=2 → (0,0)，state=RETURNING",
    ),
    (
        "completed=True，直接 FINISHED",
        1.0, 1.0, 0.5,
        [(1.0, 0.0), (0.0, 0.0)], 0, True,
        "提前标记 completed=True，无论位置如何都应 FINISHED",
    ),
    (
        "空航点列表，FINISHED",
        0.0, 0.0, 0.0,
        [], 0, False,
        "waypoints 为空列表 → 立即 FINISHED",
    ),
    (
        "目标较近，接近减速",
        0.0, 0.0, 0.0,
        [(0.2, 0.0), (0.0, 0.0)], 0, False,
        "目标仅 0.2m 远，linear_x 应为 min(0.35, 0.2)=0.2，减速接近",
    ),
    (
        "heading_error 正好在阈值边界内",
        0.5, 0.0, 0.2,
        [(1.0, 0.0), (0.0, 0.0)], 0, False,
        "朝向误差约 0.2rad < 0.25rad，允许前进",
    ),
]


def main():
    out_path = os.path.join(REPO_ROOT, 'reports', 'nav', 'manual_waypoint_test_record.md')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    lines = []
    lines.append("# compute_waypoint_command 手工运行记录\n")
    lines.append("> 生成时间：2026-07-04 | 参数：linear_speed=0.35, angular_speed=0.8, "
                 "goal_tolerance_m=0.5, heading_tolerance_rad=0.25\n")

    lines.append("| # | 场景 | 当前位置 (x,y,yaw) | 航点列表 | g_idx |"
                 " linear_x | angular_z | state | g_idx(新) | completed | 说明 |")
    lines.append("|---|------|--------------------|----------|-------|"
                 "----------|-----------|-------|-----------|-----------|------|")

    for i, (label, x, y, yaw, wpts, gidx, completed, desc) in enumerate(TEST_CASES, 1):
        result = compute_waypoint_command(
            x=x, y=y, yaw=yaw,
            waypoints=wpts,
            goal_index=gidx,
            completed=completed,
        )
        yaw_deg = round(math.degrees(yaw) % 360, 1)
        wpts_str = str(wpts).replace(" ", "")
        lines.append(
            f"| {i} | {label} | ({x},{y}, {yaw_deg}°) | {wpts_str} | {gidx} |"
            f" {result.linear_x:.3f} | {result.angular_z:+.3f} |"
            f" `{result.state}` | {result.goal_index} |"
            f" {result.completed} | {desc} |"
        )

    lines.append("\n---\n")
    lines.append("## 关键结论\n")
    lines.append("1. **先转后走**：|heading_error| > 0.25rad 时 linear_x=0，确保先对准方向。")
    lines.append("2. **到达切换**：距离 <= 0.5m 时 goal_index 自动 +1，不需要外部干预。")
    lines.append("3. **接近减速**：linear_x = min(0.35, distance)，防止冲过目标。")
    lines.append("4. **RETURNING 判定**：goal_index == len(waypoints)-1 时为 RETURNING，表示最后一个航点（返航）。")
    lines.append("5. **FINISHED 条件**：completed=True 或 waypoints 为空 或 goal_index 超出范围。")
    lines.append("6. **角速度饱和**：angular_z 被 clamp 到 [-0.8, +0.8] rad/s。")

    content = "\n".join(lines)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(content)
    print(f"Written to {out_path}")
    print(content)


if __name__ == '__main__':
    main()
