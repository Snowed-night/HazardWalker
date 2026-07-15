"""仅使用激光扫描与 IMU 的二维增量定位纯函数。

用于官方 SimEnv 的感知定位链路：不读取 Gazebo 里程计、ground truth 或场景布局，
只把公开 LaserScan 端点逐帧对齐到本进程构建的局部占据点图。它提供 ``start``
坐标系下的机体位姿，供 RGB-D 红球反投影使用；不承担导航、地图探索或控制职责。
"""

from dataclasses import dataclass
import math


@dataclass
class Pose2D:
    """start 坐标系内的平面位姿。"""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class ScanMatchResult:
    """一次扫描更新的位姿与可审计质量字段。"""

    pose: Pose2D
    status: str
    matched_endpoint_count: int
    score: float


@dataclass
class ScanImuLocalizerConfig:
    """相关扫描匹配配置；默认值优先保证小范围稳定移动时可解释。"""

    occupancy_resolution_m: float = 0.08
    search_radius_m: float = 0.60
    search_step_m: float = 0.05
    min_range_m: float = 0.15
    max_range_m: float = 18.0
    endpoint_stride: int = 4
    min_match_count: int = 12
    max_map_points: int = 12000
    # official A1 中激光相对 base 的公开静态外参；可按平台标定显式覆盖。
    laser_offset_x_m: float = 0.20
    laser_offset_y_m: float = 0.0


class ScanImuLocalizer:
    """使用 IMU 固定朝向、用局部占据端点图估计平移的轻量定位器。"""

    def __init__(self, config=None):
        self.config = config or ScanImuLocalizerConfig()
        self.pose = Pose2D()
        self._initial_imu_yaw = None
        self._occupancy = set()
        self._map_points = []

    def update_scan(self, ranges, angle_min, angle_increment, imu_yaw_rad):
        """用一帧 LaserScan 与最新 IMU 朝向更新 start→base 位姿。"""

        points = scan_ranges_to_points(
            ranges, angle_min, angle_increment,
            self.config.min_range_m, self.config.max_range_m, self.config.endpoint_stride,
        )
        return self.update_points(points, imu_yaw_rad)

    def update_points(self, laser_points, imu_yaw_rad):
        """允许测试或点云前端直接输入 laser_link 坐标系二维端点。"""

        if self._initial_imu_yaw is None:
            self._initial_imu_yaw = float(imu_yaw_rad)
        yaw = normalize_angle(float(imu_yaw_rad) - self._initial_imu_yaw)
        base_points = _laser_to_base_points(laser_points, self.config)
        if not base_points:
            return ScanMatchResult(self.pose, 'no_valid_scan_points', 0, 0.0)

        if not self._occupancy:
            self.pose = Pose2D(0.0, 0.0, yaw)
            self._integrate_points(base_points, self.pose)
            return ScanMatchResult(self.pose, 'initialized', 0, 0.0)

        best_pose, best_count = self._search_translation(base_points, yaw)
        if best_count < self.config.min_match_count:
            # 朝向仍由 IMU 更新；平移保持上一帧，避免低纹理/开阔区把错误相关写入地图。
            self.pose = Pose2D(self.pose.x, self.pose.y, yaw)
            return ScanMatchResult(self.pose, 'weak_scan_match', best_count,
                                   best_count / float(max(1, len(base_points))))

        self.pose = best_pose
        self._integrate_points(base_points, self.pose)
        return ScanMatchResult(self.pose, 'tracking', best_count,
                               best_count / float(max(1, len(base_points))))

    def _search_translation(self, base_points, yaw):
        best_pose = Pose2D(self.pose.x, self.pose.y, yaw)
        best_count = -1
        radius_steps = int(round(self.config.search_radius_m / self.config.search_step_m))
        for dx_step in range(-radius_steps, radius_steps + 1):
            for dy_step in range(-radius_steps, radius_steps + 1):
                candidate = Pose2D(
                    self.pose.x + dx_step * self.config.search_step_m,
                    self.pose.y + dy_step * self.config.search_step_m,
                    yaw,
                )
                count = self._occupancy_score(base_points, candidate)
                if count > best_count:
                    best_pose, best_count = candidate, count
        return best_pose, best_count

    def _occupancy_score(self, points, pose):
        count = 0
        cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
        for point_x, point_y in points:
            world_x = pose.x + cosine * point_x - sine * point_y
            world_y = pose.y + sine * point_x + cosine * point_y
            cell_x, cell_y = _cell(world_x, world_y, self.config.occupancy_resolution_m)
            # 一格邻域抵抗公开传感器离散化和四足机体小幅振动。
            if any((cell_x + offset_x, cell_y + offset_y) in self._occupancy
                   for offset_x in (-1, 0, 1) for offset_y in (-1, 0, 1)):
                count += 1
        return count

    def _integrate_points(self, points, pose):
        cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
        for point_x, point_y in points:
            world_x = pose.x + cosine * point_x - sine * point_y
            world_y = pose.y + sine * point_x + cosine * point_y
            self._map_points.append((world_x, world_y))
            self._occupancy.add(_cell(world_x, world_y, self.config.occupancy_resolution_m))
        if len(self._map_points) > self.config.max_map_points:
            self._map_points = self._map_points[-self.config.max_map_points:]
            self._occupancy = {
                _cell(x, y, self.config.occupancy_resolution_m)
                for x, y in self._map_points
            }


def scan_ranges_to_points(ranges, angle_min, angle_increment, min_range_m, max_range_m, stride):
    """将 LaserScan 有效量测转为稀疏二维端点，过滤 NaN、无穷远和自车近距点。"""

    points = []
    step = max(1, int(stride))
    for index in range(0, len(ranges), step):
        distance = float(ranges[index])
        if not math.isfinite(distance) or distance < min_range_m or distance > max_range_m:
            continue
        angle = float(angle_min) + index * float(angle_increment)
        points.append((distance * math.cos(angle), distance * math.sin(angle)))
    return points


def quaternion_to_yaw(x, y, z, w):
    """从公开 IMU 四元数计算偏航角。"""

    numerator = 2.0 * (float(w) * float(z) + float(x) * float(y))
    denominator = 1.0 - 2.0 * (float(y) * float(y) + float(z) * float(z))
    return math.atan2(numerator, denominator)


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _laser_to_base_points(points, config):
    return [
        (float(x) + config.laser_offset_x_m, float(y) + config.laser_offset_y_m)
        for x, y in points
    ]


def _cell(x, y, resolution):
    return (
        int(math.floor(float(x) / float(resolution))),
        int(math.floor(float(y) / float(resolution))),
    )
