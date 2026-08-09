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
    laser_offset_z_m: float = 0.08
    # 官方 Livox Mid-360 相对 base 绕 Y 轴上仰 45°。PointCloud2 必须先按
    # 公开外参转换到 base，不能把传感器局部 x/y 直接冒充水平平面。
    laser_pitch_rad: float = 0.785
    # 排除随机器人运动的近地面点和过高顶棚点，保留墙面、家具等稳定配准端点。
    min_endpoint_z_m: float = -0.25
    max_endpoint_z_m: float = 1.50
    # IMU 已锁定旋转后，用相邻帧点集 ICP 只估计平移。相邻帧比累计占据图
    # 更不易在长直墙上发生“沿墙滑移”；历史图只在 ICP 证据不足时回退。
    icp_max_correspondence_m: float = 0.45
    icp_iteration_count: int = 4
    icp_trim_fraction: float = 0.70
    # cmd_vel 只作为 ICP 初值，不是位移证据。最终增量必须由扫描匹配支持，
    # 并限制单帧跳变量；否则机器人倒地、打滑或受阻时会产生“命令发出即移动”
    # 的虚假里程计，进而污染覆盖率和危险源三维位置。
    scan_correction_gain: float = 1.0
    max_scan_correction_m: float = 0.25


class ScanImuLocalizer:
    """使用 IMU 固定朝向、用局部占据端点图估计平移的轻量定位器。"""

    def __init__(self, config=None):
        self.config = config or ScanImuLocalizerConfig()
        self.pose = Pose2D()
        self._initial_imu_yaw = None
        self._occupancy = set()
        self._map_points = []
        self._previous_world_points = []

    def reset_matching_map(self):
        """换层后保留平面位姿，但清空只属于旧楼层的扫描匹配地图。

        官方楼层平面结构可能高度相似。若电梯或楼梯到层后仍拿上一层端点做 ICP，
        定位器会把不同楼层错误拼成同一张二维地图。楼层编号由公开动作状态提供；
        本方法只隔离各层匹配历史，不读取场景布局或真值。
        """

        self._occupancy.clear()
        self._map_points = []
        self._previous_world_points = []

    def update_scan(
            self, ranges, angle_min, angle_increment, imu_yaw_rad,
            motion_prior_base=(0.0, 0.0), allow_translation_update=True):
        """用一帧 LaserScan 与最新 IMU 朝向更新 start→base 位姿。"""

        points = scan_ranges_to_points(
            ranges, angle_min, angle_increment,
            self.config.min_range_m, self.config.max_range_m, self.config.endpoint_stride,
        )
        return self.update_points(
            points,
            imu_yaw_rad,
            motion_prior_base,
            allow_translation_update=allow_translation_update,
        )

    def update_points(
            self, laser_points, imu_yaw_rad, motion_prior_base=(0.0, 0.0),
            allow_translation_update=True):
        """允许测试或点云前端直接输入 laser_link 坐标系二维端点。"""

        base_points = _laser_to_base_points(laser_points, self.config)
        return self.update_base_points(
            base_points,
            imu_yaw_rad,
            motion_prior_base,
            allow_translation_update=allow_translation_update,
        )

    def update_base_points(
            self, base_points, imu_yaw_rad, motion_prior_base=(0.0, 0.0),
            allow_translation_update=True):
        """使用已经按公开外参转换到 base 坐标系的二维端点。"""

        if self._initial_imu_yaw is None:
            self._initial_imu_yaw = float(imu_yaw_rad)
        yaw = normalize_angle(float(imu_yaw_rad) - self._initial_imu_yaw)
        base_points = list(base_points)
        if not base_points:
            return ScanMatchResult(self.pose, 'no_valid_scan_points', 0, 0.0)

        if not self._occupancy:
            self.pose = Pose2D(0.0, 0.0, yaw)
            self._integrate_points(base_points, self.pose)
            self._previous_world_points = _transform_planar_points(base_points, self.pose)
            return ScanMatchResult(self.pose, 'initialized', 0, 0.0)

        try:
            prior_forward = float(motion_prior_base[0])
            prior_left = float(motion_prior_base[1])
        except (IndexError, TypeError, ValueError):
            prior_forward = 0.0
            prior_left = 0.0
        if not math.isfinite(prior_forward) or not math.isfinite(prior_left):
            prior_forward = 0.0
            prior_left = 0.0
        predicted_pose = Pose2D(
            self.pose.x + math.cos(yaw) * prior_forward
            - math.sin(yaw) * prior_left,
            self.pose.y + math.sin(yaw) * prior_forward
            + math.cos(yaw) * prior_left,
            yaw,
        )

        if not bool(allow_translation_update):
            # 四足机体在固定站立时仍有激光振动和姿态微摆。没有新鲜线速度请求时
            # 这些变化不能解释为平移；只更新公开 IMU 给出的 yaw，并用当前扫描
            # 刷新相邻帧参考，避免停车越久里程越远。
            self.pose = Pose2D(self.pose.x, self.pose.y, yaw)
            self._integrate_points(base_points, self.pose)
            self._previous_world_points = _transform_planar_points(
                base_points, self.pose)
            return ScanMatchResult(
                self.pose,
                'stationary_command_hold',
                len(base_points),
                1.0,
            )

        best_pose, best_count = self._match_translation_icp(
            base_points, yaw, predicted_pose)
        if best_count < self.config.min_match_count:
            best_pose, best_count = self._search_translation(
                base_points, yaw, predicted_pose)
        if best_count < self.config.min_match_count:
            # 命令只表示“期望运动”，不能证明机体真的移动。弱纹理、倒地、打滑
            # 或受阻时保持上一平移并提高协方差，避免伪造覆盖和目标位置。
            self.pose = Pose2D(self.pose.x, self.pose.y, yaw)
            return ScanMatchResult(self.pose, 'insufficient_scan_evidence', best_count,
                                   best_count / float(max(1, len(base_points))))

        self.pose = _bound_translation_correction(
            best_pose,
            Pose2D(self.pose.x, self.pose.y, yaw),
            gain=self.config.scan_correction_gain,
            max_correction_m=self.config.max_scan_correction_m,
        )
        self._integrate_points(base_points, self.pose)
        self._previous_world_points = _transform_planar_points(base_points, self.pose)
        return ScanMatchResult(self.pose, 'tracking', best_count,
                               best_count / float(max(1, len(base_points))))

    def _match_translation_icp(self, base_points, yaw, predicted_pose=None):
        """在 IMU 给定 yaw 下，以鲁棒相邻帧 ICP 仅估计 x/y 平移。"""

        if not self._previous_world_points:
            return Pose2D(self.pose.x, self.pose.y, yaw), 0
        candidate = predicted_pose or Pose2D(self.pose.x, self.pose.y, yaw)
        matched_count = 0
        iteration_count = max(1, int(self.config.icp_iteration_count))
        trim_fraction = min(1.0, max(0.20, float(self.config.icp_trim_fraction)))
        max_distance_sq = float(self.config.icp_max_correspondence_m) ** 2
        for _iteration in range(iteration_count):
            current_world = _transform_planar_points(base_points, candidate)
            residuals = []
            for current_x, current_y in current_world:
                nearest = None
                nearest_distance_sq = float('inf')
                for previous_x, previous_y in self._previous_world_points:
                    distance_sq = (
                        (previous_x - current_x) ** 2
                        + (previous_y - current_y) ** 2
                    )
                    if distance_sq < nearest_distance_sq:
                        nearest = (previous_x, previous_y)
                        nearest_distance_sq = distance_sq
                if nearest is not None and nearest_distance_sq <= max_distance_sq:
                    residuals.append((
                        nearest_distance_sq,
                        nearest[0] - current_x,
                        nearest[1] - current_y,
                    ))
            if not residuals:
                return candidate, 0
            residuals.sort(key=lambda item: item[0])
            keep_count = max(1, int(math.ceil(len(residuals) * trim_fraction)))
            trimmed = residuals[:keep_count]
            matched_count = len(trimmed)
            delta_x = _median([item[1] for item in trimmed])
            delta_y = _median([item[2] for item in trimmed])
            candidate = Pose2D(
                candidate.x + delta_x,
                candidate.y + delta_y,
                yaw,
            )
            if math.hypot(delta_x, delta_y) < 0.002:
                break
        return candidate, matched_count

    def _search_translation(self, base_points, yaw, predicted_pose=None):
        # 离散回退搜索必须围绕上一条已证实位姿。predicted_pose 只供 ICP 初值；
        # 若围绕命令积分中心搜索，退化走廊中的同分候选会再次把命令伪装成位移。
        center = Pose2D(self.pose.x, self.pose.y, yaw)
        best_pose = Pose2D(center.x, center.y, yaw)
        best_count = -1
        best_motion_sq = float('inf')
        radius_steps = int(round(self.config.search_radius_m / self.config.search_step_m))
        for dx_step in range(-radius_steps, radius_steps + 1):
            for dy_step in range(-radius_steps, radius_steps + 1):
                candidate = Pose2D(
                    center.x + dx_step * self.config.search_step_m,
                    center.y + dy_step * self.config.search_step_m,
                    yaw,
                )
                count = self._occupancy_score(base_points, candidate)
                # 走廊、平墙和体素邻域常产生多个同分解。若只保留遍历到的
                # 第一个候选，静止机器人也会每帧向搜索窗口左下角漂移。
                # 同分时选择相对上一位姿运动最小的解，符合连续运动先验。
                motion_sq = (
                    (candidate.x - center.x) ** 2
                    + (candidate.y - center.y) ** 2
                )
                if (count > best_count
                        or (count == best_count and motion_sq < best_motion_sq)):
                    best_pose, best_count = candidate, count
                    best_motion_sq = motion_sq
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


def quaternion_upright_cosine(x, y, z, w):
    """返回机体 z 轴与世界 z 轴夹角的余弦，用于识别明显倒地。

    输入来自公开 trunk IMU。返回值接近 1 表示直立，低于 0.5 表示倾斜超过
    60°；该检查不读取 Gazebo 姿态真值。
    """

    values = [float(x), float(y), float(z), float(w)]
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm <= 1e-12:
        return float('-inf')
    qx, qy, _qz, _qw = (value / norm for value in values)
    return 1.0 - 2.0 * (qx * qx + qy * qy)


def floor_index_to_elevation(floor_index, floor_height_m=2.6,
                             min_floor_index=0, max_floor_index=31):
    """把公开动作链确认的楼层编号转换为相对起点高度。

    官方随机楼栋生成器公开固定层高为 2.6 m。该函数只接受整数且限制合理范围，
    避免损坏的导航消息把危险源写到异常高度；它不读取 layout、manifest 或真值。
    """

    if isinstance(floor_index, bool):
        raise ValueError('floor_index 必须是整数，不能是布尔值。')
    try:
        value = int(floor_index)
    except (TypeError, ValueError):
        raise ValueError('floor_index 必须是整数。')
    if value != floor_index:
        raise ValueError('floor_index 必须是整数。')
    if value < int(min_floor_index) or value > int(max_floor_index):
        raise ValueError('floor_index 超出允许范围。')
    height = float(floor_height_m)
    if not math.isfinite(height) or height <= 0.0:
        raise ValueError('floor_height_m 必须是正有限数。')
    return value * height


def point_cloud_xyz_to_base_points(points_xyz, config):
    """把官方 Livox PointCloud2 的 xyz 转为可用于二维匹配的 base 平面端点。

    输入只来自公开点云。转换使用文档公开的 ``base -> laser_livox`` 静态外参，
    并按 base 高度过滤地面/顶棚；不读取 Gazebo TF、场景布局或真值。
    """

    cosine, sine = math.cos(config.laser_pitch_rad), math.sin(config.laser_pitch_rad)
    result = []
    stride = max(1, int(config.endpoint_stride))
    for index, point in enumerate(points_xyz):
        if index % stride:
            continue
        try:
            laser_x, laser_y, laser_z = (
                float(point[0]), float(point[1]), float(point[2]),
            )
        except (IndexError, TypeError, ValueError):
            continue
        if not all(math.isfinite(value) for value in (laser_x, laser_y, laser_z)):
            continue
        range_m = math.sqrt(laser_x * laser_x + laser_y * laser_y + laser_z * laser_z)
        if range_m < config.min_range_m or range_m > config.max_range_m:
            continue
        base_x = (
            cosine * laser_x + sine * laser_z + config.laser_offset_x_m
        )
        base_y = laser_y + config.laser_offset_y_m
        base_z = (
            -sine * laser_x + cosine * laser_z + config.laser_offset_z_m
        )
        if base_z < config.min_endpoint_z_m or base_z > config.max_endpoint_z_m:
            continue
        result.append((base_x, base_y))
    return result


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _laser_to_base_points(points, config):
    return [
        (float(x) + config.laser_offset_x_m, float(y) + config.laser_offset_y_m)
        for x, y in points
    ]


def _transform_planar_points(points, pose):
    """把 base 平面点按给定位姿转换到 start 坐标。"""

    cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
    return [
        (
            pose.x + cosine * float(x) - sine * float(y),
            pose.y + sine * float(x) + cosine * float(y),
        )
        for x, y in points
    ]


def _median(values):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return 0.0
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def _bound_translation_correction(candidate, predicted, gain, max_correction_m):
    """限制扫描匹配相对上一条已证实位姿的单帧增量。

    参数名 ``predicted`` 为兼容既有调用保留，当前传入的是上一位姿。控制命令
    只参与 ICP 初值，不进入这里的最终平移，因此受阻或倒地不会凭空累计里程。
    """

    delta_x = float(candidate.x) - float(predicted.x)
    delta_y = float(candidate.y) - float(predicted.y)
    distance = math.hypot(delta_x, delta_y)
    safe_gain = min(1.0, max(0.0, float(gain)))
    safe_limit = max(0.0, float(max_correction_m))
    if distance <= 1e-12 or safe_gain <= 0.0 or safe_limit <= 0.0:
        return Pose2D(predicted.x, predicted.y, candidate.yaw)
    correction = min(distance * safe_gain, safe_limit)
    return Pose2D(
        predicted.x + delta_x / distance * correction,
        predicted.y + delta_y / distance * correction,
        candidate.yaw,
    )


def _cell(x, y, resolution):
    return (
        int(math.floor(float(x) / float(resolution))),
        int(math.floor(float(y) / float(resolution))),
    )
