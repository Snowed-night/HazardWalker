#!/usr/bin/env python3
"""已停用的旧 JSON 管道桥接器。

负责人：姜晨。该实现无法完整转发 RGB/深度 Image.data，且 /hw/cmd_vel 曾是空回调；官方
SimEnv 现必须使用 scripts/official_simenv_rosbridge_ros2_adapter_node.py 的 rosbridge 适配器。
仅在排查历史录包时可显式开启本文件，不能用于官方控制或业务闭环。
"""
import os
import subprocess, json, base64, threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField, Image, LaserScan
from geometry_msgs.msg import Twist, TransformStamped
from tf2_msgs.msg import TFMessage

C = 'simenv_ros1_hazard_platform'

class HwBridge(Node):
    def __init__(self):
        super().__init__('hw_bridge')
        self._pub = {}
        self._lock = threading.Lock()
        self._cmd_sub = self.create_subscription(Twist, '/hw/cmd_vel', lambda m: None, 10)
        # Pre-create depth/points publisher (data too large for JSON pipe, created empty)
        self._pub['/hw/real_sense/depth/points'] = self.create_publisher(PointCloud2, '/hw/real_sense/depth/points', 10)
        self._pipe_thread = threading.Thread(target=self._run_pipe, daemon=True)
        self._pipe_thread.start()
        self.get_logger().info('Bridge started, waiting for Docker pipe...')

    def _ensure_pub(self, name, cls):
        if name not in self._pub:
            with self._lock:
                if name not in self._pub:
                    self._pub[name] = self.create_publisher(cls, name, 10)

    def _run_pipe(self):
        import time
        while rclpy.ok():
            try:
                proc = subprocess.Popen(
                    ['docker', 'exec', '-i', C, 'bash', '-c',
                     'source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3'],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL, text=True)
                pipe_script = open('/home/hazard_platform/HazardWalker/ros2_ws/src/hazardwalker_platform/docker_pipe.py').read()
                proc.stdin.write(pipe_script)
                proc.stdin.close()
                for line in proc.stdout:
                    try:
                        msg = json.loads(line.strip())
                        self._handle(msg)
                    except:
                        pass
                proc.wait()
            except:
                pass
            time.sleep(2)

    def _handle(self, m):
        t = m.get('t','')
        if t == 'odom':
            self._ensure_pub('/hw/Odometry_gazebo', Odometry)
            msg = Odometry()
            msg.header.frame_id = m.get('fid','odom')
            msg.child_frame_id = m.get('cid','base_link')
            msg.pose.pose.position.x = m['x']; msg.pose.pose.position.y = m['y']; msg.pose.pose.position.z = m['z']
            msg.pose.pose.orientation.x = m['ox']; msg.pose.pose.orientation.y = m['oy']
            msg.pose.pose.orientation.z = m['oz']; msg.pose.pose.orientation.w = m['ow']
            msg.twist.twist.linear.x = m['vx']; msg.twist.twist.angular.z = m['wz']
            self._pub['/hw/Odometry_gazebo'].publish(msg)
        elif t == 'imu':
            fid = m.get('fid','imu_link')
            hw_topic = '/hw/trunk_imu' if fid == 'imu_link' else '/hw/livox/imu'
            self._ensure_pub(hw_topic, Imu)
            msg = Imu()
            msg.header.frame_id = fid
            msg.linear_acceleration.x = m['ax']; msg.linear_acceleration.y = m['ay']; msg.linear_acceleration.z = m['az']
            msg.angular_velocity.x = m['wx']; msg.angular_velocity.y = m['wy']; msg.angular_velocity.z = m['wz']
            self._pub[hw_topic].publish(msg)
        elif t == 'pc2':
            self._ensure_pub('/hw/livox/Pointcloud2', PointCloud2)
            msg = PointCloud2()
            msg.header.frame_id = m.get('fid','laser_livox')
            msg.height = m['h']; msg.width = m['w']
            msg.point_step = m['ps']; msg.row_step = m['rs']
            msg.is_dense = True
            msg.fields = [PointField(name=f['n'],offset=f['o'],datatype=f['d'],count=f['c']) for f in m['fields']]
            msg.data = base64.b64decode(m['data'])
            self._pub['/hw/livox/Pointcloud2'].publish(msg)
        elif t == 'tf':
            self._ensure_pub('/hw/tf', TFMessage)
            msg = TFMessage()
            for tf in m['tfs']:
                ts = TransformStamped()
                ts.header.frame_id = tf['fid']; ts.child_frame_id = tf['cid']
                ts.transform.translation.x = tf['tx']; ts.transform.translation.y = tf['ty']; ts.transform.translation.z = tf['tz']
                ts.transform.rotation.x = tf['rx']; ts.transform.rotation.y = tf['ry']
                ts.transform.rotation.z = tf['rz']; ts.transform.rotation.w = tf['rw']
                msg.transforms.append(ts)
            self._pub['/hw/tf'].publish(msg)
        elif t == 'pc2_c':
            fid = m.get('fid','')
            ci = m['ci']; ct = m['ct']
            chunk_key = f'{fid}_{int(m["w"])}_{int(m["h"])}'
            if not hasattr(self, '_pc2_chunks'):
                self._pc2_chunks = {}
            if chunk_key not in self._pc2_chunks:
                self._pc2_chunks[chunk_key] = {'data':'','ct':ct,'m':m}
            self._pc2_chunks[chunk_key]['data'] += m['data']
            if ci == ct - 1:
                # Last chunk: assemble and publish
                saved = self._pc2_chunks.pop(chunk_key)
                mm = saved['m']
                if fid == 'real_sense':
                    self._ensure_pub('/hw/real_sense/depth/points', PointCloud2)
                    msg = PointCloud2()
                    msg.header.frame_id = fid
                    msg.height = mm['h']; msg.width = mm['w']
                    msg.point_step = mm['ps']; msg.row_step = mm['rs']
                    msg.is_dense = True
                    msg.fields = [PointField(name=f['n'],offset=f['o'],datatype=f['d'],count=f['c']) for f in mm['fs']]
                    msg.data = base64.b64decode(saved['data'])
                    self._pub['/hw/real_sense/depth/points'].publish(msg)
        elif t == 'scan':
            self._ensure_pub('/hw/scan', LaserScan)
            msg = LaserScan()
            msg.header.frame_id = m.get('fid','laser_livox')
            msg.angle_min = m['angle_min']; msg.angle_max = m['angle_max']
            msg.angle_increment = m['angle_inc']
            msg.range_min = m['range_min']; msg.range_max = m['range_max']
            msg.ranges = [float(r) for r in m.get('ranges',[])]
            self._pub['/hw/scan'].publish(msg)
        elif t == 'img':
            hw_topic = m.get('topic','/hw/real_sense/rgb/image_raw')
            self._ensure_pub(hw_topic, Image)
            msg = Image()
            msg.header.frame_id = 'real_sense'
            msg.height = m['h']; msg.width = m['w']
            msg.encoding = m['enc']; msg.step = m['step']
            self._pub[hw_topic].publish(msg)
        elif t == 'pc2':
            fid = m.get('fid','')
            if fid == 'real_sense':
                self._ensure_pub('/hw/real_sense/depth/points', PointCloud2)
                msg = PointCloud2()
                msg.header.frame_id = fid
                msg.height = m['h']; msg.width = m['w']
                msg.point_step = m['ps']; msg.row_step = m['rs']
                msg.is_dense = True
                msg.fields = [PointField(name=f['n'],offset=f['o'],datatype=f['d'],count=f['c']) for f in m['fields']]
                msg.data = base64.b64decode(m['data']) if m.get('data') else b''
                self._pub['/hw/real_sense/depth/points'].publish(msg)
            else:
                self._ensure_pub('/hw/livox/Pointcloud2', PointCloud2)
                msg = PointCloud2()
                msg.header.frame_id = fid
                msg.height = m['h']; msg.width = m['w']
                msg.point_step = m['ps']; msg.row_step = m['rs']
                msg.is_dense = True
                msg.fields = [PointField(name=f['n'],offset=f['o'],datatype=f['d'],count=f['c']) for f in m['fields']]
                msg.data = base64.b64decode(m['data'])
                self._pub['/hw/livox/Pointcloud2'].publish(msg)

def main():
    # 失败优先：避免成员按旧手册启动后误以为 /hw/* 与控制链路已完整可用。
    if os.environ.get('HAZARDWALKER_ENABLE_LEGACY_JSON_BRIDGE') != '1':
        raise SystemExit(
            '旧 JSON bridge 已停用：请运行 scripts/run_official_simenv_rosbridge_adapter.sh；'
            '仅历史诊断可设置 HAZARDWALKER_ENABLE_LEGACY_JSON_BRIDGE=1。')
    rclpy.init()
    rclpy.spin(HwBridge())

if __name__ == '__main__':
    main()
