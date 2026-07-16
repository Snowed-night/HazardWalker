#!/usr/bin/env python3
"""Host side: read JSON from Docker pipe, publish ROS2 /hw/* topics."""
import subprocess, json, base64, threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField, Image, LaserScan
from geometry_msgs.msg import Twist, TransformStamped
from tf2_msgs.msg import TFMessage

import os
C = os.environ.get('SIMENV_CONTAINER', 'simenv_run')
PIPE_SCRIPT = os.path.join(
    os.environ.get('HOME', '/home/hazard_platform'),
    'HazardWalker/ros2_ws/src/hazardwalker_platform/docker_pipe.py')

class HwBridge(Node):
    def __init__(self):
        super().__init__('hw_bridge')
        self._pub = {}
        self._lock = threading.Lock()
        self._cmd_sub = self.create_subscription(Twist, '/hw/cmd_vel', self._on_cmd_vel, 10)
        self._last_cmd = None
        self._cmd_thread = threading.Thread(target=self._cmd_forward_loop, daemon=True)
        self._cmd_thread.start()
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

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg

    def _cmd_forward_loop(self) -> None:
        """10Hz loop: forward latest /hw/cmd_vel to ROS1 /cmd_vel inside Docker."""
        import time
        while rclpy.ok():
            if self._last_cmd is not None:
                try:
                    vx = self._last_cmd.linear.x
                    vz = self._last_cmd.angular.z
                    cmd = (
                        "source /opt/ros/noetic/setup.bash && "
                        "rostopic pub /cmd_vel geometry_msgs/Twist "
                        f"'{{linear: {{x: {vx}, y: 0.0, z: 0.0}}, "
                        f"angular: {{x: 0.0, y: 0.0, z: {vz}}}}}' "
                        "--once 2>/dev/null"
                    )
                    subprocess.run(
                        ['docker', 'exec', C, 'bash', '-c', cmd],
                        timeout=0.3, capture_output=True)
                except Exception:
                    pass
            time.sleep(0.1)

    def _run_pipe(self):
        import time
        while rclpy.ok():
            try:
                proc = subprocess.Popen(
                    ['docker', 'exec', '-i', C, 'bash', '-c',
                     'source /opt/ros/noetic/setup.bash && python3'],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL, text=True)
                pipe_script = open(PIPE_SCRIPT).read()
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
            self._ensure_pub('/tf', TFMessage)     # 原生话题供 SLAM/nav2 使用
            msg = TFMessage()
            for tf in m['tfs']:
                ts = TransformStamped()
                ts.header.frame_id = tf['fid']; ts.child_frame_id = tf['cid']
                ts.transform.translation.x = tf['tx']; ts.transform.translation.y = tf['ty']; ts.transform.translation.z = tf['tz']
                ts.transform.rotation.x = tf['rx']; ts.transform.rotation.y = tf['ry']
                ts.transform.rotation.z = tf['rz']; ts.transform.rotation.w = tf['rw']
                msg.transforms.append(ts)
            self._pub['/hw/tf'].publish(msg)
            self._pub['/tf'].publish(msg)
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
            self._ensure_pub('/scan', LaserScan)   # 原生话题供 SLAM Toolbox
            msg = LaserScan()
            msg.header.frame_id = m.get('fid','laser_livox')
            msg.angle_min = m['angle_min']; msg.angle_max = m['angle_max']
            msg.angle_increment = m['angle_inc']
            msg.range_min = m['range_min']; msg.range_max = m['range_max']
            msg.ranges = [float(r) for r in m.get('ranges',[])]
            self._pub['/hw/scan'].publish(msg)
            self._pub['/scan'].publish(msg)
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
    rclpy.init()
    rclpy.spin(HwBridge())

if __name__ == '__main__':
    main()
