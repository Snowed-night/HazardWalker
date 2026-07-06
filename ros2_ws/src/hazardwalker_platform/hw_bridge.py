#!/usr/bin/env python3
"""Host side: read JSON from Docker pipe, publish ROS2 /hw/* topics."""
import subprocess, json, base64, threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField, Image
from geometry_msgs.msg import Twist, TransformStamped
from tf2_msgs.msg import TFMessage

C = 'simenv_ros1_hazard_platform'

class HwBridge(Node):
    def __init__(self):
        super().__init__('hw_bridge')
        self._pub = {}
        self._lock = threading.Lock()
        self._cmd_sub = self.create_subscription(Twist, '/hw/cmd_vel', lambda m: None, 10)
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
                # Send pipe script
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

def main():
    rclpy.init()
    rclpy.spin(HwBridge())

if __name__ == '__main__':
    main()
