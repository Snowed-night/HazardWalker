#!/usr/bin/env python3
"""Docker side: subscribe ROS1, output JSON lines to stdout."""
import rospy, json, sys, struct
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage

def odom(m):
    print(json.dumps({'t':'odom','x':m.pose.pose.position.x,'y':m.pose.pose.position.y,
        'z':m.pose.pose.position.z,'ox':m.pose.pose.orientation.x,'oy':m.pose.pose.orientation.y,
        'oz':m.pose.pose.orientation.z,'ow':m.pose.pose.orientation.w,
        'vx':m.twist.twist.linear.x,'wz':m.twist.twist.angular.z,'fid':m.header.frame_id,
        'cid':m.child_frame_id}))
    sys.stdout.flush()

def imu_cb(m, fid='imu_link'):
    print(json.dumps({'t':'imu','fid':m.header.frame_id or fid,
        'ax':m.linear_acceleration.x,'ay':m.linear_acceleration.y,'az':m.linear_acceleration.z,
        'wx':m.angular_velocity.x,'wy':m.angular_velocity.y,'wz':m.angular_velocity.z}))
    sys.stdout.flush()

def pc2(m):
    # For PointCloud2, output point count and raw data as base64
    import base64
    print(json.dumps({'t':'pc2','w':m.width,'h':m.height,'ps':m.point_step,
        'rs':m.row_step,'fid':m.header.frame_id,'fields':[{'n':f.name,'o':f.offset,'d':f.datatype,'c':f.count} for f in m.fields],
        'data':base64.b64encode(m.data).decode()}))
    sys.stdout.flush()

def tf_cb(m):
    tfs = []
    for t in m.transforms:
        tfs.append({'fid':t.header.frame_id,'cid':t.child_frame_id,
            'tx':t.transform.translation.x,'ty':t.transform.translation.y,'tz':t.transform.translation.z,
            'rx':t.transform.rotation.x,'ry':t.transform.rotation.y,'rz':t.transform.rotation.z,'rw':t.transform.rotation.w})
    print(json.dumps({'t':'tf','tfs':tfs}))
    sys.stdout.flush()

rospy.init_node('pipe_out', anonymous=True, disable_signals=True)
rospy.Subscriber('/Odometry_gazebo', Odometry, odom)
rospy.Subscriber('/livox/Pointcloud2', PointCloud2, pc2)
rospy.Subscriber('/livox/imu', Imu, imu_cb, callback_args='livox_imu_link')
rospy.Subscriber('/trunk_imu', Imu, imu_cb, callback_args='imu_link')
rospy.Subscriber('/tf', TFMessage, tf_cb)
print(json.dumps({'t':'ready'}), flush=True)
rospy.spin()
