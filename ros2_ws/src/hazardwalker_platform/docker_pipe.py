#!/usr/bin/env python3
"""Docker side: subscribe ROS1, output JSON lines to stdout."""
import rospy, json, sys, struct
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, LaserScan, Image
from tf2_msgs.msg import TFMessage
import base64

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

CHUNK_SIZE = 512 * 1024  # 512KB per chunk

def pc2(m, fid_override=None):
    fid = fid_override or m.header.frame_id
    data_b64 = base64.b64encode(m.data).decode()
    fields = [{'n':f.name,'o':f.offset,'d':f.datatype,'c':f.count} for f in m.fields]
    total = (len(data_b64) + CHUNK_SIZE - 1) // CHUNK_SIZE
    for i in range(total):
        chunk = data_b64[i*CHUNK_SIZE:(i+1)*CHUNK_SIZE]
        print(json.dumps({'t':'pc2_c','w':m.width,'h':m.height,'ps':m.point_step,
            'rs':m.row_step,'fid':fid,'fs':fields,'ci':i,'ct':total,'data':chunk}))
        sys.stdout.flush()

def scan_cb(m):
    print(json.dumps({'t':'scan','fid':m.header.frame_id,
        'angle_min':m.angle_min,'angle_max':m.angle_max,'angle_inc':m.angle_increment,
        'range_min':m.range_min,'range_max':m.range_max,
        'ranges':list(m.ranges)[:10],'ranges_len':len(m.ranges)}))
    sys.stdout.flush()

def img_cb(m, topic_name):
    print(json.dumps({'t':'img','topic':topic_name,'h':m.height,'w':m.width,
        'enc':m.encoding,'step':m.step,'data_len':len(m.data)}))
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
rospy.Subscriber('/livox/imu', Imu, imu_cb, callback_args='livox_imu_link')
rospy.Subscriber('/trunk_imu', Imu, imu_cb, callback_args='imu_link')
rospy.Subscriber('/tf', TFMessage, tf_cb)
rospy.Subscriber('/scan', LaserScan, scan_cb)
rospy.Subscriber('/real_sense/rgb/image_raw', Image, img_cb, callback_args='/hw/real_sense/rgb/image_raw')
rospy.Subscriber('/real_sense/depth/image_raw', Image, img_cb, callback_args='/hw/real_sense/depth/image_raw')
rospy.Subscriber('/real_sense/depth/points', PointCloud2, pc2, callback_args='real_sense')
print(json.dumps({'t':'ready'}), flush=True)
rospy.spin()
