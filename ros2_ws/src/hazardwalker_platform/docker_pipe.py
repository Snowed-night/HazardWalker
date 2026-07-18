#!/usr/bin/env python3
"""Docker side: subscribe ROS1, output JSON lines to stdout."""
import rospy, json, sys, struct
from sensor_msgs.msg import Imu, PointCloud2, LaserScan, Image
import base64

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
        'ranges':list(m.ranges)}))
    sys.stdout.flush()

def img_cb(m, topic_name):
    print(json.dumps({'t':'img','topic':topic_name,'h':m.height,'w':m.width,
        'enc':m.encoding,'step':m.step,'data_len':len(m.data)}))
    sys.stdout.flush()

rospy.init_node('pipe_out', anonymous=True, disable_signals=True)
rospy.Subscriber('/livox/imu', Imu, imu_cb, callback_args='livox_imu_link')
rospy.Subscriber('/trunk_imu', Imu, imu_cb, callback_args='imu_link')
rospy.Subscriber('/scan', LaserScan, scan_cb)
rospy.Subscriber('/real_sense/rgb/image_raw', Image, img_cb, callback_args='/hw/real_sense/rgb/image_raw')
rospy.Subscriber('/real_sense/depth/image_raw', Image, img_cb, callback_args='/hw/real_sense/depth/image_raw')
rospy.Subscriber('/real_sense/depth/points', PointCloud2, pc2, callback_args='real_sense')
print(json.dumps({'t':'ready'}), flush=True)
rospy.spin()
