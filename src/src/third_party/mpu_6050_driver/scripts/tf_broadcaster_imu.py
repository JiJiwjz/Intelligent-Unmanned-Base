#!/usr/bin/env python  
import rospy
import tf2_ros
from math import *
import numpy as np
import geometry_msgs.msg
from sensor_msgs.msg import Imu

def qua2rot(x, y, z, w):
    rot_matrix = np.matrix(
        [[1.0 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (w * y + x * z)],
        [2 * (x * y + w * z), 1.0 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1.0 - 2 * (x * x + y * y)]])

    return rot_matrix

def rot2qua(M):
    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = M.flat
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0              ],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0              ],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0              ],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
        ) / 3.0
    vals, vecs = np.linalg.eigh(K)
    q = vecs[[3, 0, 1, 2], np.argmax(vals)]
    if q[0] < 0:
        q *= -1
    return [q[1], q[2], q[3], q[0]]

def handle_imu_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = target_frame
    t.child_frame_id = IMU_FRAME
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    rot = qua2rot(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    qua = rot2qua(rot.I)
    
    t.transform.rotation.x = qua[0]
    t.transform.rotation.y = qua[1]
    t.transform.rotation.z = qua[2]
    t.transform.rotation.w = qua[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster_imu')
    target_frame = rospy.get_param('~imu_link', 'imu_link')
    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_frame')
    imu_topic = rospy.get_param('~imu_topic', 'imu_data')
    rospy.Subscriber(imu_topic, Imu, handle_imu_pose)
    rospy.spin()
