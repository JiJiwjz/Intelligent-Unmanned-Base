import os
import numpy as np
from geometry_msgs.msg import Pose, Quaternion
from math import degrees, radians, atan2, asin, sqrt
'''
Modified DH
----------------------------------------------
i | α(i-1) | a(i-1) |       θ(i)      | d(i) |
----------------------------------------------
1 |   0°   |   0    |  θ1(-120, 120)  |   0  |
----------------------------------------------
2 |  -90°  |   0    |  θ2(-180, 0)    |   0  |
----------------------------------------------
3 |   0°   | link1  |  θ3(-120, 120)  |   0  |
----------------------------------------------
4 |   0°   | link2  |  θ4(-200, 20)   |   0  |
----------------------------------------------
5 |  -90°  |   0    |  θ5(-120, 120)  |   0  |
----------------------------------------------
'''

# 连杆长度(m)
# 底座的高度，这里把第一个坐标系和第二个坐标的原点重合到一起了
base_link = 0.23

link1 = 0.130
link2 = 0.130

# 计算tool_link时取值为link3 + tool_link，因为把末端的坐标系原点和前一个重合到一起了
# 这里的tool_link指实际上的夹持器长度
link3 = 0.055
tool_link = 0.117

# 各关节角度限制，取决于是否碰撞以及舵机的转动范围
# 多加0.2为了防止计算时数值的不稳定，会比设定值大一点点
joint1 = [-120.2, 120.2]
joint2 = [-180.2, 0.2]
joint3 = [-120.2, 120.2]
joint4 = [-200.2, 20.2]
joint5 = [-120.2, 120.2]

#         舵机脉宽范围，中位值，对应的角度范围，中位值
joint1_map = [0, 1000, 500, -120, 120, 0]
joint2_map = [0, 1000, 500, 30, -210, -90]
joint3_map = [0, 1000, 500, 120, -120, 0]
joint4_map = [0, 1000, 500, 30, -210, -90]
joint5_map = [0, 1000, 500, -120, 120, 0]

# 判断是否为旋转矩阵
def isRotationMatrix(r):
    rt = np.transpose(r)
    shouldBeIdentity = np.dot(rt, r)
    i = np.identity(3, dtype=r.dtype)
    n = np.linalg.norm(i - shouldBeIdentity)
    return n < 1e-6


# 旋转矩阵--->欧拉角
def rot2rpy(R):
    assert (isRotationMatrix(R))

    sy = sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        r = atan2(R[2, 1], R[2, 2])
        p = atan2(-R[2, 0], sy)
        y = atan2(R[1, 0], R[0, 0])
    else:
        r = atan2(-R[1, 2], R[1, 1])
        p = atan2(-R[2, 0], sy)
        y = 0

    return [degrees(r), degrees(p), degrees(y)]

def rot2qua(M):
    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = M.flat
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0              ],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0              ],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0              ],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
        ) / 3.0
    vals, vecs = np.linalg.eigh(K)
    qua = vecs[[3, 0, 1, 2], np.argmax(vals)]
    if qua[0] < 0:
        qua *= -1

    q = Pose()
    q.orientation.w = qua[0]
    q.orientation.x = qua[1]
    q.orientation.y = qua[2]
    q.orientation.z = qua[3]

    return q.orientation

def qua2rpy(qua):
    if type(qua) == Quaternion:
        x, y, z, w = qua.x, qua.y, qua.z, qua.w
    else:
        x, y, z, w = qua[0], qua[1], qua[2], qua[3]
    roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = asin(2 * (w * y - x * z))
    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
  
    return degrees(roll), degrees(pitch), degrees(yaw)

# 等比例映射
def angle_transform(angle, param, inverse=False):
    if inverse:
        new_angle = ((angle - param[5]) / (param[4] - param[3])) * (param[1] - param[0]) + param[2]
    else:
        new_angle = ((angle - param[2]) / (param[1] - param[0])) * (param[4] - param[3]) + param[5]

    return new_angle

def pulse2angle(pulse):
    theta1 = angle_transform(pulse[0], joint1_map)
    theta2 = angle_transform(pulse[1], joint2_map)
    theta3 = angle_transform(pulse[2], joint3_map)
    theta4 = angle_transform(pulse[3], joint4_map)
    theta5 = angle_transform(pulse[4], joint5_map)
    
    #print(theta1, theta2, theta3, theta4, theta5)
    return radians(theta1), radians(theta2), radians(theta3), radians(theta4), radians(theta5)

def angle2pulse(angle):
    pluse = []
    
    for i in angle:
        theta1 = angle_transform(degrees(i[0]), joint1_map, True)
        theta2 = angle_transform(degrees(i[1]), joint2_map, True)
        theta3 = angle_transform(degrees(i[2]), joint3_map, True)
        theta4 = angle_transform(degrees(i[3]), joint4_map, True)
        theta5 = angle_transform(degrees(i[4]), joint5_map, True)
        
        #print(theta1, theta2, theta3, theta4, theta5)
        pluse.extend([[theta1, theta2, theta3, theta4, theta5]])

    return pluse

