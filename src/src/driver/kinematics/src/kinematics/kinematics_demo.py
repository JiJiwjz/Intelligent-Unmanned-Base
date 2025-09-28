#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/21
# @author:aiden
# 机械臂运动学库使用实例
import kinematics.transform as transform
from kinematics.forward_kinematics import ForwardKinematics
from kinematics.inverse_kinematics import get_ik, get_position_ik, set_link, get_link, set_joint_range, get_joint_range

###########forward_kinematics##################
fk = ForwardKinematics(debug=True)  # 实例化正运动学，开启打印

print('当前各连杆长度(m):', fk.get_link())  # 详细说明请参考transform里的注释
print('当期各关节范围(deg):', fk.get_joint_range('deg'))  #以角度为单位返回
pulse = transform.pulse2angle([500, 500, 500, 500, 500])  # 舵机脉宽值转为弧度
print('input:', pulse)
res = fk.get_fk(pulse)  #获取运动学正解
print('output:', res)
print('rpy:', transform.qua2rpy(res[1]))
# 设置连杆长度(m)base_link, link1, link2, link3, tool_link
fk.set_link(0.2, 0.13, 0.13, 0.055, 0.12)  

# 设置关节范围(deg)joint1, joint2, joint3, joint4, joint5
fk.set_joint_range([-90, 0], [-90, 0], [-90, 0], [-90, 0], [-90, 0], 'deg')  
print('当前各连杆长度(m):', fk.get_link())  # 详细说明请参考transform里的注释
print('当期各关节范围(deg):', fk.get_joint_range('deg'))

print('---------------------------------------------------------------------')
###########inverse_kinematics##################
print('当前各连杆长度(m):', get_link())  # 详细说明请参考transform里的注释
print('当期各关节范围(deg):', get_joint_range('deg'))  #以角度为单位返回
# x, y, z(m), roll, pitch, yaw(deg)获取运动学逆解
res = get_position_ik(0.3, 0, 0.3, 0, 0, 0)  
if res != []:
    pulse = transform.angle2pulse(res)  # 转为舵机脉宽值
    for i in range(len(pulse)):
        print('output%s:'%(i + 1), pulse[i])
else:
    print('no solution')

# [x, y, z(m)], pitch, [pitch_min, pitch_max](deg)获取运动学逆解
res = get_ik([0.3, 0, 0.3], 0, [-180, 180])
if res != []:
    for i in range(len(res)):
        print('rpy%s:'%(i + 1), res[i][1])  # 解对应的rpy值
        pulse = transform.angle2pulse(res[i][0])  # 转为舵机脉宽值
        for j in range(len(pulse)):
            print('output%s:'%(j + 1), pulse[j])
else:
    print('no solution')
# 设置连杆长度(m)base_link, link1, link2, link3, tool_link
set_link(0.2, 0.13, 0.13, 0.055, 0.12)  

# 设置关节范围(deg)joint1, joint2, joint3, joint4, joint5
set_joint_range([-90, 0], [-90, 0], [-90, 0], [-90, 0], [-90, 0], 'deg')  
print('当前各连杆长度(m):', get_link())  # 详细说明请参考transform里的注释
print('当期各关节范围(deg):', get_joint_range('deg'))

