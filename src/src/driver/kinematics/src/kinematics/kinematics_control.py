#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/21
# @author:aiden
# 机械臂运动学调用
import rospy
import kinematics.transform as transform
from interfaces.srv import SetRobotPose, SetJointValue

def set_pose_target(position, pitch, pitch_range=[-180, 180], resolution=1):
    '''
    给定坐标和俯仰角，返回逆运动学解
    position: 目标位置，列表形式[x, y, z]，单位m
    pitch: 目标俯仰角，单位度，范围-180~180
    pitch_range: 如果在目标俯仰角找不到解，则在这个范围内寻找解
    resolution: pitch_range范围角度的分辨率
    return: 调用是否成功， 舵机的目标位置， 当前舵机的位置， 机械臂的目标姿态， 最优解所有舵机转动的变化量
    '''
    res = rospy.ServiceProxy('/kinematics/set_pose_target', SetRobotPose, persistent=True)(position, pitch, pitch_range, resolution)
    return [res.success, list(res.pulse), list(res.current_pulse), list(res.rpy), res.min_variation]

def set_joint_value_target(joint_value):
    '''
    给定每个舵机的转动角度，返回机械臂到达的目标位置姿态
    joint_value: 每个舵机转动的角度，列表形式[joint1, joint2, joint3, joint4, joint5]，单位脉宽
    return: 目标位置的3D坐标和位姿，格式geometry_msgs/Pose
    '''
    return rospy.ServiceProxy('/kinematics/set_joint_value_target', SetJointValue, persistent=True)(joint_value)
    
if __name__ == "__main__":
    # 初始化节点
    rospy.init_node('kinematics_controller', anonymous=True)
    res = set_pose_target([transform.link3 + transform.tool_link, 0, 0.36], 0, [-180, 180], 1)
    print('ik', res)
    if res[1] != []:
        res = set_joint_value_target(res[1])
        print('fk', res)
