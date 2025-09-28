#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/20
# @author:aiden
# 实时获取角度反馈，根据当前位置和
# 目标位置的最小差值来获取最优解
import rospy
import numpy as np
from sdk import common
from geometry_msgs.msg import Pose
import kinematics.transform as transform
from kinematics.forward_kinematics import ForwardKinematics
from kinematics.inverse_kinematics import get_ik, set_link, get_link, set_joint_range, get_joint_range

from servo_msgs.msg import ServoStateList
from interfaces.msg import JointRange, JointsRange, Link
from interfaces.srv import SetRobotPose, SetJointValue, GetRobotPose, SetLink, GetLink, SetJointRange, GetJointRange

fk = ForwardKinematics(debug=False)  # 不开启打印
class SearchKinematicsSolutionsNode:
    def __init__(self, name):
        # 初始化节点
        rospy.init_node(name)
        self.name = name
        
        self.current_servo_positions = []

        rospy.Subscriber('/servo_controllers/port_id_1/servo_states', ServoStateList, self.get_servo_position)
       
        rospy.Service('~set_link', SetLink, self.set_link_srv)
        rospy.Service('~get_link', GetLink, self.get_link_srv)
        rospy.Service('~set_joint_range', SetJointRange, self.set_joint_range_srv)
        rospy.Service('~get_joint_range', GetJointRange, self.get_joint_range_srv)
        rospy.Service('~set_pose_target', SetRobotPose, self.set_pose_target)
        rospy.Service('~get_current_pose', GetRobotPose, self.get_current_pose)
        rospy.Service('~set_joint_value_target', SetJointValue, self.set_joint_value_target)
        common.loginfo('kinematics init finish') 
        rospy.set_param('~init_finish', True)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")

    def set_link_srv(self, msg):
        # 设置link长度
        base_link = msg.data.base_link
        link1 = msg.data.link1
        link2 = msg.data.link2
        link3 = msg.data.link3
        end_effector_link = msg.data.end_effector_link
        set_link(base_link, link1, link2, link3, end_effector_link)
        fk.set_link(base_link, link1, link2, link3, end_effector_link)

        return [True, 'set_link']

    def get_link_srv(self, msg):
        # 获取各个link长度
        data = get_link()
        data1 = fk.get_link()
        link = Link()
        if data == data1:
            link.base_link = data[0]
            link.link1 = data[1]
            link.link2 = data[2]
            link.link3 = data[3]
            link.end_effector_link = data[4]
            return [True, link]
        else:
            return [True, []]

    def set_joint_range_srv(self, msg):
        # 设置关节范围
        joint1 = msg.data.joint1
        joint2 = msg.data.joint2
        joint3 = msg.data.joint3
        joint4 = msg.data.joint4
        joint5 = msg.data.joint5
        set_joint_range([joint1.min, joint1.max], [joint2.min, joint2.max], [joint3.min, joint3.max], [joint4.min, joint4.max], [joint5.min, joint5.max], 'deg')
        fk.set_joint_range([joint1.min, joint1.max], [joint2.min, joint2.max], [joint3.min, joint3.max], [joint4.min, joint4.max], [joint5.min, joint5.max], 'deg')

        return [True, 'set_joint_range']

    def get_joint_range_srv(self, msg):
        # 获取各个关节范围
        data = get_joint_range('deg')
        data1 = fk.get_joint_range('deg')
        joint_range = JointsRange()
        joint_range.joint1.min = data[0][0]
        joint_range.joint1.max = data[0][1]
        joint_range.joint2.min = data[1][0]
        joint_range.joint2.max = data[1][1]
        joint_range.joint3.min = data[2][0]
        joint_range.joint3.max = data[2][1]
        joint_range.joint4.min = data[3][0]
        joint_range.joint4.max = data[3][1]
        joint_range.joint5.min = data[4][0]
        joint_range.joint5.max = data[4][1]
        if data == data1:
            return [True, joint_range]
        else:
            reutrn [True, []]

    def set_joint_value_target(self, msg):
        # 正运动学解
        joint_value = msg.joint_value
        angle = transform.pulse2angle(joint_value)
        res = fk.get_fk(angle)
        pose = Pose() 
        if res:
            pose.position.x = res[0][0]
            pose.position.y = res[0][1]
            pose.position.z = res[0][2]
            pose.orientation = res[1]
            return [True, True, pose]
        else:
            return [True, False, pose]

    def get_current_pose(self, msg):
        # 获取机械臂当前位置
        angle = transform.pulse2angle(self.current_servo_positions)
        res = fk.get_fk(angle)
        pose = Pose() 
        if res:
            pose.position.x = res[0][0]
            pose.position.y = res[0][1]
            pose.position.z = res[0][2]
            pose.orientation = res[1]
            return [True, True, pose]
        else:
            return [True, False, pose]

    def get_servo_position(self, msg):
        # 获取舵机当前角度
        servo_positions = []
        for i in msg.servo_states:
            if 0 < i.id < 6:
                servo_positions.append(i.position)

        self.current_servo_positions = np.array(servo_positions) 
        # print(self.current_servo_positions)

    def set_pose_target(self, msg):
        # 逆运动学解，获取最优解(所有电机转动最小)
        position, pitch, pitch_range, resolution = msg.position, msg.pitch, msg.pitch_range, msg.resolution
        position = list(position)

        # t1 = rospy.get_time()
        all_solutions = get_ik(position, pitch, list(pitch_range), resolution)
        # t2 = rospy.get_time()
        # print(t2 - t1)
        # print(all_solutions, self.current_servo_positions)
        if all_solutions != [] and self.current_servo_positions != []:
            rpy = []
            min_d = 1000*5
            optimal_solution = []
            for s in all_solutions:
                pulse_solutions = transform.angle2pulse(s[0])
                try:
                    for i in pulse_solutions:
                        d = np.array(i) - self.current_servo_positions
                        d_abs = np.maximum(d, -d)
                        min_sum = np.sum(d_abs)
                        if min_sum < min_d:
                            min_d = min_sum
                            for k in range(len(i)):
                                if i[k] < 0:
                                    i[k] = 0
                                elif i[k] > 1000:
                                    i[k] = 1000
                            rpy = s[1]
                            optimal_solution = i
                except BaseException as e:
                    print('choose solution error', e)
                    #print(pulse_solutions, current_servo_positions)
                # print(rospy.get_time() - t2)
            return [True, optimal_solution, self.current_servo_positions.tolist(), rpy, min_d]
        else:
            return [True, [], [], [], 0]

if __name__ == "__main__":
    SearchKinematicsSolutionsNode('kinematics')
