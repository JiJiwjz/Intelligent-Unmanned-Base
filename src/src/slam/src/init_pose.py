#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/08
import math
import rospy
from servo_msgs.msg import CommandDuration, JointState

class InitPose:
    def __init__(self):
        rospy.init_node('init_pose')
        horizontal = rospy.get_param('~horizontal', False)
        namespace = rospy.get_namespace()
        
        joint1_angle = rospy.get_param('~joint1', 0)
        joint2_angle = rospy.get_param('~joint2', -40)
        joint3_angle = rospy.get_param('~joint3', 110)
        joint4_angle = rospy.get_param('~joint4', 70)
        joint5_angle = rospy.get_param('~joint5', 0)
        jointr_angle = rospy.get_param('~jointr', 48)
        
        joint1 = rospy.Publisher(namespace + 'joint1_controller/command_duration', CommandDuration, queue_size=1)
        joint2 = rospy.Publisher(namespace + 'joint2_controller/command_duration', CommandDuration, queue_size=1)
        joint3 = rospy.Publisher(namespace + 'joint3_controller/command_duration', CommandDuration, queue_size=1)
        joint4 = rospy.Publisher(namespace + 'joint4_controller/command_duration', CommandDuration, queue_size=1)
        joint5 = rospy.Publisher(namespace + 'joint5_controller/command_duration', CommandDuration, queue_size=1)
        jointr = rospy.Publisher(namespace + 'r_joint_controller/command_duration', CommandDuration, queue_size=1)
        while not rospy.is_shutdown():
            try:
                if rospy.get_param(namespace + 'servo_manager/init_finish') and rospy.get_param(namespace + 'joint_states_publisher/init_finish') and rospy.get_param(namespace + 'ros_robot_controller/init_finish'):
                    print('******init_pose******')
                    break
            except:
                rospy.sleep(0.1)
        
        rospy.sleep(0.2)
        if horizontal:
            joint1.publish(CommandDuration(data=math.radians(0), duration=2))
            joint2.publish(CommandDuration(data=math.radians(-59), duration=2))
            joint3.publish(CommandDuration(data=math.radians(117), duration=2))
            joint4.publish(CommandDuration(data=math.radians(31), duration=2))
            joint5.publish(CommandDuration(data=math.radians(0), duration=2))
            jointr.publish(CommandDuration(data=math.radians(48), duration=2))
        else:
            joint1.publish(CommandDuration(data=math.radians(joint1_angle), duration=2))
            joint2.publish(CommandDuration(data=math.radians(joint2_angle), duration=2))
            joint3.publish(CommandDuration(data=math.radians(joint3_angle), duration=2))
            joint4.publish(CommandDuration(data=math.radians(joint4_angle), duration=2))
            joint5.publish(CommandDuration(data=math.radians(joint5_angle), duration=2))
            jointr.publish(CommandDuration(data=math.radians(jointr_angle), duration=2))

if __name__ == '__main__':
    InitPose()
