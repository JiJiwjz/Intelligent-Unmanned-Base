#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 手跟随
import rospy
import signal
import sdk.pid as pid
from geometry_msgs.msg import Twist
from kinematics import transform
from interfaces.msg import Point2D
from servo_msgs.msg import MultiRawIdPosDur
from kinematics.kinematics_control import set_pose_target
from servo_controllers.bus_servo_control import set_servos

class HandTrackNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.image = None
        self.center = None
        self.running = True
        self.z_dis = 0.41
        self.y_dis = 500
        self.x_init = transform.link3 + transform.tool_link

        self.pid_z = pid.PID(0.00005, 0.0, 0.0)
        self.pid_y = pid.PID(0.04, 0.0, 0.0)

        signal.signal(signal.SIGINT, self.shutdown)

        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)  # 底盘控制
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制

        rospy.Subscriber('/hand_detect/center', Point2D, self.get_hand_callback)

        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/servo_manager/init_finish') and rospy.get_param('/joint_states_publisher/init_finish') and rospy.get_param('/kinematics/init_finish'):
                    break
            except:
                rospy.sleep(0.1)

        rospy.sleep(0.2)
        self.init_action()
        rospy.set_param('~init_finish', True)

        self.hand_track() 

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def init_action(self):
        res = set_pose_target([self.x_init, 0, self.z_dis], 0, [-180, 180], 1)
        if res[1]:
            servo_data = res[1]
            set_servos(self.joints_pub, 1.5, ((10, 500), (5, 500), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, servo_data[0])))
            rospy.sleep(1.8)

        self.mecanum_pub.publish(Twist())

    def get_hand_callback(self, msg):
        if msg.width != 0:
            self.center = msg
        else:
            self.center = None

    def hand_track(self):
        while self.running:
            if self.center is not None:
                self.pid_y.SetPoint = self.center.width / 2
                self.pid_y.update(self.center.width - self.center.x)
                self.y_dis += self.pid_y.output
                if self.y_dis < 200:
                    self.y_dis = 200
                if self.y_dis > 800:
                    self.y_dis = 800

                self.pid_z.SetPoint = self.center.height / 2
                self.pid_z.update(self.center.y)
                self.z_dis += self.pid_z.output
                if self.z_dis > 0.46:
                    self.z_dis = 0.46
                if self.z_dis < 0.36:
                    self.z_dis = 0.36

                res = set_pose_target([self.x_init, 0, self.z_dis], 0, [-180, 180], 1)
                if res[1]:
                    servo_data = res[1]
                    set_servos(self.joints_pub, 0.02, ((10, 500), (5, 500), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, self.y_dis)))
                else:
                    set_servos(self.joints_pub, 0.02, ((1, self.y_dis), ))
                rospy.sleep(0.02)
            else:
                rospy.sleep(0.01)

        self.init_action() 
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    HandTrackNode('hand_track')
