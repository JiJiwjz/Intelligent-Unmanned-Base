#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/19
# @author:aiden
# 手势控制
import sys
import cv2
import math
import rospy
import signal
import numpy as np
from interfaces.msg import Points
from servo_msgs.msg import MultiRawIdPosDur
from servo_controllers.bus_servo_control import set_servos
sys.path.append('/home/ubuntu/software/arm_pc')
from action_group_controller import ActionGroupController

class HandGestureControlNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.image = None
        self.points = []
        self.running = True
        self.left_and_right = 0
        self.up_and_down = 0
        self.last_point = [0, 0]

        signal.signal(signal.SIGINT, self.shutdown)
        rospy.Subscriber('/hand_trajectory/points', Points, self.get_hand_points_callback)
        self.controller = ActionGroupController(use_ros=True)
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/servo_manager/init_finish') and rospy.get_param('/joint_states_publisher/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        rospy.sleep(1)
        set_servos(self.joints_pub, 1.5, ((10, 300), (5, 500), (4, 150), (3, 15), (2, 760), (1, 500)))
        self.hand_gesture_control()
    
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def get_hand_points_callback(self, msg):
        points = []
        left_and_right = [0]
        up_and_down = [0]
        if len(msg.points) > 10:
            for i in msg.points:
                if int(i.x) - self.last_point[0] > 0:
                    left_and_right.append(1)
                else:
                    left_and_right.append(-1)
                if int(i.y) - self.last_point[1] > 0:
                    up_and_down.append(1)
                else:
                    up_and_down.append(-1)
                points.extend([(int(i.x), int(i.y))])
                self.last_point = [int(i.x), int(i.y)]
            self.left_and_right = sum(left_and_right)
            self.up_and_down = sum(up_and_down)
            self.points = np.array(points)

    def hand_gesture_control(self):
        while self.running:
            if self.points != []:
                line = cv2.fitLine(self.points, cv2.DIST_L2, 0, 0.01, 0.01)
                angle = int(abs(math.degrees(math.acos(line[0][0]))))
                print('>>>>>>', angle)
                if 90 >= angle > 60:
                    if self.up_and_down > 0:
                        print('down')
                    else:
                        print('up')
                    rospy.sleep(0.3)
                    self.controller.runAction('hand_control_pick')
                    set_servos(self.joints_pub, 1.5, ((10, 300), (5, 500), (4, 150), (3, 15), (2, 760), (1, 500)))
                elif 30 > angle >= 0:
                    if self.left_and_right > 0:
                        print('right')
                    else:
                        print('left')
                    rospy.sleep(0.3)
                    self.controller.runAction('hand_control_place')
                    set_servos(self.joints_pub, 1.5, ((10, 300), (5, 500), (4, 150), (3, 15), (2, 760), (1, 500)))
                self.points = []
            else:
                rospy.sleep(0.01)

        self.controller.runAction('horizontal')
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    HandGestureControlNode('hand_gesture_control')
