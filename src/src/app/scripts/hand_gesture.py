#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/19
# @author:aiden
# 手势控制
import cv2
import math
import rospy
import threading
import numpy as np
from geometry_msgs.msg import Twist
from app.common import Heart
from interfaces.msg import Points
from servo_msgs.msg import MultiRawIdPosDur
from servo_controllers.bus_servo_control import set_servos
from ros_robot_controller.msg import MotorsState, MotorState
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse, Trigger, TriggerResponse

class HandGestureControlNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.image = None
        self.image_sub = None
        self.running = True
        self.last_point = [0, 0]
        self.linear_speed = 0.3
        self.angular_speed = 2
        self.th = None
        self.thread_running = True
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist,
                                           queue_size=1)  # 底盘控制(chassis control)

        self.enter_srv = rospy.Service('~enter', Trigger, self.enter_srv_callback)  # 进入玩法(enter the game)
        self.exit_srv = rospy.Service('~exit', Trigger, self.exit_srv_callback)  # 退出玩法(exit the game)
        self.set_running_srv = rospy.Service('~set_running', SetBool, self.set_running_srv_callback)  # 开启玩法(start the game)
        self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(None))  # 心跳包(heartbeat package)
        self.motor_pub = rospy.Publisher('/ros_robot_controller/set_motor', MotorsState, queue_size=1)
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制(servo control)
        rospy.sleep(0.2)
        self.mecanum_pub.publish(Twist())

    def enter_srv_callback(self, _):
        set_servos(self.joints_pub, 1.5, ((10, 300), (5, 500), (4, 600), (3, 0), (2, 750), (1, 500)))
        self.mecanum_pub.publish(Twist())
        if self.image_sub is None:
            self.image_sub = rospy.Subscriber('/hand_trajectory/points', Points, self.get_hand_points_callback)
        self.thread_running = False
        rospy.ServiceProxy('/hand_trajectory/stop', Trigger)()
        rospy.loginfo("hand gesture control enter")
        
        return TriggerResponse(success=True)

    def exit_srv_callback(self, _):
        self.mecanum_pub.publish(Twist())
        if self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None
        self.thread_running = False
        rospy.ServiceProxy('/hand_trajectory/stop', Trigger)()
        rospy.loginfo("hand gesture control exit")
        
        return TriggerResponse(success=True)
   
    def set_running_srv_callback(self, req: SetBoolRequest):
        rospy.loginfo("set_running")
        
        if req.data:
            self.thread_running = True
            rospy.ServiceProxy('/hand_trajectory/start', Trigger)()
        else:
            self.thread_running = False
            rospy.ServiceProxy('/hand_trajectory/stop', Trigger)()
        self.mecanum_pub.publish(Twist())

        return SetBoolResponse(success=req.data)

    def move_action(self, *args):
        status = 0
        t_start = rospy.get_time()
        while self.thread_running:
            current_time = rospy.get_time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = args[0]
                self.mecanum_pub.publish(twist)
                t_start = current_time + args[1]/50.0/self.linear_speed
            elif status == 1 and t_start < current_time:
                status = 0
                break
            rospy.sleep(0.01)
        self.mecanum_pub.publish(Twist())

    def get_hand_points_callback(self, msg):
        points = []
        left_and_right = [0]
        up_and_down = [0]
        if len(msg.points) >= 5:
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
            line = cv2.fitLine(np.array(points), cv2.DIST_L2, 0, 0.01, 0.01)
            angle = int(abs(math.degrees(math.acos(line[0][0]))))
            twist = Twist()
            if 0 <= angle < 30:
                if sum(left_and_right) > 0:
                    twist.linear.y = self.linear_speed
                else:
                    twist.linear.y = -self.linear_speed

            elif 60 < angle <= 90:
                if sum(up_and_down) > 0:
                    twist.linear.x = self.linear_speed
                else:
                    twist.linear.x = -self.linear_speed
            if self.th is None:
                self.th = threading.Thread(target=self.move_action, args=(twist, len(points)))
                self.th.start()
            else:
                if not self.th.is_alive():
                    self.th = threading.Thread(target=self.move_action, args=(twist, len(points)))
                    self.th.start()
                else:
                    self.thread_running = False
                    rospy.sleep(0.1)

if __name__ == "__main__":
    HandGestureControlNode('hand_gesture')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
    
