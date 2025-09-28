#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/23
# @author:aiden
# 颜色跟踪
import rospy
import signal
import sdk.pid as pid
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from kinematics import transform
from std_srvs.srv import Trigger, TriggerResponse
from servo_msgs.msg import MultiRawIdPosDur
from interfaces.msg import ColorsInfo, ColorDetect
from interfaces.srv import SetColorDetectParam, SetString
from kinematics.kinematics_control import set_pose_target
from servo_controllers.bus_servo_control import set_servos

class ColorTrackNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.z_dis = 0.36
        self.y_dis = 500
        self.x_init = transform.link3 + transform.tool_link
        self.center = None
        self.running = True
        self.start = False
        self.name = name

        self.pid_z = pid.PID(0.00008, 0.0, 0.0)
        self.pid_y = pid.PID(0.05, 0.0, 0.0)

        signal.signal(signal.SIGINT, self.shutdown)

        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)  # 底盘控制

        rospy.Subscriber('/color_detect/color_info', ColorsInfo, self.get_color_callback)

        rospy.Service('~start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色

        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/servo_manager/init_finish') and rospy.get_param('/joint_states_publisher/init_finish') and rospy.get_param('/kinematics/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        rospy.sleep(0.2)
        self.init_action()
        
        if rospy.get_param('~start', True):
            self.start_srv_callback(None)
            self.set_color_srv_callback(String('red'))

        rospy.set_param('~init_finish', True)

        self.color_track()

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

    def set_color_srv_callback(self, msg):
        rospy.loginfo("set_color")

        msg_red = ColorDetect()
        msg_red.color_name = msg.data
        msg_red.detect_type = 'circle'
        res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)([msg_red])
        if res.success and msg.data != '':
            print('start_track_' + msg_red.color_name)
        else:
            print('track_fail')

        return [True, 'set_color']

    def start_srv_callback(self, msg):
        rospy.loginfo("start color track")

        self.start = True

        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop color track')

        self.start = False
        res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)()
        if res.success:
            print('set color success')
        else:
            print('set color fail')

        return TriggerResponse(success=True)

    def get_color_callback(self, msg):
        if msg.data != []:
            if msg.data[0].radius/msg.data[0].width > 1/64:
                self.center = msg.data[0]
            else:
                self.center = None 
        else:
            self.center = None

    def color_track(self):
        while self.running:
            if self.center is not None and self.start:
                self.pid_y.SetPoint = self.center.width/2 
                self.pid_y.update(self.center.x)
                self.y_dis += self.pid_y.output
                if self.y_dis < 200:
                    self.y_dis = 200
                if self.y_dis > 800:
                    self.y_dis = 800

                self.pid_z.SetPoint = self.center.height/2 
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
        
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    ColorTrackNode('color_track')
