#!/usr/bin/env python3
# encoding: utf-8
import time
import math
import rospy
from enum import Enum
from std_srvs.srv import Trigger
import sdk.misc as misc
import geometry_msgs.msg as geo_msg
import sensor_msgs.msg as sensor_msg
from ros_robot_controller.msg import BuzzerState
from servo_msgs.msg import CommandDuration

AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_control')
        self.min_value = 0.1
        self.max_linear = rospy.get_param('~max_linear', 0.7)
        self.max_angular = rospy.get_param('~max_angular', 3.0)
        self.disable_servo_control = rospy.get_param('~disable_servo_control', 'true')
        cmd_vel = rospy.get_param('~cmd_vel', 'controller/cmd_vel')

        self.last_axes =  dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0, ] * len(BUTTON_MAP)))
        self.mode = 0

        self.jointw = rospy.Publisher('w_joint_controller/command_duration', CommandDuration, queue_size=1)
        self.joy_sub = rospy.Subscriber('ros_robot_controller/joy', sensor_msg.Joy, self.joy_callback)
        self.buzzer_pub = rospy.Publisher('ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.mecanum_pub = rospy.Publisher(cmd_vel, geo_msg.Twist, queue_size=1)

    def axes_callback(self, axes):
        twist = geo_msg.Twist()
        if abs(axes['lx']) < self.min_value:
            axes['lx'] = 0
        if abs(axes['ly']) < self.min_value:
            axes['ly'] = 0
        if abs(axes['rx']) < self.min_value:
            axes['rx'] = 0
        if abs(axes['ry']) < self.min_value:
            axes['ry'] = 0

        twist.linear.y = misc.val_map(axes['lx'], -1, 1, -self.max_linear, self.max_linear)
        twist.linear.x = misc.val_map(axes['ly'], -1, 1, -self.max_linear, self.max_linear)
        twist.angular.z = misc.val_map(axes['rx'], -1, 1, -self.max_angular, self.max_angular)
        self.mecanum_pub.publish(twist)

    def select_callback(self, new_state):
        pass

    def l1_callback(self, new_state):
        pass

    def l2_callback(self, new_state):
        pass

    def r1_callback(self, new_state):
        pass

    def r2_callback(self, new_state):
        pass

    def square_callback(self, new_state):
        pass

    def cross_callback(self, new_state):
        pass

    def circle_callback(self, new_state):
        pass

    def triangle_callback(self, new_state):
        pass

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            msg = BuzzerState()
            msg.freq = 2500
            msg.on_time = 0.1
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)

    def hat_xl_callback(self, new_state):
        pass

    def hat_xr_callback(self, new_state):
        pass

    def hat_yd_callback(self, new_state):
        pass

    def hat_yu_callback(self, new_state):
        pass

    def joy_callback(self, joy_msg: sensor_msg.Joy):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        hat_x, hat_y = axes['hat_x'], axes['hat_y']
        hat_xl, hat_xr = 1 if hat_x > 0.5 else 0, 1 if hat_x < -0.5 else 0
        hat_yu, hat_yd = 1 if hat_y > 0.5 else 0, 1 if hat_y < -0.5 else 0
        buttons = list(joy_msg.buttons)
        buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd, 0])
        buttons = dict(zip(BUTTON_MAP, buttons))
        for key, value in axes.items(): # 轴的值被改变
            if self.last_axes[key] != value:
                axes_changed = True
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(str(e))
        for key, value in buttons.items():
            new_state = ButtonState.Normal
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal
            callback = "".join([key, '_callback'])
            if new_state != ButtonState.Normal:
                rospy.loginfo(key + ': ' + str(new_state))
                if  hasattr(self, callback):
                    try:
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        rospy.logerr(str(e))
        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    node = JoystickController()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

