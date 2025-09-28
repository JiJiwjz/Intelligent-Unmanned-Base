#!/usr/bin/python3
# coding=utf8
# 麦克纳姆轮底盘控制(Mecanum wheel chassis control)
import math
from ros_robot_controller.msg import MotorState

class MecanumChassis:
    # wheelbase = 0.206   # 前后轴距
    # track_width = 0.194 # 左右轴距
    # wheel_diameter = 0.0965  # 轮子直径
    def __init__(self, wheelbase=0.206, track_width=0.194, wheel_diameter=0.0965):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_covert(self, speed):
        """
        covert speed m/s to rps/s
        :param speed:
        :return:
        """
        return speed / (math.pi * self.wheel_diameter)

    def set_velocity(self, speed, direction, angular_rate):
        """
        Use polar coordinates to control moving
                    x
        v1 motor1|  ↑  |motor3 v2
          +  y - |     |
        v4 motor2|     |motor4 v3
        :param speed: m/s
        :param direction: Moving direction 0~2pi, 1/2pi<--- ↑ ---> 3/2pi
        :param angular_rate:  The speed at which the chassis rotates rad/sec
        :param fake:
        :return:
        """
        vx = speed * math.sin(direction)
        vy = speed * math.cos(direction)
        vp = angular_rate * (self.wheelbase + self.track_width) / 2
        v1 = vx - vy - vp
        v2 = vx + vy - vp
        v3 = -vx - vy - vp
        v4 = -vx + vy - vp
        v_s = [self.speed_covert(v) for v in [v1, v2, v3, v4]]
        data = []
        for i in range(len(v_s)):
            msg = MotorState()
            msg.id = i + 1
            msg.rps = v_s[i]
            data.append(msg)    
        return data
