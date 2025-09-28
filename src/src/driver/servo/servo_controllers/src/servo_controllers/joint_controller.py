#!/usr/bin/env python3
# encoding: utf-8
import rospy
from servo_msgs.msg import ServoStateList
from servo_msgs.msg import JointState
from servo_msgs.msg import CommandDuration

from std_msgs.msg import Float64

class JointController:
    def __init__(self, servo_io, controller_namespace, port_id):
        self.running = False
        self.servo_io = servo_io
        self.controller_namespace = controller_namespace
        self.param_namespace = "~controllers/" + controller_namespace
        self.port_id = str(port_id)
        self.joint_name = rospy.get_param(self.param_namespace + '/joint_name')
        self.joint_speed = rospy.get_param(self.param_namespace + '/joint_speed', 1.0)
    
    def initialize(self):
        raise NotImplementedError

    def start(self):
        self.running = True
        self.joint_state_pub = rospy.Publisher(self.controller_namespace + '/state', JointState, queue_size=1)
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', Float64, self.process_command)
        self.command_time_sub = rospy.Subscriber(self.controller_namespace + '/command_duration', CommandDuration,
                                                 self.process_command_duration, queue_size=10)
        self.servo_states_sub = rospy.Subscriber('servo_controllers/port_id_%s/servo_states' % self.port_id, ServoStateList,
                                                 self.process_servo_states)

    def stop(self):
        self.running = False
        self.joint_state_pub.unregister()
        self.servo_states_sub.unregister()
        self.command_sub.unregister()
        self.command_time_sub.unregister()

    def process_servo_states(self,  req):
        pass

    def process_command_duration(self, req):
        pass

    def process_command(self, msg):
        raise NotImplementedError

    def rad_to_raw(self, angle, initial_position_raw, flipped, encoder_ticks_per_radian):
        """ angle is in radians """
        angle_raw = angle * encoder_ticks_per_radian
        return int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw))

    def raw_to_rad(self, raw, initial_position_raw, flipped, radians_per_encoder_tick):
        return (initial_position_raw - raw if flipped else raw - initial_position_raw) * radians_per_encoder_tick
