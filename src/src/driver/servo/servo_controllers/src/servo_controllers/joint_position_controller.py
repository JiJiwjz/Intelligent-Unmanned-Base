#!/usr/bin/env python3
# encoding: utf-8
import rospy
from servo_msgs.msg import JointState
from servo_controllers.joint_controller import JointController

class JointPositionController(JointController):
    def __init__(self, servo_io, controller_namespace, port_id):
        JointController.__init__(self, servo_io, controller_namespace, port_id)

        self.param_namespace = "~controllers/" + controller_namespace
        self.port_id = str(port_id)

        self.servo_id = rospy.get_param(self.param_namespace + '/servo/id')
        self.initial_position_raw = rospy.get_param(self.param_namespace + '/servo/init')
        self.min_angle_raw = rospy.get_param(self.param_namespace + '/servo/min')
        self.max_angle_raw = rospy.get_param(self.param_namespace + '/servo/max')
        if rospy.has_param(self.param_namespace + '/servo/acceleration'):
            self.acceleration = rospy.get_param(self.param_namespace + '/servo/acceleration')
        else:
            self.acceleration = None

        self.flipped = self.min_angle_raw > self.max_angle_raw

        self.joint_state = JointState(name=self.joint_name, servo_ids=[self.servo_id])

    def initialize(self):
        # verify that the expected motor is connected and responding
        # available_ids = rospy.get_param('%s/serial_ports/%s/connected_ids' % (rospy.get_name(), self.port_namespace),
        #                                 [])
        # if not self.servo_id in available_ids:
        #     rospy.logwarn('The specified servo id is not connected and responding.')
        #     rospy.logwarn('Available ids: %s' % str(available_ids))
        #     rospy.logwarn('Specified id: %d' % self.servo_id)
        #     return False

        self.RADIANS_PER_ENCODER_TICK = 24 / 36 * (3.1415926 * 2) / 1000
        # self.RADIANS_PER_ENCODER_TICK = rospy.get_param(
        # 'servo/%s/%d/radians_per_encoder_tick' % (self.port_namespace, self.servo_id))
        self.ENCODER_TICKS_PER_RADIAN = 1000 / (24 / 36 * (3.1415926 * 2))
        # self.ENCODER_TICKS_PER_RADIAN = rospy.get_param(
        # 'servo/%s/%d/encoder_ticks_per_radian' % (self.port_namespace, self.servo_id))

        if self.flipped:
            self.min_angle = (self.initial_position_raw - self.min_angle_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.initial_position_raw - self.max_angle_raw) * self.RADIANS_PER_ENCODER_TICK
        else:
            self.min_angle = (self.min_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.max_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK

        self.ENCODER_RESOLUTION = 1000  # rospy.get_param(
        # 'servo/%s/%d/encoder_resolution' % (self.port_namespace, self.servo_id))
        self.MAX_POSITION = self.ENCODER_RESOLUTION - 1
        self.VELOCITY_PER_TICK = 10  # rospy.get_param(
        # 'servo/%s/%d/radians_second_per_encoder_tick' % (self.port_namespace, self.servo_id))
        self.MAX_VELOCITY = 100
        # rospy.get_param('servo/%s/%d/max_velocity' % (self.port_namespace, self.servo_id))
        self.MIN_VELOCITY = self.VELOCITY_PER_TICK

        return True

    def pos_rad_to_raw(self, pos_rad):
        if pos_rad < self.min_angle:
            pos_rad = self.min_angle
        elif pos_rad > self.max_angle:
            pos_rad = self.max_angle
        return self.rad_to_raw(pos_rad, self.initial_position_raw, self.flipped, self.ENCODER_TICKS_PER_RADIAN)

    def spd_rad_to_raw(self, spd_rad):
        if spd_rad < self.MIN_VELOCITY:
            spd_rad = self.MIN_VELOCITY
        elif spd_rad > self.joint_max_speed:
            spd_rad = self.joint_max_speed
        # velocity of 0 means maximum, make sure that doesn't happen
        return max(1, int(round(spd_rad / self.VELOCITY_PER_TICK)))


    def process_servo_states(self, state_list):
        if self.running:
            state = list(filter(lambda state: state.id == self.servo_id, state_list.servo_states))
            if state:
                state = state[0]
                self.joint_state.servo_temps = [0]
                self.joint_state.goal_pos = self.raw_to_rad(state.goal, self.initial_position_raw, self.flipped,
                                                            self.RADIANS_PER_ENCODER_TICK)
                self.joint_state.current_pos = self.raw_to_rad(state.position, self.initial_position_raw, self.flipped,
                                                               self.RADIANS_PER_ENCODER_TICK)
                self.joint_state.error = state.error * self.RADIANS_PER_ENCODER_TICK
                self.joint_state.velocity = 10  # state.speed * self.VELOCITY_PER_TICK
                self.joint_state.header.stamp = rospy.Time.from_sec(state.timestamp)
                self.joint_state_pub.publish(self.joint_state)

    def process_command(self, msg):
        angle = msg.data
        pos = self.pos_rad_to_raw(angle)
        self.servo_io.set_position(self.servo_id, pos)

    def process_command_duration(self, req):
        angle = req.data
        duration = req.duration
        pos = self.pos_rad_to_raw(angle)
        self.servo_io.set_position(self.servo_id, int(pos), int(duration))

    def set_position(self, pos, duration):
        self.servo_io.set_position(self.servo_id, int(pos), int(duration))

    def set_position_in_rad(self, rad, duration):
        pos = self.pos_rad_to_raw(rad)
        self.servo_io.set_position(self.servo_id, pos, duration)
