#!/usr/bin/env python3
# encoding: utf-8
import os
import sys
import rospy
from threading import Thread
from collections import deque

from servo_driver import servo_io
from ros_robot_controller.msg import SetBusServoState, BusServoState
from servo_msgs.msg import MultiRawIdPosDur, ServoState, ServoStateList, RawIdPosDur

class SerialProxy:
    def __init__(self,
                 port_id= 1,
                 min_motor_id=1,
                 max_motor_id=25,
                 connected_ids=[],
                 update_rate=5,
                 fake_read=False):
        self.port_id = str(port_id)
        self.min_servo_id = min_motor_id
        self.max_servo_id = max_motor_id
        self.servos = connected_ids
        self.update_rate = update_rate
        self.fake_read = fake_read
        self.running = False
        self.servo_io = None
        self.servos_static_info = {}

        self.actual_rate = update_rate
        self.error_counts = {'non_fatal': 0, 'checksum': 0, 'dropped': 0}
        self.current_state = ServoStateList()
        self.servo_states_pub = rospy.Publisher('servo_controllers/port_id_{}/servo_states'.format(self.port_id),
                                                ServoStateList, queue_size=1)

        self.servo_command_sub = rospy.Subscriber('servo_controllers/port_id_{}/id_pos_dur'.format(self.port_id),
                                                  RawIdPosDur,
                                                  self.id_pos_dur_cb)
        self.servo_command_sub = rospy.Subscriber('servo_controllers/port_id_{}/multi_id_pos_dur'.format(self.port_id),
                                                  MultiRawIdPosDur,
                                                  self.multi_id_pos_dur_cb)
        self.servo_state_pub = rospy.Publisher('ros_robot_controller/bus_servo/set_state', SetBusServoState, queue_size=10)

    def id_pos_dur_cb(self, msg):
        self.servo_io.set_position(msg.id, msg.position, msg.duration)

    def multi_id_pos_dur_cb(self, msg):
        for id_pos_dur in msg.id_pos_dur_list:
            self.servo_io.set_position(id_pos_dur.id, id_pos_dur.position, id_pos_dur.duration)

    def connect(self):
        try:
            self.servo_io = servo_io.ServoIO()
            self.__find_motors()
        except servo_io.SerialOpenError as e:
            rospy.logfatal(e.message)
            sys.exit(1)
        self.running = True
        if self.update_rate > 0:
            Thread(target=self.__update_servo_states).start()

    def disconnect(self):
        self.running = False

    def __find_motors(self):
        rospy.loginfo(
            'Pinging motor IDs %d through %d...' % (self.min_servo_id, self.max_servo_id))
        if not self.servos:
            for servo_id in range(self.min_servo_id, self.max_servo_id + 1):
                result = self.servo_io.ping(servo_id)
                if result:
                    self.servos.append(servo_id)
        if not self.servos:
            rospy.logfatal('No motors found.')
            sys.exit(1)
        status_str = 'Found %d motors - ' % (len(self.servos))
        rospy.loginfo('%s, initialization complete.' % status_str[:-2])

    def __update_servo_states(self):
        num_events = 50
        rates = deque([float(self.update_rate)] * num_events, maxlen=num_events)
        last_time = rospy.Time.now()

        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown() and self.running:
            # get current state of all motors and publish to motor_states topic
            servo_states = []
            for servo_id in self.servos:
                try:
                    state = self.servo_io.get_feedback(servo_id, self.fake_read)
                    if state:
                        servo_states.append(ServoState(**state))
                        if servo_io.exception:
                            raise servo_io.exception
                except Exception as e:
                    rospy.logerr(e)

            if servo_states:
                msl = ServoStateList()
                msl.servo_states = servo_states
                self.servo_states_pub.publish(msl)
                self.current_state = msl
                # calculate actual update rate
                current_time = rospy.Time.now()
                rates.append(1.0 / (current_time - last_time).to_sec())
                self.actual_rate = round(sum(rates) / num_events, 2)
                last_time = current_time

            rate.sleep()
