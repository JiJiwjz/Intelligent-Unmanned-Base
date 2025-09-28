#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 语音控制移动
import os
import json
import math
import rospy
import signal
import numpy as np
import sdk.pid as pid
import sdk.misc as misc
import sensor_msgs.msg as sensor_msg
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
from xf_mic_asr_offline import voice_play
from ros_robot_controller.msg import BuzzerState

MAX_SCAN_ANGLE = 240  # 激光的扫描角度,去掉总是被遮挡的部分degree
CAR_WIDTH = 0.4  # meter


class VoiceControlNode:
    def __init__(self, name):
        rospy.init_node(name)

        self.angle = None
        self.words = None
        self.running = True
        self.haved_stop = False
        self.lidar_follow = False
        self.start_follow = False
        self.last_status = Twist()
        self.threshold = 3
        self.speed = 0.3
        self.stop_dist = 0.4
        self.count = 0
        self.scan_angle = math.radians(45)

        self.pid_yaw = pid.PID(1.6, 0, 0.16)
        self.pid_dist = pid.PID(1.7, 0, 0.16)

        self.language = os.environ['ASR_LANGUAGE']
        self.lidar_type = os.environ.get('LIDAR_TYPE')
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
        self.lidar_sub = rospy.Subscriber('/scan', sensor_msg.LaserScan, self.lidar_callback)
        rospy.Subscriber('/asr_node/voice_words', String, self.words_callback)
        rospy.Subscriber('/awake_node/angle', Int32, self.angle_callback)
        rospy.wait_for_service('/voice_control/get_offline_result')
        rospy.sleep(rospy.get_param('~delay', 0))
        self.play('running')
        self.mecanum_pub.publish(Twist())
        signal.signal(signal.SIGINT, self.shutdown)
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        rospy.loginfo('唤醒口令: 小迈小迈(Wake up word: hello robot)')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        rospy.loginfo('控制指令: 左转 右转 前进 后退 过来(Voice command: turn left/turn right/go forward/go backward/come here)')

        self.time_stamp = rospy.get_time()
        self.current_time_stamp = rospy.get_time()
        self.run()

    def play(self, name):
        voice_play.play(name, language=self.language)

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')

    def words_callback(self, msg):
        self.words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            self.words = self.words.replace(' ', '')
        print('words:', self.words)
        if self.words is not None and self.words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)',
                                                         '失败10次(Fail-10-times']:
            pass
        elif self.words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif self.words == '休眠(Sleep)':
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)

    def angle_callback(self, msg):
        self.angle = msg.data
        print('angle:', self.angle)
        self.start_follow = False
        self.mecanum_pub.publish(Twist())

    def lidar_callback(self, lidar_data: sensor_msg.LaserScan):
        twist = Twist()
        if self.lidar_type != 'G4':
            max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[:max_index]  # 左半边数据
            right_ranges = lidar_data.ranges[::-1][:max_index]  # 右半边数据
        elif self.lidar_type == 'G4':
            min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / lidar_data.angle_increment)
            max_index = int(math.radians(180) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[::-1][min_index:max_index][::-1]  # 左半边数据
            right_ranges = lidar_data.ranges[min_index:max_index][::-1]  # 右半边数据
        if self.start_follow:
            angle = self.scan_angle / 2
            angle_index = int(angle / lidar_data.angle_increment + 0.50)
            left_range, right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])

            ranges = np.append(right_range[::-1], left_range)
            nonzero = ranges.nonzero()
            dist = ranges[nonzero].min()
            min_index = list(ranges).index(dist)
            angle = -angle + lidar_data.angle_increment * min_index  # 计算最小值对应的角度
            if dist < self.threshold and abs(math.degrees(angle)) > 5:  # 控制左右
                self.pid_yaw.update(-angle)
                twist.angular.z = misc.set_range(self.pid_yaw.output, -self.speed * 6, self.speed * 6)
            else:
                self.pid_yaw.clear()

            if dist < self.threshold and abs(self.stop_dist - dist) > 0.02:
                self.pid_dist.update(self.stop_dist - dist)
                twist.linear.x = misc.set_range(self.pid_dist.output, -self.speed, self.speed)
            else:
                self.pid_dist.clear()
            if abs(twist.angular.z) < 0.008:
                twist.angular.z = 0
            if abs(twist.linear.x) < 0.05:
                twist.linear.x = 0
            if twist.linear.x == 0 and twist.angular.z == 0:
                self.count += 1
            if self.count >= 10:
                self.count = 0
                self.start_follow = False
            self.mecanum_pub.publish(twist)

    def run(self):
        while not rospy.is_shutdown() and self.running:
            if self.words is not None:
                twist = Twist()
                if self.words == '前进' or self.words == 'go forward':
                    self.play('go')
                    self.time_stamp = rospy.get_time() + 2
                    twist.linear.x = 0.2
                elif self.words == '后退' or self.words == 'go backward':
                    self.play('back')
                    self.time_stamp = rospy.get_time() + 2
                    twist.linear.x = -0.2
                elif self.words == '左转' or self.words == 'turn left':
                    self.play('turn_left')
                    self.time_stamp = rospy.get_time() + 2
                    twist.angular.z = 0.8
                elif self.words == '右转' or self.words == 'turn right':
                    self.play('turn_right')
                    self.time_stamp = rospy.get_time() + 2
                    twist.angular.z = -0.8
                elif self.words == '过来' or self.words == 'come here':
                    self.play('come')
                    if 330 > self.angle > 150:
                        twist.angular.z = 1
                        self.angle = self.angle - 150
                    else:
                        twist.angular.z = -1
                        if self.angle <= 150:
                            self.angle = 150 - self.angle
                        else:
                            self.angle = 150 + 360 - self.angle
                    self.time_stamp = rospy.get_time() + math.radians(self.angle)
                    self.lidar_follow = True
                elif self.words == '休眠(Sleep)':
                    rospy.sleep(0.01)
                self.words = None
                self.haved_stop = False
                self.mecanum_pub.publish(twist)
            else:
                rospy.sleep(0.01)
            self.current_time_stamp = rospy.get_time()
            if self.time_stamp < self.current_time_stamp and not self.haved_stop:
                self.mecanum_pub.publish(Twist())
                self.haved_stop = True
                if self.lidar_follow:
                    self.lidar_follow = False
                    self.start_follow = True


if __name__ == "__main__":
    VoiceControlNode('voice_control')
