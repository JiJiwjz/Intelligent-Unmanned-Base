#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 语音控制导航
import os
import json
import rospy
import signal
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Int32
from xf_mic_asr_offline import voice_play
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from ros_robot_controller.msg import BuzzerState

class VoiceControlNavNode:
    def __init__(self, name):
        rospy.init_node(name)

        self.angle = None
        self.words = None
        self.running = True
        self.haved_stop = False
        self.last_status = Twist()

        self.language = os.environ['ASR_LANGUAGE']
        self.costmap = rospy.get_param('~costmap', '/move_base/local_costmap/costmap') 
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.mecanum_pub = rospy.Publisher('controller/cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.goal_status_pub = rospy.Publisher('move_base/result', MoveBaseActionResult, queue_size=1)
        rospy.Subscriber('/asr_node/voice_words', String, self.words_callback)
        rospy.Subscriber('/awake_node/angle', Int32, self.angle_callback)
        rospy.wait_for_service('/voice_control/get_offline_result')
        rospy.wait_for_message(self.costmap, OccupancyGrid)
        self.play('running')
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.mecanum_pub.publish(Twist())
        signal.signal(signal.SIGINT, self.shutdown)

        rospy.loginfo('唤醒口令: 小迈小迈(Wake up word: hello robot)')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        rospy.loginfo('控制指令: 去A点 去B点 去C点 回原点(Voice command: go to A/B/C point go back to the start')

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

    def run(self):
        while not rospy.is_shutdown() and self.running:
            if self.words is not None:
                pose = PoseStamped()
                pose.header.frame_id = self.map_frame
                pose.header.stamp = rospy.Time.now()
                if self.words == '去A点' or self.words == 'go to A point':
                    print('>>>>>> go a')
                    pose.pose.position.x = 1
                    pose.pose.position.y = -1
                    pose.pose.orientation.w = 1
                    self.play('go_a')
                    self.goal_pub.publish(pose)
                elif self.words == '去B点' or self.words == 'go to B point':
                    print('>>>>>> go b')
                    pose.pose.position.x = 2
                    pose.pose.position.y = 0
                    pose.pose.orientation.w = 1
                    self.play('go_b')
                    self.goal_pub.publish(pose)
                elif self.words == '去C点' or self.words == 'go to C point':
                    print('>>>>>> go c')
                    pose.pose.position.x = 1
                    pose.pose.position.y = 1
                    pose.pose.orientation.w = 1
                    self.play('go_c')
                    self.goal_pub.publish(pose)
                elif self.words == '回原点' or self.words == 'go back to the start':
                    print('>>>>>> go origin')
                    pose.pose.position.x = 0
                    pose.pose.position.y = 0
                    pose.pose.orientation.w = 1
                    self.play('go_origin')
                    self.goal_pub.publish(pose)
                elif self.words == '休眠(Sleep)':
                    rospy.sleep(0.01)
                self.words = None
            else:
                rospy.sleep(0.01)


if __name__ == "__main__":
    VoiceControlNavNode('voice_control_nav')
