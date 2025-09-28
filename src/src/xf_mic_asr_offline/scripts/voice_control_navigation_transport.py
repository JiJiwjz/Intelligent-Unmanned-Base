#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# 语音导航搬运
import os
import json
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from xf_mic_asr_offline import voice_play
from interfaces.msg import Pose2D
from interfaces.srv import SetPose2D
from ros_robot_controller.msg import BuzzerState

class VoiceControlNavigationTransportNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.running = True

        self.language = os.environ['ASR_LANGUAGE']
        self.pick_position = rospy.get_param('~pick_position')
        rospy.Subscriber('/asr_node/voice_words', String, self.words_callback)
        rospy.wait_for_service('/voice_control/get_offline_result')
        rospy.wait_for_message("/move_base/local_costmap/costmap", OccupancyGrid)
        self.play('running')
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        rospy.loginfo('唤醒口令: 小迈小迈(Wake up word: hello robot)')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        rospy.loginfo('控制指令: 导航搬运(Voice command: navigate and transport)')
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerr(str(e))
            rospy.loginfo("Shutting down")

    def play(self, name):
        voice_play.play(name, language=self.language)

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            words = words.replace(' ', '')
        print('words:', words)
        if words is not None and words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)',
                                               '失败10次(Fail-10-times']:
            if words == '导航搬运' or words == 'navigate and transport':
                pose = Pose2D()
                pose.x = self.pick_position[0]
                pose.y = self.pick_position[1]
                pose.roll = self.pick_position[2]
                pose.pitch = self.pick_position[3]
                pose.yaw = self.pick_position[4]
                res = rospy.ServiceProxy('/navigation_transport/pick', SetPose2D)(pose)
                if res.success:
                    self.play('start_navigating')
                else:
                    self.play('open_fail')
        elif words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif words == '休眠(Sleep)':
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)


if __name__ == "__main__":
    VoiceControlNavigationTransportNode('voice_control_navigation_transport')
