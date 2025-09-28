#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# 语音控制颜色追踪
import os
import json
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from interfaces.srv import SetString
from xf_mic_asr_offline import voice_play
from ros_robot_controller.msg import BuzzerState

class VoiceControlGrabNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.language = os.environ['ASR_LANGUAGE']
        rospy.Subscriber('/asr_node/voice_words', String, self.words_callback)
        camera = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')  # 获取参数
        rospy.wait_for_message('/%s/rgb/image_raw' % camera, Image)
        rospy.wait_for_service('/voice_control/get_offline_result')
        self.play('running')
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        rospy.loginfo('唤醒口令: 小奇小奇(Wake up word: hello robot)')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        rospy.loginfo('控制指令: 追踪红色 追踪绿色 追踪蓝色 停止追踪(Voice command: track red/green/blue)')
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
            if words == '追踪红色' or words == 'track red':
                res = rospy.ServiceProxy('/track_and_grab/set_color', SetString)("red")
                if res.success:
                    self.play('ok')
                else:
                    self.play('open_fail')
            elif words == '追踪绿色' or words == 'track green':
                res = rospy.ServiceProxy('/track_and_grab/set_color', SetString)("green")
                if res.success:
                    self.play('ok')
                else:
                    self.play('open_fail')
            elif words == '追踪蓝色' or words == 'track blue':
                res = rospy.ServiceProxy('/track_and_grab/set_color', SetString)("blue")
                if res.success:
                    self.play('ok')
                else:
                    self.play('open_fail')
            elif words == '停止追踪' or words == 'stop tracking':
                res = rospy.ServiceProxy('/track_and_grab/stop', Trigger)()
                if res.success:
                    self.play('stop')
                else:
                    self.play('stop_fail')
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
    VoiceControlGrabNode('vc_track_and_grab')
