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
from interfaces.srv import SetStringList
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
        rospy.loginfo('控制指令: 夹取球体 夹取圆柱体 夹取立方体 夹取红色 夹取蓝色 关闭夹取(Voice command: gripping the sphere/cylinder/cuboid red/blue stop gripping)')
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
            if words == '夹取球体' or words == 'gripping the sphere':
                res = rospy.ServiceProxy('/object_classification/set_shape', SetStringList)(["sphere"])
                if res.success:
                    self.play('ok')
                else:
                    self.play('open_fail')
            elif words == '夹取圆柱体' or words == 'gripping the cylinder':
                res = rospy.ServiceProxy('/object_classification/set_shape', SetStringList)(["cylinder"])
                if res.success:
                    self.play('ok')
                else:
                    self.play('open_fail')
            elif words == '夹取立方体' or words == 'gripping the cuboid':
                res = rospy.ServiceProxy('/object_classification/set_shape', SetStringList)(["cuboid"])
                if res.success:
                    self.play('ok')
                else:
                    self.play('open_fail')
            elif words == '夹取红色' or words == 'gripping red':
                res = rospy.ServiceProxy('/object_classification/set_color', SetStringList)(["red"])
                if res.success:
                    self.play('ok')
                else:
                    self.play('open_fail')
            elif words == '夹取蓝色' or words == 'gripping blue':
                res = rospy.ServiceProxy('/object_classification/set_color', SetStringList)(["blue"])
                if res.success:
                    self.play('ok')
                else:
                    self.play('open_fail')
            elif words == '关闭夹取' or words == 'stop gripping':
                res = rospy.ServiceProxy('/object_classification/stop', Trigger)()
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
    VoiceControlGrabNode('vc_object_classification')
