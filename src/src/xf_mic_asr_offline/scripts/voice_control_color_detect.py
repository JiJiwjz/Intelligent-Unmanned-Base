#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 语音开启颜色检测
import os
import json
import rospy
import signal
from std_msgs.msg import String
from sensor_msgs.msg import Image
from xf_mic_asr_offline import voice_play
from ros_robot_controller.msg import BuzzerState
from interfaces.srv import SetColorDetectParam
from interfaces.msg import ColorsInfo, ColorDetect


class VoiceControlColorDetectNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.count = 0
        self.color = None
        self.running = True
        self.last_color = None
        signal.signal(signal.SIGINT, self.shutdown)

        self.language = os.environ['ASR_LANGUAGE']
        rospy.Subscriber('/asr_node/voice_words', String, self.words_callback)
        rospy.Subscriber('/color_detect/color_info', ColorsInfo, self.get_color_callback)
        
        camera = rospy.get_param('/astra_camera/camera_name', 'depth_cam')  # 获取参数
        rospy.wait_for_message('/%s/rgb/image_raw' % camera, Image)
        rospy.wait_for_service('/voice_control/get_offline_result')

        self.play('running')
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        rospy.loginfo('唤醒口令: 小迈小迈(Wake up word: hello robot)')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        rospy.loginfo('控制指令: 开启颜色识别 关闭颜色识别(Voice command: start color recognition/stop color recognition)')

        while self.running:
            if self.color == 'red' and self.last_color != 'red':
                self.last_color = 'red'
                self.play('red')
                print('red')
            elif self.color == 'green' and self.last_color != 'green':
                self.last_color = 'green'
                self.play('green')
                print('green')
            elif self.color == 'blue' and self.last_color != 'blue':
                self.last_color = 'blue'
                self.play('blue')
                print('blue')
            else:
                self.count += 1
                rospy.sleep(0.01)
                if self.count > 50:
                    self.count = 0
                    self.last_color = self.color
        rospy.signal_shutdown('shutdown')

    def play(self, name):
        voice_play.play(name, language=self.language)

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def get_color_callback(self, msg):
        data = msg.data
        if data != []:
            if data[0].radius > 30:
                self.color = data[0].color
            else:
                self.color = None
        else:
            self.color = None

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            words = words.replace(' ', '')
        print('words:', words)
        if words is not None and words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)',
                                               '失败10次(Fail-10-times']:
            if words == '开启颜色识别' or words == 'start color recognition':
                msg_red = ColorDetect()
                msg_red.color_name = 'red'
                msg_red.detect_type = 'circle'
                msg_green = ColorDetect()
                msg_green.color_name = 'green'
                msg_green.detect_type = 'circle'
                msg_blue = ColorDetect()
                msg_blue.color_name = 'blue'
                msg_blue.detect_type = 'circle'
                res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)([msg_red, msg_green, msg_blue])
                if res.success:
                    self.play('open_success')
                else:
                    self.play('open_fail')
            elif words == '关闭颜色识别' or words == 'stop color recognition':
                res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)([])
                if res.success:
                    self.play('close_success')
                else:
                    self.play('close_fail')
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
    VoiceControlColorDetectNode('voice_control_color_detect')
