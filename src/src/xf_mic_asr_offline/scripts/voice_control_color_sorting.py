#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 语音开启颜色分拣
import os
import json
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from xf_mic_asr_offline import voice_play
from ros_robot_controller.msg import BuzzerState

class VoiceControlColorSortingNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.running = True

        self.language = os.environ['ASR_LANGUAGE']
        rospy.Subscriber('/asr_node/voice_words', String, self.words_callback)
        camera = rospy.get_param('/gemini_camera/camera_name', 'depth_cam')  # 获取参数
        rospy.wait_for_message('/%s/rgb/image_raw' % camera, Image)
        rospy.wait_for_service('/voice_control/get_offline_result')
        self.play('running')
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        rospy.loginfo('唤醒口令: 小迈小迈(Wake up word: hello robot)')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        rospy.loginfo('控制指令: 开启颜色分拣 关闭颜色分拣(Voice command: start color sorting/stop color sorting)')
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
            if words == '开启颜色分拣' or words == 'start color sorting':
                res = rospy.ServiceProxy('/color_sorting/start', Trigger)()
                if res.success:
                    self.play('open_success')
                else:
                    self.play('open_fail')
            elif words == '关闭颜色分拣' or words == 'stop color sorting':
                res = rospy.ServiceProxy('/color_sorting/stop', Trigger)()
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
    VoiceControlColorSortingNode('voice_control_colorsorting')
