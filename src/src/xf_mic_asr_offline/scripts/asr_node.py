#!/usr/bin/env python3
# coding=utf-8
# @Author: Aiden
import rospy
import signal
from std_msgs.msg import String, Bool
from xf_mic_asr_offline.srv import GetOfflineResult

class ASRNode:
    def __init__(self, name):
        rospy.init_node(name)

        self.init_finish = False
        self.awake_flag = False
        self.recognize_fail_count = 0
        self.recognize_fail_count_threshold = 5 #yct add:两次失败直接休眠
        self.running = True
        signal.signal(signal.SIGINT, self.shutdown)

        self.confidence_threshold = rospy.get_param('~confidence', 18)
        self.seconds_per_order = rospy.get_param('~seconds_per_order', 3)

        self.control = rospy.Publisher('~voice_words', String, queue_size=1)
        rospy.Subscriber('/awake_node/awake_flag', Bool, self.awake_flag_callback)

        rospy.wait_for_service('/voice_control/get_offline_result')
        rate = rospy.Rate(10)
        while self.running:
            self.proces_regogniztion()
            rate.sleep()
        rospy.signal_shutdown('shutdown')

    def shutdown(self):
        self.running = False
        rospy.loginfo('shutdown')

    def awake_flag_callback(self, msg):
        self.awake_flag = msg.data
        self.recognize_fail_count = 0
        count_msg = String()
        count_msg.data = "唤醒成功(wake-up-success)"
        self.control.publish(count_msg)

    def proces_regogniztion(self)->None:
        if self.awake_flag:
            response = rospy.ServiceProxy('/voice_control/get_offline_result', GetOfflineResult)(1, self.confidence_threshold, self.seconds_per_order)
            if response.text == "休眠(Sleep)":  # 主动休眠(active sleep)
                self.awake_flag = 0
                self.recognize_fail_count = 0
                print('\033[1;32m休眠(Sleep)\033[0m')
            elif response.result == "ok":  # 清零被动休眠相关变量(clear passive sleep relative variable)
                self.awake_flag = 0
                self.recognize_fail_count = 0
                count_msg = String()
                count_msg.data = response.text
                self.control.publish(count_msg)
                print('\033[1;32mok\033[0m')
            elif response.result == "fail":  # 记录识别失败次数(record the number of recognition failures)
                self.recognize_fail_count += 1
                if self.recognize_fail_count == 5:  # 连续识别失败5次，用户界面显示提醒信息(fail to recognize for consecutive 5 times.Warning occurs on user interface)
                    count_msg = String()
                    count_msg.data = "失败5次(Fail-5-times)"
                    self.control.publish(count_msg)
                    print('\033[1;32m失败5次(Fail-5-times)\033[0m')
                elif self.recognize_fail_count == 10:  # 连续识别失败10次，用户界面显示提醒信息(fail to recognize for consecutive 10 times.Warning occurs on user interface)
                    count_msg = String()
                    count_msg.data = "失败10次(Fail-10-times)"
                    self.control.publish(count_msg)
                    print('\033[1;32m失败10次(Fail-10-times)\033[0m')
                elif self.recognize_fail_count >= self.recognize_fail_count_threshold:  # 被动休眠(passive sleep)
                    self.awake_flag = 0
                    count_msg = String()
                    count_msg.data = "休眠(Sleep)"
                    self.control.publish(count_msg)
                    self.recognize_fail_count = 0
                    print('\033[1;32m休眠(Sleep)\033[0m')

if __name__ == "__main__":
    ASRNode('asr_node')
