#!/usr/bin/env python3
# coding=utf-8
# @Author: Aiden
# @Date: 2023/11/16
# 环形麦克风阵列Python SDK
import re
import time
import json
import rospy
import serial
import signal
from std_srvs.srv import Trigger
from std_msgs.msg import Int32, Bool
from xf_mic_asr_offline.srv import SetString

class CircleMic:
    def __init__(self, port, flag_pub, angle_pub):
        self.serialHandle = serial.Serial(None, 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=5)
        self.serialHandle.rts = False
        self.serialHandle.dtr = False
        self.serialHandle.setPort(port)
        self.serialHandle.open()

        self.running = True
        self.key_type = r"{\"code.*?\"}"
        self.pattern = re.compile(r"{\"content.*?aiui_event\"}")
        self.flag_pub = flag_pub
        self.angle_pub = angle_pub
        self.start_time = time.time()

    # 麦克风阵列切换
    def switch_mic(self, mic="mic6_circle"):
        # mic：麦克风阵列类型，mic4：线性4麦，mic6：线性6麦， mic6_circle：环形6麦
        param ={
            "type": "switch_mic",
            "content": {
                "mic": "mic6_circle"
            }
        }
        param['content']['mic'] = mic 
        header = [0xA5, 0x01, 0x05]  
        res = self.send(header, param)
        if res is not None:
            pattern = re.compile(self.key_type)
            m = re.search(pattern, str(res))
            if m is not None:
                m = m.group(0)
                if m is not None:
                    return m
        
        return False

    # 获取版本信息
    def get_setting(self):
        param ={
            "type": "version"
        }
        
        header = [0xA5, 0x01, 0x05]  
        res = self.send(header, param)
        if res is not None:
            pattern = re.compile(self.key_type)
            m = re.search(pattern, str(res))
            if m is not None:
                m = m.group(0)
                if m is not None:
                    return m
        
        return False

    # 唤醒词更换（浅定制）
    #def set_wakeup_word(self, str_pinyin="xiao3 qi2 xiao3 qi2"): 
    def set_wakeup_word(self, str_pinyin="tang2 xiao3 tian1"):
        # 参数为中文拼音
        # 更多参数请参考https://aiui.xfyun.cn/doc/aiui/3_access_service/access_hardware/r818/protocol.html
        param ={
            "type": "wakeup_keywords",
            "content": {
                "keyword": "tang2 xiao3 tian1",
                "threshold": "500" 
            }
        }
        
        param['content']['keyword'] = str_pinyin
        header = [0xA5, 0x01, 0x05]  
        print('setting wakeup keywords need about 30s')
        print('setting ...')
        self.send(header, param)
        while time.time() - self.start_time < 30:
            time.sleep(0.1)

    # 计算校验和
    def calculate_checksum(self, bytes_list):
        checksum = sum(bytes_list) & 0xFF
        checksum = (~checksum + 1) & 0xFF
        return checksum

    # 数据串口发送
    def send_data(self, header, args):
        packet = header 

        data = bytes(json.dumps(args), encoding="utf8")

        length = len(data)
        low_length = int(length & 0xFF)
        high_length = int(length >> 8)
        
        packet.extend([low_length, high_length])
        packet.extend([0x00, 0x00])
        
        packet.extend(data)
        checksum = self.calculate_checksum(packet)
        packet.append(checksum)
        
        self.serialHandle.write(packet)  # 发送主控消息

    # 发送数据
    def send(self, header, args):
        self.serialHandle.write([0xa5, 0x01, 0x01, 0x04, 0x00, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0xb0])  # 发送握手请求
        while True:
            result = None
            recv_data = self.serialHandle.read()
            header_ = [b'\xa5', b'\x01', b'\xff'] 
            if recv_data == header_[0]:
                recv_data = self.serialHandle.read()
                if recv_data == header_[1]:
                    recv_data = self.serialHandle.read()
                    if recv_data == header_[2]:
                        recv_data = self.serialHandle.read(4)
                        self.serialHandle.read((recv_data[1] << 8 | recv_data[0]) + 1)
                        self.send_data(header, args)
                        self.start_time = time.time()
                        break
                        
                    else:  # 没有收到确认
                        recv_data = self.serialHandle.read(4)
                        self.serialHandle.read((recv_data[1] << 8 | recv_data[0]) + 1)
                        
                        time.sleep(0.1)
                        self.serialHandle.write([0xa5, 0x01, 0x01, 0x04, 0x00, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0xb0])  # 继续发送发送握手请求
        
        result = None
        while True:
            recv_data = self.serialHandle.read()
            header_ = [b'\xa5', b'\x01', b'\x04'] 
            if recv_data == header_[0]:
                recv_data = self.serialHandle.read()
                if recv_data == header_[1]:
                    recv_data = self.serialHandle.read()
                    if recv_data == header_[2]:
                        recv_data = self.serialHandle.read(4)
                        result = self.serialHandle.read((recv_data[1] << 8 | recv_data[0]) + 1)
                        break
        return result

    def val_map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # 检测是否唤醒以及唤醒对应角度
    def get_awake_result(self):
        recv_data = self.serialHandle.read()
        if recv_data == b'\xa5':
            recv_data = self.serialHandle.read()
            if recv_data == b'\x01':
                recv_data = self.serialHandle.read()
                if recv_data == b'\x04':
                    time.sleep(0.01)
                    recv_data = self.serialHandle.read(4)
                    result = self.serialHandle.read((recv_data[1] << 8 | recv_data[0]) + 1)
                    if b'content' in result:
                        m = re.search(self.pattern, str(result).replace('\\', ''))
                        if m is not None:
                            m = m.group(0).replace('"{"', '{"').replace('}"', '}')
                            if m is not None:
                                angle = int(json.loads(m)['content']['info']['ivw']['angle'])
                                if self.flag_pub is not None and self.angle_pub is not None:
                                    msg = Bool()
                                    msg.data = True
                                    self.flag_pub.publish(msg)
                                    msg = Int32()
                                    msg.data = int(angle)
                                    self.angle_pub.publish(msg)
    
class AwakeNode:
    def __init__(self, name):
        rospy.init_node(name)
        
        self.running = True
        signal.signal(signal.SIGINT, self.shutdown)

        mic_type = rospy.get_param('~mic_type', 'mic6_circle')
        port = rospy.get_param('~port', '/dev/ring_mic')
        #awake_word = rospy.get_param('~awake_word', 'xiao3 mai4 xiao3 mai4')
        awake_word = rospy.get_param('~awake_word', 'tang2 xiao3 tian1') #自定义唤醒词为唐小天
        enable_setting = rospy.get_param('~enable_setting', False)
        self.awake_angle_pub = rospy.Publisher('~angle', Int32, queue_size=1)
        self.awake_flag_pub = rospy.Publisher('~awake_flag', Bool, queue_size=1) 
        self.mic = CircleMic(port, self.awake_flag_pub, self.awake_angle_pub)
        rospy.Service('~set_mic_type', SetString, self.set_mic_type_srv)
        rospy.Service('~get_setting', Trigger, self.get_setting_srv)
        rospy.Service('~set_wakeup_word', SetString, self.set_wakeup_word_srv)
        if enable_setting:
            self.mic.switch_mic(mic_type)
            self.mic.set_wakeup_word(awake_word)
        print('>>>>>Wake up word: %s' % awake_word)
        rate = rospy.Rate(50)
        while self.running:
            self.mic.get_awake_result()
            rate.sleep()
        rospy.signal_shutdown('shutdown')

    # 设置麦克风类型
    def set_mic_type_srv(self, msg):
        res = self.mic.switch_mic(msg.data) 
        return [True, str(res)]

    # 获取麦克风版本信息
    def get_setting_srv(self, msg):
        res = self.mic.get_setting()
        return [True, str(res)]

    # 设置唤醒词服务
    def set_wakeup_word_srv(self, msg):
        self.mic.set_wakeup_word(msg.data)
        return [True, 'success']

    def shutdown(self):
        self.running = False
        self.mic.serialHandle.close()
        rospy.loginfo('shutdown')

if __name__ == "__main__":
    AwakeNode('awake_node')

