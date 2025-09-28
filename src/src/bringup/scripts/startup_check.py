#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/04
# @author:aiden
# 自检程序(self-test program)
import os
import psutil
import threading

def check_mic():
    data = os.popen('ls /dev/ |grep ring_mic').read()
    if data == 'ring_mic\n':
        os.system("roslaunch xf_mic_asr_offline startup_test.launch")

def get_cpu_serial_number():
    address = "/sys/class/net/eth0/address"
    if os.path.exists(address):
        with open(address, 'r') as f:
            serial_num = f.read().replace('\n', '').replace(':', '').upper()
            serial_num = serial_num[-8:]
    else:
        device_serial_number = open("/proc/device-tree/serial-number")
        serial_num = device_serial_number.readlines()[0][-10:-1]

    sn = (serial_num + "00000000000000000000000000")[:32]
    WIFI_AP_SSID = ''.join(["WN-", sn[0:8]])

    return WIFI_AP_SSID

def get_wlan():
    ip = ''
    info = psutil.net_if_addrs()
    for k, v in info.items():
        if 'wlan0' in k:
            for i in v:
                if i[2] is not None:
                    ip = i[1]
                    break
                else:
                    ip = None

    if ip != '' and ip is not None:
        return ip
    else:
        return '0.0.0.0'


if __name__ == '__main__':
    import rospy
    from ros_robot_controller.msg import BuzzerState, OLEDState
    
    threading.Thread(target=check_mic, daemon=False).start()
    rospy.init_node('startup')
    buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
    oled_pub = rospy.Publisher('/ros_robot_controller/set_oled', OLEDState, queue_size=1)
    rospy.sleep(45)
    
    msg = BuzzerState()
    msg.freq = 1900
    msg.on_time = 0.2
    msg.off_time = 0.01
    msg.repeat = 1
    buzzer_pub.publish(msg)
    msg = OLEDState()
    msg.index = 1
    msg.text = 'SSID:' + get_cpu_serial_number()
    oled_pub.publish(msg)
    rospy.sleep(0.2)
    msg = OLEDState()
    msg.index = 2
    msg.text = 'IP:' + get_wlan()
    oled_pub.publish(msg)
