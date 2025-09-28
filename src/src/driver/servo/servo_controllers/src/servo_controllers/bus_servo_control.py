#!/usr/bin/env python3
# encoding: utf-8
# Date:2021/07/24
# 串口舵机代码控制例程
import rospy
import signal
from servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur

def set_servos(pub, duration, pos_s):
    msg = MultiRawIdPosDur(id_pos_dur_list=list(map(lambda x: RawIdPosDur(int(x[0]), int(x[1]), int(duration)), pos_s)))
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('servo_control_example', anonymous=True)

    running = True
    def shutdown(signum, frame):
        global running

        running = False
        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')

    signal.signal(signal.SIGINT, shutdown)

    # 发布舵机位置
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    while running:
        try:
            # 参数1:发布句柄
            # 参数2:运行时间(ms)
            # 参数3:((舵机id, 舵机位置(脉宽)), (舵机id, 舵机位置(脉宽), ...)
            #set_servos(joints_pub, 0.5, ((10, 460), (5, 633), (4, 500), (3, 410), (2, 666)))
            # 延时，等待舵机转到指定位置
            #rospy.sleep(0.5)
            
            set_servos(joints_pub, 1, ((5, 400), ))
            rospy.sleep(0.5)
            
            set_servos(joints_pub, 1, ((5, 600),))
            rospy.sleep(0.5)
        except Exception as e:
            print(e)
            break
