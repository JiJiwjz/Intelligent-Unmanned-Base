#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 颜色分拣
import os
import sys
import cv2
import queue
import rospy
import signal
import threading
import numpy as np
from sdk import common
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from interfaces.msg import ColorsInfo, ColorDetect, ROI
from interfaces.srv import SetColorDetectParam, SetCircleROI
sys.path.append('/home/ubuntu/software/arm_pc')
from action_group_controller import ActionGroupController

class ColorSortingNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.center = None
        pick_roi = rospy.get_param('~roi')
        self.pick_roi = [pick_roi['y_min'], pick_roi['y_max'], pick_roi['x_min'], pick_roi['x_max']] #[y_min, y_max, x_min, x_max]
        self.start_pick = False
        self.target_color = ''
        self.count = 0
        self.image_queue = queue.Queue(maxsize=1)
        signal.signal(signal.SIGINT, self.shutdown)
        
        rospy.Subscriber('/color_detect/color_info', ColorsInfo, self.get_color_callback)
        rospy.Subscriber('/color_detect/image_result/', Image, self.image_callback)
        rospy.Service('~start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法
        self.controller = ActionGroupController(use_ros=True)
        rospy.sleep(1)
        self.controller.runAction('pick_init')
        
        self.debug = rospy.get_param('~debug', False)
        if self.debug:
            self.pick_roi = [2/48, 46/48, 2/64, 62/64]
            self.controller.runAction('pick_debug')
            rospy.sleep(5)
            self.controller.runAction('pick_init')
            rospy.sleep(2)
        if rospy.get_param('~start'):
            self.start_srv_callback(None)
        
        threading.Thread(target=self.pick, daemon=True).start()

        rospy.set_param('~init_finish', True)
        
        self.color_track() 

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def start_srv_callback(self, msg):
        rospy.loginfo("start color sorting")

        roi = ROI()
        roi.x_min = self.pick_roi[2] - 2/64
        roi.x_max = self.pick_roi[3] + 2/64
        roi.y_min = self.pick_roi[0] - 2/48
        roi.y_max = self.pick_roi[1] + 2/48

        res = rospy.ServiceProxy('/color_detect/set_circle_roi', SetCircleROI)(roi)
        if res.success:
            print('set roi success')
        else:
            print('set roi fail')

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
            print('set color success')
        else:
            print('set color fail')
        self.start = True
         
        return TriggerResponse(success=True)
    
    def stop_srv_callback(self, msg):
        rospy.loginfo('stop color sorting')
        res = rospy.ServiceProxy('/color_detect/set_param', SetColorDetectParam)()
        if res.success:
            print('set color success')
        else:
            print('set color fail')
        return TriggerResponse(success=True)   

    def get_color_callback(self, msg):
        data = msg.data
        if data != []:
            for i in data:
                if i.radius/i.width > 1/64 and self.pick_roi[2] < i.x/i.width < self.pick_roi[3] and self.pick_roi[0] < i.y/i.height < self.pick_roi[1]:
                    self.center = i
                    self.target_color = i.color
                    break
                else:
                    self.target_color = ''
        else:
            self.target_color = ''

    def pick(self):
        while self.running:
            if self.start_pick:
                self.stop_srv_callback(Trigger())
                if self.target_color == 'red':
                    self.controller.runAction('pick')
                    self.controller.runAction('place_center')
                elif self.target_color == 'green':
                    self.controller.runAction('pick')
                    self.controller.runAction('place_left')
                elif self.target_color == 'blue':
                    self.controller.runAction('pick')
                    self.controller.runAction('place_right')
                self.controller.runAction('pick_init')
                rospy.sleep(0.5)
                self.start_pick = False
                self.start_srv_callback(Trigger())
            else:
                rospy.sleep(0.01)

    def color_track(self):
        count = 0
        while self.running:
            image = self.image_queue.get(block=True)
            h, w = image.shape[:2]
            if self.target_color != '' and self.start:
                if not self.start_pick and not self.debug:
                    self.count += 1
                    if self.count > 30:
                        self.count = 0
                        self.start_pick = True
                elif self.debug:
                    count += 1
                    if count > 50:
                        count = 0
                        self.pick_roi = [(self.center.y - 10)/h, (self.center.y + 10)/h, (self.center.x - 10)/w, (self.center.x + 10)/w]
                        data = {'roi': {}}
                        data['roi']['x_min'] = self.pick_roi[2]
                        data['roi']['x_max'] = self.pick_roi[3]
                        data['roi']['y_min'] = self.pick_roi[0]
                        data['roi']['y_max'] = self.pick_roi[1]
                        common.save_yaml_data(data, os.path.join(
                            os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../..')),
                            'config/color_sorting_roi.yaml'))
                        self.debug = False
                        self.start_srv_callback(None)
                    print(self.center.y - 10, self.center.y + 10, self.center.x - 10, self.center.x + 10)
                    cv2.rectangle(image, (self.center.x - 25, self.center.y - 25,), (self.center.x + 25, self.center.y + 25), (0, 0, 255), 2)
                else:
                    count = 0
            if image is not None:
                if not self.start_pick and not self.debug:
                    cv2.rectangle(image, (int(self.pick_roi[2]*w) - 25, int(self.pick_roi[0]*h) - 25), (int(self.pick_roi[3]*w) + 25, int(self.pick_roi[1]*h) + 25), (0, 255, 255), 2)
                cv2.imshow('image', image)
                key = cv2.waitKey(1)
                if key != -1:
                    self.running = False
        self.controller.runAction('init')
        rospy.signal_shutdown('shutdown')

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)

if __name__ == "__main__":
    ColorSortingNode('color_sorting')
