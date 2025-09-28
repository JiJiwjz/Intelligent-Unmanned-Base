#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 垃圾分类
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
import geometry_msgs.msg as geo_msg
from xf_mic_asr_offline import voice_play
from interfaces.msg import ObjectsInfo
from std_srvs.srv import Trigger, TriggerResponse
sys.path.append('/home/ubuntu/software/arm_pc')
from action_group_controller import ActionGroupController

WASTE_CLASSES = {
    'food_waste': ('BananaPeel', 'BrokenBones', 'Ketchup'),
    'hazardous_waste': ('Marker', 'OralLiquidBottle', 'StorageBattery'),
    'recyclable_waste': ('PlasticBottle', 'Toothbrush', 'Umbrella'),
    'residual_waste': ('Plate', 'CigaretteEnd', 'DisposableChopsticks'),
}

class GarbageClassificationNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.running = True
        self.center = None
        self.count = 0
        self.class_name = None
        self.start_pick = False
        self.current_class_name = None
        self.language = os.environ['ASR_LANGUAGE']
        pick_roi = rospy.get_param('~roi')
        self.pick_roi = [pick_roi['y_min'], pick_roi['y_max'], pick_roi['x_min'], pick_roi['x_max']] #[y_min, y_max, x_min, x_max]
        
        signal.signal(signal.SIGINT, self.shutdown)
        self.image_queue = queue.Queue(maxsize=1)
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', geo_msg.Twist, queue_size=1)  # 底盘控制
        rospy.Service('/garbage_classification/start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('/garbage_classification/stop', Trigger, self.stop_srv_callback)  # 退出玩法
        rospy.Subscriber('/yolov5/object_image', Image, self.image_callback)  # 摄像头订阅
        rospy.Subscriber('/yolov5/object_detect', ObjectsInfo, self.get_object_callback)
        
        self.broadcast = rospy.get_param('~broadcast', False)
        self.debug = rospy.get_param('~debug', False)
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/yolov5/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        self.controller = ActionGroupController(use_ros=True)
        rospy.sleep(1)
        self.controller.runAction('garbage_pick_init')
        if self.debug:
            self.pick_roi = [3/48, 45/48, 3/64, 61/64]
            self.controller.runAction('garbage_pick_debug')
            rospy.sleep(5)
            self.controller.runAction('garbage_pick_init')
            rospy.sleep(2)
        if rospy.get_param('~start', True):
            self.start_srv_callback(None)

        threading.Thread(target=self.pick, daemon=True).start()
        self.mecanum_pub.publish(geo_msg.Twist())

        rospy.set_param('~init_finish', True)

        self.image_proc()

    def play(self, name):
        if self.broadcast:
            voice_play.play(name, language=self.language)

    def start_srv_callback(self, msg):
        rospy.loginfo("start garbage classification")

        rospy.ServiceProxy('/yolov5/start', Trigger)()

        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop garbage classification')

        rospy.ServiceProxy('/yolov5/stop', Trigger)()

        return TriggerResponse(success=True)

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)

    def pick(self):
        while self.running:
            waste_category = None
            if self.start_pick:
                rospy.sleep(0.2)
                for k, v in WASTE_CLASSES.items():
                    if self.current_class_name in v:
                        waste_category = k
                        break
                self.class_name = None
                print(waste_category)
                self.stop_srv_callback(Trigger())
                self.controller.runAction('garbage_pick')
                if waste_category == 'food_waste':
                    self.play('food_waste')
                    self.controller.runAction('place_food_waste')
                elif waste_category == 'hazardous_waste':
                    self.play('hazardous_waste')
                    self.controller.runAction('place_hazardous_waste')
                elif waste_category == 'recyclable_waste':
                    self.play('recyclable_waste')
                    self.controller.runAction('place_recyclable_waste')
                elif waste_category == 'residual_waste':
                    self.play('residual_waste')
                    self.controller.runAction('place_residual_waste')
                self.controller.runAction('garbage_pick_init')
                rospy.sleep(0.5)
                self.start_pick = False
                self.start_srv_callback(Trigger())
            else:
                rospy.sleep(0.01)

    def image_proc(self):
        count = 0
        while self.running:
            image = self.image_queue.get(block=True)
            h, w = image.shape[:2]
            if self.class_name is not None and not self.start_pick and not self.debug:
                self.count += 1
                if self.count > 50:
                    self.current_class_name = self.class_name
                    self.start_pick = True
                    self.count = 0
            elif self.debug and self.class_name is not None:
                count += 1
                if count > 50:
                    count = 0
                    self.pick_roi = [(self.center[1] - 15)/h, (self.center[1] + 15)/h, (self.center[0] - 15)/w, (self.center[0] + 15)/w]
                    data = {'roi': {}}
                    data['roi']['x_min'] = self.pick_roi[2]
                    data['roi']['x_max'] = self.pick_roi[3]
                    data['roi']['y_min'] = self.pick_roi[0]
                    data['roi']['y_max'] = self.pick_roi[1]
                    common.save_yaml_data(data, os.path.join(
                        os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../..')),
                        'config/garbage_classification_roi.yaml'))
                    self.debug = False
                print(self.center[1] - 15, self.center[1] + 15, self.center[0] - 15, self.center[0] + 15)
                cv2.rectangle(image, (self.center[0] - 45, self.center[1] - 45), (self.center[0] + 45, self.center[1] + 45), (0, 0, 255), 2)
            else:
                self.count = 0
                rospy.sleep(0.01)
            if image is not None:
                if not self.start_pick and not self.debug:
                    cv2.rectangle(image, (int(self.pick_roi[2]*w) - 30, int(self.pick_roi[0]*h) - 30), (int(self.pick_roi[3]*w) + 30, int(self.pick_roi[1]*h) + 30), (0, 255, 255), 2)
                cv2.imshow('image', image)
                key = cv2.waitKey(1)
                if key != -1:
                    self.running = False
        self.mecanum_pub.publish(geo_msg.Twist())
        self.controller.runAction('garbage_pick_init')
        rospy.signal_shutdown('shutdown')

    def get_object_callback(self, msg):
        objects = msg.objects
        if objects == []:
            self.center = None
            self.class_name = None
        else:
            for i in objects:
                center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2))
                if self.pick_roi[2] < center[0]/i.width < self.pick_roi[3] and self.pick_roi[0] < center[1]/i.height < self.pick_roi[1]:
                    self.center = center
                    self.class_name = i.class_name

if __name__ == "__main__":
    GarbageClassificationNode('garbage_classification')
