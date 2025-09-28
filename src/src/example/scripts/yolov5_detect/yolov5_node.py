#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# yolov5目标检测
import os
import cv2
import queue
import rospy
import signal
import numpy as np
import sdk.fps as fps
from sdk import common
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from interfaces.msg import ObjectInfo, ObjectsInfo

MODE_PATH = os.path.split(os.path.realpath(__file__))[0]

class Yolov5Node:
    def __init__(self, name):
        rospy.init_node(name)

        self.start = False
        self.running = True
        self.image_queue = queue.Queue(maxsize=1)
        signal.signal(signal.SIGINT, self.shutdown)
        
        self.fps = fps.FPS()  # fps计算器
        engine = rospy.get_param('~engine')
        lib = rospy.get_param('~lib')
        conf_thresh = rospy.get_param('~conf_thresh', 0.8)
        self.classes = rospy.get_param('~classes')
        if engine == 'garbage_classification_320s_6_2.engine':
            from yolov5_trt_6_2 import YoLov5TRT
        else:
            from yolov5_trt_7_0 import YoLov5TRT
        
        self.yolov5 = YoLov5TRT(os.path.join(MODE_PATH, engine), os.path.join(MODE_PATH, lib), self.classes, conf_thresh)
        rospy.Service('/yolov5/start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('/yolov5/stop', Trigger, self.stop_srv_callback)  # 退出玩法
        if rospy.get_param('~use_astra_camera', False):
            camera = rospy.get_param('/astra_camera/camera_name', 'gemini_camera')
        else:
            camera = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')
        self.image_sub = rospy.Subscriber('/%s/rgb/image_raw' % camera, Image, self.image_callback, queue_size=1)

        self.object_pub = rospy.Publisher('~object_detect', ObjectsInfo, queue_size=1)
        self.result_image_pub = rospy.Publisher('~object_image', Image, queue_size=1)
        rospy.set_param('~init_finish', True)
        self.image_proc()

    def start_srv_callback(self, msg):
        rospy.loginfo("start yolov5 detect")

        self.start = True

        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop yolov5 detect')

        self.start = False

        return TriggerResponse(success=True)

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # 将图像放入队列
        self.image_queue.put(bgr_image)
   
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def image_proc(self):
        while self.running:
            image = self.image_queue.get(block=True)
            try:
                if self.start:
                    objects_info = []
                    h, w = image.shape[:2]
                    boxes, scores, classid = self.yolov5.infer(image)
                    for box, cls_conf, cls_id in zip(boxes, scores, classid):
                        color = common.colors(cls_id, True)
                        object_info = ObjectInfo()
                        object_info.class_name = self.classes[cls_id]
                        object_info.box = box.astype(int)
                        object_info.score = cls_conf
                        object_info.width = w
                        object_info.height = h
                        objects_info.append(object_info)

                        common.plot_one_box(
                        box,
                        image,
                        color=color,
                        label="{}:{:.2f}".format(
                            self.classes[cls_id], cls_conf
                        ),
                    )
                    object_msg = ObjectsInfo()
                    object_msg.objects = objects_info
                    self.object_pub.publish(object_msg)
                else:
                    rospy.sleep(0.01)
            except BaseException as e:
                print(e)

            self.fps.update()
            result_image = self.fps.show_fps(image)
            self.result_image_pub.publish(common.cv2_image2ros(result_image, frame_id='yolov5'))
        self.yolov5.destroy() 
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    node = Yolov5Node('yolov5')
