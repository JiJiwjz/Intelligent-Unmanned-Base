#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 手检测
import cv2
import rospy
import queue
import numpy as np
import faulthandler
import mediapipe as mp
import sdk.fps as fps
from sensor_msgs.msg import Image
from interfaces.msg import Point2D
from sdk.common import cv2_image2ros

mp_hands = mp.solutions.hands
WRIST = mp_hands.HandLandmark.WRIST
MIDDLE_FINGER_MCP = mp_hands.HandLandmark.MIDDLE_FINGER_MCP
THUMB_MCP = mp_hands.HandLandmark.THUMB_MCP
PINKY_MCP = mp_hands.HandLandmark.PINKY_MCP

faulthandler.enable()

def get_hand_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标
    :param img: 像素坐标对应的图片
    :param landmarks: 归一化的关键点
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)

class HandDetectNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.drawing = mp.solutions.drawing_utils

        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6
        )
        self.fps = fps.FPS()  # fps计算器
        self.running = True
        self.image_queue = queue.Queue(maxsize=1)
        self.result_publisher = rospy.Publisher('~image_result', Image, queue_size=5)  # 图像处理结果发布

        camera = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')  # 获取参数
        rospy.Subscriber('/%s/rgb/image_raw' % camera, Image, self.image_callback)  # 摄像头订阅

        self.point_publisher = rospy.Publisher('~center', Point2D, queue_size=1)

        self.image_proc()

    def image_proc(self):
        while self.running:
            point = Point2D()
            image = self.image_queue.get(block=True)
            h, w = image.shape[:2]
            image_flip = cv2.flip(image, 1)
            bgr_image = cv2.cvtColor(image_flip, cv2.COLOR_RGB2BGR)
            try:
                cv2.waitKey(10)
                results = self.hand_detector.process(image_flip)
                if results is not None and results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.drawing.draw_landmarks(
                            bgr_image,
                            hand_landmarks,
                            mp.solutions.hands.HAND_CONNECTIONS)
                        landmarks = get_hand_landmarks(image_flip, hand_landmarks.landmark)
                        center_x1 = (landmarks[WRIST][0] + landmarks[MIDDLE_FINGER_MCP][0])/2
                        center_y1 = (landmarks[WRIST][1] + landmarks[MIDDLE_FINGER_MCP][1])/2
                        center_x2 = (landmarks[THUMB_MCP][0] + landmarks[PINKY_MCP][0])/2
                        center_y2 = (landmarks[THUMB_MCP][1] + landmarks[PINKY_MCP][1])/2
                        center_x = int((center_x1 + center_x2)/2)
                        center_y = int((center_y1 + center_y2)/2)

                        cv2.circle(bgr_image, (center_x, center_y), 10, (0, 255, 255), -1)

                        point.x = center_x
                        point.y = center_y
                        point.width = w
                        point.height = h
            except Exception as e:
                print('>>>>>>', e)
            self.fps.update()
            result_image = self.fps.show_fps(bgr_image)
            self.result_publisher.publish(cv2_image2ros(result_image, self.name))
            self.point_publisher.publish(point)

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 原始 RGB 画面
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)

if __name__ == "__main__":
    HandDetectNode('hand_detect')
