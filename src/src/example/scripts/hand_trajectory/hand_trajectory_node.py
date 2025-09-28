#!/usr/bin/env python3
# encoding: utf-8
# 指尖轨迹点发布
import cv2
import enum
import queue
import rospy
import numpy as np
import faulthandler
import mediapipe as mp
import sdk.fps as fps
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from interfaces.msg import Points, PixelPosition
from sdk.common import vector_2d_angle, distance, cv2_image2ros

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

def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度
    :param landmarks: 手部关键点
    :return: 各个手指的角度
    """
    angle_list = []
    # thumb 大拇指
    angle_ = vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2])
    angle_list.append(angle_)
    # index 食指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8])
    angle_list.append(angle_)
    # middle 中指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12])
    angle_list.append(angle_)
    # ring 无名指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16])
    angle_list.append(angle_)
    # pink 小拇指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20])
    angle_list.append(angle_)
    angle_list = [abs(a) for a in angle_list]
    return angle_list

def h_gesture(angle_list):
    """
    通过二维特征确定手指所摆出的手势
    :param angle_list: 各个手指弯曲的角度
    :return : 手势名称字符串
    """
    thr_angle = 65.
    thr_angle_thumb = 53.
    thr_angle_s = 49.
    gesture_str = "none"
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "nico-nico-ni"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "two"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
        gesture_str = "three"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "OK"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "four"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "five"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "six"
    else:
        "none"
    return gesture_str

class State(enum.Enum):
    NULL = 0
    START = 1
    TRACKING = 2
    RUNNING = 3

def draw_points(img, points, thickness=4, color=(0, 0, 255)):
    points = np.array(points).astype(dtype=np.int)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, thickness)

class HandTrajectoryNode:
    def __init__(self, name):
        rospy.init_node(name)  # launch里的name会覆盖此处的name，所以要修改name，需要修改launch里的name, 为了一致性此处name会和launch name保持一致

        self.drawing = mp.solutions.drawing_utils

        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6
        )

        self.name = name
        self.start = False
        self.running = True
        self.fps = fps.FPS()  # fps计算器
        self.state = State.NULL
        self.points = []
        self.count = 0
        self.image_queue = queue.Queue(maxsize=1)
        camera = rospy.get_param('/astra_camera/camera_name', 'gemini_camera')  # 获取参数
        rospy.Subscriber('/%s/rgb/image_raw' % camera, Image, self.image_callback)  # 摄像头订阅

        self.point_publisher = rospy.Publisher('~points', Points, queue_size=1)  # 使用~可以自动加上前缀名称
        self.result_publisher = rospy.Publisher('~image_result', Image, queue_size=1)  # 图像处理结果发布
        rospy.Service('~start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法
        self.display = rospy.get_param('~display')
        if rospy.get_param('~start'):
            self.start_srv_callback(None)
        
        rospy.set_param('~init_finish', True)

        self.image_proc()
    
    def start_srv_callback(self, msg):
        rospy.loginfo('start hand trajectory')
        self.start = True

        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop hand trajectory')
        self.start = False

        return TriggerResponse(success=True)

    def image_proc(self):
        points_list = []
        while self.running:
            image = self.image_queue.get(block=True)
            image_flip = cv2.flip(image, 1)
            bgr_image = cv2.cvtColor(image_flip, cv2.COLOR_RGB2BGR)
            if self.start:
                try:
                    cv2.waitKey(10)
                    results = self.hand_detector.process(image_flip)
                    if results is not None and results.multi_hand_landmarks:
                        gesture = "none"
                        index_finger_tip = [0, 0]
                        for hand_landmarks in results.multi_hand_landmarks:
                            self.drawing.draw_landmarks(
                                bgr_image,
                                hand_landmarks,
                                mp.solutions.hands.HAND_CONNECTIONS)
                            landmarks = get_hand_landmarks(image_flip, hand_landmarks.landmark)
                            angle_list = (hand_angle(landmarks))
                            gesture = (h_gesture(angle_list))
                            index_finger_tip = landmarks[8].tolist()
                        if self.state != State.TRACKING:
                            if gesture == "one":  # 检测食指手势， 开始指尖追踪
                                self.count += 1
                                if self.count > 5:
                                    self.count = 0
                                    self.state = State.TRACKING
                                    self.points = []
                                    points_list = []
                            else:
                                self.count = 0

                        elif self.state == State.TRACKING:
                            if gesture != "two":
                                if len(self.points) > 0:
                                    last_point = self.points[-1]
                                    if distance(last_point, index_finger_tip) < 5:
                                        self.count += 1
                                    else:
                                        self.count = 0
                                        pixels = PixelPosition()
                                        pixels.x = int(index_finger_tip[0])
                                        pixels.y = int(index_finger_tip[1])
                                        points_list.append(pixels)
                                        self.points.append(index_finger_tip)
                                else:
                                    pixels = PixelPosition()
                                    pixels.x = int(index_finger_tip[0])
                                    pixels.y = int(index_finger_tip[1])
                                    points_list.append(pixels)
                                    self.points.append(index_finger_tip)
                            draw_points(bgr_image, self.points)
                        if gesture == "five":
                            self.state = State.NULL
                            if points_list:
                                points = Points()
                                points.points = points_list
                                self.point_publisher.publish(points)
                            self.points = []
                            points_list = []
                            draw_points(bgr_image, self.points)
                except Exception as e:
                    print(e)
            else:
                rospy.sleep(0.01)
            #self.fps.update()
            #result_image = self.fps.show_fps(bgr_image)
            self.result_publisher.publish(cv2_image2ros(bgr_image, self.name))
            if self.display:
                cv2.imshow(self.name, bgr_image)
                key = cv2.waitKey(1)
                if key != -1:
                    break

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)

if __name__ == "__main__":
    print('\n******Press any key to exit!******')
    HandTrajectoryNode('hand_trajectory')
