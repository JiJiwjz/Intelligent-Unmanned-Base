#!/usr/bin/env python3
# encoding: utf-8
# 指尖轨迹点发布
import cv2
import math
import queue
import rospy
import numpy as np
import mediapipe as mp
import sdk.fps as fps
from sensor_msgs.msg import Image
from kinematics import transform
from servo_msgs.msg import MultiRawIdPosDur
from kinematics.kinematics_control import set_pose_target
from servo_controllers.bus_servo_control import set_servos

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

class FingerControlNode:
    def __init__(self, name):
        rospy.init_node(name)  # launch里的name会覆盖此处的name，所以要修改name，需要修改launch里的name, 为了一致性此处name会和launch name保持一致

        self.drawing = mp.solutions.drawing_utils
        self.image_queue = queue.Queue(maxsize=1)
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6
        )

        self.name = name
        self.running = True
        self.fps = fps.FPS()  # fps计算器
        self.z_dis = 0.36
        self.y_dis = 500
        self.last_d = 0
        self.x_init = transform.link3 + transform.tool_link
        
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控
        camera = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')  # 获取参数
        rospy.Subscriber('/%s/rgb/image_raw' % camera, Image, self.image_callback)  # 摄像头订阅
        rospy.set_param('~init_finish', True)
        self.init_action()
        self.image_proc()

    def init_action(self):
        res = set_pose_target([self.x_init, 0, self.z_dis], 0, [-180, 180], 1)
        if res[1]:
            servo_data = res[1]
            set_servos(self.joints_pub, 1.5, ((10, 500), (5, 500), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, servo_data[0])))
            rospy.sleep(1.8)

    def image_proc(self):
        while self.running:
            image = self.image_queue.get(block=True)
            image_flip = cv2.flip(image, 1)
            bgr_image = cv2.cvtColor(image_flip, cv2.COLOR_RGB2BGR)
            results = self.hand_detector.process(image_flip)
            if results is not None and results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.drawing.draw_landmarks(
                        bgr_image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS)
                    landmarks = get_hand_landmarks(image_flip, hand_landmarks.landmark)
                    try:
                        index_finger_tip = landmarks[8].tolist()
                        thumb_finger_tip = landmarks[4].tolist()
                        cv2.circle(bgr_image, (int(index_finger_tip[0]), int(index_finger_tip[1])), 10, (0, 255, 255), -1)
                        cv2.circle(bgr_image, (int(thumb_finger_tip[0]), int(thumb_finger_tip[1])), 10, (0, 255, 255), -1)
                        cv2.line(bgr_image, (int(index_finger_tip[0]), int(index_finger_tip[1])), (int(thumb_finger_tip[0]), int(thumb_finger_tip[1])), (0, 255, 255), 5)

                        d = math.sqrt(math.pow(thumb_finger_tip[0] - index_finger_tip[0], 2) + math.pow(thumb_finger_tip[1] - index_finger_tip[1], 2))

                        # print(d, d - self.last_d)
                        if abs(d - self.last_d) > 10:
                            if d -self.last_d > 0:
                                self.z_dis += 0.01
                            else:
                                self.z_dis -= 0.01

                            if self.z_dis > 0.46:
                                self.z_dis = 0.46
                            if self.z_dis < 0.36:
                                self.z_dis = 0.36

                            res = set_pose_target([self.x_init, 0, self.z_dis], 0, [-180, 180], 1)
                            if res[1]:
                                servo_data = res[1]
                                set_servos(self.joints_pub, 0.02, ((10, 500), (5, 500), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, self.y_dis)))
                        self.last_d = d
                    except BaseException as e:
                        print(e)


            #self.fps.update()
            #result_image = self.fps.show_fps(bgr_image)
            cv2.imshow(self.name, bgr_image)
            key = cv2.waitKey(10)
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
    FingerControlNode('finger_control')
