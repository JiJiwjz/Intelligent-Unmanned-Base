#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 肌体控制
import cv2
import queue
import rospy
import threading
import numpy as np
import faulthandler
import mediapipe as mp
import sdk.fps as fps
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from servo_msgs.msg import MultiRawIdPosDur
from servo_controllers.bus_servo_control import set_servos
from ros_robot_controller.msg import BuzzerState, MotorsState, MotorState

faulthandler.enable()

mp_pose = mp.solutions.pose

LEFT_SHOULDER = mp_pose.PoseLandmark.LEFT_SHOULDER
LEFT_ELBOW = mp_pose.PoseLandmark.LEFT_ELBOW
LEFT_WRIST = mp_pose.PoseLandmark.LEFT_WRIST
LEFT_HIP = mp_pose.PoseLandmark.LEFT_HIP

RIGHT_SHOULDER = mp_pose.PoseLandmark.RIGHT_SHOULDER
RIGHT_ELBOW = mp_pose.PoseLandmark.RIGHT_ELBOW
RIGHT_WRIST = mp_pose.PoseLandmark.RIGHT_WRIST
RIGHT_HIP = mp_pose.PoseLandmark.RIGHT_HIP

LEFT_KNEE = mp_pose.PoseLandmark.LEFT_KNEE
LEFT_ANKLE = mp_pose.PoseLandmark.LEFT_ANKLE

RIGHT_KNEE = mp_pose.PoseLandmark.RIGHT_KNEE
RIGHT_ANKLE = mp_pose.PoseLandmark.RIGHT_ANKLE

def get_joint_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(Convert landmarks from medipipe's normalized output to pixel coordinates)
    :param img: 像素坐标对应的图片(picture corresponding to pixel coordinate)
    :param landmarks: 归一化的关键点(normalized keypoint)
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)

def joint_distance(landmarks):
    distance_list = []

    d1 = landmarks[LEFT_HIP] - landmarks[LEFT_SHOULDER]
    d2 = landmarks[LEFT_HIP] - landmarks[LEFT_WRIST]
    dis1 = d1[0]**2 + d1[1]**2
    dis2 = d2[0]**2 + d2[1]**2
    distance_list.append(round(dis1/dis2, 1))
   
    d1 = landmarks[RIGHT_HIP] - landmarks[RIGHT_SHOULDER]
    d2 = landmarks[RIGHT_HIP] - landmarks[RIGHT_WRIST]
    dis1 = d1[0]**2 + d1[1]**2
    dis2 = d2[0]**2 + d2[1]**2
    distance_list.append(round(dis1/dis2, 1))
    
    d1 = landmarks[LEFT_HIP] - landmarks[LEFT_ANKLE]
    d2 = landmarks[LEFT_ANKLE] - landmarks[LEFT_KNEE]
    dis1 = d1[0]**2 + d1[1]**2
    dis2 = d2[0]**2 + d2[1]**2
    distance_list.append(round(dis1/dis2, 1))
   
    d1 = landmarks[RIGHT_HIP] - landmarks[RIGHT_ANKLE]
    d2 = landmarks[RIGHT_ANKLE] - landmarks[RIGHT_KNEE]
    dis1 = d1[0]**2 + d1[1]**2
    dis2 = d2[0]**2 + d2[1]**2
    distance_list.append(round(dis1/dis2, 1))
    
    return distance_list

class BodyControlNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.drawing = mp.solutions.drawing_utils
        self.body_detector = mp_pose.Pose(
            static_image_mode=False,
            min_tracking_confidence=0.5,
            min_detection_confidence=0.5)
        self.running = True
        self.fps = fps.FPS()  # fps计算器

        self.move_finish = True
        self.stop_flag = False
        self.left_hand_count = []
        self.right_hand_count = []
        self.left_leg_count = []
        self.right_leg_count = []

        self.detect_status = [0, 0, 0, 0]
        self.move_status = [0, 0, 0, 0]
        self.last_status = 0
        self.image_queue = queue.Queue(maxsize=1)

        camera = rospy.get_param('/astra_camera/camera_name', 'gemini_camera')
        self.image_sub = rospy.Subscriber('/%s/rgb/image_raw' % camera, Image, self.image_callback, queue_size=1)
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制(servo control)
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.motor_pub = rospy.Publisher('/ros_robot_controller/set_motor', MotorsState, queue_size=1)
        self.mecanum_pub.publish(Twist())
        rospy.set_param('~init_finish', True)
        self.run()

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)

    def move(self, *args):
        self.mecanum_pub.publish(args[0])
        rospy.sleep(args[1])
        self.mecanum_pub.publish(Twist())
        rospy.sleep(0.1)
        self.stop_flag =True
        self.move_finish = True

    def buzzer_warn(self):
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)

    def image_proc(self, image):
        image_flip = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)
        results = self.body_detector.process(image)
        if results is not None and results.pose_landmarks is not None:
            if self.move_finish:
                twist = Twist()
                landmarks = get_joint_landmarks(image, results.pose_landmarks.landmark)
                distance_list = (joint_distance(landmarks))
              
                if distance_list[0] < 1:
                    self.detect_status[0] = 1
                if distance_list[1] < 1:
                    self.detect_status[1] = 1
                if 0 < distance_list[2] < 2:
                    self.detect_status[2] = 1
                if 0 < distance_list[3] < 2:
                    self.detect_status[3] = 1
                
                self.left_hand_count.append(self.detect_status[0])
                self.right_hand_count.append(self.detect_status[1])
                self.left_leg_count.append(self.detect_status[2])
                self.right_leg_count.append(self.detect_status[3])                   
                #print(distance_list) 
               
                if len(self.left_hand_count) == 4:
                    count = [sum(self.left_hand_count), 
                             sum(self.right_hand_count), 
                             sum(self.left_leg_count), 
                             sum(self.right_leg_count)]

                    self.left_hand_count = []
                    self.right_hand_count = []
                    self.left_leg_count = []
                    self.right_leg_count = []
                
                    if self.stop_flag:
                        if count[self.last_status - 1] <= 1:
                            self.stop_flag = False
                            self.move_status = [0, 0, 0, 0]
                            self.buzzer_warn()
                    else:
                        if count[0] > 2:
                            self.move_status[0] = 1
                        if count[1] > 2:
                            self.move_status[1] = 1
                        if count[2] > 2:
                            self.move_status[2] = 1
                        if count[3] > 2:
                            self.move_status[3] = 1

                        if self.move_status[0]:
                            self.move_finish = False
                            self.last_status = 1
                            twist.linear.y = -0.3
                            threading.Thread(target=self.move, args=(twist, 1)).start()
                        elif self.move_status[1]:
                            self.move_finish = False
                            self.last_status = 2
                            twist.linear.y = 0.3
                            threading.Thread(target=self.move, args=(twist, 1)).start()
                        elif self.move_status[2]:
                            self.move_finish = False
                            self.last_status = 3
                            twist.linear.x = 0.3
                            threading.Thread(target=self.move, args=(twist, 1)).start()
                        elif self.move_status[3]:
                            self.move_finish = False
                            self.last_status = 4
                            twist.linear.x = -0.3
                            threading.Thread(target=self.move, args=(twist, 1)).start()

                self.detect_status = [0, 0, 0, 0]
            
            result_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            self.drawing.draw_landmarks(
                result_image,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS)
            return cv2.flip(result_image, 1)
        else:
            return image_flip

    def run(self):
        while self.running:
            image = self.image_queue.get(block=True)
            try:
                result_image = self.image_proc(np.copy(image))
            except BaseException as e:
                print(e)
                result_image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)
            self.fps.update()
            result_image = self.fps.show_fps(result_image)
            cv2.imshow(self.name, result_image)
            key = cv2.waitKey(1)
            if key != -1:
                self.mecanum_pub.publish(Twist())
                rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    print('\n******Press any key to exit!******')
    BodyControlNode('body_control')
