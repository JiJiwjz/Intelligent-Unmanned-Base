#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 肌体控制融合RGB
import cv2
import queue
import rospy
import threading
import numpy as np
import faulthandler
import mediapipe as mp
import sdk.fps as fps
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
from app.common import ColorPicker
from sdk.common import vector_2d_angle
from servo_msgs.msg import MultiRawIdPosDur
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

def get_body_center(h, w, landmarks):
    landmarks = np.array([(lm.x * w, lm.y * h) for lm in landmarks])
    center = ((landmarks[LEFT_HIP] + landmarks[LEFT_SHOULDER] + landmarks[RIGHT_HIP] + landmarks[RIGHT_SHOULDER])/4).astype(int)
    return center.tolist()

def get_joint_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(Convert landmarks from medipipe's normalized output to pixel coordinates)
    :param img: 像素坐标对应的图片(picture corresponding to pixel coordinate)
    :param landmarks: 归一化的关键点(normalized keypoint)
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)

def get_dif(list1, list2):
    if len(list1) != len(list2):
        return 255*3
    else:
        d = np.absolute(np.array(list1) - np.array(list2))
        return sum(d)

def joint_angle(landmarks):
    """
    计算各个关节弯曲角度(calculate flex angle of each joint)
    :param landmarks: 手部关键点(hand keypoints)
    :return: 关节角度(joint angle)
    """
    angle_list = []
    left_hand_angle1 = vector_2d_angle(landmarks[LEFT_SHOULDER] - landmarks[LEFT_ELBOW], landmarks[LEFT_WRIST] - landmarks[LEFT_ELBOW])
    angle_list.append(int(left_hand_angle1))
   
    left_hand_angle2 = vector_2d_angle(landmarks[LEFT_HIP] - landmarks[LEFT_SHOULDER], landmarks[LEFT_WRIST] - landmarks[LEFT_SHOULDER])
    angle_list.append(int(left_hand_angle2))

    right_hand_angle1 = vector_2d_angle(landmarks[RIGHT_SHOULDER] - landmarks[RIGHT_ELBOW], landmarks[RIGHT_WRIST] - landmarks[RIGHT_ELBOW])
    angle_list.append(int(right_hand_angle1))

    right_hand_angle2 = vector_2d_angle(landmarks[RIGHT_HIP] - landmarks[RIGHT_SHOULDER], landmarks[RIGHT_WRIST] - landmarks[RIGHT_SHOULDER])
    angle_list.append(int(right_hand_angle2))
    
    return angle_list

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
        
        self.color_picker = ColorPicker(Point(), 2)
        self.running = True
        self.fps = fps.FPS()  # fps计算器
    
        self.current_color = None
        self.lock_color = None
        self.calibrating = False
        self.move_finish = True
        self.stop_flag = False
        self.count_akimbo = 0
        self.count_no_akimbo = 0
        self.can_control = False
        self.have_lock = False
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
            twist = Twist()

            landmarks = get_joint_landmarks(image, results.pose_landmarks.landmark)

            # 叉腰标定
            angle_list = joint_angle(landmarks)
            # print(angle_list)
            if -150 < angle_list[0] < -90 and -30 < angle_list[1] < -10 and 90 < angle_list[2] < 150 and 10 < \
                    angle_list[3] < 30:
                self.count_akimbo += 1  # 叉腰检测+1(hands-on-hips detection+1)
                self.count_no_akimbo = 0  # 没有叉腰检测归零(clear no hands-on-hips detection)
            else:
                self.count_akimbo = 0  # 叉腰检测归零(clear hands-on-hips detection)
                self.count_no_akimbo += 1  # 没有叉腰检测+1(no hands-on-hips detection+1)
                # 当连续5次都检测到叉腰且当前不在标定状态(If hands-on-hips posture is detected for 5 consecutive times, and not under calibrated status)
            if self.count_akimbo > 5 and not self.calibrating:
                self.count_akimbo = 0  # 检测归零(clear detection)
                self.calibrating = True  # 正在标定(calibrating)
                self.color_picker.reset()  # 颜色拾取器重置(reset color picker)
                self.lock_color = None  # 标定颜色置空
                # 当连续5次都检测不到叉腰(hands-on-hips posture is detected for 5 consecutive times)
            if self.count_no_akimbo > 5:
                # 且已经标定过(was calibrated)
                if self.calibrating:
                    self.calibrating = False  # 没在标定(not calibrating)
                    self.have_lock = False  # 没标定好(not be calibrated)
                self.count_akimbo = 0  # 检测归零

                # 获取躯体中心颜色(acquire the color of body center)
            h, w = image.shape[:-1]
            center = get_body_center(h, w, results.pose_landmarks.landmark)
            point = Point()
            point.x = center[0] / w
            point.y = center[1] / h
            self.color_picker.set_point(point)

            if self.move_finish:
                self.current_color, image = self.color_picker(image, image.copy())
                # 没有标定颜色且检测到当前中心颜色，且在标定状态，还没标定好(Color is not calibrated.
                # Color of body center is detected and calibrated but the calibration isn't finished)
                if self.lock_color is None and self.current_color is not None and self.calibrating and not self.have_lock:
                    self.have_lock = True  # 标定好了
                    self.buzzer_warn()  # 标定好蜂鸣器提示(calibrate buzzer warning)
                    self.lock_color = self.current_color[1]  # 保存标定的颜色(save calibrated)
                # print(self.current_color[1])
                # 已经有标定颜色且测到当前中心颜色，不在标定状态(There is calibrated color and color of body center is detected but not be calibrated)
                if self.lock_color is not None and self.current_color is not None and not self.calibrating:
                    # 比较当前颜色和标定颜色是否一致(compare the current color and the calibrated color)
                    res = get_dif(list(self.lock_color), list(self.current_color[1]))
                    # print(res)
                    # print(self.lock_color, self.current_color[1])
                    if res < 50:  # 如果是则表示当前可以控制(if they are consistent, it means that control function can be implemented)
                        self.can_control = True
                    else:  # 如果不是则表示当前不可以控制(if they are not consistent, it means that control function cannot be implemented)
                        self.can_control = False
                if self.can_control:  # 可以控制则进入姿态检测部分(if control function can be implemented, move to posture detection part)
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
                    # print(distance_list)

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
                            if count[self.last_status - 1] <= 2:
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
            cv2.circle(image, tuple(center), 10, (255, 255, 0), -1)
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
