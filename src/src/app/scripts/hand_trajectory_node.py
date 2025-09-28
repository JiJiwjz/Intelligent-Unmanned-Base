#!/usr/bin/env python3
# encoding: utf-8
# 轨迹点发布
import cv2
import math
import enum
import rospy
import numpy as np
import faulthandler
from model import HandLandmark
from model import PalmDetection
from sensor_msgs.msg import Image
from ros_robot_controller.msg import BuzzerState
from utils.utils import rotate_and_crop_rectangle
from std_srvs.srv import Trigger, TriggerResponse
from interfaces.msg import Points, PixelPosition
from sdk.common import vector_2d_angle, distance, cv2_image2ros

faulthandler.enable()

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

def draw_points(img, points, thickness=4, color=(100, 0, 200)):
    points = np.array(points).astype(dtype=np.int)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, thickness)

class State(enum.Enum):
    NULL = 0
    START = 1
    TRACKING = 2
    RUNNING = 3

class HandTrajectoryNode:
    def __init__(self, name):
        rospy.init_node(name)  # launch里的name会覆盖此处的name，所以要修改name，需要修改launch里的name, 为了一致性此处name会和launch name保持一致

        self.name = name
        #t1 = rospy.get_time()
        self.start = False
        self.running = True
        self.image = None
        self.state = State.NULL
        self.points = []
        self.count = 0
        self.count_miss = 0
        self.last_point = [0, 0]
        self.lines_hand = [
            [0,1],[1,2],[2,3],[3,4],
            [0,5],[5,6],[6,7],[7,8],
            [5,9],[9,10],[10,11],[11,12],
            [9,13],[13,14],[14,15],[15,16],
            [13,17],[17,18],[18,19],[19,20],[0,17],
        ]

        self.palm_detection = PalmDetection(score_threshold=0.6)
        self.hand_landmark = HandLandmark()

        camera = rospy.get_param('/astra_camera/camera_name', 'gemini_camera')  # 获取参数
        rospy.Subscriber('/%s/rgb/image_raw' % camera, Image, self.image_callback)  # 摄像头订阅
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.point_publisher = rospy.Publisher('~points', Points, queue_size=1)  # 使用~可以自动加上前缀名称
        self.result_publisher = rospy.Publisher('~image_result', Image, queue_size=1)  # 图像处理结果发布
        rospy.Service('~start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法
        if rospy.get_param('~start'):
            self.start_srv_callback(None)
        
        rospy.set_param('~init_finish', True)
        #print(rospy.get_time() - t1)
        print('hand gesture init')
        self.image_proc()
   
    def start_srv_callback(self, msg):
        rospy.loginfo('start hand trajectory')
        self.state = State.NULL
        self.points = []
        self.count = 0
        self.count_miss = 0
        self.start = True
        self.last_point = [0, 0]

        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop hand trajectory')
        self.start = False

        return TriggerResponse(success=True)

    def buzzer_warn(self):
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)

    def image_proc(self):
        points_list = []
        while self.running:
            if self.image is not None:
                h, w = self.image.shape[:2]
                image_flip = cv2.cvtColor(cv2.flip(self.image, 1), cv2.COLOR_RGB2BGR)
                self.image = None
                bgr_image = image_flip.copy()
                if self.start:
                    try:
                        #cv2.waitKey(10)
                        hands = self.palm_detection(image_flip)
                        rects = []
                        cropted_rotated_hands_images = []

                        if len(hands) > 0:
                            for hand in hands:
                                # hand: sqn_rr_size, rotation, sqn_rr_center_x, sqn_rr_center_y
                                rotation = hand[1]
                                sqn_rr_size = hand[0]
                                sqn_rr_center_x = hand[2]
                                sqn_rr_center_y = hand[3]

                                cx = int(sqn_rr_center_x * w)
                                cy = int(sqn_rr_center_y * h)
                                xmin = int((sqn_rr_center_x - (sqn_rr_size / 2)) * w)
                                xmax = int((sqn_rr_center_x + (sqn_rr_size / 2)) * w)
                                ymin = int((sqn_rr_center_y - (sqn_rr_size * w / h / 2)) * h)
                                ymax = int((sqn_rr_center_y + (sqn_rr_size * w / h / 2)) * h)
                                xmin = max(0, xmin)
                                xmax = min(w, xmax)
                                ymin = max(0, ymin)
                                ymax = min(h, ymax)
                                degree = math.degrees(rotation)
                                # [boxcount, cx, cy, width, height, degree]
                                rects.append([cx, cy, (xmax - xmin), (ymax - ymin), degree])

                            rects = np.asarray(rects, dtype=np.float32)

                            cropted_rotated_hands_images = rotate_and_crop_rectangle(
                                image=image_flip,
                                rects_tmp=rects,
                                operation_when_cropping_out_of_range='padding',
                            )
                         
                        if len(cropted_rotated_hands_images) > 0:
                            self.count_miss = 0
                            gesture = "none"
                            hand_center = [0, 0]
                            hand_landmarks, rotated_image_size_leftrights = self.hand_landmark(
                                images=cropted_rotated_hands_images,
                                rects=rects,
                            )
                            if len(hand_landmarks) > 0:
                                for landmark, rotated_image_size_leftright in zip(hand_landmarks, rotated_image_size_leftrights):
                                    if rotated_image_size_leftright[-1] == 0:
                                        thick_coef = rotated_image_size_leftright[0] / 400
                                        lines = np.asarray(
                                            [
                                                np.array([landmark[point] for point in line]).astype(np.int32) for line in self.lines_hand
                                            ]
                                        )
                                        radius = int(1 + thick_coef*5)
                                        cv2.polylines(
                                            bgr_image,
                                            lines,
                                            False,
                                            (255, 0, 0),
                                            int(radius),
                                            cv2.LINE_AA,
                                        )
                                        p1 = landmark[:,:2][0]
                                        p2 = landmark[:,:2][2]
                                        p3 = landmark[:,:2][9]
                                        p4 = landmark[:,:2][17]
                                        hand_center = [int((p1[0] + p2[0] + p3[0] + p4[0])/4), int((p1[1] + p2[1] + p3[1] + p4[1])/4)]
                                        cv2.circle(bgr_image, tuple(hand_center), 10, (0, 255, 255), -1)
                                        _ = [cv2.circle(bgr_image, (int(x), int(y)), radius, (0, 128, 255), -1) for x, y in landmark[:,:2]]
                                        angle_list = (hand_angle(landmark[:,:2]))
                                        gesture = (h_gesture(angle_list))
                            if self.state != State.TRACKING:
                                if gesture == "five":  
                                    self.count += 1
                                    if self.count > 5:
                                        self.count = 0
                                        if self.state != State.TRACKING:
                                            self.buzzer_warn()

                                        self.state = State.TRACKING
                                        self.points = []
                                        points_list = []
                                else:
                                    self.count = 0
                            elif self.state == State.TRACKING:
                                if gesture != "fist":
                                    self.count += 1
                                    if self.count > 1:
                                        self.count = 1
                                        if 40 > distance(self.last_point, hand_center) > 5:
                                            pixels = PixelPosition()
                                            pixels.x = int(hand_center[0])
                                            pixels.y = int(hand_center[1])
                                            points_list.append(pixels)
                                            self.points.append(hand_center)
                                            draw_points(bgr_image, self.points)
                                        self.last_point = hand_center
                            if gesture == "fist":
                                if self.state != State.NULL:
                                    self.buzzer_warn()
                                self.state = State.NULL
                                if points_list:
                                    points = Points()
                                    points.points = points_list
                                    self.point_publisher.publish(points)
                                self.count = 0
                                self.last_point = [0, 0]
                                self.points = []
                                points_list = []
                                draw_points(bgr_image, self.points)
                        else:
                            self.count_miss += 1
                            if self.count_miss > 5:
                                self.state = State.NULL
                                self.count_miss = 5
                                self.count = 0
                                self.last_point = [0, 0]
                                self.points = []
                    except Exception as e:
                        print(e)
                else:
                    rospy.sleep(0.01)
                self.result_publisher.publish(cv2_image2ros(cv2.resize(bgr_image, (640, 480)), self.name))
                #cv2.imshow(self.name, cv2.resize(bgr_image, (640, 480)))
                #key = cv2.waitKey(1)
                #if key != -1:
                #    break

    def image_callback(self, ros_image):
        self.image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面

if __name__ == "__main__":
    print('\n******Press any key to exit!******')
    HandTrajectoryNode('hand_trajectory')
