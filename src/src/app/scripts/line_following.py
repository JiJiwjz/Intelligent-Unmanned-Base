#!/usr/bin/env python3
# encoding: utf-8
# 巡线(line following)
import os
import cv2
import math
import rospy
import threading
import numpy as np
import collections
import sdk.pid as pid
import sdk.misc as misc
import geometry_msgs.msg as geo_msg
from app.common import ColorPicker, Heart
from servo_msgs.msg import MultiRawIdPosDur
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from servo_controllers.bus_servo_control import set_servos
from sensor_msgs.msg import CameraInfo, Image, CompressedImage, LaserScan
from interfaces.srv import SetPoint, SetPointRequest, SetPointResponse
from interfaces.srv import SetFloat64, SetFloat64Request, SetFloat64Response

MAX_SCAN_ANGLE = 240  # 激光的扫描角度,去掉总是被遮挡的部分degree(the scanning angle of lidar. The covered part is always eliminated)
class LineFollower:
    def __init__(self, color, node):
        self.node = node
        self.target_lab, self.target_rgb = color
        self.rois = ((400, 430, 0, 640, 0.7), (340, 370, 0, 640, 0.2), (280, 310, 0, 640, 0.1))
        self.weight_sum = 1.0

    @staticmethod
    def get_area_max_contour(contours, threshold=100):
        '''
        获取最大面积对应的轮廓(get the contour of the largest area)
        :param contours:
        :param threshold:
        :return:
        '''
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None

    def __call__(self, image, result_image, threshold):
        centroid_sum = 0
        h, w = image.shape[:2]
        min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                     int(self.target_lab[1] - 50 * threshold),
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
        target_color = self.target_lab, min_color, max_color
        for roi in self.rois:
            blob = image[roi[0]:roi[1], roi[2]:roi[3]]  # 截取roi(intercept roi)
            img_lab = cv2.cvtColor(blob, cv2.COLOR_RGB2LAB)  # rgb转lab(convert rgb into lab)
            img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪(perform Gaussian filtering to reduce noise)
            mask = cv2.inRange(img_blur, tuple(target_color[1]), tuple(target_color[2]))  # 二值化(image binarization)
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(corrode)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
            # cv2.imshow('section:{}:{}'.format(roi[0], roi[1]), cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓(find the contour)
            max_contour_area = self.get_area_max_contour(contours, 30)  # 获取最大面积对应轮廓(get the contour corresponding to the largest contour)
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形(minimum circumscribed rectangle)
                box = np.int0(cv2.boxPoints(rect))  # 四个角(four corners)
                for j in range(4):
                    box[j, 1] = box[j, 1] + roi[0]
                cv2.drawContours(result_image, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形(draw the rectangle composed of four points)

                # 获取矩形对角点(acquire the diagonal points of the rectangle)
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # 线的中心点(center point of the line)
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)   # 画出中心点(draw the center point)
                centroid_sum += line_center_x * roi[-1]
        if centroid_sum == 0:
            return result_image, None
        center_pos = centroid_sum / self.weight_sum  # 按比重计算中心点(calculate the center point according to the ratio)
        deflection_angle = -math.atan((center_pos - (w / 2.0)) / (h / 2.0))   # 计算线角度(calculate the line angle)
        return result_image, deflection_angle

class LineFollowingNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = rospy.get_name()
        self.is_running = False
        self.color_picker = None
        self.follower = None
        self.scan_angle = math.radians(45)
        self.pid = pid.PID(0.01, 0.0, 0.0)
        self.empty = 0
        self.count = 0
        self.stop = False
        self.imgs = collections.deque(maxlen=20)
        self.threshold = 0.1
        self.stop_threshold = 0.4
        self.lock = threading.RLock()
        self.image_sub = None
        self.lidar_sub = None
        self.lidar_type = os.environ.get('LIDAR_TYPE')
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', geo_msg.Twist, queue_size=1)  # 底盘控制(chassis control)
        self.result_publisher = rospy.Publisher('~image_result', Image, queue_size=1)  # 图像处理结果发布(publish the image processing result)
        self.enter_srv = rospy.Service('~enter', Trigger,  self.enter_srv_callback)  # 进入玩法(enter the game)
        self.exit_srv = rospy.Service('~exit', Trigger, self.exit_srv_callback)  # 退出玩法(exit the game)
        self.set_running_srv = rospy.Service('~set_running', SetBool, self.set_running_srv_callback)  # 开启玩法(start the game)
        self.set_target_color_srv = rospy.Service('~set_target_color', SetPoint, self.set_target_color_srv_callback)  # 设置颜色(set the color)
        self.get_target_color_srv = rospy.Service('~get_target_color', Trigger, self.get_target_color_srv_callback)   # 获取颜色(get the color)
        self.set_threshold_srv = rospy.Service('~set_threshold', SetFloat64, self.set_threshold_srv_callback)  # 设置阈值(set the threshold)
        self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(None))  # 心跳包(heartbeat package)
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制(servo control)

    def enter_srv_callback(self, _):
        rospy.loginfo("line following enter")
        try:
            if self.image_sub is not None:
                self.image_sub.unregister()
        except Exception as e:
            rospy.logerr(str(e))
        with self.lock:
            self.stop = False
            self.is_running = False
            self.color_picker = None
            self.pid = pid.PID(1.1, 0.0, 0.0)
            self.follower = None
            self.threshold = 0.1
            self.empty = 0
            depth_camera = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')  # 获取参数(acquire the parameter)
            self.image_sub = rospy.Subscriber('/%s/rgb/image_raw'%depth_camera, Image, self.image_callback)  # 摄像头订阅(subscribe to the camera)
            self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)  # 订阅雷达(subscribe to Lidar)
            set_servos(self.joints_pub, 1, ((10, 300), (5, 500), (4, 210), (3, 40), (2, 665), (1, 500)))
            self.mecanum_pub.publish(geo_msg.Twist())
        return TriggerResponse(success=True)

    def exit_srv_callback(self, _):
        rospy.loginfo("line following exit")
        try:
            if self.image_sub is not None:
                self.image_sub.unregister()
            if self.lidar_sub is not None:
                self.lidar_sub.unregister()
        except Exception as e:
            rospy.logerr(str(e))
        with self.lock:
            self.is_running = False
            self.color_picker = None
            self.pid = pid.PID(0.01, 0.0, 0.0)
            self.follower = None
            self.threshold = 0.1
            self.mecanum_pub.publish(geo_msg.Twist())
        return TriggerResponse(success=True)

    def set_target_color_srv_callback(self, req: SetPointRequest):
        rospy.loginfo("set_target_color")
        with self.lock:
            x, y = req.data.x, req.data.y
            self.follower = None
            if x == -1 and y == -1:
                self.color_picker = None
            else:
                self.color_picker = ColorPicker(req.data, 20)
                self.mecanum_pub.publish(geo_msg.Twist())
        return SetPointResponse(success=True)

    def get_target_color_srv_callback(self, _):
        rospy.loginfo("get_target_color")
        rsp = TriggerResponse(success=False, message="")
        with self.lock:
            if self.follower is not None:
                rsp.success = True
                rgb = self.follower.target_rgb
                rsp.message = "{},{},{}".format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        return rsp

    def set_running_srv_callback(self, req: SetBoolRequest):
        rospy.loginfo("set_running")
        with self.lock:
            self.is_running = req.data
            self.empty = 0
            if not self.is_running:
                self.mecanum_pub.publish(geo_msg.Twist())
        return SetBoolResponse(success=req.data)

    def set_threshold_srv_callback(self, req: SetFloat64Request):
        rospy.loginfo("set threshold")
        with self.lock:
            self.threshold = req.data
        return SetFloat64Response(success=True)

    def lidar_callback(self, lidar_data):
        # 数据大小 = 扫描角度/每扫描一次增加的角度(data size= scanning angle/ the increased angle per scan)
        if self.lidar_type != 'G4':
            max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[:max_index]  # 左半边数据(left data)
            right_ranges = lidar_data.ranges[::-1][:max_index]  # 右半边数据(right data)
        elif self.lidar_type == 'G4':
            min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / lidar_data.angle_increment)
            max_index = int(math.radians(180) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[::-1][min_index:max_index][::-1]  # 左半边数据(left data)
            right_ranges = lidar_data.ranges[min_index:max_index][::-1]  # 右半边数据(right data)

        # 根据设定取数据(Get data according to settings)
        angle = self.scan_angle / 2
        angle_index = int(angle / lidar_data.angle_increment + 0.50)
        left_range, right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])

        left_nonzero = left_range.nonzero()
        right_nonzero = right_range.nonzero()
        # 取左右最近的距离(get the shortest distance left and right)
        min_dist_left = left_range[left_nonzero].min()
        min_dist_right = right_range[right_nonzero].min()
        if min_dist_left < self.stop_threshold or min_dist_right < self.stop_threshold:
            self.stop = True
        else:
            self.count += 1
            if self.count > 5:
                self.count = 0
                self.stop = False

    def image_callback(self, ros_image: Image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 原始 RGB 画面(original RGB image)
        result_image = np.copy(rgb_image)  # 显示结果用的画面 (the image used to display the result)
        with self.lock:
            # 颜色拾取器和识别巡线互斥, 如果拾取器存在就开始拾取(color picker and line recognition are exclusive. If there is color picker, start picking)
            if self.color_picker is not None:  # 拾取器存在(color picker exists)
                try:
                    target_color, result_image = self.color_picker(rgb_image, result_image)
                    if target_color is not None:
                        self.color_picker = None
                        self.follower = LineFollower(target_color, self) 
                except Exception as e:
                    rospy.logerr(str(e))
            else:
                twist = geo_msg.Twist()
                twist.linear.x = 0.15
                if self.follower is not None:
                    try:
                        result_image, deflection_angle = self.follower(rgb_image, result_image, self.threshold)
                        if deflection_angle is not None and self.is_running and not self.stop:
                            self.pid.update(deflection_angle)
                            twist.angular.z = misc.set_range(-self.pid.output, -1.0, 1.0)
                            self.mecanum_pub.publish(twist)
                        elif self.stop:
                            self.mecanum_pub.publish(geo_msg.Twist())
                        else:
                            self.pid.clear()
                    except Exception as e:
                        rospy.logerr(str(e))

        ros_image.data = result_image.tostring()
        self.result_publisher.publish(ros_image)

if __name__ == "__main__":
    LineFollowingNode('line_following')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
