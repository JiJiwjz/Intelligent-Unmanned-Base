#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/12/18
# @author:aiden
# 过独木桥
import os
import cv2
import math
import rospy
import queue
import signal
import numpy as np
from sdk import common
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from servo_msgs.msg import MultiRawIdPosDur
from servo_controllers import bus_servo_control

class CrossBridgeNode():
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        signal.signal(signal.SIGINT, self.shutdown)
        self.running = True
        self.turn = False
        self.plane_high = rospy.get_param('~bridge_plane_distance', 0.34)
        self.twist = Twist()
        self.time_stamp = rospy.get_time()
        self.image_queue = queue.Queue(maxsize=1)
        self.left_roi = [405/480, 415/480, 165/640, 175/640]
        self.center_roi = [405/480, 415/480, 315/640, 325/640]
        self.right_roi = [405/480, 415/480, 465/640, 475/640]

        servos_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
        rospy.sleep(0.2)
        bus_servo_control.set_servos(servos_pub, 1, ((1, 500), (2, 700), (3, 85), (4, 150), (5, 500), (10, 200)))
        rospy.sleep(1)
        camera_name = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')
        rospy.wait_for_service('/%s/set_ldp'%camera_name)
        rospy.ServiceProxy('/%s/set_ldp'%camera_name, SetBool)(False)
        rospy.sleep(1)
        self.mecanum_pub.publish(Twist())
        self.debug = rospy.get_param('~debug', False)
        rospy.Subscriber('/%s/depth/image_raw'%camera_name, Image, self.depth_callback)
        self.run()

    def depth_callback(self, ros_depth_image):
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(depth_image)
          
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')   

    def get_roi_distance(self, depth_image, roi):
        h, w = depth_image.shape[:2]
        roi_image = depth_image[int(roi[0]*h):int(roi[1]*h), int(roi[2]*w):int(roi[3]*w)]
        try:
            distance = round(float(np.mean(roi_image[np.logical_and(roi_image>0, roi_image<30000)])/1000), 3)
        except:
            distance = 0
        return distance

    def move_policy(self, left_distance, center_distance, right_distance):
        if abs(left_distance - self.plane_high) > 0.02:
            self.twist.angular.z = -0.1
        elif abs(right_distance - self.plane_high) > 0.02:
            self.twist.angular.z = 0.1
        else:
            self.twist.angular.z = 0
        if abs(center_distance - self.plane_high) > 0.02:
            self.twist = Twist()
            self.runnning = False
        else:
            self.twist.linear.x = 0.2

        self.mecanum_pub.publish(self.twist)

    def run(self):
        count = 0
        while self.running:
            depth_image = self.image_queue.get(block=True)
            h, w = depth_image.shape[:2]
            depth_color_map = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.45), cv2.COLORMAP_JET)
            cv2.circle(depth_color_map, (int(w*(self.left_roi[2] + self.left_roi[3]) / 2), int(h*(self.left_roi[0] + self.left_roi[1]) / 2)), 10, (0, 0, 0), -1)
            cv2.circle(depth_color_map, (int(w*(self.center_roi[2] + self.center_roi[3]) / 2), int(h*(self.center_roi[0] + self.center_roi[1]) / 2)), 10, (0, 0, 0), -1)
            cv2.circle(depth_color_map, (int(w*(self.right_roi[2] + self.right_roi[3]) / 2), int(h*(self.right_roi[0] + self.right_roi[1]) / 2)), 10, (0, 0, 0), -1)
            left_distance = self.get_roi_distance(depth_image, self.left_roi)
            center_distance = self.get_roi_distance(depth_image, self.center_roi)
            right_distance = self.get_roi_distance(depth_image, self.right_roi)
            if self.debug:
                count += 1
                print(left_distance, center_distance, right_distance)
                if count > 50 and not math.isnan(center_distance):
                    count = 0
                    self.plane_high = center_distance
                    common.save_yaml_data({'plane_high': self.plane_high}, os.path.join(
                        os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../..')),
                        'config/bridge_plane_distance.yaml'))
                    self.debug = False
            else:
                if math.isnan(left_distance):
                    left_distance = 0
                if math.isnan(center_distance):
                    center_distance = 0
                if math.isnan(right_distance):
                    right_distance = 0
                self.move_policy(left_distance, center_distance, right_distance)
            cv2.imshow('depth_color_map', depth_color_map)
            k = cv2.waitKey(1)
            if k != -1:
                self.running = False
        self.mecanum_pub.publish(Twist())
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    CrossBridgeNode('cross_bridge')
