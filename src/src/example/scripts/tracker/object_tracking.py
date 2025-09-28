#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 物体跟踪
import cv2
import queue
import rospy
import numpy as np
import sdk.pid as pid
import sdk.fps as fps
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class KCFNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.pid_d = pid.PID(1.8, 0, 0)
        # self.pid_d = pid.PID(0, 0, 0)

        self.pid_angular = pid.PID(0.005, 0, 0)
        self.go_speed, self.turn_speed = 0.007, 0.04
        self.linear_x, self.angular = 0, 0
        self.image_queue = queue.Queue(maxsize=1)
        self.fps = fps.FPS()

        self.running = True
        self.DISTANCE = 100  # 停止距离单位cm
        self.distance = self.DISTANCE  # 选取时停止距离单位cm
        self.mouse_click = False
        self.selection = None  # 实时跟踪鼠标的跟踪区域
        self.track_window = None  # 要检测的物体所在区域
        self.drag_start = None  # 标记，是否开始拖动鼠标
        self.start_circle = True
        self.start_click = False
        self.depth_frame = None

        self.tracker = cv2.legacy.TrackerMedianFlow_create()
        fs = cv2.FileStorage("custom_csrt.xml", cv2.FILE_STORAGE_READ)
        fn = fs.getFirstTopLevelNode()
        # self.tracker.save('default_csrt.xml')
        self.tracker.read(fn)

        cv2.namedWindow(name, 1)
        cv2.setMouseCallback(name, self.onmouse)
        camera = rospy.get_param('/astra_camera/camera_name', 'gemini_camera')
        self.image_sub = rospy.Subscriber('/%s/rgb/image_raw' % camera, Image, self.image_callback,
                                          queue_size=1)  # 订阅目标检测节点
        self.depth_image_sub = rospy.Subscriber('/%s/depth/image_raw' % camera, Image, self.depth_image_callback,
                                                queue_size=1)
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
        rospy.sleep(0.2)
        self.mecanum_pub.publish(Twist())
        rospy.set_param('~init_finish', True)
        self.run()

    def depth_image_callback(self, depth_image):
        self.depth_frame = np.ndarray(shape=(depth_image.height, depth_image.width), dtype=np.uint16,
                                      buffer=depth_image.data)

    def image_callback(self, ros_image: Image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 将自定义图像消息转化为图像
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)

    # 鼠标点击事件回调函数
    def onmouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # 鼠标左键按下
            self.mouse_click = True
            self.drag_start = (x, y)  # 鼠标起始位置
            self.track_window = None
        if self.drag_start:  # 是否开始拖动鼠标，记录鼠标位置
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax, ymax)
        if event == cv2.EVENT_LBUTTONUP:  # 鼠标左键松开
            self.mouse_click = False
            self.drag_start = None
            self.track_window = self.selection
            self.selection = None
            roi = self.depth_frame[self.track_window[1]:self.track_window[3], self.track_window[0]:self.track_window[2]]
            try:
                h, w = self.depth_frame.shape[:2]
                roi_h, roi_w = roi.shape
                if roi_h > 0 and roi_w > 0:
                    roi_cut = [1 / 3, 1 / 3]
                    if roi_w/w < 10/64:
                        roi_cut[1] = 0
                    if roi_h/h < 10/48:
                        roi_cut[0] = 0
                    roi_distance = roi[int(roi_h * roi_cut[0]):(roi_h - int(roi_h * roi_cut[0])),
                                   int(roi_w * roi_cut[1]):(roi_w - int(roi_w * roi_cut[1]))]
                    self.distance = int(
                        np.mean(roi_distance[np.logical_and(roi_distance > 0, roi_distance < 30000)]) / 10)
                else:
                    self.distance = self.DISTANCE
            except:
                self.distance = self.DISTANCE
        if event == cv2.EVENT_RBUTTONDOWN:
            self.mouse_click = False
            self.selection = None  # 实时跟踪鼠标的跟踪区域
            self.track_window = None  # 要检测的物体所在区域
            self.drag_start = None  # 标记，是否开始拖动鼠标
            self.start_circle = True
            self.start_click = False
            self.mecanum_pub.publish(Twist())
            self.tracker = cv2.legacy.TrackerMedianFlow_create()
            fs = cv2.FileStorage("custom_csrt.xml", cv2.FILE_STORAGE_READ)
            fn = fs.getFirstTopLevelNode()
            # tracker.save('default_csrt.xml')
            self.tracker.read(fn)

    def image_proc(self, image):
        if self.start_circle:
            # 用鼠标拖拽一个框来指定区域
            if self.track_window:  # 跟踪目标的窗口画出后，实时标出跟踪目标
                cv2.rectangle(image, (self.track_window[0], self.track_window[1]),
                              (self.track_window[2], self.track_window[3]), (0, 0, 255), 2)
            elif self.selection:  # 跟踪目标的窗口随鼠标拖动实时显示
                cv2.rectangle(image, (self.selection[0], self.selection[1]), (self.selection[2], self.selection[3]),
                              (0, 0, 255), 2)
            if self.mouse_click:
                self.start_click = True
            if self.start_click:
                if not self.mouse_click:
                    self.start_circle = False
            if not self.start_circle:
                print('start tracking')
                bbox = (self.track_window[0], self.track_window[1], self.track_window[2] - self.track_window[0],
                        self.track_window[3] - self.track_window[1])
                self.tracker.init(image, bbox)
        else:
            twist = Twist()
            ok, bbox = self.tracker.update(image)
            if ok and min(bbox) > 0:
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                center = [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]
                roi = self.depth_frame[p1[1]:p2[1], p1[0]:p2[0]]
                h, w = self.depth_frame.shape[:2]
                roi_h, roi_w = roi.shape
                if roi_h > 0 and roi_w > 0:
                    roi_cut = [1 / 3, 1 / 3]
                    if roi_w/w < 10/64:
                        roi_cut[1] = 0
                    if roi_h/h < 10/48:
                        roi_cut[0] = 0
                    roi_distance = roi[int(roi_h * roi_cut[0]):(roi_h - int(roi_h * roi_cut[0])),
                                   int(roi_w * roi_cut[1]):(roi_w - int(roi_w * roi_cut[1]))]
                    try:
                        distance = int(
                            np.mean(roi_distance[np.logical_and(roi_distance > 0, roi_distance < 30000)]) / 10)
                    except:
                        distance = self.DISTANCE
                    cv2.putText(image, 'Distance: ' + str(distance) + 'cm', (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 255, 0), 1)
                    # cv2.drawMarker(image, (int(center[0]), int(center[1])), (0, 255, 0), markerType=0, thickness=2, line_type=cv2.LINE_AA)
                    cv2.rectangle(image, p1, p2, (0, 255, 0), 2, 1)
                    h, w = image.shape[:2]

                    self.pid_d.SetPoint = self.distance
                    if abs(distance - self.distance) < 10:
                        distance = self.distance
                    self.pid_d.update(distance)  # 更新pid
                    tmp = self.go_speed - self.pid_d.output
                    self.linear_x = tmp
                    if tmp > 0.3:
                        self.linear_x = 0.3
                    if tmp < -0.3:
                        self.linear_x = -0.3
                    if abs(tmp) < 0.008:
                        self.linear_x = 0
                    twist.linear.x = self.linear_x

                    self.pid_angular.SetPoint = w / 2
                    if abs(center[0] - w / 2)/w < 1/64:
                        center[0] = w / 2
                    self.pid_angular.update(center[0])  # 更新pid
                    tmp = self.turn_speed + self.pid_angular.output
                    self.angular = tmp
                    if tmp > 1:
                        self.angular = 1
                    if tmp < -1:
                        self.angular = -1
                    if abs(tmp) < 0.05:
                        self.angular = 0
                    twist.angular.z = self.angular
                else:
                    cv2.putText(image, "Tracking failure detected !", (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 255, 0), 2)
            else:
                # Tracking failure
                cv2.putText(image, "Tracking failure detected !", (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 0), 1)
            self.mecanum_pub.publish(twist)
        self.fps.update()
        result_image = self.fps.show_fps(image)

        return result_image

    def run(self):
        while self.running:
            image = self.image_queue.get(block=True)
            try:
                result_image = self.image_proc(np.copy(image))
            except BaseException as e:
                print(e)
                result_image = image
            cv2.imshow(self.name, cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
            key = cv2.waitKey(1)
            if key != -1:
                self.mecanum_pub.publish(Twist())
                rospy.signal_shutdown('shutdown')

if __name__ == '__main__':
    KCFNode('kcf_tracking')
