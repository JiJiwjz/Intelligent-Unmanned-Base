#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/12/19
# 机械前超前看识别追踪空中指定颜色物品
# 通过深度相机识别计算物品的空间位置
# 完成抓取并放到指定位置
import cv2
import math
import time
import rospy
import queue
import signal
import threading
import numpy as np
import message_filters
from std_srvs.srv import SetBool
from sdk import pid, common, fps
from interfaces.srv import SetString
from interfaces.srv import GetRobotPose
from kinematics import kinematics_control
from servo_msgs.msg import MultiRawIdPosDur
from sensor_msgs.msg import Image, CameraInfo
from servo_controllers import bus_servo_control
from std_srvs.srv import Trigger, TriggerResponse

def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])

class ColorTracker:
    def __init__(self, target_color):
        self.target_color = target_color
        self.pid_yaw = pid.PID(20.5, 1.0, 1.2)
        self.pid_pitch = pid.PID(20.5, 1.0, 1.2)
        self.yaw = 500
        self.pitch = 150
    
    def proc(self, source_image, result_image, color_ranges):
        h, w = source_image.shape[:2]
        color = color_ranges['lab']['gemini_camera'][self.target_color]

        img = cv2.resize(source_image, (int(w/2), int(h/2)))
        img_blur = cv2.GaussianBlur(img, (3, 3), 3) # 高斯模糊
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间
        mask = cv2.inRange(img_lab, tuple(color['min']), tuple(color['max'])) # 二值化

        # 平滑边缘，去除小块，合并靠近的块
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        # 找出最大轮廓
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        min_c = None
        for c in contours:
            if math.fabs(cv2.contourArea(c))/(h*w) < 50/(640*480):
                continue
            (center_x, center_y), radius = cv2.minEnclosingCircle(c) # 最小外接圆
            if min_c is None:
                min_c = (c, center_x)
            elif center_x < min_c[1]:
                if center_x < min_c[1]:
                    min_c = (c, center_x)

        # 如果有符合要求的轮廓
        if min_c is not None:
            (center_x, center_y), radius = cv2.minEnclosingCircle(min_c[0]) # 最小外接圆

            # 圈出识别的的要追踪的色块
            circle_color = common.range_rgb[self.target_color] if self.target_color in common.range_rgb else (0x55, 0x55, 0x55)
            cv2.circle(result_image, (int(center_x * 2), int(center_y * 2)), int(radius * 2), circle_color, 2)

            center_x = center_x * 2
            center_x_1 = center_x / w
            if abs(center_x_1 - 0.5) > 0.02: # 相差范围小于一定值就不用再动了
                self.pid_yaw.SetPoint = 0.5 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置
                self.pid_yaw.update(center_x_1)
                self.yaw = min(max(self.yaw + self.pid_yaw.output, 0), 1000)
            else:
                self.pid_yaw.clear() # 如果已经到达中心了就复位一下 pid 控制器

            center_y = center_y * 2
            center_y_1 = center_y / h
            if abs(center_y_1 - 0.5) > 0.02:
                self.pid_pitch.SetPoint = 0.5
                self.pid_pitch.update(center_y_1)
                self.pitch = min(max(self.pitch + self.pid_pitch.output, 100), 720)
            else:
                self.pid_pitch.clear()
            # rospy.loginfo("x:{:.2f}\ty:{:.2f}".format(self.x , self.y))
            return (result_image, (self.pitch, self.yaw), (center_x, center_y), radius * 2)
        else:
            return (result_image, None, None, 0)


class TrackAndGrapNode:
    hand2cam_tf_matrix = [
    [0.0, 0.0, 1.0, -0.101],
    [-1.0, 0.0, 0.0, 0.011],
    [0.0, -1.0, 0.0, 0.045],
    [0.0, 0.0, 0.0, 1.0]
]

    def __init__(self, name):
        rospy.init_node(name, anonymous=True, log_level=rospy.INFO)
        self.fps = fps.FPS()
        self.moving = False
        self.count = 0
        self.start = False
        self.running = True
        self.last_pitch_yaw = (0, 0)
        self.first_time = time.time()
        signal.signal(signal.SIGINT, self.shutdown)
        self.enable_disp = 1
        self.lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
        self.last_position = (0, 0, 0)
        self.stamp = time.time()
        self.servos_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        rospy.sleep(0.2)
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 200)))
        rospy.sleep(1)
        self.target_color = None
        camera_name = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')
        rospy.wait_for_service('/%s/set_ldp'%camera_name)
       
        rospy.Service('~start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)
        self.tracker = None
        if rospy.get_param('~start', True):
            self.target_color = rospy.get_param('~color', 'blue')
 
            msg = SetString()
            msg.data = self.target_color
            self.set_color_srv_callback(msg)

        self.image_queue = queue.Queue(maxsize=1)
        self.endpoint = None

        self.ttt = time.time() + 3
        rospy.ServiceProxy('/%s/set_ldp'%camera_name, SetBool)(False)
        rgb_sub = message_filters.Subscriber('/%s/rgb/image_raw'%camera_name, Image, queue_size=1)
        depth_sub = message_filters.Subscriber('/%s/depth/image_raw'%camera_name, Image, queue_size=1)
        info_sub = message_filters.Subscriber('/%s/depth/camera_info'%camera_name, CameraInfo, queue_size=1)

        # 同步时间戳, 时间允许有误差在0.03s
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.02)
        sync.registerCallback(self.multi_callback) #执行反馈函数

        common.loginfo("TrackAndGrapNode initailized")
        self.image_proc()

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def set_color_srv_callback(self, msg):
        rospy.loginfo("set_color")
        self.target_color = msg.data
        self.tracker = ColorTracker(self.target_color)
        self.start = True
        return [True, 'set_color']

    def start_srv_callback(self, msg):
        rospy.loginfo("start")
        self.start = True
        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop')
        self.start = False
        self.moving = False
        self.count = 0
        self.last_pitch_yaw = (0, 0)
        self.last_position = (0, 0, 0)
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 200)))
        return TriggerResponse(success=True)

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))

    def get_endpoint(self):
        endpoint = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose)().pose
        self.endpoint = common.xyz_quat_to_mat([endpoint.position.x, endpoint.position.y, endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
        return self.endpoint

    def pick(self, position):
        if position[2] < 0.2:
            yaw = 80
        else:
            yaw = 30
        ret = kinematics_control.set_pose_target(position, yaw)
        # print(ret, position, yaw)
        if len(ret[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1, ((1, ret[1][0]), ))
            rospy.sleep(1)
            bus_servo_control.set_servos(self.servos_pub, 1.5, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, ret[1][4])))
            rospy.sleep(1.5)
        bus_servo_control.set_servos(self.servos_pub, 0.5, ((10, 600),))
        rospy.sleep(1)
        position[2] += 0.03
        ret = kinematics_control.set_pose_target(position, yaw)
        if len(ret[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1, ((1, ret[1][0]),(2, ret[1][1]), (3, ret[1][2]),(4, ret[1][3]), (5, ret[1][4])))
            rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 600)))
        rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 125), (2, 635), (3, 120), (4, 200), (5, 500)))
        rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 1.5, ((1, 125), (2, 325), (3, 200), (4, 290), (5, 500)))
        rospy.sleep(1.5)
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 125), (2, 325), (3, 200), (4, 290), (5, 500), (10, 200)))
        rospy.sleep(1.5)
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 200)))
        rospy.sleep(2)
        self.tracker.yaw = 500
        self.tracker.pitch = 150
        self.tracker.pid_yaw.clear()
        self.tracker.pid_pitch.clear()
        self.stamp = time.time()
        self.first_time = time.time() + 2
        self.moving = False

    def image_proc(self):
        while self.running:
            ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True)
            try:
                rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
                h, w = depth_image.shape[:2]
                rgb_h, rgb_w = rgb_image.shape[:2]
                rgb_image = rgb_image[int((rgb_h - h)/2):h+int((rgb_h - h)/2), :]
                result_image = np.copy(rgb_image)
                h, w = depth_image.shape[:2]
                depth = np.copy(depth_image).reshape((-1, ))
                depth[depth<=0] = 55555
                
                sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64)

                sim_depth_image = sim_depth_image / 2000.0 * 255.0
                bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

                if self.tracker is not None and self.moving is False and time.time() > self.ttt and self.start:
                    result_image, p_y, center, r = self.tracker.proc(rgb_image, result_image, self.lab_data)
                    if p_y is not None:
                        bus_servo_control.set_servos(self.servos_pub, 0.02, ((1, p_y[1]), (4, p_y[0])))
                        center_x, center_y = center
                        if center_x > w:
                            center_x = w
                        if center_y > h:
                            center_y = h
                        if abs(self.last_pitch_yaw[0] - p_y[0]) < 3 and abs(self.last_pitch_yaw[1] - p_y[1]) < 3:
                            if time.time() - self.stamp > 2:
                                self.stamp = time.time()
                                roi = [int(center_y) - 5, int(center_y) + 5, int(center_x) - 5, int(center_x) + 5]
                                if roi[0] < 0:
                                    roi[0] = 0
                                if roi[1] > h:
                                    roi[1] = h
                                if roi[2] < 0:
                                    roi[2] = 0
                                if roi[3] > w:
                                    roi[3] = w
                                roi_distance = depth_image[roi[0]:roi[1], roi[2]:roi[3]] 
                                try:
                                    dist = round(float(np.mean(roi_distance[np.logical_and(roi_distance>0, roi_distance<10000)])/1000.0), 3)
                                except BaseException as e:
                                    print(e)
                                    txt = "DISTANCE ERROR !!!"
                                    return
                                if np.isnan(dist):
                                    txt = "DISTANCE ERROR !!!"
                                    return
                                dist += 0.015 # 物体半径补偿
                                dist += 0.015 # 误差补偿
                                K = depth_camera_info.K
                                self.get_endpoint()
                                position = depth_pixel_to_camera((center_x, center_y), dist, (K[0], K[4], K[2], K[5]))
                                position[0] -= 0.01  # rgb相机和深度相机tf有1cm偏移
                                pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标
                                world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标
                                pose_t, pose_R = common.mat_to_xyz_euler(world_pose)
                                self.stamp = time.time()
                                self.moving = True
                                threading.Thread(target=self.pick, args=(pose_t,)).start()
                        else:
                            self.stamp = time.time()
                        dist = depth_image[int(center_y),int(center_x)]
                        if dist < 100:
                            txt = "TOO CLOSE !!!"
                        else:
                            txt = "Dist: {}mm".format(dist)
                        cv2.circle(result_image, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                        cv2.circle(depth_color_map, (int(center_x), int(center_y)), 5, (255, 255, 255), -1)
                        cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 0), 10, cv2.LINE_AA)
                        cv2.putText(depth_color_map, txt, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 2, cv2.LINE_AA)
                        self.last_pitch_yaw = p_y
                    else:
                        self.stamp = time.time()
                if self.enable_disp:
                    result_image = np.concatenate([cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR), depth_color_map, ], axis=1)
                    cv2.imshow("depth", result_image)
                    key = cv2.waitKey(1)
                    if key != -1:
                        rospy.signal_shutdown('shutdown1')

            except Exception as e:
                rospy.logerr('callback error:', str(e))

if __name__ == "__main__":
    TrackAndGrapNode('track_and_grap')

