#!/usr/bin/python3
#coding=utf8
# @data:2023/12/25
# 通过深度图识别物体进行分类
# 机械臂向下识别
# 可以识别长方体，球，圆柱体，以及他们的颜色
import os
import sys
import cv2
import math
import rospy
import queue
import signal
import threading
import numpy as np
import message_filters
from std_srvs.srv import SetBool
from sdk import pid, common, fps
from interfaces.srv import SetStringList
from kinematics import kinematics_control
from xf_mic_asr_offline import voice_play
from servo_msgs.msg import MultiRawIdPosDur
from sensor_msgs.msg import Image, CameraInfo
from servo_controllers import bus_servo_control
from ros_robot_controller.msg import BuzzerState
from std_srvs.srv import Trigger, TriggerResponse
from position_change_detect import position_reorder
sys.path.append('/home/ubuntu/software/arm_pc')
from action_group_controller import ActionGroupController

def depth_pixel_to_camera(pixel_coords, intrinsic_matrix):
    fx, fy, cx, cy = intrinsic_matrix[0], intrinsic_matrix[4], intrinsic_matrix[2], intrinsic_matrix[5]
    px, py, pz = pixel_coords
    x = (px - cx) * pz / fx
    y = (py - cy) * pz / fy
    z = pz
    return np.array([x, y, z])

class ObjectClassificationNode:
    hand2cam_tf_matrix = [
    [0.0, 0.0, 1.0, -0.101],
    [-1.0, 0.0, 0.0, 0.011],
    [0.0, -1.0, 0.0, 0.045],
    [0.0, 0.0, 0.0, 1.0]
]
    pick_offset = [0.01, 0.01, 0.0, -0.005, 0.02] # x1, x2, y1, y2, z 
    '''
                x1
        y1    center    y2
                x2

                arm
                car
    '''


    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.fps = fps.FPS()
        self.moving = False
        self.count = 0
        self.start = False
        self.shapes = None
        self.colors = None
        self.running = True
        self.target_shapes = ''
        self.roi = [70, 280, 170, 470]
        self.endpoint = None
        self.last_position = 0, 0
        self.last_object_info_list = []
        signal.signal(signal.SIGINT, self.shutdown)
        self.language = os.environ['ASR_LANGUAGE']
        self.image_queue = queue.Queue(maxsize=1)
        self.lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.servos_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.controller = ActionGroupController(use_ros=True)
        camera_name = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')
        rospy.wait_for_service('/%s/set_ldp'%camera_name)
        rospy.wait_for_service('/kinematics/set_joint_value_target')
        rospy.sleep(0.2)
        self.goto_default()
        rospy.ServiceProxy('/%s/set_ldp'%camera_name, SetBool)(False)
        rospy.Service('~start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法
        rospy.Service('~set_shape', SetStringList, self.set_shape_srv_callback)
        rospy.Service('~set_color', SetStringList, self.set_color_srv_callback)

        rgb_sub = message_filters.Subscriber('/%s/rgb/image_raw'%camera_name, Image, queue_size=1)
        depth_sub = message_filters.Subscriber('/%s/depth/image_raw'%camera_name, Image, queue_size=1)
        info_sub = message_filters.Subscriber('/%s/depth/camera_info'%camera_name, CameraInfo, queue_size=1)
    
        # 同步时间戳, 时间允许有误差在0.03s
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.02)
        sync.registerCallback(self.multi_callback) #执行反馈函数

        if rospy.get_param('~start', True):
            if rospy.get_param('~category', 'shape') == 'shape':
                msg = SetStringList()
                msg.data = ['sphere', 'cuboid', 'cylinder']
                self.set_shape_srv_callback(msg)
            else:
                msg = SetStringList()
                msg.data = ['red', 'green', 'blue']
                self.set_color_srv_callback(msg)

        self.image_proc()

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def set_shape_srv_callback(self, msg):
        rospy.loginfo("set_shape")
        self.colors = None
        self.shapes = msg.data
        self.start = True
        return [True, 'set_shape']

    def set_color_srv_callback(self, msg):
        rospy.loginfo("set_color")
        self.shapes = None
        self.colors = msg.data
        self.start = True
        return [True, 'set_color']

    def start_srv_callback(self, msg):
        rospy.loginfo("start")
        self.start = True
        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop')
        self.start = False
        self.colors = None
        self.shapes = None
        self.moving = False
        self.count = 0
        self.target_shapes = ''
        self.last_position = 0, 0
        self.last_object_info_list = []
        return TriggerResponse(success=True)

    def goto_default(self):
        endpoint = kinematics_control.set_joint_value_target([500, 470, 220, 70, 500])
        pose_t = endpoint.pose.position
        pose_r = endpoint.pose.orientation
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 470), (3, 220), (4, 70), (5, 500), (10, 200)))
        self.endpoint = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])

    def move(self, obejct_info):
        shape, pose_t = obejct_info[:2]
        color, angle = obejct_info[-2:]
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg) 
        rospy.sleep(1)
        if 'sphere' in shape:
            offset_z = -0.015 + self.pick_offset[-1]
        else:
            offset_z = 0.01 + self.pick_offset[-1]
        if pose_t[0] > 0.21:
            offset_x = self.pick_offset[0]
        else:
            offset_x = self.pick_offset[1]
        if pose_t[1] > 0:
            offset_y = self.pick_offset[2]
        else:
            offset_y = self.pick_offset[3]
        pose_t[0] += offset_x
        pose_t[1] += offset_y
        pose_t[2] += offset_z
        ret1 = kinematics_control.set_pose_target(pose_t, 85)
        if len(ret1[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1.5, ((1, ret1[1][0]), (2, ret1[1][1]), (3, ret1[1][2]), (4, ret1[1][3]),(5, ret1[1][4])))
            rospy.sleep(1.5)
        pose_t[2] -= 0.05
        ret2 = kinematics_control.set_pose_target(pose_t, 85)
        if angle != 0:
            if 'sphere' in shape or ('cylinder' in shape and 'cylinder_horizontal_' not in shape):
                angle = 500
            else:
                angle = angle % 180
                angle = angle - 180 if angle > 90 else (angle + 180 if angle < -90 else angle)
                if angle ==  90:
                    angle = 0
                angle = 500 + int(1000 * (angle + ret2[3][-1]) / 240)
        else:
            angle = 500
        if len(ret2[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 0.5, ((5, angle),))
            rospy.sleep(0.5)
            bus_servo_control.set_servos(self.servos_pub, 1, ((1, ret2[1][0]), (2, ret2[1][1]), (3, ret2[1][2]), (4, ret2[1][3]),(5, angle)))
            rospy.sleep(1)
            if shape == "sphere":
                bus_servo_control.set_servos(self.servos_pub, 0.6, ((10, 700),))
            else:
                bus_servo_control.set_servos(self.servos_pub, 0.6, ((10, 550),))
            rospy.sleep(0.6)
        if len(ret1[1]) > 0:
            bus_servo_control.set_servos(self.servos_pub, 1, ((1, ret1[1][0]), (2, ret1[1][1]), (3, ret1[1][2]), (4, ret1[1][3]),(5, angle)))
            rospy.sleep(1)
        bus_servo_control.set_servos(self.servos_pub, 1, ((1, 500), (2, 640), (3, 150), (4, 130), (5, 500), (10, 650)))
        rospy.sleep(1)
        if self.colors is None:
            print("shape: ", shape.split("_")[0])
            if "sphere" in shape:
                self.controller.runAction("target_1")
            if "cylinder" in shape:
                self.controller.runAction("target_2")
            if "cuboid" in shape:
                self.controller.runAction("target_3")
        else:
            color = self.color_comparison(color)
            print("color: ", color)
            if "red" == color:
                self.controller.runAction("target_1")
            if "blue" == color:
                self.controller.runAction("target_3")
        self.goto_default()
        rospy.sleep(2)
        self.moving = False

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))

    def cal_position(self, x, y, depth, intrinsic_matrix):
        position = depth_pixel_to_camera([x, y, depth / 1000], intrinsic_matrix)
        position[0] -= 0.01
        pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))
        world_pose = np.matmul(self.endpoint, pose_end)
        pose_t, pose_r = common.mat_to_xyz_euler(world_pose)
        return pose_t

    def get_min_distance(self, depth_image):
        ih, iw = depth_image.shape[:2]
        # 屏蔽掉一些区域，降低识别条件，使识别跟可靠
        depth_image[:, :self.roi[2]] = np.array([[1000, ] * self.roi[2]] * ih)
        depth_image[:, self.roi[3]:] = np.array([[1000, ] * (iw - self.roi[3])] * ih)
        depth_image[self.roi[1]:, :] = np.array([[1000, ] * iw] * (ih - self.roi[1]))
        depth_image[:self.roi[0], :] = np.array([[1000, ] * iw] * self.roi[0])
        depth = np.copy(depth_image).reshape((-1,))
        depth[depth <= 0] = 55555  # 距离为0可能是进入死区，或者颜色问题识别不到，将距离赋一个大值

        min_index = np.argmin(depth)  # 距离最小的像素
        min_y = min_index // iw
        min_x = min_index - min_y * iw

        min_dist = depth_image[min_y, min_x]  # 获取最小距离值
        # print("min_dist: ", min_dist)
        return min_dist

    def get_contours(self, depth_image, min_dist):
        depth_image = np.where(depth_image > 310, 0, depth_image)
        depth_image = np.where(depth_image > min_dist + 40, 0, depth_image)  # 将深度值大于最小距离15mm的像素置0
        sim_depth_image_sort = np.clip(depth_image, 0, 310).astype(np.float64) / 310 * 255
        depth_gray = sim_depth_image_sort.astype(np.uint8)
        _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
        # cv2.imshow('depth_bit', depth_bit)
        contours, hierarchy = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        return contours

    def shape_recognition(self, rgb_image, depth_image, depth_color_map, intrinsic_matrix, min_dist):
        object_info_list = []
        image_height, image_width = depth_image.shape[:2]
        if min_dist <= 320:
            sphere_index = 0
            cuboid_index = 0
            cylinder_index = 0
            cylinder_horizontal_index = 0
            contours = self.get_contours(depth_image, min_dist)
            h, w = depth_image.shape[:2]
            for obj in contours:
                area = cv2.contourArea(obj)
                if area/(h*w) < 300/(640*480):
                    continue
                # cv2.drawContours(depth_color_map, obj, -1, (255, 255, 0), 10)  # 绘制轮廓线
                perimeter = cv2.arcLength(obj, True)  # 计算轮廓周长
                approx = cv2.approxPolyDP(obj, 0.035 * perimeter, True)  # 获取轮廓角点坐标
                # cv2.drawContours(depth_color_map, approx, -1, (255, 0, 0), 4)  # 绘制轮廓线

                CornerNum = len(approx)
                (cx, cy), r = cv2.minEnclosingCircle(obj)
                center, (width, height), angle = cv2.minAreaRect(obj)
                if angle < -45:
                    angle += 89
                if width > height and width / height > 1.5:
                    angle = angle + 90
                depth = depth_image[int(cy), int(cx)]
                position = self.cal_position(cx, cy, depth, intrinsic_matrix)
                x, y, w, h = cv2.boundingRect(approx)
                mask = np.full((image_height, image_width), 0, dtype=np.uint8)
                cv2.drawContours(mask, [obj], -1, (255), cv2.FILLED)
                # 计算轮廓区域内像素值的标准差
                depth_image_mask = np.where(depth_image == 0, np.nan, depth_image)
                depth_std = np.nanstd(np.where(mask == 0, np.nan, depth_image_mask))
                # cv2.imshow('mask', mask)
                # arr = np.where(mask == 0, np.nan, depth_image_mask)
                # print(arr[~np.isnan(arr)])
                # print(depth_std, CornerNum)

                objType = None
                if depth_std > 3.0 and CornerNum > 4:
                    sphere_index += 1
                    angle = 0
                    objType = 'sphere_' + str(sphere_index)
                elif depth_std > 1.5:
                    cylinder_horizontal_index += 1
                    objType = "cylinder_horizontal_" + str(cylinder_horizontal_index)
                else:
                    if 4 <= CornerNum <= 6:
                        hull = cv2.convexHull(obj)
                        area = cv2.contourArea(hull)
                        # 计算轮廓的凸包
                        perimeter = cv2.arcLength(hull, closed=True)
                        # print(width * height / area, depth_std, perimeter/area, 2*(1/width + 1/height), 2/r)

                        # cv2.drawContours(depth_color_map, hull, -1, (255, 255, 0), 10)
                        # print(perimeter/area)
                        if perimeter / area < 0.07:
                            cuboid_index += 1
                            objType = "cuboid_" + str(cuboid_index)
                        else:
                            cylinder_index += 1
                            angle = 0
                            objType = "cylinder_" + str(cylinder_index)
                    elif CornerNum > 6:
                        cylinder_index += 1
                        angle = 0
                        objType = "cylinder_" + str(cylinder_index)
                if objType is not None:
                    object_info_list.append([objType, position, depth, [x, y, w, h, center, width, height], rgb_image[int(center[1]), int(center[0])], angle])
                    # cv2.putText(depth_color_map, objType[:-2], (x + w // 2, y + (h //2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0, 0), 2, cv2.LINE_AA)
                    # cv2.putText(depth_color_map, objType[:-2], (x + w // 2, y + (h //2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 1)
                    cv2.rectangle(depth_color_map, (x, y), (x + w, y + h), (255, 255, 255), 2)

        return object_info_list

    def color_comparison(self, rgb):
        if rgb[0] > rgb[1] and rgb[0] > rgb[2]:
            return 'red'
        elif rgb[2] > rgb[1] and rgb[2] > rgb[1]:
            return 'blue'
        else:
            return None

    def image_proc(self):
        while self.running:
            ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True)
            try:
                rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
                # cv2.imshow('rgb', cv2.applyColorMap(depth_image.astype(np.uint8), cv2.COLORMAP_JET))
                h, w = depth_image.shape[:2]
                rgb_h, rgb_w = rgb_image.shape[:2]
                rgb_image = rgb_image[int((rgb_h - h)/2):h+int((rgb_h - h)/2), :]
                depth_image = depth_image.copy()
                min_dist = self.get_min_distance(depth_image)
                sim_depth_image = np.clip(depth_image, 0, 380).astype(np.float64) / 380 * 255
                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                h, w = rgb_image.shape[:2]
                if not self.moving:
                    object_info_list = self.shape_recognition(rgb_image, depth_image, depth_color_map, depth_camera_info.K, min_dist)
                    if self.start:
                        reorder_object_info_list = object_info_list
                        if object_info_list:
                            if self.last_object_info_list:
                                # 对比上一次的物体的位置来重新排序
                                reorder_object_info_list = position_reorder(object_info_list, self.last_object_info_list, 20)
                        if reorder_object_info_list:
                            if not self.target_shapes:
                                if self.shapes is not None:
                                    indices = [i for i, info in enumerate(reorder_object_info_list) if info[0].split('_')[0] in self.shapes]
                                else:
                                    indices = [i for i, info in enumerate(reorder_object_info_list) if self.color_comparison(info[-2]) in self.colors]
                                if indices:
                                    min_depth_index = min(indices, key=lambda i: reorder_object_info_list[i][2])
                                    self.target_shapes = reorder_object_info_list[min_depth_index][0].split('_')[0]
                            else:
                                # print(1, self.target_shapes, reorder_object_info_list)
                                # for i, info in enumerate(reorder_object_info_list):
                                    # print(info[0].split('_')[0])
                                # print(reorder_object_info_list, self.target_shapes)
                                target_index = [i for i, info in enumerate(reorder_object_info_list) if info[0].split('_')[0] == self.target_shapes]
                                if target_index:
                                    target_index = target_index[0]
                                    # print(target_index)
                                    obejct_info = reorder_object_info_list[target_index]
                                    x, y, w, h, center, width, height = obejct_info[3]
                                    angle = obejct_info[-1]
                                    cv2.putText(depth_color_map, self.target_shapes, (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0,
                                                (0, 0, 0), 2, cv2.LINE_AA)
                                    cv2.putText(depth_color_map, self.target_shapes, (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0,
                                                (255, 255, 255), 1)
                                    cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (width, height), angle)))], -1,
                                                     (0, 0, 255), 2, cv2.LINE_AA)
                                    position = obejct_info[1]
                                    e_distance = round(math.sqrt(pow(self.last_position[0] - position[0], 2)) + math.sqrt(
                                        pow(self.last_position[1] - position[1], 2)), 5)
                                    if e_distance <= 0.005:
                                        self.count += 1
                                    else:
                                        self.count = 0
                                    if self.count > 5:
                                        self.count = 0
                                        self.target_shapes = None
                                        self.moving = True
                                        if self.colors is not None:
                                            voice_play.play(self.color_comparison(obejct_info[-2]), language=self.language)
                                        else:
                                            voice_play.play(obejct_info[0].split('_')[0], language=self.language)
                                        threading.Thread(target=self.move, args=(obejct_info,)).start()
                                    self.last_position = position
                                else:
                                    self.target_shapes = None

                        self.last_object_info_list = reorder_object_info_list

                cv2.rectangle(rgb_image, (self.roi[2], self.roi[0]), (self.roi[3], self.roi[1]), (255, 255, 0), 1)
                bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

                self.fps.update()
                result_image = np.concatenate([depth_color_map, bgr_image], axis=1)
                cv2.imshow("depth", result_image)
                key = cv2.waitKey(1)
                if key != -1:
                    rospy.signal_shutdown('shutdown1')
            except Exception as e:
                rospy.logerr('callback error:', str(e))

if __name__ == "__main__":
    ObjectClassificationNode('object_classification')
