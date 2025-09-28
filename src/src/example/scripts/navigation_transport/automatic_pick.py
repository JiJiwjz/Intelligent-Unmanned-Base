#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# 追踪拾取
import os
import sys
import cv2
import time
import math
import queue
import rospy
import threading
import numpy as np
from sdk.pid import PID
import sdk.misc as misc
from sensor_msgs.msg import Image
import sdk.common as common
from geometry_msgs.msg import Twist
from xf_mic_asr_offline import voice_play
from interfaces.msg import Pose2D
from interfaces.srv import SetPose2D
from std_srvs.srv import Trigger, TriggerResponse
from servo_msgs.msg import MultiRawIdPosDur
from servo_controllers import bus_servo_control
sys.path.append('/home/ubuntu/software/arm_pc')
from action_group_controller import ActionGroupController

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

config_path = os.path.join(os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../..')), 
        'config/automatic_pick_roi.yaml')

debug = False
start_pick = False
start_place = False
target_color = ""
linear_base_speed = 0.007
angular_base_speed = 0.03

yaw_pid = PID(P=0.015, I=0, D=0.000)
# linear_pid = PID(P=0, I=0, D=0)
linear_pid = PID(P=0.0028, I=0, D=0)
# angular_pid = PID(P=0, I=0, D=0)
angular_pid = PID(P=0.003, I=0, D=0)

linear_speed = 0
angular_speed = 0
yaw_angle = 90

pick_stop_x = 32/64
pick_stop_y = 388/480
place_stop_x = 32/64
place_stop_y = 388/480
stop = True

d_y = 1/48
d_x = 1/64

pick = False
place = False

broadcast_status = ''
status = "approach"
count_stop = 0
count_turn = 0

lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
image_queue = queue.Queue(maxsize=1)

def cancel_callback(msg):
    global start_pick, start_place

    start_pick = False
    start_place = False

    return TriggerResponse(success=True)


def start_pick_callback(msg):
    global start_pick, yaw_angle, pick_stop_x, pick_stop_y, stop, status, target_color, broadcast_status
    global linear_speed, angular_speed
    global d_x, d_y
    global pick, place
    global count_turn, count_stop

    rospy.loginfo("start pick")
    rospy.set_param('~status', 'start_pick')

    linear_speed = 0
    angular_speed = 0
    yaw_angle = 90

    param = rospy.get_param('/automatic_pick/pick_stop_pixel_coordinate')
    pick_stop_x = param[0] 
    pick_stop_y = param[1]
    stop = True

    d_y = 1/48
    d_x = 1/64

    pick = False
    place = False

    status = "approach"
    target_color = 'blue'
    broadcast_status = 'find_target'
    count_stop = 0
    count_turn = 0

    linear_pid.clear()
    angular_pid.clear()
    start_pick = True

    return TriggerResponse(success=True)


def start_place_callback(msg):
    global start_place, place_stop_x, place_stop_y, stop, target_color, broadcast_status
    global linear_speed, angular_speed
    global d_x, d_y
    global pick, place

    rospy.loginfo("start place")
    rospy.set_param('~status', 'start_place')

    linear_speed = 0
    angular_speed = 0
    d_y = 3/48
    d_x = 3/64
    
    param = rospy.get_param('~place_stop_pixel_coordinate')
    place_stop_x = param[0] 
    place_stop_y = param[1]
    stop = True
    pick = False
    place = False
    target_color = 'red'
    broadcast_status = 'mission_completed'

    linear_pid.clear()
    angular_pid.clear()
    start_place = True

    return TriggerResponse(success=True)


# 颜色识别
size = (0.5, 0.5)

def colorDetect(img):
    img_h, img_w = img.shape[:2]
    resize_image = (int(size[0]*img_w), int(size[1]*img_h))
    frame_resize = cv2.resize(img, resize_image, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
    frame_mask = cv2.inRange(frame_lab, tuple(lab_data['lab'][camera_name][target_color]['min']),
                             tuple(lab_data['lab'][camera_name][target_color]['max']))  # 对原图像和掩模进行位运算

    eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀

    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓

    center_x, center_y, angle = -1, -1, -1
    if len(contours) != 0:
        areaMaxContour, area_max = common.get_area_max_contour(contours, 10)  # 找出最大轮廓
        if areaMaxContour is not None:
            #print(111111111111, area_max)
            if 10 < area_max:  # 有找到最大面积
                rect = cv2.minAreaRect(areaMaxContour)  # 最小外接矩形
                angle = rect[2]
                box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
                for j in range(4):
                    box[j, 0] = int(misc.val_map(box[j, 0], 0, resize_image[0], 0, img_w))
                    box[j, 1] = int(misc.val_map(box[j, 1], 0, resize_image[1], 0, img_h))

                cv2.drawContours(img, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形
                # 获取矩形的对角点
                ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                radius = abs(ptime_start_x - pt3_x)
                center_x, center_y = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2)  # 中心点
                cv2.circle(img, (center_x, center_y), 5, (0, 255, 255), -1)  # 画出中心点

    return center_x, center_y, angle


def action_thread():
    global pick, place, start_pick, start_place, broadcast_status
    while True:
        if pick:
            mecnum_pub.publish(Twist())
            rospy.sleep(0.5)
            bus_servo_control.set_servos(joints_pub, 1.5, ((1, 500), (2, 190), (3, 300), (4, 300), (5, 500), (10, 200)))
            time.sleep(2)
            bus_servo_control.set_servos(joints_pub, 0.5, ((1, 500), (2, 190), (3, 300), (4, 300), (5, 500), (10, 200)))
            time.sleep(0.5)
            bus_servo_control.set_servos(joints_pub, 0.5, ((1, 500), (2, 190), (3, 300), (4, 300), (5, 500), (10, 560)))
            time.sleep(0.5)
            bus_servo_control.set_servos(joints_pub, 0.5, ((1, 500), (2, 190), (3, 300), (4, 300), (5, 500), (10, 560)))
            time.sleep(0.3)
            bus_servo_control.set_servos(joints_pub, 1.5, ((1, 500), (2, 700), (3, 15), (4, 215), (5, 500), (10, 560)))    
            time.sleep(1.5)
            bus_servo_control.set_servos(joints_pub, 0.3, ((1, 500), (2, 700), (3, 15), (4, 215), (5, 500), (10, 560)))
            time.sleep(0.3)
            # controller.runAction('navigation_pick')
            if broadcast and broadcast_status == 'crawl_succeeded':
                broadcast_status = 'mission_completed'
                voice_play.play('crawl_succeeded', language=language)
            rospy.set_param('~status', 'pick_finish')
            start_pick = False
            pick = False
            if broadcast:
                pose = Pose2D()
                pose.x = place_position[0]
                pose.y = place_position[1]
                pose.roll = place_position[2]
                pose.pitch = place_position[3]
                pose.yaw = place_position[4]
                rospy.ServiceProxy('/navigation_transport/place', SetPose2D)(pose)
            print('pick finish')
        elif place:
            mecnum_pub.publish(Twist())
            rospy.sleep(1)
            # controller.runAction('navigation_place')
            bus_servo_control.set_servos(joints_pub, 1.5, ((1, 500), (2, 200), (3, 310), (4, 325), (5, 500), (10, 560)))
            time.sleep(1.5)
            bus_servo_control.set_servos(joints_pub, 0.3, ((1, 500), (2, 200), (3, 310), (4, 325), (5, 500), (10, 560)))
            time.sleep(0.3)
            bus_servo_control.set_servos(joints_pub, 0.5, ((1, 500), (2, 200), (3, 310), (4, 325), (5, 500), (10, 200)))
            time.sleep(0.5)
            bus_servo_control.set_servos(joints_pub, 0.3, ((1, 500), (2, 200), (3, 310), (4, 325), (5, 500), (10, 200)))
            time.sleep(0.3)
            bus_servo_control.set_servos(joints_pub, 1.5, ((1, 500), (2, 700), (3, 15), (4, 215), (5, 500), (10, 200)))    
            time.sleep(1.5)
            bus_servo_control.set_servos(joints_pub, 0.3, ((1, 500), (2, 700), (3, 15), (4, 215), (5, 500), (10, 200)))
            time.sleep(0.3)
            if broadcast and broadcast_status == 'mission_completed':
                broadcast_status = ''
                voice_play.play('mission_completed', language=language)
            rospy.set_param('~status', 'place_finish')
            start_place = False
            place = False
            print('place finish')
        else:
            rospy.sleep(0.01)

count = 0
def pick_handle(image):
    global pick, count_turn, count_stop, angular_speed, linear_speed, status, d_x, d_y, broadcast_status, count, pick_stop_y, pick_stop_x, debug

    h, w = image.shape[:2]

    twist = Twist()
    if not pick or debug:
        object_center_x, object_center_y, object_angle = colorDetect(image)  # 获取物体颜色的中心和角度
        if debug:
            count += 1
            if count > 10:
                count = 0
                pick_stop_y = object_center_y/h
                pick_stop_x = object_center_x/w
                config = common.get_yaml_data(config_path)
                config['pick_stop_pixel_coordinate'] = [pick_stop_x, pick_stop_y]
                common.save_yaml_data(config, config_path)
                debug = False
            # print(object_center_x, object_center_y)  # 打印当前物体中心的像素
        elif object_center_x > 0:
            if broadcast and broadcast_status == 'find_target':
                broadcast_status = 'crawl_succeeded'
                voice_play.play('find_target', language=language)
            ########电机pid处理#########
            # 以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入#
            linear_pid.SetPoint = int(pick_stop_y*h)
            if abs(object_center_y/h - pick_stop_y) <= d_y:
                object_center_y = int(pick_stop_y*h)
            if status != "align":
                linear_pid.update(object_center_y)  # 更新pid
                tmp = linear_base_speed + linear_pid.output

                linear_speed = tmp
                # print('tmp', tmp)
                if tmp > 0.15:
                    linear_speed = 0.15
                if tmp < -0.15:
                    linear_speed = -0.15
                if abs(tmp) <= 0.0075:
                    linear_speed = 0

            angular_pid.SetPoint = int(pick_stop_x*w)
            if abs(object_center_x/w - pick_stop_x) <= d_x:
                object_center_x = int(pick_stop_x*w)
            # print(pick_stop_x*w, pick_stop_y*h)
            if status != "align":
                angular_pid.update(object_center_x)  # 更新pid
                tmp = angular_base_speed + angular_pid.output

                angular_speed = tmp
                if tmp > 1.2:
                    angular_speed = 1.2
                if tmp < -1.2:
                    angular_speed = -1.2
                if abs(tmp) <= 0.038:
                    angular_speed = 0
            
            if abs(linear_speed) == 0 and abs(angular_speed) == 0:
                count_turn += 1
                if count_turn > 5:
                    count_turn = 5
                    status = "align"
                    if count_stop < 10:  # 连续10次都没在移动
                        if object_angle < 40: # 不取45，因为如果在45时值的不稳定会导致反复移动
                            object_angle += 90
                        yaw_pid.SetPoint = 90
                        if abs(object_angle - 90) <= 1:
                            object_angle = 90
                        yaw_pid.update(object_angle)  # 更新pid
                        yaw_angle = yaw_pid.output
                        if object_angle != 90:
                            if abs(yaw_angle) <=0.038:
                                count += 1
                            else:
                                count_stop = 0
                            twist.linear.y = -2 * 0.3 * math.sin(yaw_angle / 2)
                            twist.angular.z = yaw_angle
                        else:
                            count_stop += 1
                    elif count_stop <= 20:
                        d_x = 5/640
                        d_y = 5/480
                        count_stop += 1
                        status = "adjust"
                    else:
                        count_stop = 0
                        pick = True

            else:
                if count_stop >= 10:
                    count_stop = 10
                count_turn = 0
                if status != 'align':
                    twist.linear.x = linear_speed
                    twist.angular.z = angular_speed

    mecnum_pub.publish(twist)

    return image


def place_handle(image):
    global place, angular_speed, linear_speed

    twist = Twist()
    h, w = image.shape[:2]
    if not place or debug:
        object_center_x, object_center_y, object_angle = colorDetect(image)  # 获取物体颜色的中心和角度
        if debug:
            print(object_center_x/w, object_center_y/h)  # 打印当前物体离中心的像素距离
        elif object_center_x > 0:
            ########电机pid处理#########
            # 以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入#
            linear_pid.SetPoint = int(place_stop_y*h)
            if abs(object_center_y/h - place_stop_y) <= d_y:
                object_center_y = int(place_stop_y*h)
            linear_pid.update(object_center_y)  # 更新pid
            tmp = linear_base_speed + linear_pid.output

            linear_speed = tmp
            if tmp > 0.15:
                linear_speed = 0.15
            if tmp < -0.15:
                linear_speed = -0.15
            if abs(tmp) <= 0.0075:
                linear_speed = 0

            angular_pid.SetPoint = int(place_stop_x*w)
            if abs(object_center_x/w- place_stop_x) <= d_x:
                object_center_x = int(place_stop_x*w)

            angular_pid.update(object_center_x)  # 更新pid
            tmp = angular_base_speed + angular_pid.output

            angular_speed = tmp
            if tmp > 1.2:
                angular_speed = 1.2
            if tmp < -1.2:
                angular_speed = -1.2
            if abs(tmp) <= 0.035:
                angular_speed = 0

            if abs(linear_speed) == 0 and abs(angular_speed) == 0:
                place = True
            else:
                twist.linear.x = linear_speed
                twist.angular.z = angular_speed

    mecnum_pub.publish(twist)

    return image


def image_callback(ros_image):
    rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                           buffer=ros_image.data)  # 将自定义图像消息转化为图像
    if image_queue.full():
        # 如果队列已满，丢弃最旧的图像
        image_queue.get()
    # 将图像放入队列
    image_queue.put(rgb_image)

def run():
    global place, stop
    
    while True:
        image = image_queue.get(block=True)
        if start_pick:
            stop = True
            result_image = pick_handle(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        elif start_place:
            stop = True
            if place_without_color:
                place = True
                result_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            else:
                result_image = place_handle(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        else:
            if stop:
                stop = False
                mecnum_pub.publish(Twist())
            result_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        ros_image = common.cv2_image2ros(result_image)
        image_pub.publish(ros_image)

if __name__ == '__main__':
    rospy.init_node('automatic_pick', anonymous=True)
    
    rospy.set_param('~status', 'start')
    broadcast = rospy.get_param('~broadcast', False)
    language = os.environ['ASR_LANGUAGE']

    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    mecnum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
    image_pub = rospy.Publisher('~image_result', Image, queue_size=1)
    camera_name = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')
    rospy.Subscriber(camera_name + '/rgb/image_raw', Image, image_callback)

    rospy.Service('~pick', Trigger, start_pick_callback)
    rospy.Service('~place', Trigger, start_place_callback)
    rospy.Service('~cancel', Trigger, cancel_callback)

    place_position = rospy.get_param('~place_position', [1.0, 1.0, 0, 0, 0])
    place_without_color = rospy.get_param('~place_without_color', True)
    debug = rospy.get_param('~debug', False)
    controller = ActionGroupController(use_ros=True)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        try:
            if rospy.get_param('/servo_manager/init_finish') and rospy.get_param(
                    '/joint_states_publisher/init_finish'):
                break
        except:
            rospy.sleep(0.1)
    # controller.runAction('navigation_pick_init')
   
    bus_servo_control.set_servos(joints_pub, 2, ((1, 500), (2, 700), (3, 15), (4, 215), (5, 500), (10, 200)))
    time.sleep(2)
    mecnum_pub.publish(Twist())
    threading.Thread(target=action_thread, daemon=True).start()
    if debug:
        controller.runAction('navigation_pick_debug')
        rospy.sleep(5)
        bus_servo_control.set_servos(joints_pub, 2, ((1, 500), (2, 700), (3, 15), (4, 215), (5, 500), (10, 200)))
        time.sleep(2)
        start_pick_callback(None)
    
    rospy.set_param('~init_finish', True)
    run()
