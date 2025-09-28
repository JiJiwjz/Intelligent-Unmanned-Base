#!/usr/bin/env python3
# encoding: utf-8
import os
import sys
import cv2
import time
import math
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

config_path = os.path.join(os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../../..')), 'config/config.yaml')
image_sub = None
debug = False
start_pick = False
close = False
start_place = False
target_color = ""
linear_base_speed = 0.007
angular_base_speed = 0.03

yaw_pid = PID(P=0.015, I=0, D=0.000)
linear_pid = PID(P=0.0018, I=0, D=0)
angular_pid = PID(P=0.003, I=0, D=0)

linear_speed = 0
angular_speed = 0
yaw_angle = 90

pick_stop_x = 320
pick_stop_y = 388
place_stop_x = 320
place_stop_y = 388
stop = True

d_y = 10
d_x = 10

pick = False
place = False

broadcast_status = ''
status = "approach"
count_stop = 0
count_turn = 0

lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")

def start_pick_callback(msg):
    global start_pick, yaw_angle, pick_stop_x, pick_stop_y, stop, status, target_color, broadcast_status
    global linear_speed, angular_speed
    global d_x, d_y
    global pick, place
    global count_turn, count_stop

    rospy.loginfo("start pick_1")
    rospy.set_param('~status', 'pick')

    bus_servo_control.set_servos(joints_pub, 2, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 200)))
    rospy.sleep(2)
    linear_speed = 0
    angular_speed = 0
    yaw_angle = 90

    param = rospy.get_param('/pick_stop_pixel_coordinate')
    pick_stop_x = param[0] 
    pick_stop_y = param[1]
    stop = True

    d_y = 5
    d_x = 5

    pick = False
    place = False

    status = "approach"
    target_color = 'box'
    broadcast_status = 'find_target'
    count_stop = 0
    count_turn = 0

    linear_pid.clear()
    angular_pid.clear()
    start_pick = True

    return TriggerResponse(success=True)

def start_pick_2_callback(msg):
    global start_pick, yaw_angle, pick_stop_x, pick_stop_y, stop, status, target_color, broadcast_status
    global linear_speed, angular_speed
    global d_x, d_y
    global pick, place
    global count_turn, count_stop

    rospy.loginfo("start pick_2")
    rospy.set_param('~status', 'pick')

    bus_servo_control.set_servos(joints_pub, 1, ((1, 500), (2, 500), (3, 150), (4, 130), (5, 500), (10, 200)))
    rospy.sleep(2)

    rospy.set_param('~status', 'stop')
    return TriggerResponse(success=True)

def start_pick_3_callback(msg):
    global start_pick, yaw_angle, pick_stop_x, pick_stop_y, stop, status, target_color, broadcast_status
    global linear_speed, angular_speed
    global d_x, d_y
    global pick, place
    global count_turn, count_stop

    rospy.loginfo("start pick_3")
    rospy.set_param('~status', 'pick')

    # bus_servo_control.set_servos(joints_pub, 2, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 200)))
    # rospy.sleep(2)
    linear_speed = 0
    angular_speed = 0
    yaw_angle = 90

    pick_stop_x = 320 
    pick_stop_y = 180
    stop = True

    d_y = 5
    d_x = 5

    pick = False
    place = False

    status = "approach"
    target_color = 'box'
    broadcast_status = 'find_target'
    count_stop = 0
    count_turn = 0

    linear_pid.clear()
    angular_pid.clear()
    start_pick = True

    return TriggerResponse(success=True)

def start_place_callback(msg):
    global start_pick, yaw_angle, pick_stop_x, pick_stop_y, stop, status, target_color, broadcast_status
    global linear_speed, angular_speed
    global d_x, d_y
    global pick, place
    global count_turn, count_stop

    rospy.loginfo("start place_1")
    rospy.set_param('~status', 'place')

    bus_servo_control.set_servos(joints_pub, 2, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 650)))
    rospy.sleep(2)
    linear_speed = 0
    angular_speed = 0
    yaw_angle = 90

    pick_stop_x = 360 
    pick_stop_y = 240
    stop = True

    d_y = 5
    d_x = 5

    pick = False
    place = False

    status = "approach"
    target_color = 'orange'
    broadcast_status = 'find_target'
    count_stop = 0
    count_turn = 0

    linear_pid.clear()
    angular_pid.clear()
    start_pick = True

    return TriggerResponse(success=True)

def start_place_2_callback(msg):
    global start_pick, yaw_angle, pick_stop_x, pick_stop_y, stop, status, target_color, broadcast_status
    global linear_speed, angular_speed
    global d_x, d_y
    global pick, place
    global count_turn, count_stop
    bus_servo_control.set_servos(joints_pub, 2, ((1, 500), (2, 720), (3, 100), (4, 100), (5, 500), (10, 600)))
    rospy.sleep(2)

    rospy.loginfo("start place_2")
    rospy.set_param('~status', 'place')

    linear_speed = 0
    angular_speed = 0
    yaw_angle = 90

    pick_stop_x = 320
    pick_stop_y = 200
    stop = True

    d_y = 5
    d_x = 5

    pick = False
    place = False

    status = "approach"
    target_color = 'orange'
    broadcast_status = 'find_target'
    count_stop = 0
    count_turn = 0

    linear_pid.clear()
    angular_pid.clear()
    start_pick = True

    return TriggerResponse(success=True)

def start_place_3_callback(msg):
    global start_pick, yaw_angle, pick_stop_x, pick_stop_y, stop, status, target_color, broadcast_status
    global linear_speed, angular_speed
    global d_x, d_y
    global pick, place
    global count_turn, count_stop

    rospy.loginfo("start place_3")
    bus_servo_control.set_servos(joints_pub, 2, ((1, 500), (2, 210), (3, 320), (4, 350), (5, 500), (10, 650)))
    rospy.sleep(2)
    bus_servo_control.set_servos(joints_pub, 0.5, ((10, 200),))
    rospy.sleep(0.5)
    bus_servo_control.set_servos(joints_pub, 2, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 200)))
    rospy.sleep(2)

    return TriggerResponse(success=True)

# 颜色识别
size = (320, 240)

def colorDetect(img):
    global target_color,calibration
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
    frame_mask = cv2.inRange(frame_lab, tuple(lab_data['lab']['gemini_camera'][target_color]['min']),
                             tuple(lab_data['lab']['gemini_camera'][target_color]['max']))  # 对原图像和掩模进行位运算

    eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀

    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓

    center_x, center_y, angle = -1, -1, -1
    if len(contours) != 0:
        areaMaxContour, area_max = common.get_area_max_contour(contours, 10)  # 找出最大轮廓
        if areaMaxContour is not None:
            if 10 < area_max:  # 有找到最大面积
                rect = cv2.minAreaRect(areaMaxContour)  # 最小外接矩形
                angle = rect[2]
                box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
                for j in range(4):
                    box[j, 0] = int(misc.val_map(box[j, 0], 0, size[0], 0, img_w))
                    box[j, 1] = int(misc.val_map(box[j, 1], 0, size[1], 0, img_h))

                cv2.drawContours(img, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形
                # 获取矩形的对角点
                ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                radius = abs(ptime_start_x - pt3_x)
                center_x, center_y = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2)  # 中心点
                cv2.circle(img, (center_x, center_y), 5, (0, 255, 255), -1)  # 画出中心点
    if calibration:
        print("坐标为:")
        return center_x, center_y, angle, img
    else:
        return center_x, center_y, angle


count = 0
def pick_handle(image):
    global pick, count_turn, count_stop, angular_speed, linear_speed, status, d_x, d_y, broadcast_status, count, pick_stop_y, pick_stop_x, debug, target_color

    img_center_x = image.shape[:2][1] / 2  # 获取缩小图像的宽度值的一半, 即图像中心
    img_center_y = image.shape[:2][0] / 2

    twist = Twist()
    if not pick or debug:
        object_center_x, object_center_y, object_angle = colorDetect(image)  # 获取物体颜色的中心和角度
        if debug:
            count += 1
            if count > 10:
                count = 0
                pick_stop_y = object_center_y
                pick_stop_x = object_center_x
                config = common.get_yaml_data(config_path)
                config['pick_stop_pixel_coordinate'] = [pick_stop_x, pick_stop_y]
                common.save_yaml_data(config, config_path)
                debug = False
            print(object_center_x, object_center_y)  # 打印当前物体中心的像素
        elif object_center_x > 0:
            if broadcast and broadcast_status == 'find_target':
                broadcast_status = 'crawl_succeeded'
                voice_play.play('find_target', language=language)
            ########电机pid处理#########
            # 以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入#
            linear_pid.SetPoint = pick_stop_y
            if abs(object_center_y - pick_stop_y) <= d_y:
                object_center_y = pick_stop_y
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

            angular_pid.SetPoint = pick_stop_x
            if abs(object_center_x - pick_stop_x) <= d_x:
                object_center_x = pick_stop_x
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
            print(linear_speed, angular_speed)
            if abs(linear_speed) == 0 and abs(angular_speed) == 0 and target_color != 'box':
                pick = True 
                count_stop = 0
                pick = True
                rospy.set_param('~status', 'stop')
                start_pick = False

            # else:
                # twist.linear.x = linear_speed
                # twist.angular.z = angular_speed
            # print(target_color)
            elif abs(linear_speed) == 0 and abs(angular_speed) == 0 and target_color == 'box':
                # print(target_color)
                if machine_type == 'ROSLander':
                    # print(target_color)
                    count_turn += 1
                    print("count_stop",count_stop)
                    print("count_turn",count_turn)
                    if count_turn > 5:
                        count_turn = 5
                        status = "align"
                        # print(count_stop)
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
                            d_x = 5
                            d_y = 5
                            count_stop += 1
                            status = "adjust"
                        else:
                            count_stop = 0
                            pick = True
                            rospy.set_param('~status', 'stop')
                            start_pick = False
                # else:
                    # print(count_stop)
                    # if count_stop > 15:
                        # count_stop = 0
                        # pick = True
                        # rospy.set_param('~status', 'stop')
                        # start_pick = False
            else:
                if count_stop >= 10:
                    count_stop = 10
                count_turn = 0
                if status != 'align':
                    twist.linear.x = linear_speed
                    twist.angular.z = angular_speed

        mecnum_pub.publish(twist)

    return image

def image_callback(ros_image):
    global place, stop,image_sub,count 
    global target_color,calibration

    rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                           buffer=ros_image.data)  # 将自定义图像消息转化为图像
    if start_pick:
        stop = True
        result_image = pick_handle(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
    else:
        rospy.sleep(0.1)
        if stop:
            stop = False
    if close:
        rospy.signal_shutdown('shutdown')
    if calibration:
        target_color = "box"
        center_x, center_y, angle, result_image = colorDetect(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
        cv2.imshow("RGB", result_image)
        key = cv2.waitKey(1)
        print("x:",center_x, "y:",center_y, "angle:",angle)

        if center_x != -1:
            count += 1
            if count > 100:
                print('正在保存参数,pick_stop_pixel_coordinate')
                config = common.get_yaml_data(config_path)
                config['pick_stop_pixel_coordinate'][0] = center_x
                config['pick_stop_pixel_coordinate'][1] = center_y
                common.save_yaml_data(config, config_path)
                rospy.sleep(2)
                print('保存完毕,ramp_calibration[0],ramp_flat')
                count = 0
                image_sub.unregister()
        else:
            count = 0

def calibration_callback(msg):
    global calibration
    calibration = True
    bus_servo_control.set_servos(joints_pub, 2, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 200)))
    rospy.sleep(2)
    return TriggerResponse(success=True)

def start_callback(msg):
    global image_sub,image_pub 
    # image_pub = rospy.Publisher('~image_result', Image, queue_size=1)
    # image_sub = rospy.Subscriber("/robot_1/gemini_camera/rgb/image_raw", Image, image_callback)
    image_sub = rospy.Subscriber("/gemini_camera/rgb/image_raw", Image, image_callback)
    return TriggerResponse(success=True)

def stop_callback(msg):
    global image_sub 
    image_sub.unregister()
    return TriggerResponse(success=True)

def colse_callback(msg):
    global close
    close = True
    return TriggerResponse(success=True)

if __name__ == '__main__':
    rospy.init_node('position_correction', anonymous=True)
    
    rospy.set_param('~status', 'stop')
    broadcast = rospy.get_param('~broadcast', False)
    calibration = rospy.get_param('~calibration', False)
    language = os.environ['ASR_LANGUAGE']
    machine_type = os.environ['MACHINE_TYPE']

    # wait_yolo_status()

    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    # joints_pub = rospy.Publisher('/robot_1/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    mecnum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
    # mecnum_pub = rospy.Publisher('/robot_1/controller/cmd_vel', Twist, queue_size=1)
    # image_pub = rospy.Publisher('~image_result', Image, queue_size=1)

    rospy.Service('~start', Trigger, start_callback)
    rospy.Service('~stop', Trigger, stop_callback)
    rospy.Service('~colse', Trigger, colse_callback)
    rospy.Service('~calibration', Trigger, calibration_callback)
    rospy.Service('~pick_1', Trigger, start_pick_callback)
    rospy.Service('~pick_2', Trigger, start_pick_2_callback)
    rospy.Service('~pick_3', Trigger, start_pick_3_callback)
    rospy.Service('~place_1', Trigger, start_place_callback)
    rospy.Service('~place_2', Trigger, start_place_2_callback)
    rospy.Service('~place_3', Trigger, start_place_3_callback)
    debug = rospy.get_param('~debug', False)
    rospy.sleep(1)
    # while not rospy.is_shutdown():
        # try:
            # if rospy.get_param('/robot_1/servo_manager/init_finish') and rospy.get_param(
                    # '/robot_1/joint_states_publisher/init_finish'):
                # break
        # except:
            # rospy.sleep(0.1)
    while not rospy.is_shutdown():
        try:
            if rospy.get_param('/servo_manager/init_finish') and rospy.get_param(
                    '/joint_states_publisher/init_finish'):
                break
        except:
            rospy.sleep(0.1)
    rospy.sleep(2)
    mecnum_pub.publish(Twist())
    rospy.set_param('~init_finish', True)
    try:
        rospy.spin()
    except exception as e:
        mecnum_pub.publish(twist())
        rospy.logerr(str(e))
