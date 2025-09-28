#!/usr/bin/env python3
# encoding: utf-8
from sklearn.linear_model import LinearRegression #此必须放置于第一行
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
from sensor_msgs.msg import Image ,LaserScan
import sdk.common as common
from geometry_msgs.msg import Twist
from xf_mic_asr_offline import voice_play
from interfaces.msg import Pose2D
from interfaces.srv import SetPose2D
from std_srvs.srv import SetBool,Trigger, TriggerResponse
from servo_msgs.msg import MultiRawIdPosDur
from servo_controllers import bus_servo_control
config_path = os.path.join(os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../../..')), 'config/config.yaml')

debug = False 
start_pick = False 
start_place = False
target_color = ""
linear_base_speed = 0.007
angular_base_speed = 0.005
image_sub = None
yaw_pid = PID(P=0.015, I=0, D=0.000)
linear_pid = PID(P=0.001, I=0, D=0)
# y_linear_pid = PID(P=0.0028, I=0, D=0)
# linear_pid = PID(P=0.0056, I=0, D=0)
angular_pid = PID(P=0.0015, I=0, D=0)
# angular_pid = PID(P=0.006, I=0, D=0)

#由于相机是斜着看地面的，使用线性回归校准数据成平面
line_compensation = LinearRegression()
#此处参数皆为测试所得
ramp_flat = rospy.get_param('/ramp_flat')

line_compensation.fit([[10],[200],[390]],[[ramp_flat[0]],[1],[ramp_flat[1]]])

line_depth_compensation = []
transition_depth_image = np.zeros((400, 640), dtype=float)
# 由于相机只是再y轴翻转，使用这里只需要校准y轴 
for i in range(399):
    line_depth_compensation.append(line_compensation.predict([[i]]))

linear_speed = 0
angular_speed = 0
yaw_angle = 0
y_linear_speed = 0
pick_stop_x = 320
pick_stop_y = 388
place_stop_x = 320
place_stop_y = 388
stop = True
line = [200,250]
x = [40,600]
d_y = 10
d_x = 10
dis = 0

pick = False
place = False

broadcast_status = ''
status = "approach"
count_stop = 0
count_turn = 0
calibration_flat = False
calibration_ramp = False
lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
#开始上坡
def start_up_callback(msg):
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

    ramp_up_pixel_coordinate = rospy.get_param('/ramp_up_pixel_coordinate')
    pick_stop_x = ramp_up_pixel_coordinate[0]
    pick_stop_y = ramp_up_pixel_coordinate[1]
    stop = True

    d_y = 10
    d_x = 10

    pick = False
    place = False

    status = "approach"
    target_color = 'red'
    broadcast_status = 'find_target'
    count_stop = 0
    count_turn = 0

    linear_pid.clear()
    angular_pid.clear()
    start_pick = True

    return TriggerResponse(success=True)

#开始下坡
def start_down_callback(msg):
    global start_pick, yaw_angle, pick_stop_x, pick_stop_y, stop, status, target_color, broadcast_status
    global linear_speed, angular_speed
    global d_x, d_y
    global pick, place
    global count_turn, count_stop

    rospy.loginfo("start pick")
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
    target_color = 'blue'
    broadcast_status = 'find_target'
    count_stop = 0
    count_turn = 0

    linear_pid.clear()
    angular_pid.clear()
    start_pick = True

    return TriggerResponse(success=True)

# 坡道检测，需要输入深度图
size = (640,400)
def depth_Detect(img):
    global line , x, calibration_ramp,calibration_config,count,image_sub
    angle = None
    center_x, center_y = 0,0
    img_h, img_w = img.shape[:2]
    depth_image = img
    # 若得到值小于calibration_config[0]则为0，识别到坡道部分
    print("calibration_config: ",calibration_config)
    depth_image = np.where(depth_image > calibration_config[0] , 0, depth_image)
    # 高斯
    depth_image = cv2.GaussianBlur(depth_image, (3, 3), 0)
    # 深度最大为2000，最小为0，并归一化，减少坡道之间的差
    depth_image = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
    if calibration_ramp :
        print("calibration_config: ",calibration_config)
        print("depth_200: ",depth_image[200][320])
        print("depth_180: ",depth_image[180][320])
        if int(depth_image[200][320]) != 0 and int(depth_image[180][320]) != 0 :
            count += 1
            if count > 20 :
                print('正在保存参数,ramp_calibration[1],ramp_calibration[2]')
                config = common.get_yaml_data(config_path)
                config['ramp_calibration'][1] = int(depth_image[200][320]) 
                config['ramp_calibration'][2] = int(depth_image[180][320]) 
                common.save_yaml_data(config, config_path)
                rospy.sleep(2)
                print('保存完毕,ramp_calibration[0],ramp_flat')
                count = 0
                image_sub.unregister()
        else:
            count = 0
    # 深度小于七十的就保存，大于70就设0，仅仅识别坡道头部
    depth_image = np.where(depth_image > calibration_config[1] , 0, depth_image)
    depth_image = np.where(depth_image < calibration_config[2] , 0, depth_image)
    result_image = depth_image
    result_image = cv2.applyColorMap(img.astype(np.uint8), cv2.COLORMAP_HOT)
    depth_gray = depth_image.astype(np.uint8)
    # 二值化
    _,depth_bit = cv2.threshold(depth_gray, 0, 1000, cv2.THRESH_BINARY)
    #腐蚀
    depth_bit = cv2.erode(depth_bit, np.ones((9, 9), np.uint8))
    #膨胀
    depth_bit = cv2.dilate(depth_bit, np.ones((3, 3), np.uint8))
    #得到轮廓
    contours = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    center_x, center_y, angle = -1, -1, -1
    if len(contours) != 0:
        #得到最大轮廓
        areaMaxContour, area_max = common.get_area_max_contour(contours, 10)  # 找出最大轮廓
        # print(area_max)
        if calibration_ramp :
            print(area_max)
        if areaMaxContour is not None :
            if 1000 < area_max < 15000 :  # 有找到最大面积
                rect = cv2.minAreaRect(areaMaxContour)  # 最小外接矩形
                angle = rect[2]
                box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
                for j in range(4):
                    box[j, 0] = int(misc.val_map(box[j, 0], 0, size[0], 0, img_w))
                    box[j, 1] = int(misc.val_map(box[j, 1], 0, size[1], 0, img_h))
                # print(box) 
                # cv2.drawContours(result_image, [box], -1, (0, 255, 255), 4)  # 画出四个点组成的矩形
                # 获取矩形的对角点
                ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                radius = abs(ptime_start_x - pt3_x)
                center_x, center_y = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2)  # 中心点
                print(depth_image[center_y][center_x],depth_image[center_y-2][center_x],depth_image[center_y-30][center_x])
                #根据坡道特性过滤其他识别到的物体
                if abs(depth_image[center_y][center_x] - depth_image[center_y-2][center_x]) < 2 and  depth_image[center_y][center_x] - depth_image[center_y][center_x-2] < 2 and abs(depth_image[center_y][center_x] - depth_image[center_y-30][center_x]) > 30 :
                    cv2.drawContours(result_image, [box], -1, (0, 255, 255), 4)  # 画出四个点组成的矩形
                    cv2.circle(result_image, (center_x, center_y), 5, (0, 255, 255), -1)  # 画出中心点
                else:
                    center_x, center_y, angle = -1, -1, -1

    print(center_x, center_y, angle)
    # return result_image,depth_bit,center_x, center_y, angle
    return result_image,center_x, center_y, angle
    
def lidar_callback(lidar_msg):
    global lidar_data
    lidar_data = lidar_msg.ranges.copy()
    rospy.loginfo("Forward: %f",lidar_data[180])

def image_callback(ros_image):
    global place, stop,line_compensation,calibration_flat,calibration_ramp,transition_depth_image,count,dis

    depth_image = np.ndarray(shape=(ros_image.height, ros_image.width), dtype=np.uint16,
                             buffer=ros_image.data)  # 将自定义图像消息转化为图像
    depth_image = depth_image.copy()
    # print(transition_depth_image.shape)
    #补偿深度
    # for i in range(399):
        # transition_depth_image[i] = depth_image[i]*line_depth_compensation[i]
    if start_pick:
        #补偿深度
        for i in range(399):
            transition_depth_image[i] = depth_image[i]*line_depth_compensation[i]
            # transition_depth_image.append(depth_image[i]*line_depth_compensation[i])
        stop = True
        result_image = ramp_align(transition_depth_image)
    elif calibration_flat:
        print("data_10: ",depth_image[10][320])
        print("data_200: ",depth_image[200][320])
        print("data_390: ",depth_image[390][320])
        print("data_200 / data_10: ",depth_image[200][320]/depth_image[10][320])
        print("data_200 / data_300: ",depth_image[200][320]/depth_image[390][320])

        calibration_compensation = LinearRegression()
        calibration_10 = float(depth_image[200][320]/depth_image[10][320])
        calibration_390 = float(depth_image[200][320]/depth_image[390][320])
        calibration_compensation.fit([[10],[200],[390]],[[depth_image[200][320]/depth_image[10][320]],[1],[depth_image[200][320]/depth_image[390][320]]])
        calibration_depth_compensation = []
        # 由于相机只是再y轴翻转，使用这里只需要校准y轴 
        for i in range(399):
            calibration_depth_compensation.append(calibration_compensation.predict([[i]]))
        for j in range(399):
            transition_depth_image[j] = depth_image[j]*calibration_depth_compensation[j]

        print("transition_data_10: ",transition_depth_image[10][320])
        print("transition_data_200: ",transition_depth_image[200][320])
        print("transition_data_390: ",transition_depth_image[390][320])
        print("target_dis: ",transition_depth_image[200][320]-10)
        # print(abs(dis - int(transition_depth_image[200][320]-20))) 
        if abs(dis - int(transition_depth_image[200][320]-20)) < 2:
            count += 1
            
            if count > 5:
                print('正在保存参数,ramp_calibration[0],ramp_flat')
                config = common.get_yaml_data(config_path)
                config['ramp_calibration'][0] = int(transition_depth_image[200][320]-10) 
                config['ramp_flat'][0] = calibration_10
                config['ramp_flat'][1] = calibration_390
                common.save_yaml_data(config, config_path)
                rospy.sleep(2)
                print('保存完毕,ramp_calibration[0],ramp_flat')
                count = 0
                image_sub.unregister()
        else:
            count = 0
            dis = int(transition_depth_image[200][320]-20)


       
    elif calibration_ramp:
        for i in range(399):
            transition_depth_image[i] = depth_image[i]*line_depth_compensation[i]
        result_image,object_center_x, object_center_y, object_angle = depth_Detect(transition_depth_image)  # 获取物体颜色的中心和角度
        cv2.line(result_image,(200,200),(480,200),(0,255,0),3,cv2.LINE_4)
        cv2.circle(result_image, (320, 200), 5, (255, 0, 0), -1)  # 画出中心点
        cv2.line(result_image,(200,180),(480,180),(0,255,0),3,cv2.LINE_4)
        cv2.circle(result_image, (320, 180), 5, (255, 0, 0), -1)  # 画出中心点
        cv2.imshow("depth", result_image)
        key = cv2.waitKey(1)

    else:
        rospy.sleep(0.1)
        if stop:
            stop = False
            mecnum_pub.publish(Twist())

count = 0
def ramp_align(image):
    global pick, count_turn, count_stop, angular_speed, linear_speed, status, d_x, d_y, broadcast_status, count, pick_stop_y, pick_stop_x, debug,y_linear_speed,start_pick

    twist = Twist()
    if not pick or debug:
        result_image,object_center_x, object_center_y, object_angle = depth_Detect(image)  # 获取物体颜色的中心和角度

        if object_center_x > 0:
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
            # print(object_center_x-pick_stop_x)
            if abs(object_center_x - pick_stop_x) <= d_x:
                object_center_x = pick_stop_x
            if status != "align":
                angular_pid.update(object_center_x)  # 更新pid
                tmp = angular_base_speed + angular_pid.output

                angular_speed = tmp
                # print(tmp)
                if tmp > 1.2:
                    angular_speed = 1.2
                if tmp < -1.2:
                    angular_speed = -1.2
                if abs(tmp) <= 0.038:
                    angular_speed = 0
            print("speed:",linear_speed,angular_speed)
            if abs(linear_speed) == 0 and abs(angular_speed) == 0:
                count_turn += 1
                if count_turn > 5:
                    count_turn = 5
                    status = "align"
                    print("count_stop",count_stop)
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

            else:
                if count_stop >= 10:
                    count_stop = 10
                count_turn = 0
                if status != 'align':
                    twist.linear.x = linear_speed
                    twist.angular.z = angular_speed


        mecnum_pub.publish(twist)
    else:
        result_image,object_center_x, object_center_y, object_angle = depth_Detect(image)  # 获取物体颜色的中心和角度
    return result_image

def calibration_flat_callback(msg):
    global calibration_flat 
    calibration_flat = True
    bus_servo_control.set_servos(joints_pub, 2, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 200)))
    rospy.sleep(2)
    return TriggerResponse(success=True)
def calibration_ramp_callback(msg):
    global calibration_ramp 
    calibration_ramp = True 
    bus_servo_control.set_servos(joints_pub, 2, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 200)))
    rospy.sleep(2)
    return TriggerResponse(success=True)
#启动坡道检测
def start_callback(msg):
    #yct add:这里添加了雷达订阅，为了不通过看破面直接上坡
    global image_sub 
    global lidar_sub
    image_sub = rospy.Subscriber("/gemini_camera/depth/image_raw", Image, image_callback)
    #lidar_sub = rospy.Subscriber("/scan",LaserScan,lidar_callback)
    # image_sub = rospy.Subscriber("/robot_1/gemini_camera/depth/image_raw", Image, image_callback)
    rospy.sleep(2)
    return TriggerResponse(success=True)
# 关闭坡道检测
def stop_callback(msg):
    global image_sub 
    image_sub.unregister()
    return TriggerResponse(success=True)

if __name__ == '__main__':
    rospy.init_node('ramp', anonymous=True)
    
    rospy.set_param('~status', 'stop')
    broadcast = rospy.get_param('~broadcast', False)
    calibration_config = rospy.get_param('/ramp_calibration')
    language = os.environ['ASR_LANGUAGE']
    machine_type = os.environ['MACHINE_TYPE']
    # 舵机控制
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    # joints_pub = rospy.Publisher('/robot_1/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    # 电机控制
    mecnum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
    # mecnum_pub = rospy.Publisher('/robot_1/controller/cmd_vel', Twist, queue_size=1)
    # 回传画面发布
    image_pub = rospy.Publisher('~image_result',Image, queue_size=1)
    
    rospy.Service('~calibration_flat', Trigger, calibration_flat_callback) #上坡
    rospy.Service('~calibration_ramp', Trigger, calibration_ramp_callback) #上坡
    rospy.Service('~up', Trigger, start_up_callback) #上坡
    rospy.Service('~down', Trigger, start_down_callback) #下坡
    rospy.Service('~start', Trigger, start_callback) #启动坡道检测
    rospy.Service('~stop', Trigger, stop_callback) #关闭坡道检测
    # 检测LDP服务
    rospy.wait_for_service('/gemini_camera/set_ldp')
    # rospy.wait_for_service('/robot_1/gemini_camera/set_ldp')
    rospy.ServiceProxy('/gemini_camera/set_ldp', SetBool)(False)
    # rospy.ServiceProxy('/robot_1/gemini_camera/set_ldp', SetBool)(False)
    rospy.sleep(2)
    while not rospy.is_shutdown():
        try:
            if rospy.get_param('/servo_manager/init_finish') and rospy.get_param(
                    '/joint_states_publisher/init_finish'):
                break
        except:
            rospy.sleep(0.1)
   
    rospy.set_param('~init_finish', True)
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
