#!/usr/bin/env python3
# encoding: utf-8
# 语音控制导航
import os
import json
import rospy
import signal
import math
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger, TriggerResponse, Empty
from xf_mic_asr_offline import voice_play
from geometry_msgs.msg import Twist, PoseStamped, Pose
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from ros_robot_controller.msg import BuzzerState
from servo_controllers import bus_servo_control
from servo_msgs.msg import MultiRawIdPosDur

import subprocess
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header

#yct add:这是一个杀死其他node的函数，node_name就是节点名字的字符串
def terminate_ros_node(node_name):
    command = ["rosnode", "kill", node_name]
    try:
        subprocess.run(command, check=True)
        print(f"Successfully killed node: {node_name}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to kill node {node_name}: {e}")

def find_pid(pattern):
    #"""根据程序名或命令行参数查找PID"""
    try:
        output = os.popen(f"pgrep -f '{pattern}'").read()
        pids = output.strip().split('\n')
        return pids
    except Exception as e:
        print(f"Error finding PID: {e}")
        return []

def terminate_process(pid):
    #"""终止指定PID的进程"""
    try:
        os.system(f"kill -9 {pid}")
        print(f"Process with PID {pid} terminated.")
    except Exception as e:
        print(f"Error terminating process: {e}")

def terminate_ros_pid(target_script_name):
    pids = find_pid(target_script_name)
    if pids:
        print(f"Found {len(pids)} matching processes:")
        for pid in pids:
            print(f"Terminating process with PID: {pid}")
            terminate_process(pid)
    else:
        print("No matching processes found.")

#将rpy转换成qua
def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation

class VoiceControlNavNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.words = None #语音识别到的内容
        self.running = True # 循环检测任务开关 
        self.move_base_status = 1 # 导航状态 1 是还在运动中，3是导航完毕
        self.pick_location_time = rospy.get_param('/pick_location_time', 3) # map节点名
        self.up_ramp_time = rospy.get_param('/up_ramp_time', 3.5) # map节点名

        #yct add:添加对清除代价地图的操作
        rospy.wait_for_service('/move_base/clear_costmaps')
        
        rospy.Service('~pick', Trigger, self.start_pick_callback) # 夹取测试
        rospy.Service('~place', Trigger, self.start_place_callback) # 放置测试
        rospy.Service('~detect', Trigger, self.start_detect_callback) # 检测测试
        rospy.Service('~back', Trigger, self.start_back_callback) # 回到起始点测试
        rospy.Service('~test', Trigger, self.test_callback) # 不通过语音识别启动
        rospy.Service('~aligning', Trigger, self.start_aligning_callback) # 不通过语音识别启动
        
        rospy.set_param('~target_shape', 'None') # 设置目标形状
        rospy.set_param('~status', 'start') # 设置状态
        
        self.language = os.environ['ASR_LANGUAGE'] # 读取语言
        self.costmap = '/move_base/local_costmap/costmap' # costmap节点名
        # self.costmap = '/robot_1/move_base/local_costmap/costmap' # costmap节点名
        self.map_frame = rospy.get_param('~map_frame', '/map') # map节点名
        # self.map_frame = rospy.get_param('~map_frame', '/robot_1/map') # map节点名
        # 麦轮运动控制节点
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
        # self.mecanum_pub = rospy.Publisher('/robot_1/controller/cmd_vel', Twist, queue_size=1)
        # 舵机控制
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        # self.joints_pub = rospy.Publisher('/robot_1/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        #yct add:位置控制，配合pose_pub(alpha,x_pos,y_pos)函数设置位置
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        # 等待舵机节点开启
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
        # 初始状态 
        # bus_servo_control.set_servos(self.joints_pub, 2, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 650)))
        bus_servo_control.set_servos(self.joints_pub, 2, ((1, 500), (2, 760), (3, 15), (4, 150), (5, 500), (10, 200)))
        # set_servos(self.servos_pub, 1, ((1, 500), (2, 720), (3, 100), (4, 150), (5, 500), (10, 650)))
        rospy.sleep(2)
        # 导航点发布
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        # self.goal_pub = rospy.Publisher('/robot_1/move_base_simple/goal', PoseStamped, queue_size=1)
        # 订阅路径规划返回话题
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_callback)
        # rospy.Subscriber('/robot_1/move_base/result', MoveBaseActionResult, self.move_callback)
        # 订阅语音识别节点
        self.vc_sub = rospy.Subscriber('/voice_control/voice_words', String, self.words_callback)
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        # 等待语音识别节点启动
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/voice_control/init_finish'):
                    break
            except:
                rospy.sleep(0.1)

        rospy.loginfo('唤醒口令: 唐小天')
        rospy.loginfo('唤醒后15秒内可以不用再唤醒')
        rospy.loginfo('控制指令: 开始进行任务')
        # 等待导航启动
        rospy.wait_for_message(self.costmap, OccupancyGrid)
        self.play('running')
        signal.signal(signal.SIGINT, self.shutdown)
        #开始任务
        self.clear_costmap()
        self.run()

    #不通过语音进行识别
    def test_callback(self,msg):
        self.words = '开始进行任务'
        return TriggerResponse(success=True)
    #夹取测试
    def start_pick_callback(self,msg):
        rospy.ServiceProxy('/shape_recognition/start', Trigger)() #启动夹取节点
        rospy.ServiceProxy('/shape_recognition/pick', Trigger)()  #开始夹取
        rospy.sleep(1)
        self.wait_pick_status()# 等待夹取完毕
        self.play('7')
        rospy.ServiceProxy('/shape_recognition/stop', Trigger)() # 暂停夹取

        return TriggerResponse(success=True)
    #放置测试
    def start_place_callback(self,msg):
        self.control(0,0,1,"place") #导航并进行放置
        return TriggerResponse(success=True)

    def start_detect_callback(self,msg):
        self.control(0,0,1,"detect") #导航并进行检测
        return TriggerResponse(success=True)

    def start_aligning_callback(self,msg):
        rospy.ServiceProxy('/position_correction/start', Trigger)() #启动对齐
        # self.play('6')
        rospy.ServiceProxy('/position_correction/' + 'pick' + "_1", Trigger)() # 夹取对齐
        rospy.sleep(1)
        self.wait_correction_status()
        rospy.ServiceProxy('/position_correction/' + 'pick' + "_2", Trigger)() # 夹取动作
        rospy.sleep(1)
        self.wait_correction_status()
        twist = Twist()
        twist.linear.x = 0.05
        self.mecanum_pub.publish(twist)
        rospy.sleep(float(self.pick_location_time[0]))
        twist = Twist()
        self.mecanum_pub.publish(twist)
        return TriggerResponse(success=True)

    def start_back_callback(self,msg):
        self.control(0,0,1,"back") # 导航并进行回到起始点
        #rospy.ServiceProxy('/ramp/start', Trigger)() #启动坡道对齐
        #rospy.ServiceProxy('/ramp/up', Trigger)() #开始进行坡道对齐
        rospy.sleep(1)
        self.wait_ramp_status()
        twist = Twist()
        twist.linear.y = -0.1
        self.mecanum_pub.publish(twist)
        rospy.sleep(0.6)
        twist = Twist()
        twist.angular.z = -0.5
        self.mecanum_pub.publish(twist)
        rospy.sleep(0.2)
        twist = Twist()
        twist.linear.x = 0.3
        self.mecanum_pub.publish(twist)
        rospy.sleep(float(self.up_ramp_time[0]))
        twist = Twist()
        twist.angular.z = 0.5
        self.mecanum_pub.publish(twist)
        rospy.sleep(6)
        self.mecanum_pub.publish(Twist())
        return TriggerResponse(success=True)

    def play(self, name):
        voice_play.play(name, language=self.language) #进行语音播报

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')
        rospy.signal_shutdown('shutdown')
    #进行语音识别
    def words_callback(self, msg):
        self.words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            self.words = self.words.replace(' ', '')
        print('words:', self.words)

        if self.words is not None and self.words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)',
                                                         '失败10次(Fail-10-times']:
            pass
        elif self.words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif self.words == '休眠(Sleep)':
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)
    #进行导航状态检测
    def move_callback(self, msg):
        print(msg)
        try:
            if msg.status.status == 3:
                self.move_base_status = msg.status.status
            else : 
                self.move_base_status = 1
        except:
            self.move_base_status = 1
    #设置的导航点
    def nav_position(self,x,y,w):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = rpy2qua(math.radians(0),math.radians(0),math.radians(w))
        self.goal_pub.publish(pose)
    #yct add:人工设置位置，配合清除代价地图，实现重新定位，来规避雷达漂移
    #使用self.pose_pub(alpha,x_pos,y_pos)来进行位置定位，alpha是弧度制单位！
    #夹完1：self.pose_pub(0,1.643,-3.04) #alpha,x_pos,y_pos
    #夹完2：self.pose_pub(3.1416,0.1697,-3.17) #alpha,x_pos,y_pos
    def pose_pub(self,alpha,x_pos,y_pos):
        rospy.loginfo("[yct]正在修改当前位置...")
        pose_msg=PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.pose.position.x = x_pos
        pose_msg.pose.pose.position.y = y_pos
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[6 * 1 + 1] = 0.25
        pose_msg.pose.covariance[6 * 5 + 5] = 0.06853892326654787
        pose_msg.pose.pose.orientation.z = math.sin(alpha/2)
        pose_msg.pose.pose.orientation.w = math.cos(alpha/2)
        self.initial_pose_pub.publish(pose_msg)
        rospy.sleep(0.5)
        self.initial_pose_pub.publish(pose_msg)
    #yct add:清除代价地图
    #直接使用内置函数，不用os.system(f"rosservice call /move_base/clear_costmaps")
    def clear_costmap(self):
        try:
            #等待服务提供
            clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_costmaps()
            rospy.loginfo("[yct]清除代价地图成功")
        except rospy.ServiceException as e:
            rospy.logerr("[yct]清除代价地图失败！原因: %s" % e)
    #等到导航状态
    def wait_nav_status(self):
        while True:
            # print(self.move_base_status)
            if self.move_base_status == 3 :
                self.move_base_status = 1
                break
            else:
                rospy.sleep(2)
    #等待对齐状态
    def wait_correction_status(self):
        while True:
            correction_status = rospy.get_param('/position_correction/status',"stop")
            print('correction_status:',correction_status)
            if correction_status == "stop":
                break
            else:
                rospy.sleep(2)
    #等待上下坡状态
    def wait_ramp_status(self):
        while True:
            correction_status = rospy.get_param('/ramp/status',"stop")
            print('ramp:',correction_status)
            if correction_status == "stop":
                break
            else:
                rospy.sleep(2)
    #等到夹取状态
    def wait_pick_status(self):
        while True:
            pick_status = rospy.get_param('/shape_recognition/status',"start")
            print('grab_status:',pick_status)
            if pick_status == "stop":
                rospy.set_param('/shape_recognition/status',"start")
                break
            else:
                rospy.sleep(2)
    #等待识别形状
    def wait_yolo_status(self):
        self.count = 0                 #zcl add
        while True:
            shape = rospy.get_param('/yolov5/shape','None')
            print('yolo_shape:',shape)
            if shape != "None":
                break
            else:
                rospy.sleep(0.5)
                self.count += 1        #zcl add
                if self.count >= 10:   #zcl add
                    shape = "cube"     #zcl add
    #总控制，x，y 为坐标
    #          x
    #       y    -y
    #         -x 
    # w为角度
    #        
    #     左：w    右：-w
    # set_status 为控制小车任务 pick、place、detect、back
    def control(self,x,y,w,set_status):
        #在detect任务下不导航
        if set_status != 'detect':
            self.nav_position(x,y,w) #导航
            rospy.sleep(2)
            self.wait_nav_status() #等待导航
        if set_status == 'pick' : #夹取
            twist = Twist()
            twist.linear.x = 0.2
            self.mecanum_pub.publish(twist)
            rospy.sleep(2)
            self.mecanum_pub.publish(Twist())
            rospy.ServiceProxy('/position_correction/start', Trigger)() #启动对齐
            #self.play('6')
            rospy.ServiceProxy('/position_correction/' + set_status + "_1", Trigger)() # 夹取对齐
            rospy.sleep(1)
            self.wait_correction_status()
            rospy.ServiceProxy('/position_correction/' + set_status + "_2", Trigger)() # 夹取动作
            rospy.sleep(1)
            self.wait_correction_status()
            twist = Twist()
            twist.linear.x = 0.05
            self.mecanum_pub.publish(twist)
            rospy.sleep(float(self.pick_location_time[0]))
            twist = Twist()
            self.mecanum_pub.publish(twist)

            rospy.ServiceProxy('/position_correction/stop', Trigger)() # 停止对齐
            rospy.ServiceProxy('/shape_recognition/start', Trigger)() # 启动夹取话题
            rospy.ServiceProxy('/shape_recognition/' + set_status, Trigger)() #开始夹取
            rospy.sleep(1)
            self.wait_pick_status()
            self.play('7')
            rospy.ServiceProxy('/shape_recognition/stop', Trigger)() #关闭夹取话题
            #开环向后跑的部分直接丢到run函数中了
            
        elif set_status == "place" : #放置
            # rospy.ServiceProxy('/position_correction/start', Trigger)()
            # self.play('8')
            # rospy.ServiceProxy('/position_correction/' + set_status + "_1" , Trigger)()
            # rospy.sleep(1)
            # self.wait_correction_status()

            # rospy.ServiceProxy('/position_correction/' + set_status + "_2" , Trigger)()
            # rospy.sleep(1)
            # self.wait_correction_status()
            twist = Twist()
            twist.linear.x = 0.2
            self.mecanum_pub.publish(twist)
            rospy.sleep(1.95)
            self.mecanum_pub.publish(Twist())
            rospy.ServiceProxy('/position_correction/' + set_status + "_3" , Trigger)() #放置物体
            rospy.sleep(1)
            self.wait_correction_status()
            self.play('9')
            # twist = Twist()
            # twist.angular.z = -0.5
            # self.mecanum_pub.publish(twist)
            # rospy.sleep(5)
            # self.mecanum_pub.publish(Twist())
            twist = Twist()
            twist.linear.x = -0.2
            self.mecanum_pub.publish(twist)
            rospy.sleep(2)
            self.mecanum_pub.publish(Twist())
            twist = Twist()
            twist.angular.z = -0.5
            self.mecanum_pub.publish(twist)
            rospy.sleep(5)
            self.mecanum_pub.publish(Twist())
            # rospy.ServiceProxy('/position_correction/stop', Trigger)()
        elif set_status == "detect" : #检测
            self.play('2')
            rospy.ServiceProxy('/yolov5/start', Trigger)()
            rospy.sleep(1)
            self.wait_yolo_status()
            shape = rospy.get_param('/yolov5/shape','None')
            #yct add:这里优化了一下顺序，这样就不用播完再关掉，而是先关掉再播放
            rospy.ServiceProxy('/yolov5/stop', Trigger)()
            if shape == "cube":
                self.play('3')
            elif shape == "box":
                self.play('4')
            else:
                self.play('5')
            # shape = "box"
            rospy.set_param('/shape_recognition/target_shape',shape)
            rospy.sleep(1)
            # self.play('4')
        elif set_status == "back" : #回出发点(看坡面)
            rospy.ServiceProxy('/ramp/start', Trigger)() #启动坡道对齐
            rospy.set_param('~status', 'stop')
            #self.play('10')
            rospy.ServiceProxy('/ramp/up', Trigger)() #开始进行坡道对齐
            rospy.sleep(1)
            self.wait_ramp_status()

            ####
            twist = Twist()
            twist.angular.z = 0.5
            self.mecanum_pub.publish(twist)
            rospy.sleep(6)
            self.mecanum_pub.publish(Twist())
            ####
            rospy.sleep(0.6)

            twist = Twist()
            twist.linear.y = 0.1
            self.mecanum_pub.publish(twist)
            rospy.sleep(0.6)
            twist = Twist()
            '''
            twist.angular.z = -0.5
            self.mecanum_pub.publish(twist)
            rospy.sleep(0.2)
'''
            twist = Twist()
            twist.linear.x = -0.3
            self.mecanum_pub.publish(twist)
            rospy.sleep(4)
            self.mecanum_pub.publish(Twist())

            '''
            twist = Twist()
            twist.angular.z = 0.5
            self.mecanum_pub.publish(twist)
            rospy.sleep(6)
            self.mecanum_pub.publish(Twist())
            '''
            self.play('finish') #wjz add：播报任务完成

        elif set_status == "back_noramp" : #回出发点(看坡面)
            rospy.sleep(1)


    def run(self):
        while not rospy.is_shutdown() and self.running: 
            # print(self.words)
            if self.words is not None :
                if self.words == '开始执行任务':
                    print('>>>>>>>>>>>>>>>>>>>> 开始进行任务<<<<<<<<<<<<<<<<<')
                    
                    #yct add:识别完成之后，就注销语音识别话题，因为已经不需要了，但是官方给的似乎有时候会自动触发，很奇怪
                    self.vc_sub.unregister() #注销语音识别话题
                    self.buzzer_pub.unregister()
                    #强行关闭节点:
                    
                    print("[yct]尝试杀死node：/voice_control \n")
                    terminate_ros_node("/voice_control");
                    
                    print("[yct]尝试杀死node：/asr_node \n")
                    terminate_ros_node("/asr_node");
                    terminate_ros_pid("python3 /home/ubuntu/ros_ws/src/xf_mic_asr_offline/scripts/asr_node.py")
                    print("[yct]尝试杀死node：/awake_node \n")
                    terminate_ros_node("/awake_node");
                    terminate_ros_pid("python3 /home/ubuntu/ros_ws/src/xf_mic_asr_offline/scripts/awake_node.py")
                    self.play('1')
                    
                    #yct add:这段最开始的速度指定是为了能够先下坡，但是看起来官方给的很浪费时间
                    twist = Twist()
                    twist.linear.x = 0.3
                    self.mecanum_pub.publish(twist)
                    rospy.sleep(3)
                    #self.mecanum_pub.publish(Twist())
                    
                    #twist = Twist()
                    twist.angular.z = -0.5
                    self.mecanum_pub.publish(twist)
                    rospy.sleep(3)
                    self.mecanum_pub.publish(Twist())

                    # twist = Twist()
                    # twist.angular.z = -0.5
                    # self.mecanum_pub.publish(twist)
                    # rospy.sleep(3)
                    # self.mecanum_pub.publish(Twist())
                    #yct add:下坡完成了，接下来才会去正确点位
                    print("go")
                    self.move_base_status = 1
                    self.control(1.5,  -0.3,    90,  "detect") # 进行检测，control逻辑更改了，对于检测不用导航
                    # self.pose_pub(-1.57,1.1259,-0.4931) #alpha,x_pos,y_pos
                    # self.clear_costmap()
                    # twist = Twist()
                    # twist.angular.z = -0.5
                    # self.mecanum_pub.publish(twist)
                    # rospy.sleep(2) #yct add:看完图片直接自转
                    # self.mecanum_pub.publish(Twist())
                    # print("[yct]尝试杀死node：/astra_camera \n")
                    # terminate_ros_node("/astra_camera");
                    self.control(0.95,  -3.12,    0,   "pick")  # 进行夹取
                    #第一次夹取完成，重新设置位置，并且清除代价地图
                    self.pose_pub(0,1.643,-3.04) #alpha,x_pos,y_pos
                    self.clear_costmap()
                    #rospy.ServiceProxy('/move_base/clear_costmaps', Trigger)()
                    twist = Twist()
                    twist.linear.x = -0.2
                    self.mecanum_pub.publish(twist)
                    rospy.sleep(2)
                    self.mecanum_pub.publish(Twist())
                    twist = Twist()
                    twist.angular.z = 0.7
                    self.mecanum_pub.publish(twist)
                    rospy.sleep(3) #yct add:夹取完成后后退，后退后旋转90度面朝出口
                    self.mecanum_pub.publish(Twist())
                    self.control(1.3,  -0.35,    39,  "place") # 进行放置

                
                    self.control(0.95,  -3.12,   -180,"pick")  # 进行夹取
                    #第二次夹取完成，重新设置位置，并且清除代价地图
                    self.pose_pub(3.1416,0.1697,-3.17) #alpha,x_pos,y_pos
                    self.clear_costmap()
                    #rospy.ServiceProxy('/move_base/clear_costmaps', Trigger)()
                    twist = Twist()
                    twist.linear.x = -0.2
                    self.mecanum_pub.publish(twist)
                    rospy.sleep(2)
                    self.mecanum_pub.publish(Twist())
                    twist = Twist()
                    twist.angular.z = -0.5
                    self.mecanum_pub.publish(twist)
                    rospy.sleep(3) #yct add:夹取完成后后退，后退后旋转90度面朝出口
                    self.control(1.3,  -0.35,   39,  "place") # 进行放置
                    

                    # rospy.ServiceProxy('/yolov5/stop', Trigger)()
                    rospy.ServiceProxy('/position_correction/colse', Trigger)() #关闭对齐节点
                    rospy.ServiceProxy('/shape_recognition/colse', Trigger)()   #关闭夹取节点
                    self.control(0.2 ,-0.1, 180," ") #回到起始点 
                    twist = Twist()
                    twist.angular.z = -1.35
                    self.mecanum_pub.publish(twist)
                    rospy.sleep(2)
                    twist=Twist()
                    twist.linear.y = 0.2
                    self.mecanum_pub.publish(twist)
                    rospy.sleep(0.7)
                    self.mecanum_pub.publish(Twist())
                    
                    
                
                    self.play('finish')
                    print('>>>>>>>>>>>>>>>>>>>> 结束任务<<<<<<<<<<<<<<<<<')
                elif self.words == '休眠(Sleep)':
                    rospy.sleep(0.01)
                self.words = None
            else:
                rospy.sleep(0.01)

        self.mecanum_pub.publish(Twist())

if __name__ == "__main__":
    VoiceControlNavNode('voice_control_nav')
