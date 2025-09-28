#!/usr/bin/env python3
# encoding: utf-8
# yolov5目标检测
import os
import cv2
import queue
import rospy
import signal
import numpy as np
import sdk.fps as fps
from sdk import common
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from interfaces.msg import ObjectInfo, ObjectsInfo
from yolov5_trt_6_2 import YoLov5TRT
#此处为必须，用于初始化cuda
import pycuda.driver as cuda
import pycuda.driver as drv 
import pycuda.autoinit
MODE_PATH = os.path.split(os.path.realpath(__file__))[0]

class Yolov5Node:
    def __init__(self, name):
        rospy.init_node(name)  #初始化节点
        self.start = False     #是否开始识别
        self.running = True    #是否继续循环识别
        self.yolov5 = None     #给yolov5模型预赋一个属性
        self.image_sub = None  #相机节点
        self.image_queue = queue.Queue(maxsize=1) #图像队列
        
        self.engine = rospy.get_param('~engine')  #模型名
        self.calibration = rospy.get_param('~calibration',False)  #模型名
        self.lib = rospy.get_param('~lib')   #模型对应.so文件
        self.conf_thresh = rospy.get_param('~conf_thresh', 0.8) #置信度
        self.classes = rospy.get_param('/classes') #识别的类别
        rospy.set_param('~close', False) #设置此节点close参数，方便后续关闭节点
        rospy.set_param('~shape', 'None') #设置此节点shape参数，方便将识别到的内容传输
        self.close = rospy.get_param('~close') #得到目前close参数
        rospy.wait_for_service('/astra_camera/set_ldp') #等待相机启动
        # rospy.wait_for_service('/robot_2/astra_camera/set_ldp') #等待相机启动
        rospy.Service('/yolov5/start', Trigger, self.start_srv_callback)  # 开始识别
        rospy.Service('/yolov5/stop', Trigger, self.stop_srv_callback)  # 关闭识别并退出节点
        rospy.Service('/yolov5/calibration', Trigger, self.calibration_srv_callback)  # 关闭识别并退出节点
        rospy.set_param('~init_finish', True) #设置初始化状态参数

        #yct add: 初始化就启动YOLO
        self.yolov5 = YoLov5TRT(os.path.join(MODE_PATH, self.engine), os.path.join(MODE_PATH, self.lib), self.classes, self.conf_thresh) #初始化模型
        drv.Context.pop()
        signal.signal(signal.SIGINT, self.shutdown)

    #开始识别
    def calibration_srv_callback(self, msg):
        self.calibration = True
        self.result_image_pub = rospy.Publisher('~object_image', Image, queue_size=1)
        return TriggerResponse(success=True)
    #yct add: 这里开始yolo5的资源检测，极其容易报功率超限的错误
    def start_srv_callback(self, msg):
        rospy.loginfo("start yolov5 detect")
        #yct add:初始化yolov5的我放在init里面了，免得再启动太麻烦
        # self.yolov5 = YoLov5TRT(os.path.join(MODE_PATH, self.engine), os.path.join(MODE_PATH, self.lib), self.classes, self.conf_thresh) #初始化模型
        # drv.Context.pop()
        self.image_sub = rospy.Subscriber('/%s/rgb/image_raw' % 'astra_camera', Image, self.image_callback, queue_size=1)  #接受图像信息
        # self.image_sub = rospy.Subscriber('/robot_1/%s/rgb/image_raw' % 'astra_camera', Image, self.image_callback, queue_size=1)  #接受图像信息
        self.start = True #启动识别
        return TriggerResponse(success=True)
        
    #暂停并退出节点
    def stop_srv_callback(self, msg):
        rospy.loginfo('stop yolov5 detect')
        self.start = False
        self.close = True
        return TriggerResponse(success=True)
    #图像回调函数
    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # 将图像放入队列
        self.image_queue.put(bgr_image)
        #识别
        self.image_proc()
   
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')
    #识别函数
    def image_proc(self):
        # while self.running :
        try:
            if self.close :  #根据参数close判断是否需要关闭此节点
                self.image_sub.unregister()
                # self.yolov5.destroy() 
                rospy.signal_shutdown('shutdown')
            elif self.start:   #根据参数close判断是否需要进行识别
                image = self.image_queue.get(block=True) #得到队列内图像
                h, w = image.shape[:2]  #读取图像高宽
                boxes, scores, classid = self.yolov5.infer(image) #得到识别内容
                for box, cls_conf, cls_id in zip(boxes, scores, classid):#将内容读出
                    rospy.set_param('~shape', self.classes[cls_id])#根据形状设置参数shape
                    print("shape",self.classes[cls_id])
                    if self.calibration:
                        color = common.colors(cls_id, True)
                        common.plot_one_box(
                            box,
                            image,
                            color=color,
                            label="{}:{:.2f}".format(
                                self.classes[cls_id], cls_conf
                            ),
                        )
                if self.calibration:
                    self.result_image_pub.publish(common.cv2_image2ros(image, frame_id='yolov5'))
            else:
                rospy.sleep(0.01)
        except BaseException as e:
            print(e)

if __name__ == "__main__":
    node = Yolov5Node('yolov5')
    try:
        rospy.spin()
    except exception as e:
        mecnum_pub.publish(twist())
        rospy.logerr(str(e))
