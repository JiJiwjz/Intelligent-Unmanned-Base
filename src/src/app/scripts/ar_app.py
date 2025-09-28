#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/03/24
# @author:aiden
# ar增强(augmented reality)
import os
import cv2
import rospy
import numpy as np
import threading
from apriltag import apriltag
from app.common import Heart
from scipy.spatial.transform import Rotation as R
from app.objloader_simple import OBJ as obj_load
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from interfaces.srv import SetString, SetStringRequest, SetStringResponse

# 获取模型默认存放路径(acquire the default storage path for the model)
MODEL_PATH = os.path.join(os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '..')), 'models')

# 求解pnp的点，正方形的四个角和中心点(solve for the points of pnp, the four corners and the center point of the square)
OBJP = np.array([[-1, -1,  0],
                 [ 1, -1,  0],
                 [-1,  1,  0],
                 [ 1,  1,  0],
                 [ 0,  0,  0]], dtype=np.float32)

# 绘制立方体的坐标(draw the coordinate of the cube)
AXIS = np.float32([[-1, -1, 0], 
                   [-1,  1, 0], 
                   [ 1,  1, 0], 
                   [ 1, -1, 0],
                   [-1, -1, 2],
                   [-1,  1, 2],
                   [ 1,  1, 2],
                   [ 1, -1, 2]])

# 模型缩放比例(model scaling)
MODELS_SCALE = {
                'bicycle': 50, 
                'fox': 4, 
                'chair': 400, 
                'cow': 0.4,
                'wolf': 0.6,
                }

def draw_rectangle(img, imgpts):
    '''
    绘制立方体(draw the cube)
    :param img: 要绘制立方体的图像(the image to draw the cube)
    :param imgpts: 立方体的角点(angular point of the cube)
    :return: 要绘制立方体的图像(the image to draw the cube)
    '''
    imgpts = np.int32(imgpts).reshape(-1, 2)
    cv2.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), -3)  # 绘制轮廓点，填充形式
    for i, j in zip(range(4), range(4, 8)):
        cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 3)  # 绘制线连接点
    cv2.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 3)  # 绘制轮廓点，不填充
    
    return img

class ARNode:
    def __init__(self, name):
        rospy.init_node(name)
        # 获取模型存放路径(obtain the storage path for the model)
        self.model_path = rospy.get_param("~model_path", MODEL_PATH)
        
        self.name = name
        # 摄像头内参
        self.camera_intrinsic = np.matrix([[619.063979, 0,          302.560920],
                                           [0,          613.745352, 237.714934],
                                           [0,          0,          1]])
        self.dist_coeffs = np.array([0.103085, -0.175586, -0.001190, -0.007046, 0.000000])
        
        self.obj = None
        self.image_sub = None
        self.target_model = None
        self.camera_info_sub = None
        
        self.tag_detector = apriltag("tag36h11")  # 实例化apriltag(instantiate apriltag)
        self.lock = threading.RLock()  # 线程锁(thread lock)
        
        self.result_publisher = rospy.Publisher('~image_result', Image, queue_size=1)  # 发布最终图像(publish the final image)
        self.enter_srv = rospy.Service('~enter', Trigger, self.enter_srv_callback)  # 进入发玩法服务(enter the game service)
        self.exit_srv = rospy.Service('~exit', Trigger, self.exit_srv_callback)  # 退出玩法服务(exit the game service)
        self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(None))  # 心跳包(heartbeat package)
        self.set_model_srv = rospy.Service('~set_model', SetString, self.set_model_srv_callback)  # 设置模型服务(set the model service)

    def enter_srv_callback(self, msg):
        # 进入服务(enter the service)
        rospy.loginfo("ar enter")
        # 进入服务时如果节点还在则注销订阅，重新订阅(if there is a node when entering the service, cancel subscription and subscribe again)
        try:
            if self.image_sub is not None:
                self.image_sub.unregister()
        except Exception as e:
            rospy.logerr(str(e))
        try:
            if self.camera_info_sub is not None:
                self.camera_info_sub.unregister()
        except Exception as e:
            rospy.logerr(str(e))
         
        with self.lock:
            self.obj = None
            self.target_model = None
            depth_camera = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')  # 获取摄像头名称(get the camera name)
            self.image_sub = rospy.Subscriber('/%s/rgb/image_raw'%depth_camera, Image, self.image_callback, queue_size=1)  # 订阅图像(subscribe to the image)
            self.camera_info_sub = rospy.Subscriber('/%s/rgb/camera_info'%depth_camera, CameraInfo, self.camera_info_callback) # 订阅摄像头信息(subscribe to the camera information)
        
        return TriggerResponse(success=True)

    def exit_srv_callback(self, _):
        # 退出服务(exit the service)
        rospy.loginfo("ar exit")
        # 退出服务时注销订阅，节省开销(cancel the subscribtion when exiting the service to save the expenditure)
        try:
            if self.image_sub is not None:
                self.image_sub.unregister()
        except Exception as e:
            rospy.logerr(str(e))
        try:
            if self.camera_info_sub is not None:
                self.camera_info_sub.unregister()
        except Exception as e:
            rospy.logerr(str(e))
            
        return TriggerResponse(success=True)
        
    def set_model_srv_callback(self, model: SetStringRequest):
        # 设置模型
        with self.lock:
            rospy.loginfo("set model {}".format(model.data))
            if model.data == "":
                self.target_model = None
            else:
                self.target_model = model.data
                if self.target_model != 'rectangle':  # 如果不是绘制立方体(if the cube is not being drawn)
                    # 加载模型(load the model)
                    obj = obj_load(os.path.join(self.model_path, self.target_model + '.obj'), swapyz=True)
                    obj.faces = obj.faces[::-1]
                    new_faces = []
                    # 对模型进行解析，获取点坐标(analyze the model and get the point coordinates)
                    for face in obj.faces:
                        face_vertices = face[0]
                        points = []
                        colors = []
                        for vertex in face_vertices:
                            data = obj.vertices[vertex - 1]
                            points.append(data[:3])
                            if self.target_model != 'cow' and self.target_model != 'wolf':
                                colors.append(data[3:])
                        scale_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) * MODELS_SCALE[self.target_model]  # 缩放(scale)
                        points = np.dot(np.array(points), scale_matrix)
                        if self.target_model == 'bicycle':
                            points = np.array([[p[0] - 670, p[1] - 350, p[2]] for p in points])
                            points = R.from_euler('xyz', (0, 0, 180), degrees=True).apply(points)
                        elif self.target_model == 'fox':
                            points = np.array([[p[0], p[1], p[2]] for p in points])
                            points = R.from_euler('xyz', (0, 0, -90), degrees=True).apply(points)
                        elif self.target_model == 'chair':
                            points = np.array([[p[0], p[1], p[2]] for p in points])
                            points = R.from_euler('xyz', (0, 0, -90), degrees=True).apply(points)
                        else:
                            points = np.array([[p[0], p[1], p[2]] for p in points])
                        if len(colors) > 0:
                            color = tuple(255 * np.array(colors[0]))
                        else:
                            color = None
                        new_faces.append((points, color))
                    self.obj = new_faces
                    
        return SetStringResponse(success=True)

    def camera_info_callback(self, msg):
        # 摄像头内参信息获取回调(camera internal parameter callback)
        with self.lock:
            self.camera_intrinsic = np.array(msg.K).reshape(3, -1)
            self.dist_coeffs = msg.D

    def image_callback(self, ros_image):
        # 图像回调(image callback)
        # 将ROS图像消息转化为numpy格式(convert ROS image into numpy format)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        result_image = np.copy(rgb_image)
        with self.lock:
            try:
                # 图像处理(process image)
                result_image = self.image_proc(rgb_image, result_image)
            except Exception as e:
                rospy.logerr(str(e))
        # opencv格式转为ros格式(convert opencv format into ros format)
        ros_image.data = result_image.tostring()
        self.result_publisher.publish(ros_image)  # 发布图像(publish image)

    def image_proc(self, rgb_image, result_image):
        # 图像处理
        if self.target_model is not None: 
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)  # 转为灰度图(convert into gray image)
            detections = self.tag_detector.detect(gray)  # aprilatg识别(aprilatg recognition)
            if detections != ():
                for detection in detections:  # 遍历(traverse)
                    # 获取四个角点和中心(acquire four angular points and center point)
                    tag_center = detection['center']
                    tag_corners = detection['lb-rb-rt-lt']
                    lb = tag_corners[0]
                    rb = tag_corners[1]
                    rt = tag_corners[2]
                    lt = tag_corners[3]
                    # 绘制四个角点(draw four angular points)
                    cv2.circle(result_image, (int(lb[0]), int(lb[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(lt[0]), int(lt[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(rb[0]), int(rb[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(rt[0]), int(rt[1])), 2, (0, 255, 255), -1)
                    # cv2.circle(result_image, (int(tag_center[0]), int(tag_center[1])), 3, (255, 0, 0), -1)
                    corners = np.array([lb, rb, lt, rt, tag_center]).reshape(5, -1)
                    # 使用世界坐标系k个点坐标(OBJP)，对应图像坐标系2D的k个点坐标(corners)，以及相机内参camera_intrinsic和dist_coeffs进行反推图片的外参r, t
                    # (Use the world coordinate system k point coordinates (OBJP), the k point coordinates (corners) corresponding to the 2D image coordinate system,
                    # and the camera internal parameters camera_intrinsic and dist_coeffs to reverse the external parameters r, t of the picture)
                    ret, rvecs, tvecs = cv2.solvePnP(OBJP, corners, self.camera_intrinsic, self.dist_coeffs)
                    if self.target_model == 'rectangle':  # 如果要显示立方体则单独处理(if the cube needs to be displayed, process independently)
                        # 反向投影将世界坐标系点转换到图像点(backprojection converts world coordinate system points to image points)
                        imgpts, jac = cv2.projectPoints(AXIS, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)
                        result_image = draw_rectangle(result_image, imgpts)
                    else:
                        for points, color in self.obj:
                             dst, jac = cv2.projectPoints(points.reshape(-1, 1, 3)/100.0, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)
                             imgpts = dst.astype(int)
                             # 手动上色(manual coloring)
                             if self.target_model == 'cow':
                                 cv2.fillConvexPoly(result_image, imgpts, (0, 255, 255))
                             elif self.target_model == 'wolf':
                                 cv2.fillConvexPoly(result_image, imgpts, (255, 255, 0))
                             else:
                                 cv2.fillConvexPoly(result_image, imgpts, color)

        return result_image

if __name__ == '__main__':
    ARNode('ar_app')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
        rospy.loginfo("Shutting down")
