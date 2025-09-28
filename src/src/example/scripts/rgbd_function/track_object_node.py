#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/12/18
# @author:aiden
# 物体跟踪
import cv2
import rospy
import queue
import signal
import numpy as np
from sdk import pid
import open3d as o3d
import message_filters
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from servo_msgs.msg import MultiRawIdPosDur
from sensor_msgs.msg import Image, CameraInfo
from servo_controllers import bus_servo_control

class TrackObjectNode():
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        signal.signal(signal.SIGINT, self.shutdown)
        self.pid_x = pid.PID(1.5, 0, 0)
        self.pid_y = pid.PID(1.5, 0, 0)
        self.x_speed, self.y_speed = 0.007, 0.007
        self.stop_distance = 0.4
        self.x_stop = -0.04
        self.scale = 4
        self.linear_x, self.linear_y = 0, 0
        self.haved_add = False
        self.get_point = False
        self.display = 1
        self.running = True
        self.pc_queue = queue.Queue(maxsize=1)
        self.target_cloud = o3d.geometry.PointCloud() # 要显示的点云
        # 裁剪roi
        # x, y, z
        roi = np.array([
            [-0.8, -1.5, 0],
            [-0.8, 0.3, 0],
            [0.8,  0.3, 0],
            [0.8,  -1.5, 0]], 
            dtype = np.float64)
        # y 近+， x左-
        self.vol = o3d.visualization.SelectionPolygonVolume()
        # 裁剪z轴，范围
        self.vol.orthogonal_axis = 'Z'
        self.vol.axis_max = 0.9
        self.vol.axis_min = -0.3
        self.vol.bounding_polygon = o3d.utility.Vector3dVector(roi)
        # self.intrinsic = o3d.camera.PinholeCameraIntrinsic(640, 480, 378, 310, 378, 241)#356, 260
        # self.intrinsic = o3d.camera.PinholeCameraIntrinsic(self.proc_size[0], self.proc_size[1], int(378/self.scale), int(310/self.scale), int(378/self.scale), int(241/self.scale))#356, 260

        self.t0 = rospy.get_time()
        servos_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.mecanum_pub = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
        rospy.sleep(0.2)
        bus_servo_control.set_servos(servos_pub, 1, ((1, 500), (2, 765), (3, 85), (4, 150), (5, 500), (10, 200)))
        rospy.sleep(1)
        camera_name = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')
        rospy.wait_for_service('/%s/set_ldp'%camera_name)
        rospy.ServiceProxy('/%s/set_ldp'%camera_name, SetBool)(False)
        rospy.sleep(1)
        self.mecanum_pub.publish(Twist())
        rgb_sub = message_filters.Subscriber('/%s/rgb/image_raw'%camera_name, Image, queue_size=1)
        depth_sub = message_filters.Subscriber('/%s/depth/image_raw'%camera_name, Image, queue_size=1)
        info_sub = message_filters.Subscriber('/%s/depth/camera_info'%camera_name, CameraInfo, queue_size=1)
        
        # 同步时间戳, 时间允许有误差在0.02s
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.03)
        sync.registerCallback(self.multi_callback) #执行反馈函数
        self.run()

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        try:
            # ros格式转为numpy
            rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
            depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
            h, w = depth_image.shape[:2]
            h, w = depth_image.shape[:2]
            rgb_h, rgb_w = rgb_image.shape[:2]
            rgb_image = rgb_image[int((rgb_h - h)/2):h+int((rgb_h - h)/2), :]
            intrinsic = o3d.camera.PinholeCameraIntrinsic(int(depth_camera_info.width / self.scale),
                                                               int(depth_camera_info.height / self.scale),
                                                               int(depth_camera_info.K[0] / self.scale), int(depth_camera_info.K[4] / self.scale),
                                                               int(depth_camera_info.K[2] / self.scale), int(depth_camera_info.K[5] / self.scale))
            proc_size = (int(depth_camera_info.width / self.scale), int(depth_camera_info.height / self.scale))
            rgb_image = cv2.resize(rgb_image, tuple(proc_size), interpolation=cv2.INTER_NEAREST)
            depth_image = cv2.resize(depth_image, tuple(proc_size), interpolation=cv2.INTER_NEAREST)
            o3d_image_rgb = o3d.geometry.Image(rgb_image)
            o3d_image_depth = o3d.geometry.Image(np.ascontiguousarray(depth_image))        
            
            # rgbd_function --> point_cloud
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_image_rgb, o3d_image_depth, convert_rgb_to_intensity=False)
            # cpu占用大 
            pc = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)#, extrinsic=extrinsic)ic)
            
            # 裁剪
            roi_pc = self.vol.crop_point_cloud(pc)
            
            if len(roi_pc.points) > 0:
                # 去除最大平面，即地面, 距离阈4mm，邻点数，迭代次数
                plane_model, inliers = roi_pc.segment_plane(distance_threshold=0.05,
                         ransac_n=5,
                         num_iterations=40)
                
                # 保留内点
                inlier_cloud = roi_pc.select_by_index(inliers, invert=True)
                self.target_cloud.points = inlier_cloud.points
                self.target_cloud.colors = inlier_cloud.colors
            else:
                self.target_cloud.points = roi_pc.points
                self.target_cloud.colors = roi_pc.colors
            # 转180度方便查看
            self.target_cloud.transform(np.asarray([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]))
            try:
                self.pc_queue.put_nowait(self.target_cloud)
            except queue.Full:
                pass

            fps = int(1.0/(rospy.get_time() - self.t0))
            print('\r', 'FPS: ' + str(fps), end='')
        except BaseException as e:
            print('callback error:', e)
        self.t0 = rospy.get_time()

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')   

    def run(self):
        if self.display:
            # 创建可视化窗口
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name='point cloud', width=320, height=200, visible=1)
        while self.running:
            if not self.haved_add:
                if self.display:
                    point_cloud = self.pc_queue.get(block=True)
                    vis.add_geometry(point_cloud)
                self.haved_add = True
            if self.haved_add:
                point_cloud = self.pc_queue.get(block=True)
                # 刷新
                points = np.asarray(point_cloud.points)
                if len(points) > 0:
                    min_index = np.argmax(points[:, 2])
                    min_point = points[min_index]
                    point_cloud.colors[min_index] = [255, 255, 0]
                    
                    # kdtree = o3d.geometry.KDTreeFlann(point_cloud)
                    # 搜索指定坐标点的 10 个最近邻点
                    # nearest_points = kdtree.search_knn_vector_3d(min_point, 8)
                    # 对 10 个点进行着色
                    # for i in nearest_points[1]:
                        # point_cloud.colors[i] = [255, 255, 0]

                    distance = min_point[-1]
                    self.pid_x.SetPoint = self.stop_distance
                    if abs(distance - self.stop_distance) < 0.1:
                        distance = self.stop_distance
                    self.pid_x.update(-distance)  #更新pid
                    tmp = self.x_speed - self.pid_x.output
                    self.linear_x = tmp
                    if tmp > 0.3:
                        self.linear_x = 0.3
                    if tmp < -0.3:
                        self.linear_x = -0.3
                    if abs(tmp) < 0.008:
                        self.linear_x = 0
                    twist = Twist()
                    twist.linear.x = self.linear_x
                    
                    y_distance = min_point[0]
                    self.pid_y.SetPoint = self.x_stop
                    if abs(y_distance - self.x_stop) < 0.03:
                        y_distance = self.x_stop
                    self.pid_y.update(y_distance)  #更新pid
                    tmp = self.y_speed + self.pid_y.output
                    self.linear_y = tmp
                    if tmp > 0.3:
                        self.linear_y = 0.3
                    if tmp < -0.3:
                        self.linear_y = -0.3
                    if abs(tmp) < 0.008:
                        self.linear_y = 0
                    twist.linear.y = self.linear_y

                    # print(min_point)
                    if self.display:
                        vis.update_geometry(point_cloud)
                        vis.poll_events()
                        vis.update_renderer()
                    self.mecanum_pub.publish(twist)
                else:
                    self.mecanum_pub.publish(Twist())
            else:
                rospy.sleep(0.01)
        self.mecanum_pub.publish(Twist())
        # 销毁所有显示的几何图形
        vis.destroy_window()
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    TrackObjectNode('track_object')
