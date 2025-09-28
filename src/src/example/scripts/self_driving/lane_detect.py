#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/11
# @author:aiden
# 无人驾驶车道线提取
import cv2
import math
import numpy as np
import sdk.common as common

lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")

class LaneDetector(object):
    def __init__(self, color):
        # 车道线颜色
        self.target_color = color
        # 车道线识别的区域
        self.rois = ((45/48, 1, 0, 0.5, 0.7), (39/48, 42/48, 0, 0.5, 0.2), (33/48, 36/48, 0, 0.5, 0.1))
        self.weight_sum = 1.0

    def set_roi(self, roi):
        self.rois = roi

    @staticmethod
    def get_area_max_contour(contours, threshold=100):
        '''
        获取最大面积对应的轮廓
        :param contours:
        :param threshold:
        :return:
        '''
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None
    
    def add_horizontal_line(self, image):
        #   |____  --->   |————   ---> ——
        h, w = image.shape[:2]
        roi_w_min = int(w/2)
        roi_w_max = w
        roi_h_min = 0
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]  # 截取右半边
        flip_binary = cv2.flip(roi, 0)  # 上下翻转
        max_y = cv2.minMaxLoc(flip_binary)[-1][1]  # 提取最上，最左数值为255的点坐标

        return h - max_y

    def add_vertical_line_far(self, image, near=False):
        h, w = image.shape[:2]
        if near:
            roi_w_min = int(1*w/8)
        else:
            roi_w_min = int(2*w/8)
        roi_w_max = int(4*w/8)
        roi_h_min = 0
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]
        flip_binary = cv2.flip(roi, -1)  # 图像左右上下翻转
        # cv2.imshow('1', flip_binary)
        # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(ret)
        # minVal：最小值
        # maxVal：最大值
        # minLoc：最小值的位置
        # maxLoc：最大值的位置
        # 遍历的顺序，先行再列，行从左到右，列从上到下
        (x_0, y_0) = cv2.minMaxLoc(flip_binary)[-1]  # 提取最上，最左数值为255的点坐标
        y_center = y_0 + int(55/480*h)
        roi = flip_binary[y_center:, :]
        (x_1, y_1) = cv2.minMaxLoc(roi)[-1]
        down_p = (roi_w_max - x_1, roi_h_max - (y_1 + y_center))
        # cv2.circle(flip_binary, (x_0, y_0), 20, 255, -1)
        
        y_center = y_0 + int(65/480*h)
        roi = flip_binary[y_center:, :]
        (x_2, y_2) = cv2.minMaxLoc(roi)[-1]
        up_p = (roi_w_max - x_2, roi_h_max - (y_2 + y_center))
        
        # cv2.circle(flip_binary, (x_2, y_2), 20, 255, -1)
        # cv2.imshow('2', flip_binary)
        # print(down_p, up_p)
        up_point = (0, 0)
        down_point = (0, 0)
        if up_p[1] - down_p[1] != 0 and up_p[0] - down_p[0] != 0:
            up_point = (int(-down_p[1]/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), 0)
            down_point = (int((h - down_p[1])/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), h)

        return up_point, down_point

    def add_vertical_line_near(self, image):
        # ——|         |——        |
        #   |   --->  |     --->
        h, w = image.shape[:2]
        roi_w_min = 0
        roi_w_max = int(w/2)
        roi_h_min = int(h/2)
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]
        flip_binary = cv2.flip(roi, -1)  # 图像左右上下翻转
        #cv2.imshow('1', flip_binary)
        (x_0, y_0) = cv2.minMaxLoc(flip_binary)[-1]  # 提取最上，最左数值为255的点坐标
        down_p = (roi_w_max - x_0, roi_h_max - y_0)

        (x_1, y_1) = cv2.minMaxLoc(roi)[-1]
        y_center = int((roi_h_max - roi_h_min - y_1 + y_0)/2)
        roi = flip_binary[y_center:, :] 
        (x, y) = cv2.minMaxLoc(roi)[-1]
        up_p = (roi_w_max - x, roi_h_max - (y + y_center))

        up_point = (0, 0)
        down_point = (0, 0)
        if up_p[1] - down_p[1] != 0 and up_p[0] - down_p[0] != 0:
            up_point = (int(-down_p[1]/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), 0)
            down_point = down_p

        return up_point, down_point, y_center

    def get_binary(self, image, camera_name='gemini_camera'):
        # 通过lab空间识别颜色
        img_lab = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # rgb转lab
        img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪
        mask = cv2.inRange(img_blur, tuple(lab_data['lab'][camera_name][self.target_color]['min']), tuple(lab_data['lab'][camera_name][self.target_color]['max']))  # 二值化
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀

        return dilated

    def __call__(self, image, result_image):
        # 按比重提取线中心
        centroid_sum = 0
        h, w = image.shape[:2]
        max_center_x = -1
        center_x = []
        for roi in self.rois:
            blob = image[int(roi[0]*h):int(roi[1]*h), int(roi[2]*w):int(roi[3]*w)]  # 截取roi
            contours = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓
            max_contour_area = self.get_area_max_contour(contours, 30)  # 获取最大面积对应轮廓
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形
                box = np.int0(cv2.boxPoints(rect))  # 四个角
                for j in range(4):
                    box[j, 1] = box[j, 1] + int(roi[0]*h)
                cv2.drawContours(result_image, [box], -1, (255, 255, 0), 2)  # 画出四个点组成的矩形

                # 获取矩形对角点
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # 线的中心点
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)  # 画出中心点
                center_x.append(line_center_x)
            else:
                center_x.append(-1)
        for i in range(len(center_x)):
            if center_x[i] != -1:
                if center_x[i] > max_center_x:
                    max_center_x = center_x[i]
                centroid_sum += center_x[i] * self.rois[i][-1]
        if centroid_sum == 0:
            return result_image, None, max_center_x
        center_pos = centroid_sum / self.weight_sum  # 按比重计算中心点
        angle = math.degrees(-math.atan((center_pos - (w / 2.0)) / (h / 2.0)))
        
        return result_image, angle, max_center_x

image = None
def image_callback(ros_image):
    global image
    rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
    image = rgb_image

if __name__ == '__main__':
    import rospy
    from sensor_msgs.msg import Image

    lane_detect = LaneDetector('yellow')
    rospy.init_node('lane_detect_test', anonymous=True)
    camera = rospy.get_param('/gemini_camera/camera_name', 'gemini_camera')
    rospy.Subscriber('/%s/rgb/image_raw' % camera, Image, image_callback, queue_size=1)
    while not rospy.is_shutdown():
        if image is not None:
            binary_image = lane_detect.get_binary(image)
            h, w = binary_image.shape[:2]
            cv2.imshow('binary', binary_image)
            img = image.copy()
            '''
            y = lane_detect.add_horizontal_line(binary_image)
            roi = [(0, y), (w, y), (w, 0), (0, 0)]
            cv2.fillPoly(binary_image, [np.array(roi)], [0, 0, 0])  # 将上面填充为黑色，防干扰
            min_x = cv2.minMaxLoc(binary_image)[-1][0]
            cv2.line(img, (min_x, y), (w, y), (255, 255, 255), 50)  # 画虚拟线来驱使转弯
            result_image, angle, x = lane_detect(binary_image, image.copy()) 
            '''
            up, down = lane_detect.add_vertical_line_far(binary_image)
            #up, down, center = lane_detect.add_vertical_line_near(binary_image)
            cv2.line(img, up, down, (255, 255, 255), 10)
            cv2.imshow('img', img)
            cv2.waitKey(1)
        else:
            rospy.sleep(0.01)
