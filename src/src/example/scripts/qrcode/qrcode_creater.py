#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import qrcode
import numpy as np

def create_qrcode(data, file_name):
    '''
    version：值为1~40的整数，控制二维码的大小（最小值是1，是个12×12的矩阵）。
             如果想让程序自动确定，将值设置为 None 并使用 fit 参数即可。
    error_correction：控制二维码的错误纠正功能。可取值下列4个常量。
    　　ERROR_CORRECT_L：大约7%或更少的错误能被纠正。
    　　ERROR_CORRECT_M（默认）：大约15%或更少的错误能被纠正。
    　　ROR_CORRECT_H：大约30%或更少的错误能被纠正。
    box_size：控制二维码中每个小格子包含的像素数。
    border：控制边框（二维码与图片边界的距离）包含的格子数（默认为4，是相关标准规定的最小值）
    '''
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=5,
        border=4)
    # 添加数据
    qr.add_data(data)
    # 填充数据
    qr.make(fit=True)
    # 生成图片
    img = qr.make_image(fill_color=(0, 0, 0), back_color=(255, 255, 255))
    opencv_img = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)
    cv2.imshow('img', opencv_img)
    cv2.waitKey(0)
    cv2.imwrite(file_name, opencv_img)
    print('save', data, file_name)

if __name__ == '__main__':
    file_path = os.getcwd()
    out_img = file_path + '/myQRcode.jpg'
    qrcode_text = input("Please enter：")
    create_qrcode(qrcode_text, out_img)
