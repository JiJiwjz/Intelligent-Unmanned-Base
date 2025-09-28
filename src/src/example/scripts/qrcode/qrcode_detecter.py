#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2  
import sdk.fps as fps

MODEL_PATH = os.path.join(os.path.split(os.path.realpath(__file__))[0], 'opencv_3rdparty')
model1 = os.path.join(MODEL_PATH, 'detect.prototxt')
model2 = os.path.join(MODEL_PATH, 'detect.caffemodel')
model3 = os.path.join(MODEL_PATH, 'sr.prototxt')
model4 = os.path.join(MODEL_PATH, 'sr.caffemodel')

detect_obj = cv2.wechat_qrcode_WeChatQRCode(model1, model2, model3, model4)

cap = cv2.VideoCapture("/dev/astra_camera")
# cap = cv2.VideoCapture("/dev/gemini_camera")
print('\n******Press any key to exit!******')
fps = fps.FPS()
while cap.isOpened():
    ret, img = cap.read()
    if ret:
        res, points = detect_obj.detectAndDecode(img)
        if res != ():
            print('result:', res)
        for pos in points:
            color = (0, 0, 255)
            thick = 3
            for p in [(0, 1), (1, 2), (2, 3), (3, 0)]:
                start = int(pos[p[0]][0]), int(pos[p[0]][1])
                end = int(pos[p[1]][0]), int(pos[p[1]][1])
                cv2.line(img, start, end, color, thick)
        fps.update()
        result_image = fps.show_fps(img)
        cv2.imshow('qrcode_detect', result_image)
        key = cv2.waitKey(1)
        if key != -1: 
            break
            
cap.release()
cv2.destroyAllWindows()
