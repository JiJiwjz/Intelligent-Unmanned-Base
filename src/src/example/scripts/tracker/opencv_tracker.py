#!/usr/bin/env python3
# encoding: utf-8
import cv2
import sdk.fps as fps

cap = cv2.VideoCapture("/dev/astra_camera")
# cap = cv2.VideoCapture("/dev/gemini_camera")
print('\n******Press any key to exit!******')
fps = fps.FPS()

tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT']
tracker_type = tracker_types[-1]#4 5

if tracker_type == 'BOOSTING':
    tracker = cv2.legacy.TrackerBoosting_create()
elif tracker_type == 'MIL':
    tracker = cv2.legacy.TrackerMIL_create()
elif tracker_type == 'KCF':
    tracker = cv2.TrackerKCF_create()
elif tracker_type == 'TLD':
    tracker = cv2.legacy.TrackerTLD_create()
elif tracker_type == 'MEDIANFLOW':
    tracker = cv2.legacy.TrackerMedianFlow_create()
elif tracker_type == 'MOSSE':
    tracker = cv2.legacy.TrackerMOSSE_create()
elif tracker_type == "CSRT":
    tracker = cv2.TrackerCSRT_create()

selection = None   # 实时跟踪鼠标的跟踪区域
track_window = None   # 要检测的物体所在区域
drag_start = None   # 标记，是否开始拖动鼠标
 
mouse_click = False
#鼠标点击事件回调函数
def onmouse(event, x, y, flags, param):   
    global selection, track_window, drag_start      #定义全局变量
    global mouse_click
    if event == cv2.EVENT_LBUTTONDOWN:       #鼠标左键按下
        mouse_click = True
        drag_start = (x, y)       #鼠标起始位置
        track_window = None
    if drag_start:       #是否开始拖动鼠标，记录鼠标位置
        xmin = min(x, drag_start[0])
        ymin = min(y, drag_start[1])
        xmax = max(x, drag_start[0])
        ymax = max(y, drag_start[1])
        selection = (xmin, ymin, xmax, ymax)
    if event == cv2.EVENT_LBUTTONUP:        #鼠标左键松开
        mouse_click = False
        drag_start = None
        track_window = selection
        selection = None

def main():
    global mouse_click, x_dis, get_object
    #创建图像与窗口，并将窗口与回调函数绑定
    cv2.namedWindow('image', 1)
    cv2.setMouseCallback('image', onmouse)
    
    start_circle = True
    start_click = False
    while(1):
        ret, orgframe = cap.read()  #从摄像头读入1帧，ret表明成功与否
        frame = orgframe.copy() #cv2.resize(orgframe, (320, 240))
        if not ret:
            print("Game over!")
            break 
        image = frame.copy()     #不改变原来的帧，拷贝一个新的
        # 初始化第一帧
        if start_circle:
            # 用鼠标拖拽一个框来指定区域             
            if track_window:      #跟踪目标的窗口画出后，实时标出跟踪目标
                cv2.rectangle(image, (track_window[0], track_window[1]), (track_window[2], track_window[3]), (0, 0, 255), 2)
            elif selection:      #跟踪目标的窗口随鼠标拖动实时显示
                cv2.rectangle(image, (selection[0], selection[1]), (selection[2], selection[3]), (0, 0, 255), 2)
            if mouse_click:
                start_click = True
            if start_click:
                if not mouse_click:
                    start_circle = False
            if not start_circle:
                print('track init')
                bbox = (track_window[0], track_window[1], track_window[2] - track_window[0], track_window[3] - track_window[1])
                tracker.init(image, bbox)
        else:
            ok, bbox = tracker.update(image)
            if ok:
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                center = (int((p1[0] + p2[0])/2), int((p1[1] + p2[1])/2))
                cv2.drawMarker(image, center, (0, 255, 0), markerType=0, thickness=2, line_type=cv2.LINE_AA)
                #cv2.circle(image, center, 5, (0, 255, 0), -1, cv2.LINE_AA)
                cv2.rectangle(image, p1, p2, (0, 255, 0), 2, 1)
            else:
                # Tracking failure
                cv2.putText(image, "Tracking failure detected !", (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        
        fps.update()
        result_image = fps.show_fps(image)
        cv2.imshow('image', result_image)
        c = cv2.waitKey(1)
        if c == 27:
            break

    cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main()
