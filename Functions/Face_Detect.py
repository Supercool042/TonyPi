#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import math
import time
import threading
import numpy as np
import mediapipe as mp
import hiwonder.ros_robot_controller_sdk as rrc
import hiwonder.yaml_handle as yaml_handle
import hiwonder.Camera as Camera

# 人脸检测(face detection)

# 初始化机器人底层驱动(initialize robot underlying driver)
board = rrc.Board()


# 导入人脸识别模块(import human face recognition module)
face = mp.solutions.face_detection
# 自定义人脸识别方法，最小的人脸检测置信度0.5(custom human face recognition method, the minimum face detection confidence is 0.5)
face_detection = face.FaceDetection(min_detection_confidence=0.5)

di_once = True
detect_people = False
def buzzer():
    global di_once
    global detect_people
    
    while True:
        if detect_people and di_once:
            board.set_buzzer(1900, 0.3, 0.7, 1) # 以1900Hz的频率，持续响0.3秒，关闭0.7秒，重复1次(at a frequency of 1900Hz, sound for 0.1 seconds, then pause for 0.9 seconds, repeat once)
            di_once = False
        else:
            time.sleep(0.01)
            
# 运行子线程(run sub-thread)
th = threading.Thread(target=buzzer)
th.daemon = True
th.start()

detect_count = 0
miss_count = 0
size = (320, 240)
def run(img):
    global detect_count, miss_count
    global di_once
    global detect_people
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    image_rgb = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB) # 将BGR图像转为RGB图像(convert the BGR image to RGB image)
    results = face_detection.process(image_rgb) # 将每一帧图像传给人脸识别模块(pass each frame of the image to the face recognition module)
    if results.detections:   # 如果检测不到人脸那就返回None(if no face is detected, return None)
        for index, detection in enumerate(results.detections): # 返回人脸索引index(第几张脸)，和关键点的坐标信息(return the face index (which face) and the coordinate information of the keypoints)
            bboxC = detection.location_data.relative_bounding_box # 设置一个边界框，接收所有的框的xywh及关键点信息(set up a bounding box to receive the xywh and keypoint information for all boxes)
            
             # 将边界框的坐标点,宽,高从比例坐标转换成像素坐标(convert the coordinates, width, and height of the bounding box from relative coordinates to pixel coordinates)
            bbox = (int(bboxC.xmin * img_w), int(bboxC.ymin * img_h),  
                   int(bboxC.width * img_w), int(bboxC.height * img_h))
            cv2.rectangle(img, bbox, (0,255,0), 2)  # 在每一帧图像上绘制矩形框(draw a rectangle on each frame of the image)
    
        detect_count += 1
        if detect_count > 20:
            detect_people = True
    else:
        detect_count = 0
        detect_people = False
        miss_count += 1
        if miss_count > 20:
            di_once = True

    return img

if __name__ == '__main__':
    from CameraCalibration.CalibrationConfig import *
    
    #加载参数(load parameters)
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数(get parameters)
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)    

    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open()
    
    print("Face_Detect Init")
    print("Face_Detect Start")
    
    while True:
        ret, img = my_camera.read()
        if img is not None:
            frame = img.copy()
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
           
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
