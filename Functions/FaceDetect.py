#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import math
import time
import threading
import numpy as np

import mediapipe as mp
import hiwonder.Camera as Camera
import hiwonder.Misc as Misc
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle


'''
    程序功能：人脸识别(program function: face recognition)

    运行效果：玩法开启后，摄像头云台将进行左右转动。当识别到人脸后，云台会停止转动，此时机器人将执行招手动作。(running effect: after the game mode is activated, the camera gimbal will rotate left and right. When a face is recognized, the gimbal will stop rotating, and the robot will perform a waving gesture)

    对应教程文档路径：  TonyPi智能视觉人形机器人\3.AI视觉玩法学习\第6课 人脸识别(corresponding tutorial file path: TonyPi Intelligent Vision Humanoid Robot\3.AI Vision Game Course\Lesson6 Human Face Recognition)
'''

# 检查 Python 版本是否为 Python 3，若不是则打印提示信息并退出程序(check if the Python version is Python 3. If not, print a prompt message and exit the program)
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 导入人脸识别模块(import human face recognition module)
face = mp.solutions.face_detection
# 自定义人脸识别方法，最小的人脸检测置信度0.5(custom human face recognition method, the minimum face detection confidence is 0.5)
face_detection = face.FaceDetection(min_detection_confidence=0.7)

board = rrc.Board()
ctl = Controller(board)


servo_data = None
def load_config():
    global servo_data
    
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)  

load_config()

servo2_pulse = servo_data['servo2']

# 初始化机器人舵机初始位置(initialize the servo initialization position of robot)
def initMove():
    ctl.set_pwm_servo_pulse(1, 1800, 500)
    ctl.set_pwm_servo_pulse(2, servo2_pulse, 500)


d_pulse = 10
start_greet = False
action_finish = True
# 变量重置(variable reset)
def reset():
    global d_pulse
    global start_greet
    global servo2_pulse    
    global action_finish

    d_pulse = 10
    start_greet = False
    action_finish = True
    servo2_pulse = servo_data['servo2']    
    initMove()  
    
# app初始化调用(app initialization calling)
def init():
    print("FaceDetect Init")
    reset()

__isRunning = False
# app开始玩法调用(app start program calling)
def start():
    global __isRunning
    __isRunning = True
    print("FaceDetect Start")

# app停止玩法调用(app stop program calling)
def stop():
    global __isRunning
    __isRunning = False
    reset()
    print("FaceDetect Stop")

# app退出玩法调用(app exit program calling)
def exit():
    global __isRunning
    __isRunning = False
    AGC.runActionGroup('stand_slow')
    print("FaceDetect Exit")

def move():
    global start_greet
    global action_finish
    global d_pulse, servo2_pulse    
    
    while True:
        if __isRunning:
            if start_greet:
                start_greet = False
                action_finish = False
                AGC.runActionGroup('wave') # 识别到人脸时执行的动作(the action to be performed when a face is recognized)
                action_finish = True
                time.sleep(0.5)
            else:
                if servo2_pulse > 2000 or servo2_pulse < 1000:
                    d_pulse = -d_pulse
            
                servo2_pulse += d_pulse 
                ctl.set_pwm_servo_pulse(2, servo2_pulse, 50)
                time.sleep(0.05)
        else:
            time.sleep(0.01)
            
# 运行子线程(run sub-thread)
th = threading.Thread(target=move)
th.daemon = True
th.start()

def run(img):
    global start_greet
    global action_finish
       
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning:
        return img
    
    image_rgb = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB) # 将BGR图像转为RGB图像(convert BGR image to RGB image)
    results = face_detection.process(image_rgb) # 将每一帧图像传给人脸识别模块(pass each frame of the image to the face recognition module)
    if results.detections:   # 如果检测不到人脸那就返回None
        for index, detection in enumerate(results.detections): # 返回人脸索引index(第几张脸)，和关键点的坐标信息(return the face index (which face) and the coordinate information of the keypoints)
            bboxC = detection.location_data.relative_bounding_box # 设置一个边界框，接收所有的框的xywh及关键点信息(set up a bounding box to receive the xywh and keypoint information for all boxes)
            
            # 将边界框的坐标点,宽,高从比例坐标转换成像素坐标(convert the coordinates, width, and height of the bounding box from relative coordinates to pixel coordinates)
            bbox = (int(bboxC.xmin * img_w), int(bboxC.ymin * img_h),  
                   int(bboxC.width * img_w), int(bboxC.height * img_h))
            cv2.rectangle(img, bbox, (0,255,0), 2)  # 在每一帧图像上绘制矩形框(draw a rectangle on each frame of the image)
            x, y, w, h = bbox  # 获取识别框的信息,xy为左上角坐标点(get the information of the recognition box, where xy is the coordinate point of the upper left corner)
            center_x =  int(x + (w/2))
           
            if abs(center_x - img_w/2) < img_w/4:
                if action_finish:
                    start_greet = True
    
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
    
    init()
    start()
    
    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open()                
    AGC.runActionGroup('stand')
    while True:
        ret, img = my_camera.read()
        if ret:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正(distortion correction)
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
