#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import math
import time
import threading
import numpy as np

import hiwonder.Camera as Camera
import hiwonder.Misc as Misc
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle
import hiwonder.apriltag as apriltag



'''
    程序功能：标签识别(program function: tag recognition)

    运行效果：玩法开启后，取出附带的标签卡片，依次对准摄像头模块进行识别。当识别到后，机器人便执行对应的动作。(running effect: "after the game mode is activated, take out the accompanying tag cards and align them with the camera module for recognition one by one. 
    When recognized, the robot will perform the corresponding action)

    对应教程文档路径：  TonyPi智能视觉人形机器人\3.AI视觉玩法学习\第7课 标签识别(corresponding tutorial file path: TonyPi Intelligent Humanoid Robot\3.AI Vision Game Course\Lesson7 Tag Recognition)
'''

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 调试模式标志量(debug mode flag variable)
debug = False

board = rrc.Board()
ctl = Controller(board)

# 初始化机器人舵机初始位置(initialize servo initialization position of robot)
def initMove():
    ctl.set_pwm_servo_pulse(1, 1500, 500)
    ctl.set_pwm_servo_pulse(2, 1500, 500)

    
tag_id = None
__isRunning = False
action_finish = True
# 变量重置(variable reset)
def reset():      
    global tag_id
    global action_finish
    
    tag_id = 0
    action_finish = True
    
# app初始化调用(app initialization calling)
def init():
    print("Apriltag Init")
    initMove()

# app开始玩法调用(app start program calling)
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("Apriltag Start")

# app停止玩法调用(app stop program calling)
def stop():
    global __isRunning
    __isRunning = False
    print("Apriltag Stop")

# app退出玩法调用(app exit program calling)
def exit():
    global __isRunning
    __isRunning = False
    AGC.runActionGroup('stand_slow')
    print("Apriltag Exit")

def move():
    global tag_id
    global action_finish  
    
    while True:
        if debug:
            return
        if __isRunning:
            if tag_id is not None:
                action_finish = False
                time.sleep(0.5)
                if tag_id == 1:#标签ID为1时(when tag ID is 1)
                    AGC.runActionGroup('bow')#鞠躬(bow)
                    tag_id = None
                    time.sleep(1)                  
                    action_finish = True                
                elif tag_id == 2:                    
                    AGC.runActionGroup('stepping')#原地踏步(march in place)
                    tag_id = None
                    time.sleep(1)
                    action_finish = True          
                elif tag_id == 3:                   
                    AGC.runActionGroup('twist')#扭腰(twist your waist)
                    tag_id = None
                    time.sleep(1)
                    action_finish = True
                else:
                    action_finish = True
                    time.sleep(0.01)
            else:
               time.sleep(0.01)
        else:
            time.sleep(0.01)

# 运行子线程(run sub-thread)
th = threading.Thread(target=move)
th.daemon = True
th.start()

# 检测apriltag(detect apriltag)
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
def apriltagDetect(img):   
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, return_image=False)

    if len(detections) != 0:
        for detection in detections:                       
            corners = np.int0(detection.corners)  # 获取四个角点(get four corner points)
            cv2.drawContours(img, [np.array(corners, int)], -1, (0, 255, 255), 2)

            tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family(get tag_family)
            tag_id = int(detection.tag_id)  # 获取tag_id(get tag_id)

            object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])  # 中心点(center point)
            
            object_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # 计算旋转角(calculate rotation angle)
            
            return tag_family, tag_id
            
    return None, None

def run(img):
    global tag_id
    global action_finish
     
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning:
        return img
    
    tag_family, tag_id = apriltagDetect(img) # apriltag检测(apriltag detection)
    
    if tag_id is not None:
        cv2.putText(img, "tag_id: " + str(tag_id), (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
        cv2.putText(img, "tag_family: " + tag_family, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
    else:
        cv2.putText(img, "tag_id: None", (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
        cv2.putText(img, "tag_family: None", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 255, 255], 2)
    
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
    
    debug = False
    if debug:
        print('Debug Mode')
        
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
