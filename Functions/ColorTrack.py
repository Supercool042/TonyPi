#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import math
import time
import threading
import numpy as np

import hiwonder.PID as PID
import hiwonder.Misc as Misc
import hiwonder.Camera as Camera
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle


'''
    程序功能：颜色追踪(program function: color tracking)

    运行效果：玩法开启后，手持红色小球进行缓慢移动，机器人头部将随着目标颜色的移动而跟随转动(running effect: after the game mode is activated, move the red ball slowly by hand. The robot's head will follow the movement of the target color)

    对应教程文档路径：  TonyPi智能视觉人形机器人\3.AI视觉玩法学习\第5课 颜色追踪(corresponding tutorial file path: TonyPi Intelligent Vision Humanoid Robot\3.AI Intelligent Game Course\Lesson5 Color Tracking)
'''

# 调试模式标志量(debug mode flag variable)
debug = False

# 检查 Python 版本是否为 Python 3，若不是则打印提示信息并退出程序(check if the Python version is Python 3. If not, print a prompt message and exit the program)
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('red')

# 设置检测颜色(set detected color)
def setTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, (), 'SetTargetTrackingColor')

# 找出面积最大的轮廓(find the contour with the maximal area)
# 参数为要比较的轮廓的列表(parameter is the list of contour to be compared)
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    areaMaxContour = None

    for c in contours:  # 历遍所有轮廓(iterate through all the contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 10:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only contours with an area greater than 300 are considered valid; the contour with the largest area is used to filter out interference)
                areaMaxContour = c

    return areaMaxContour, contour_area_max  # 返回最大的轮廓(return the contour with the maximal area)

board = rrc.Board()
ctl = Controller(board)

lab_data = None
servo_data = None
# 加载配置文件数据(load configuration file data)
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()

x_dis = servo_data['servo2']
y_dis = 1500

# 初始位置(initial position)
def initMove():
    ctl.set_pwm_servo_pulse(1, y_dis, 500)
    ctl.set_pwm_servo_pulse(2, x_dis, 500)
    
    
x_pid = PID.PID(P=0.145, I=0.00, D=0.0007)#pid初始化(pid initialization)
y_pid = PID.PID(P=0.145, I=0.00, D=0.0007)

# 变量重置(variable reset)
def reset():
    global x_dis, y_dis
    global __target_color
       
    x_dis = servo_data['servo2']
    y_dis = 1500
    x_pid.clear()
    y_pid.clear()
    __target_color = ()
    initMove()

# app初始化调用(app initialization calling)
def init():
    print("ColorTrack Init")
    load_config()
    reset()

__isRunning = False
# app开始玩法调用(app start program calling)
def start():
    global __isRunning
    __isRunning = True
    print("ColorTrack Start")

# app停止玩法调用(app stop program calling)
def stop():
    global __isRunning
    __isRunning = False
    reset()
    print("ColorTrack Stop")

# app退出玩法调用(app exit program calling)
def exit():
    global __isRunning
    __isRunning = False
    AGC.runActionGroup('stand_slow')
    print("ColorTrack Exit")


size = (320, 240)
def run(img):
    global x_dis, y_dis
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning or __target_color == ():
        return img

    cv2.line(img, (int(img_w/2 - 10), int(img_h/2)), (int(img_w/2 + 10), int(img_h/2)), (0, 255, 255), 2)
    cv2.line(img, (int(img_w/2), int(img_h/2 - 10)), (int(img_w/2), int(img_h/2 + 10)), (0, 255, 255), 2)

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (5, 5), 5)   
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to the LAB space)
    
    area_max = 0
    areaMaxContour = 0
    for i in lab_data:
        if i in __target_color:
            detect_color = i
            frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(corrosion)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilation)
            if debug:
                cv2.imshow(i, dilated)
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find out contours)
            areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓(find out the contour with the maximal area)
    if area_max > 20:  # 有找到最大面积(find the maximal area)
        (centerX, centerY), radius = cv2.minEnclosingCircle(areaMaxContour) #获取最小外接圆(get the minimum bounding circumcircle)
        centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
        centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, size[0], 0, img_w))
        cv2.circle(img, (int(centerX), int(centerY)), int(radius), range_rgb[detect_color], 2)
        
        use_time = 0    
        
        if abs(centerX - img_w/2.0) < 20: # 移动幅度比较小，则不需要动(if the movement amplitude is small, then no need to move)
            centerX = img_w/2.0 

        x_pid.SetPoint = img_w/2 #设定(set)
        x_pid.update(centerX) #当前(current)
        dx = int(x_pid.output)
        use_time = abs(dx*0.00025)
        x_dis += dx #输出(output)
        
        x_dis = 500 if x_dis < 500 else x_dis          
        x_dis = 2500 if x_dis > 2500 else x_dis


        if abs(centerY - img_h/2.0) < 20: # 移动幅度比较小，则不需要动(if the movement amplitude is small, then no need to move)
            centerY = img_h/2.0  
       
        y_pid.SetPoint = img_h/2
        y_pid.update(centerY)
        dy = int(y_pid.output)
        
        use_time = round(max(use_time, abs(dy*0.00025)), 5)
        
        y_dis += dy
        
        y_dis = 1000 if y_dis < 1000 else y_dis
        y_dis = 2000 if y_dis > 2000 else y_dis    
        
        if not debug:
            ctl.set_pwm_servo_pulse(1, y_dis, use_time*1000)
            ctl.set_pwm_servo_pulse(2, x_dis, use_time*1000)
            time.sleep(use_time)
            
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
    __target_color = ('green')
    
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
