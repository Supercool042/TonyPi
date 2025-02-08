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
from hiwonder.common import ColorPicker

'''
    程序功能：颜色追踪(program function: color tracking)

    运行效果：玩法开启后，手持红色小球进行缓慢移动，机器人头部将随着目标颜色的移动而跟随转动(running effect: after the game mode is activated, move the red ball slowly by hand. The robot's head will follow the movement of the target color)

    对应教程文档路径：  TonyPi智能视觉人形机器人\3.AI视觉玩法学习\第5课 颜色追踪(corresponding tutorial file path: TonyPi Intelligent Vision Humanoid Robot\3.AI Intelligent Game Course\Lesson5 Color Tracking)
'''

# 调试模式标志量(debug mode flag variable)
debug = False
color_picker = None
# 检查 Python 版本是否为 Python 3，若不是则打印提示信息并退出程序(check if the Python version is Python 3. If not, print a prompt message and exit the program)
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

target_color = []

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

servo_data = None
# 加载配置文件数据(load configuration file data)
def load_config():
    global servo_data
    
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
    global target_color
    global color_picker
       
    x_dis = servo_data['servo2']
    y_dis = 1500
    x_pid.clear()
    y_pid.clear()
    target_color = []
    color_picker = None

# app初始化调用(app initialization calling)
def init():
    global enter
    print("ColorTrack Init")
    initMove()
    # load_config()
    enter = True

enter = False
running = False
# app开始玩法调用(app start program calling)
def start():
    global running
    running = True
    print("ColorTrack Start")

# app停止玩法调用(app stop program calling)
def stop():
    global running
    running = False
    reset()
    
    initMove()
    print("ColorTrack Stop")

# app退出玩法调用(app exit program calling)
def exit():
    global enter, running
    enter = False
    running = False
    reset()
    AGC.runActionGroup('stand_slow')
    print("ColorTrack Exit")

color_picker = None
def set_point(point):
    global color_picker, target_color
    x, y = point
    target_color = []
    color_picker = ColorPicker([x, y], 20)

    return (True, (), 'set_point') 

def get_rgb_value():
    if target_color:
        return target_color[1] 
    else:
        return []

threshold = 0.3
def set_threshold(value):
    global threshold
    threshold = value
    return (True, (), 'set_threshold')

size = (320, 240)
img_w, img_h = None, None
def run(img):
    global x_dis, y_dis, target_color
    global img_w, img_h
    global color_picker

    display_image = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not enter:
        return display_image

    cv2.line(display_image, (int(img_w/2 - 10), int(img_h/2)), (int(img_w/2 + 10), int(img_h/2)), (0, 255, 255), 2)
    cv2.line(display_image, (int(img_w/2), int(img_h/2 - 10)), (int(img_w/2), int(img_h/2 + 10)), (0, 255, 255), 2)

    if color_picker is not None and not target_color:  
        target_color, display_image = color_picker(img, display_image)
        if target_color:
            color_picker = None
    elif target_color:
        frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (5, 5), 5)   
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to the LAB space)
        
        min_color = [int(target_color[0][0] - 50 * threshold * 2),
                     int(target_color[0][1] - 50 * threshold),
                     int(target_color[0][2] - 50 * threshold)]
        max_color = [int(target_color[0][0] + 50 * threshold * 2),
                     int(target_color[0][1] + 50 * threshold),
                     int(target_color[0][2] + 50 * threshold)]
        #对原图像和掩模进行位运算(perform bitwise operation to the original image and mask)
        frame_mask = cv2.inRange(frame_lab, tuple(min_color), tuple(max_color))
        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(corrosion)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilation)
        if debug:
            cv2.imshow('dilate', dilated)
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find out contours)
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓(find out the contour with the maximal area)
        if areaMaxContour is not None:  # 有找到最大面积(find the maximal area)
            (centerX, centerY), radius = cv2.minEnclosingCircle(areaMaxContour) #获取最小外接圆(get the minimum bounding circumcircle)
            centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
            centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))
            cv2.circle(display_image, (int(centerX), int(centerY)), int(radius), (0, 255, 255), 2)
            if running: 
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
            
    return display_image

if __name__ == '__main__':
    from CameraCalibration.CalibrationConfig import *
    def mouse_callback(event, x, y, flags, param):
        global color_picker
        if event == cv2.EVENT_LBUTTONDOWN:
            color_picker = ColorPicker([x/img_w, y/img_h], 20)
            # print(x, y)
        elif event == cv2.EVENT_RBUTTONDOWN:
            color_picker = None
            init()
            reset()    
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
            cv2.imshow('result_image', Frame)
            cv2.setMouseCallback("result_image", mouse_callback) 
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
