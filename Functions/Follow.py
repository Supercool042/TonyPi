#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import time
import math
import threading
import numpy as np
import pandas as pd

import hiwonder.PID as PID
import hiwonder.Camera as Camera
import hiwonder.Misc as Misc
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle
from CameraCalibration.CalibrationConfig import *

'''
    程序功能：物体追踪(program function: object tracking)

    运行效果：   玩法开启后，手持红色海绵方块或者将方块置于可以移动的载体上进行缓慢移动，
                TonyPi 机器人将随着目标颜色的移动而移动。(running effect: after the game mode is activated, move the red sponge block slowly by hand or place the block on a movable carrier. The TonyPi robot will move along with the movement of the target color)

                    
    对应教程文档路径：  TonyPi智能视觉人形机器人\4.拓展课程学习\1.语音交互及智能搬运课程（语音模块选配）\第6课 物体追踪(corresponding tutorial file path: TonyPi Intelligent Vision Humanoid Robot\4.Expanded Courses\1.Voice Interaction and Intelligent Transportation(voice module optional)\Lesson6 Object Tracking)
'''

# 调试模式标志量(debug mode flag variable)
debug = False

# 检查 Python 版本是否为 Python 3，若不是则打印提示信息并退出程序(check if the Python version is Python 3. If not, print a prompt message and exit the program)
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

#加载参数(load parameters)
param_data = np.load(calibration_param_path + '.npz')

#获取参数(get parameters)
mtx = param_data['mtx_array']
dist = param_data['dist_array']
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

lab_data = None
servo_data = None
# 加载配置文件数据(load configuration file data)
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()

__target_color = ('green')

# 设置检测颜色(set detected color)
def setBallTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, ())

# 初始化机器人底层驱动(initialize robot underlying drivers)
board = rrc.Board()
ctl = Controller(board)

# 初始位置(initial position)
def initMove():
    ctl.set_pwm_servo_pulse(1, servo_data['servo1'], 500)
    ctl.set_pwm_servo_pulse(2, servo_data['servo2'], 500)

# 初始化舵机移动水平方向和垂直方向的步长(initialize the step size for servo movement in the horizontal and vertical directions)
d_x = 20
d_y = 20

step = 1

# 设置舵机位置(set servo position)
x_dis = servo_data['servo2']
y_dis = servo_data['servo1']

# 初始化物体中心点的xy坐标(initialize the xy coordinates of the object's center point)
centerX, centerY = -2, -2

# 初始化 PID控制器(initialize PID controller)
x_pid = PID.PID(P=0.145, I=0.0, D=0.0007)#pid初始化(pid initialization)
y_pid = PID.PID(P=0.145, I=0.0, D=0.0007)

# 变量重置(variable reset)
def reset():
    global d_x, d_y
    global step
    global x_dis, y_dis
    global __target_color
    global centerX, centerY

    d_x = 20
    d_y = 20
    step = 1
    x_pid.clear()
    y_pid.clear()
    x_dis = servo_data['servo2']
    y_dis = servo_data['servo1']
    __target_color = ()
    centerX, centerY = -2, -2

# app初始化调用(app initialization calling)
def init():
    print("Follow Init")
    load_config()
    initMove()

__isRunning = False
# app开始玩法调用(app start program calling)
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("Follow Start")

# app停止玩法调用(app stop program calling)
def stop():
    global __isRunning
    __isRunning = False
    print("Follow Stop")

# app退出玩法调用(app exit program calling)
def exit():
    global __isRunning
    __isRunning = False
    AGC.runActionGroup('stand_slow')
    print("Follow Exit")

# 找出面积最大的轮廓(find out the contour with the maximal area)
# 参数为要比较的轮廓的列表(parameter is the list of contour to be compared)
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    areaMaxContour = None

    for c in contours:  # 历遍所有轮廓(iterate through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 100:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only contours with an area greater than 300 are considered valid; the contour with the largest area is used to filter out interference)
                areaMaxContour = c

    return areaMaxContour, contour_area_max  # 返回最大的轮廓(return the contour with the maximal area)


CENTER_X = 320
circle_radius = 0
#执行动作组(execute action group)
def move():
    
    while True:
        if __isRunning:
            if centerX >= 0:
                if centerX - CENTER_X > 100 or x_dis - servo_data['servo2'] < -80:  # 不在中心，根据方向让机器人转向一步(if not in the center, instruct the robot to turn one step according to the direction)
                    AGC.runActionGroup('turn_right_small_step')
                elif centerX - CENTER_X < -100 or x_dis - servo_data['servo2'] > 80:
                    AGC.runActionGroup('turn_left_small_step')                        
                elif 100 > circle_radius > 0:
                    AGC.runActionGroup('go_forward')
                elif 180 < circle_radius:
                    AGC.runActionGroup('back_fast')
            else:
                time.sleep(0.01)
        else:
            time.sleep(0.01)

#启动动作的线程(start the thread for executing actions)
th = threading.Thread(target=move)
th.daemon = True
th.start()

radius_data = []
size = (320, 240)
def run(img):
    global radius_data
    global x_dis, y_dis
    global centerX, centerY, circle_radius
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning or __target_color == ():
        return img
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
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
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find out contour)
            areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓(find out the contour with the maximal area)
    if areaMaxContour is not None and area_max > 100:  # 有找到最大面积(the maximal area is found)
        rect = cv2.minAreaRect(areaMaxContour)#最小外接矩形(the minimum bounding rectangle)
        box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点(the four vertices of the minimum bounding rectangle)
        for j in range(4):
            box[j, 0] = int(Misc.map(box[j, 0], 0, size[0], 0, img_w))
            box[j, 1] = int(Misc.map(box[j, 1], 0, size[1], 0, img_h))

        cv2.drawContours(img, [box], -1, (0,255,255), 2)#画出四个点组成的矩形(draw the rectangle formed by the four points)
        #获取矩形的对角点(get the diagonal points of the rectangle)
        ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
        pt3_x, pt3_y = box[2, 0], box[2, 1]
        radius = abs(ptime_start_x - pt3_x)
        centerX, centerY = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2)#中心点(center point)
        cv2.circle(img, (centerX, centerY), 5, (0, 255, 255), -1)#画出中心点(draw the center point)
          
        use_time = 0       
        
        radius_data.append(radius)
        data = pd.DataFrame(radius_data)
        data_ = data.copy()
        u = data_.mean()  # 计算均值(calculate the average value)
        std = data_.std()  # 计算标准差(calculate standard deviation)

        data_c = data[np.abs(data - u) <= std]
        circle_radius = round(data_c.mean()[0], 1)
        if len(radius_data) == 5:
            radius_data.remove(radius_data[0])

        # 设置水平舵机位置PID的目标值为图像宽度的一半(set the target value of the PID for the horizontal servo position to half of the image width)
        x_pid.SetPoint = img_w/2 #设定(set)
        x_pid.update(centerX) #当前(current)
        dx = int(x_pid.output)
        # 计算使用时间(calculate the usage time)
        use_time = abs(dx*0.00025)
        x_dis += dx #输出(output)
        
        # 将控制头部水平移动的舵机位置限制在预设范围内(limit the position of the servo controlling horizontal head movement within the preset range)
        x_dis = servo_data['servo2'] - 400 if x_dis < servo_data['servo2'] - 400 else x_dis          
        x_dis = servo_data['servo2'] + 400 if x_dis > servo_data['servo2'] + 400 else x_dis

        # 设置垂直舵机位置PID的目标值为图像高度的一半(set the target value of the PID for the vertical servo position to half of the image height)
        y_pid.SetPoint = img_h/2
        y_pid.update(centerY)
        dy = int(y_pid.output)
        # 计算使用时间(calculate the usage time)
        use_time = round(max(use_time, abs(dy*0.00025)), 5)
        y_dis += dy
        
        # 将控制头部垂直移动的舵机位置限制在预设范围内(limit the position of the servo controlling vertical head movement within the preset range)
        y_dis = servo_data['servo1'] if y_dis < servo_data['servo1'] else y_dis
        y_dis = 2000 if y_dis > 2000 else y_dis    
        
        ctl.set_pwm_servo_pulse(1, y_dis, use_time*1000)
        ctl.set_pwm_servo_pulse(2, x_dis, use_time*1000)

        time.sleep(use_time)
    else:
        centerX, centerY = -1, -1
   
    

    return img

if __name__ == '__main__':
    init()
    start()
    __target_color = ('red')
    
    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open() 
    AGC.runActionGroup('stand')
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
