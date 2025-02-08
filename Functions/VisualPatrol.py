#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import time
import math
import threading
import numpy as np

import hiwonder.Camera as Camera
import hiwonder.Misc as Misc
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle
from hiwonder.common import ColorPicker

'''
    程序功能：视觉巡线(program function: vision line following)

    运行效果：将机器人放置在黑线上，程序启动后，机器人会沿着黑线轨迹前进(running effect: place the robot on the black line. Upon program startup, the robot will move forward along the black line track)

    对应教程文档路径：  TonyPi智能视觉人形机器人\3.AI视觉玩法学习\第4课 视觉巡线(corresponding tutorial file path: TonyPi Intelligent Vision Humanoid Robot\4.Expanded Courses\1.Voice Interaction and Intelligent Transportation(voice module optional)\Lesson4 Vision Line Following)
'''

# 检查 Python 版本是否为 Python 3，若不是则打印提示信息并退出程序(check if the Python version is Python 3, if not, print a prompt message and exit the program)
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

target_color = []

board = rrc.Board()
ctl = Controller(board)

servo_data = None

# 加载配置文件数据(load configuration file data)
def load_config():
    global servo_data
    
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()

# 初始化机器人舵机初始位置(initial servo initialization position of robot)
def initMove():
    ctl.set_pwm_servo_pulse(1, servo_data['servo1'], 500)
    ctl.set_pwm_servo_pulse(2, servo_data['servo2'], 500)

line_center_x = -1

# 变量重置(variable reset)
def reset():
    global line_center_x
    global target_color
    global color_picker
    
    line_center_x = -1
    target_color = []
    color_picker = None

# app初始化调用(app initialization calling)
def init():
    global enter
    print("VisualPatrol Init")
    load_config()
    initMove()
    enter = True

enter = False
running = False
# app开始玩法调用(app start program calling)
def start():
    global running
    running = True
    print("VisualPatrol Start")

# app停止玩法调用(app stop program calling)
def stop():
    global running
    running = False
    reset()
    print("VisualPatrol Stop")

# app退出玩法调用(app exit program calling)
def exit():
    global enter, running
    enter = False
    running = False
    reset()
    AGC.runActionGroup('stand_slow')
    print("VisualPatrol Exit")

color_picker = None
def set_point(point):
    global color_picker, target_color
    x, y = point
    target_color = []
    color_picker = ColorPicker([x, y], 20)

def get_rgb_value():
    if target_color:
        return target_color[1] 
    else:
        return []

threshold = 0.3
def set_threshold(value):
    global threshold
    threshold = value

# 找出面积最大的轮廓(find out the contour with the maximal area)
# 参数为要比较的轮廓的列表(parameter is the list of contour to be compared)
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓(iterate through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 5:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only contours with an area greater than 300 are considered valid; the contour with the largest area is used to filter out interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the contour with the maximal area)

img_centerx = 320
def move():
    global line_center_x
    
    while True:
        if enter:
            if line_center_x != -1:
                if abs(line_center_x - img_centerx) <= 50:
                    AGC.runActionGroup('go_forward')
                elif line_center_x - img_centerx > 50:
                    AGC.runActionGroup('turn_right_small_step')
                elif line_center_x - img_centerx < -50:
                    AGC.runActionGroup('turn_left_small_step')
            else:
                time.sleep(0.01)
        else:
            time.sleep(0.1)

# 运行子线程(run sub-thread)
th = threading.Thread(target=move)
th.daemon = True
th.start()

roi = [ # [ROI, weight]
        (240, 280,  0, 640, 0.1), 
        (340, 380,  0, 640, 0.3), 
        (440, 480,  0, 640, 0.6)
       ]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]

roi_h_list = [roi_h1, roi_h2, roi_h3]

size = (640, 480)
img_w, img_h = None, None
def run(img):
    global line_center_x
    global target_color
    global img_w, img_h
    global color_picker
    
    display_image = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not enter:
        return display_image
    if color_picker is not None and not target_color:  
        target_color, display_image = color_picker(img, display_image)
        if target_color:
            color_picker = None
    elif target_color:    
        frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
                
        centroid_x_sum = 0
        weight_sum = 0
        center_ = []
        n = 0

        #将图像分割成上中下三个部分，这样处理速度会更快，更精确(divide the image into three parts: top, middle, and bottom. This will result in faster and more accurate processing)
        for r in roi:
            roi_h = roi_h_list[n]
            n += 1       
            blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
            frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)
            
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
            dilated[:, 0:160] = 0
            dilated[:, 480:640] = 0        
            cnts = cv2.findContours(dilated , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]#找出所有轮廓(find out all contours)
            cnt_large, area = getAreaMaxContour(cnts)#找到最大面积的轮廓(find out the contour with the maximal area)
            if cnt_large is not None:#如果轮廓不为空(if contour is not NONE)
                rect = cv2.minAreaRect(cnt_large)#最小外接矩形(the minimum bounding rectangle)
                box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点(the four vertices of the minimum bounding rectangle)
                for i in range(4):
                    box[i, 1] = box[i, 1] + (n - 1)*roi_h + roi[0][0]
                    box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
                for i in range(4):                
                    box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))
                    
                cv2.drawContours(display_image, [box], -1, (0,0,255,255), 2)#画出四个点组成的矩形(draw the rectangle formed by four points)
                
                #获取矩形的对角点(get the diagonal points of the rectangle)
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]            
                center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2#中心点(center point)
                cv2.circle(display_image, (int(center_x), int(center_y)), 5, (0,0,255), -1)#画出中心点(draw center point)
                
                center_.append([center_x, center_y])                        
                #按权重不同对上中下三个中心点进行求和(sum the three central points of the top, middle, and bottom sections according to different weights)
                centroid_x_sum += center_x * r[4]
                weight_sum += r[4]

        if weight_sum != 0:
            #求最终得到的中心点(seeking the final obtained center point)
            cv2.circle(display_image, (line_center_x, int(center_y)), 10, (0,255,255), -1)#画出中心点(draw center point)
            if running:
                line_center_x = int(centroid_x_sum / weight_sum)  
        else:
            line_center_x = -1

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
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
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
            cv2.imshow('result_image', Frame)
            cv2.setMouseCallback("result_image", mouse_callback)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
