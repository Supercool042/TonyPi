#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import math
import time
import threading
import numpy as np

import hiwonder.TTS as TTS
import hiwonder.Camera as Camera
import hiwonder.Misc as Misc
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle

'''
    程序功能：颜色识别播报(program function: color recognition announcement)

    运行效果：   当听到机器人播报“我准备好了”的语音后，可将不同颜色的物品依次置于摄像头前。
                当识别到后，便会播报该颜色对应的词条，并且播报音色各不相同。(running effect: when you hear the robot say 'I'm ready,' you can place items of different colors in front of the camera one by one. 
                Once recognized, it will announce the corresponding word for that color, with each color having a different tone)
                    

    对应教程文档路径：  TonyPi智能视觉人形机器人\4.拓展课程学习\1.语音交互及智能搬运课程（语音模块选配）\第3课 颜色识别播报(corresponding tutorial file path: TonyPi Intelligent Vision Humanoid Robot\4.Expanded Courses\1.Voice Interaction and Intelligent Transportation(voice module optional)\Lesson3 Color Recognition Announcement)
'''

# 检查 Python 版本是否为 Python 3，若不是则打印提示信息并退出程序(check if the Python version is Python 3. If not, print a prompt message and exit the program)
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 调试模式标志量(debug mode flag variable)
debug = False

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


# 找出面积最大的轮廓(find the contour with the maximal area)
# 参数为要比较的轮廓的列表(parameter is the list of contour to be compared)
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    areaMaxContour = None

    for c in contours:  # 历遍所有轮廓(iterate through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 50:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only contours with an area greater than 300 are considered valid; the contour with the largest area is used to filter out interference)
                areaMaxContour = c

    return areaMaxContour, contour_area_max  # 返回最大的轮廓(return the contour with the maximal area)

tts = TTS.TTS()

# 初始化机器人底层驱动(initialize robot underlying drivers)
board = rrc.Board()
ctl = Controller(board)

# 初始化机器人舵机初始位置(initialize tge servo initialization position of robot)
def initMove():
  ctl.set_pwm_servo_pulse(1, 1500, 500)
  ctl.set_pwm_servo_pulse(2, servo_data['servo2'], 500)

color_list = []
detect_color = 'None'
action_finish = True
draw_color = range_rgb["black"]
# 变量重置(variable reset)
def reset():
    global draw_color
    global color_list
    global detect_color
    global action_finish
    
    color_list = []
    detect_color = 'None'
    action_finish = True
    draw_color = range_rgb["black"]
    
# app初始化调用(app initialization calling)
def init():
    print("ColorDetect Init")
    load_config()
    initMove()
        
__isRunning = False
# app开始玩法调用(app start program calling)
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorDetect Start")

# app停止玩法调用(app stop program calling)
def stop():
    global __isRunning
    __isRunning = False
    print("ColorDetect Stop")

# app退出玩法调用(app exit program calling)
def exit():
    global __isRunning
    __isRunning = False
    AGC.runActionGroup('stand_slow')
    print("ColorDetect Exit")

color_dict = {'red': '红色',
        'green': '绿色',
        'blue': '蓝色'
        }

def move():
    global draw_color
    global detect_color
    global action_finish
    
    speaker = '[m3]'
    while True:
        if debug:
            return
        if __isRunning:
            # 根据识别到的颜色，采用不同的音色去播报(based on the recognized color, use different tones to announce)
            if detect_color != 'None':
                if detect_color == 'red':
                    speaker = '[m3]'
                elif detect_color == 'green':
                    speaker = '[m51]'
                elif detect_color == 'blue':
                    speaker = '[m55]'
                tts.TTSModuleSpeak(speaker, '检测到' + color_dict[detect_color])
                time.sleep(2)
                detect_color = 'None'
                draw_color = range_rgb["black"]
            else:
                time.sleep(0.01)
        else:
            time.sleep(0.01)

# 运行子线程(run sub-thread)
th = threading.Thread(target=move)
th.daemon = True
th.start()

size = (320, 240)
def run(img):
    global draw_color
    global color_list
    global detect_color
    global action_finish
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning:
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to the LAB space)

    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0
    
    if action_finish:
        for i in lab_data:
            if i != 'black' and i != 'white':
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
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓(find out contours)
                areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓(find out the contour with the maximal area)
                if areaMaxContour is not None:
                    if area_max > max_area:#找最大面积(find the maximal area)
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if max_area > 200:  # 有找到最大面积(the maximal area is found)
            ((object_center_x, object_center_y), radius) = cv2.minEnclosingCircle(areaMaxContour_max)  # 获取最小外接圆(get the minimum bounding circumcircle)
            object_center_x = int(Misc.map(object_center_x, 0, size[0], 0, img_w))
            object_center_y = int(Misc.map(object_center_y, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))            
            cv2.circle(img, (object_center_x, object_center_y), radius, range_rgb[color_area_max], 2)#画圆(draw circle)

            if color_area_max == 'red':  #红色最大(red is the maximal area)
                color = 1
            elif color_area_max == 'green':  #绿色最大(green is the maximal area)
                color = 2
            elif color_area_max == 'blue':  #蓝色最大(blue is the maximal area)
                color = 3
            else:
                color = 0
            color_list.append(color)

            if len(color_list) == 5:  #多次判断(multiple judgement)
                # 取平均值(take average value)
                color = int(round(np.mean(np.array(color_list))))
                color_list = []
                if color == 1:
                    detect_color = 'red'
                    draw_color = range_rgb["red"]
                elif color == 2:
                    detect_color = 'green'
                    draw_color = range_rgb["green"]
                elif color == 3:
                    detect_color = 'blue'
                    draw_color = range_rgb["blue"]
                else:
                    detect_color = 'None'
                    draw_color = range_rgb["black"]               
        else:
            detect_color = 'None'
            draw_color = range_rgb["black"]
            
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
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
    tts.TTSModuleSpeak('[h0][v10][m3]', '我准备好了')
    while True:
        ret, img = my_camera.read()
        if img is not None:
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
