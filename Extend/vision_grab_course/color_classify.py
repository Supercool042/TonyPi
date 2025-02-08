#!/usr/bin/python3
# coding=utf8
# 4.拓展课程学习\10.拓展课程之视觉抓取课程\第3课 色块分拣(4.Advanced Lessons\10.Vision Gripping Lesson\Lesson3 Color Sorting)
import sys
import cv2
import math
import time
import threading
import numpy as np
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.Misc as Misc
import hiwonder.Camera as Camera
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle

# 开合手掌色块分类(open and close hand for color block classification)

debug = False

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
            if contour_area_temp > 50:  # 只有在面积大于50时，最大面积的轮廓才是有效的，以过滤干扰(only contours with an area greater than 50 are considered valid, with the largest area being the effective one to filter out interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the contour with the maximal area)

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

board = rrc.Board()
ctl = Controller(board)

# 初始位置(initial position)
def initMove():
    servo1 = servo_data['servo1']
    ctl.set_bus_servo_pulse(17, 500, 500)
    ctl.set_bus_servo_pulse(18, 500, 500)
    ctl.set_pwm_servo_pulse(1, servo1, 500)
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
    print("ColorClassify Init")
    load_config()
    initMove()
    AGC.runActionGroup('stand_slow')
    time.sleep(1)
    AGC.runActionGroup('squat_down')

robot_is_running = False
# app开始玩法调用(app start program calling)
def start():
    global robot_is_running
    reset()
    robot_is_running = True
    print("ColorClassify Start")

# app停止玩法调用(app stop program calling)
def stop():
    global robot_is_running
    robot_is_running = False
    print("ColorClassify Stop")

# app退出玩法调用(app exit program calling)
def exit():
    global robot_is_running
    robot_is_running = False
    AGC.runActionGroup('stand_slow')
    print("ColorClassify Exit")
    
   
def move():
    global draw_color
    global detect_color
    global action_finish
    
    
    while True:
        if debug:
            return
        if robot_is_running:
            if detect_color != 'None':
                board.set_buzzer(1900, 0.1, 0.9, 1)
                action_finish = False
                
                time.sleep(1)
                if detect_color == 'red':
                    AGC.runActionGroup('grab_right')
                    detect_color = 'None'
                    draw_color = range_rgb["black"]                    
                    action_finish = True
                    
                elif detect_color == 'blue':
                    AGC.runActionGroup('grab_left')
                    detect_color = 'None'
                    draw_color = range_rgb["black"]                    
                    action_finish = True
                    
                elif detect_color == 'green':
                    for i in range(2):
                        ctl.set_pwm_servo_pulse(2, 1300, 300)
                        time.sleep(0.3)
                        ctl.set_pwm_servo_pulse(2, 1700, 300)
                        time.sleep(0.3)
                    ctl.set_pwm_servo_pulse(2, servo_data['servo2'], 500)
                    detect_color = 'None'
                    draw_color = range_rgb["black"]                    
                    action_finish = True
                    
                else:
                    detect_color = 'None'
                    draw_color = range_rgb["black"]                    
                    action_finish = True
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

    if not robot_is_running:
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)

    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0
    
    if action_finish:
        for i in lab_data:
            if i in ['red', 'green', 'blue']:
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
        if max_area > 3500:  # 有找到最大面积(the maximal area is found)
            ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)  # 获取最小外接圆(get the minimum bounding rectangle)
            centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
            centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))
            cv2.circle(img, (centerX, centerY), radius, range_rgb[color_area_max], 2)#画圆(draw circle)

            if color_area_max == 'red':  #红色最大(red is the maximal area)
                color = 1
            elif color_area_max == 'green':  #绿色最大(green is the maximal area)
                color = 2
            elif color_area_max == 'blue':  #蓝色最大(blue is the maximal area)
                color = 3
            else:
                color = 0
            color_list.append(color)

            if len(color_list) == 3:  #多次判断(multiple judgements)
                # 取平均值(take average value)
                color = round(np.mean(np.array(color_list)))
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
            color_list = []
            detect_color = 'None'
            draw_color = range_rgb["black"]
            
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
    return img


if __name__ == '__main__':
    from hiwonder.CalibrationConfig import *
    
    param_data = np.load(calibration_param_path + '.npz')
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
  
    my_camera = Camera.Camera()
    my_camera.camera_open()
    
    init()
    start()
    
    while True:
        ret,img = my_camera.read()
        if ret:
            frame = img.copy()
            frame = cv2.remap(frame.copy(), mapx, mapy, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
    
    
