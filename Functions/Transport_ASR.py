#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import time
import math
import threading
import numpy as np

import hiwonder.TTS as TTS
import hiwonder.ASR as ASR
import hiwonder.apriltag as apriltag
import hiwonder.Camera as Camera
import hiwonder.Misc as Misc
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle

'''
    程序功能：语音控制搬运(program function: voice control transportation)

    运行效果：   程序启动之后，TonyPi 机器人的头部会朝下，并反馈“准备就绪”的语音信息。此时，
                对 TonyPi 进行唤醒（说出唤醒词“开始”）。当模块上的“STA 指示灯”变为蓝色常亮再
                向机器人下达搬运指令。例如，我们说“搬运红色”，机器人接收后会反馈“收到，开始寻
                找红色”的语音信息，然后开始寻找并搬运红色物品，并放置到对应的 AprilTag 区域
     (running effect: After the program starts, the head of the TonyPi robot will be facing downwards and it will provide an auditory feedback saying "Ready". At this point, you can wake up TonyPi by saying the wake-up word "Start". 
     When the "STA indicator light" on the module turns solid blue, you can issue the transportation command to the robot. 
     For example, if you say "Transport red", the robot will acknowledge by saying "Received, starting to search for red", and then proceed to search for and transport the red item, placing it in the corresponding AprilTag area)               

    对应教程文档路径：  TonyPi智能视觉人形机器人\4.拓展课程学习\1.语音交互及智能搬运课程（语音模块选配）\第5课 语音控制搬运(corresponding tutorial file path: TonyPi Intelligent Vision Humanoid Robot\4.Expanded Courses\1.Voice Interaction and Intelligent Transportation(voice module optional)\Lesson5 Voice Control Transportation)
'''


if __name__ == '__main__':
    from CameraCalibration.CalibrationConfig import *
else:
    from Functions.CameraCalibration.CalibrationConfig import *

# 检查 Python 版本是否为 Python 3，若不是则打印提示信息并退出程序(check if the Python version is Python 3, if not, print a prompt message and exit the program)
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

# 搬运用到的动作组名称(the name of the action group used for transportation)
go_forward = 'go_forward'
back = 'back_fast'
turn_left = 'turn_left_small_step'
turn_right = 'turn_right_small_step'
left_move = 'left_move'
right_move = 'right_move'
left_move_large = 'left_move_30'
right_move_large = 'right_move_30'

try:
    asr = ASR.ASR()
    tts = TTS.TTS()
except:
    print('传感器初始化出错')

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# 颜色对应的tag编号(color corresponds to the tag number)
color_tag = {'red': 1,
             'green': 2,
             'blue': 3
             }

lab_data = None
servo_data = None
# 加载配置文件数据(load configuration file data)
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()

# 初始化机器人底层驱动(initialize robot underlying driver)
board = rrc.Board()
ctl = Controller(board)

# 初始位置(initial position)
def initMove():
    ctl.set_pwm_servo_pulse(1, servo_data['servo1'], 500)
    ctl.set_pwm_servo_pulse(2, servo_data['servo2'], 500)


# 初始化舵机移动水平方向和垂直方向的步长(initialize the step size for moving the servo motor in the horizontal and vertical directions)
d_x = 15
d_y = 15

step = 1

# 初始化开始计时的时间戳(initialize the timestamp for starting the timer)
time_start = 0

# 设置舵机位置(set servo position)
x_dis = servo_data['servo2']
y_dis = servo_data['servo1']

# 初始化机器人上一步的状态(initialize the previous state of the robot)
last_status = ''

# 初始化开始计时的标志量(initialize the flag variable for starting the timer)
start_count = True

head_turn = 'left_right'
object_center_x, object_center_y, object_angle = -2, -2, 0
# 变量重置(variable reset)
def reset():
    global time_start
    global d_x, d_y
    global last_status
    global start_count
    global step, head_turn
    global x_dis, y_dis
    global object_center_x, object_center_y, object_angle

    d_x = 15
    d_y = 15
    step = 1
    time_start = 0
    x_dis = servo_data['servo2']
    y_dis = servo_data['servo1']
    last_status = ''
    start_count = True
    head_turn = 'left_right'
    object_center_x, object_center_y, object_angle = -2, -2, 0
    
# app初始化调用(app initialization calling)
def init():
    print("Transport Init")
    load_config()
    asr_init()
    initMove()

__isRunning = False
# app开始玩法调用(app start program calling)
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("Transport Start")

# app停止玩法调用(app stop program calling)
def stop():
    global __isRunning
    __isRunning = False
    print("Transport Stop")

# app退出玩法调用(app exit program calling)
def exit():
    global __isRunning
    __isRunning = False
    AGC.runActionGroup('stand_slow')
    print("Transport Exit")

# 找出面积最大的轮廓(find out the contour with the maximal area)
# 参数为要比较的轮廓的列表(the parameter is the list of contour to be compared)
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓(iterate through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate the contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only contours with an area greater than 300 are considered valid, where the largest contour within this threshold is selected to filter out noise)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the contour with the maximal area)

# 红绿蓝颜色识别(red-green-blur color recognition)
size = (320, 240)
def colorDetect(img, _color_):
    img_h, img_w = img.shape[:2]
    
    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)
    
    center_max_distance = pow(img_w/2, 2) + pow(img_h, 2)
    color, center_x, center_y, angle = 'None', -1, -1, 0
    for i in lab_data:
        if i == _color_:
            frame_mask = cv2.inRange(frame_lab,
                                     (lab_data[i]['min'][0],
                                      lab_data[i]['min'][1],
                                      lab_data[i]['min'][2]),
                                     (lab_data[i]['max'][0],
                                      lab_data[i]['max'][1],
                                      lab_data[i]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(corrosion)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilation)
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find out contours)
            areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓(find out the contour with the maximal area)
            
            if area_max > 500:  # 有找到最大面积(the maximal area is found)
                rect = cv2.minAreaRect(areaMaxContour)#最小外接矩形(the minimal bounding rectangle)
                angle_ = rect[2]
        
                box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点(the four vertices of the minimum bounding rectangle)
                for j in range(4):
                    box[j, 0] = int(Misc.map(box[j, 0], 0, size[0], 0, img_w))
                    box[j, 1] = int(Misc.map(box[j, 1], 0, size[1], 0, img_h))

                cv2.drawContours(img, [box], -1, (0,255,255), 2)#画出四个点组成的矩形(draw a rectangle formed by four points)
            
                #获取矩形的对角点(get the diagonal points of the rectangle)
                ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]            
                center_x_, center_y_ = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2)#中心点(center point)
                cv2.circle(img, (center_x_, center_y_), 5, (0, 255, 255), -1)#画出中心点(draw center point)
                
                distance = pow(center_x_ - img_w/2, 2) + pow(center_y_ - img_h, 2)
                if distance < center_max_distance:  # 寻找距离最近的物体来搬运(find the nearest object to transport)
                    center_max_distance = distance
                    color = i
                    center_x, center_y, angle = center_x_, center_y_, angle_
                    
    return color, center_x, center_y, angle

# 检测apriltag(detect pariltag)
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
def apriltagDetect(img):   
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    detections = detector.detect(gray, return_image=False)
    tag_1 = [-1, -1, 0]
    tag_2 = [-1, -1, 0]
    tag_3 = [-1, -1, 0]

    if len(detections) != 0:
        for detection in detections:                       
            corners = np.rint(detection.corners)  # 获取四个角点(get four corner points)
            cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)

            tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family(get tag_family)
            tag_id = str(detection.tag_id)  # 获取tag_id(get tag_id)

            object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])  # 中心点(center point)
            cv2.circle(frame, (object_center_x, object_center_y), 5, (0, 255, 255), -1)
            
            object_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # 计算旋转角(calculate rotation point)
            
            if tag_family == 'tag36h11':
                if tag_id == '1':
                    tag_1 = [object_center_x, object_center_y, object_angle]
                elif tag_id == '2':
                    tag_2 = [object_center_x, object_center_y, object_angle]
                elif tag_id == '3':
                    tag_3 = [object_center_x, object_center_y, object_angle]
        
    return tag_1, tag_2, tag_3

# 通过其他apriltag判断目标apriltag位置(determine the position of the target apriltag using other apriltags)
# apriltag摆放位置：红(tag36h11_1)，绿(tag36h11_2)，蓝(tag36h11_3)(apriltag placement: red(tag36h11_1), green(tag36h11_2), blue(tag36h11_3))
def getTurn(tag_id, tag_data):
    tag_1 = tag_data[0]
    tag_2 = tag_data[1]
    tag_3 = tag_data[2]

    if tag_id == 1:  # 目标apriltag为1(target apriltag is 1)
        if tag_2[0] == -1:  # 没有检测到apriltag 2(apriltag 2 is not detected)
            if tag_3[0] != -1:  # 检测到apriltag 3， 则apriltag 1在apriltag 3左边，所以左转(apriltag 3 is not detected, then apriltag 1 is to the left of apriltag 3, so turn left)
                return 'left'
        else:  # 检测到apriltag 2，则则apriltag 1在apriltag 2左边，所以左转(apriltag 2 is  detected, then apriltag 1 is to the left of apriltag 2, so turn left)
            return 'left'
    elif tag_id == 2:
        if tag_1[0] == -1:
            if tag_3[0] != -1:
                return 'left'
        else:
            return 'right'
    elif tag_id == 3:
        if tag_1[0] == -1:
            if tag_2[0] != -1:
                return 'right'
        else:
            return 'right'

    return 'None'


# 转向方向(turning direction)
turn = 'None'

# 画面中心x坐标(the x-coordinate of the center of the screen)
CENTER_X = 350

# True为搬运阶段，False为放置变量(True is the transportation state, False is the placement variable)
find_box = True

# 执行前进动作组的次数(the number of times the forward action group is executed)
go_step = 3

# 舵机锁，用来固定搬运时手部舵机的位置(servo lock, used to fix the position of the hand servo during transportation)
lock_servos = ''

stop_detect = False
object_color = None
haved_find_tag = False
head_turn = 'left_right'
color_list = ['red', 'green', 'blue']
color_center_x, color_center_y = -1, -1
LOCK_SERVOS = {'6': 650, '7': 850, '8': 0, '14': 350, '15': 150, '16': 1000}

#执行动作组(execute action group)
def move(): 
    global d_x
    global d_y
    global step
    global turn
    global x_dis
    global y_dis
    global go_step
    global lock_servos
    global start_count
    global find_box
    global head_turn
    global time_start
    global color_list
    global stop_detect
    global haved_find_tag    
    global object_color

    while True:
        if __isRunning:
            if object_center_x == -3:  # -3表示放置阶段，且没有找到目标apriltag，但是找到其他apriltag(-3 indicates the placement phase, and the target AprilTag was not found, but other AprilTags were detected)
                # 根据其他arpiltag的相对位置来判断转向(determine the turning direction based on the relative positions of other apriltags)
                if turn == 'left':
                    AGC.runActionGroup(turn_left, lock_servos=lock_servos)
                elif turn == 'right':
                    AGC.runActionGroup(turn_right, lock_servos=lock_servos)
            elif haved_find_tag and object_center_x == -1:  # 如果转头过程找到了apriltag，且头回中时apriltag不在视野中(if an AprilTag is found during the turning process, but it is not in view when the head returns to center position)
                # 根据头转向来判断apriltag位置(determine the position of the AprilTag based on the head orientation)
                if x_dis > servo_data['servo2']:                    
                    AGC.runActionGroup(turn_left, lock_servos=lock_servos)
                elif x_dis < servo_data['servo2']:
                    AGC.runActionGroup(turn_right, lock_servos=lock_servos)
                else:
                    haved_find_tag = False
            elif object_center_x >= 0:  # 如果找到目标(if the target is found)
                if not find_box:  # 如果是放置阶段(if it isi the placement stage)
                    if color_center_y > 350:  # 搬运过程中，当太靠近其他物体时，需要绕开(when too close to other objects during the transportation process, it is necessary to navigate around them)
                        if (color_center_x - CENTER_X) > 80:
                            AGC.runActionGroup(go_forward, lock_servos=lock_servos)
                        elif (color_center_x > CENTER_X and object_center_x >= CENTER_X) or (color_center_x <= CENTER_X and object_center_x >= CENTER_X):
                            AGC.runActionGroup(right_move_large, lock_servos=lock_servos)
                            #time.sleep(0.2)
                        elif (color_center_x > CENTER_X and object_center_x < CENTER_X) or (color_center_x <= CENTER_X and object_center_x < CENTER_X):
                            AGC.runActionGroup(left_move_large, lock_servos=lock_servos)
                            #time.sleep(0.2)

                # 如果是转头阶段找到物体， 头回中(if an object is found during the head turning phase, return the head to the center position)
                if x_dis != servo_data['servo2'] and not haved_find_tag:
                    # 重置转头寻找的相关变量(reset the relevant variables for head scanning)
                    head_turn == 'left_right'
                    start_count = True
                    d_x, d_y = 15, 15
                    haved_find_tag = True
                    
                    # 头回中(the head returns to the center)
                    ctl.set_pwm_servo_pulse(1, servo_data['servo1'], 500)
                    ctl.set_pwm_servo_pulse(2, servo_data['servo2'], 500)
                    time.sleep(0.6)
                elif step == 1:  # 左右调整，保持在正中(adjust left or right to maintain in the center)
                    x_dis = servo_data['servo2']
                    y_dis = servo_data['servo1']                   
                    turn = ''
                    haved_find_tag = False
                    
                    if (object_center_x - CENTER_X) > 170 and object_center_y > 330:
                        AGC.runActionGroup(back, lock_servos=lock_servos)   
                    elif object_center_x - CENTER_X > 80:  # 不在中心，根据方向让机器人转向一步(not in the center, turn the robot one step according to the direction)
                        AGC.runActionGroup(turn_right, lock_servos=lock_servos)
                    elif object_center_x - CENTER_X < -80:
                        AGC.runActionGroup(turn_left, lock_servos=lock_servos)                        
                    elif 0 < object_center_y <= 250:
                        AGC.runActionGroup(go_forward, lock_servos=lock_servos)
                    else:
                        step = 2
                elif step == 2:  # 接近物体(approach the object)
                    if 330 < object_center_y:
                        AGC.runActionGroup(back, lock_servos=lock_servos)
                    if find_box:
                        if object_center_x - CENTER_X > 150:  
                            AGC.runActionGroup(right_move_large, lock_servos=lock_servos)
                        elif object_center_x - CENTER_X < -150:
                            AGC.runActionGroup(left_move_large, lock_servos=lock_servos)                        
                        elif -10 > object_angle > -45:# 不在中心，根据方向让机器人转向一步(not in the center, turn the robot one step according to the direction)
                            AGC.runActionGroup(turn_left, lock_servos=lock_servos)
                        elif -80 < object_angle <= -45:
                            AGC.runActionGroup(turn_right, lock_servos=lock_servos)
                        elif object_center_x - CENTER_X > 40:  # 不在中心，根据位置让机器人移动一步(not in the center, turn the robot one step according to the position)
                            AGC.runActionGroup(right_move_large, lock_servos=lock_servos)
                        elif object_center_x - CENTER_X < -40:
                            AGC.runActionGroup(left_move_large, lock_servos=lock_servos)
                        else:
                            step = 3
                    else:                        
                        if object_center_x - CENTER_X > 150:  # 不在中心，根据位置让机器人移动一步(not in the center, turn the robot one step according to the position)
                            AGC.runActionGroup(right_move_large, lock_servos=lock_servos)
                        elif object_center_x - CENTER_X < -150:
                            AGC.runActionGroup(left_move_large, lock_servos=lock_servos)                        
                        elif object_angle < -5:# 不在中心，根据方向让机器人转向一步(not in the center, turn the robot one step according to the direction)
                            AGC.runActionGroup(turn_left, lock_servos=lock_servos)
                        elif 5 < object_angle:
                            AGC.runActionGroup(turn_right, lock_servos=lock_servos)
                        elif object_center_x - CENTER_X > 40:  
                            AGC.runActionGroup(right_move_large, lock_servos=lock_servos)
                        elif object_center_x - CENTER_X < -40:
                            AGC.runActionGroup(left_move_large, lock_servos=lock_servos)
                        else:
                            step = 3
                elif step == 3:
                    if 340 < object_center_y:
                        AGC.runActionGroup(back, lock_servos=lock_servos)
                    elif 0 < object_center_y <= 250:
                        AGC.runActionGroup(go_forward, lock_servos=lock_servos)
                    elif object_center_x - CENTER_X >= 40:  # 不在中心，根据位置让机器人移动一步(not in the center, turn the robot one step according to the position)
                        AGC.runActionGroup(right_move_large, lock_servos=lock_servos)
                    elif object_center_x - CENTER_X <= -40:
                        AGC.runActionGroup(left_move_large, lock_servos=lock_servos) 
                    elif 20 <= object_center_x - CENTER_X < 40:  # 不在中心，根据位置让机器人移动一步(not in the center, turn the robot one step according to the position)
                        AGC.runActionGroup(right_move, lock_servos=lock_servos)
                    elif -40 < object_center_x - CENTER_X < -20:                      
                        AGC.runActionGroup(left_move, lock_servos=lock_servos)
                    else:
                        step = 4 
                elif step == 4:  #靠近物体(approach the object)
                    if 300 < object_center_y <= 340:
                        AGC.runActionGroup('go_forward_one_step', lock_servos=lock_servos)
                        time.sleep(0.2)
                    elif 0 <= object_center_y <= 300:
                        AGC.runActionGroup(go_forward, lock_servos=lock_servos)
                    else:
                        if object_center_y >= 370:
                            go_step = 2
                        else:
                            go_step = 3
                        if abs(object_center_x - CENTER_X) <= 20:
                            stop_detect = True
                            step = 5
                        else:
                            step = 3
                elif step == 5:  # 拿起或者放下物体(pick up or put down the object)
                    if find_box:
                        AGC.runActionGroup('go_forward_one_step', times=3)
                        AGC.runActionGroup('stand', lock_servos=lock_servos)
                        AGC.runActionGroup('move_up')
                        lock_servos = LOCK_SERVOS
                        step = 6    
                    else:
                        AGC.runActionGroup('go_forward_one_step', times=go_step, lock_servos=lock_servos)
                        AGC.runActionGroup('stand', lock_servos=lock_servos)
                        AGC.runActionGroup('put_down')
                        AGC.runActionGroup(back, times=5, with_stand=True)
                        tts.TTSModuleSpeak('', '搬运完成')
                        lock_servos = ''
                        step = 6
                        object_color = None
            elif object_center_x == -1:  # 找不到目标时，转头，转身子来寻找(when unable to find the target, turn the head or rotate the body to search)
                if start_count:
                    start_count = False
                    time_start = time.time()
                else:
                    if time.time() - time_start > 0.5:
                        if 0 < servo_data['servo2'] - x_dis <= abs(d_x) and d_y > 0:
                            x_dis = servo_data['servo2']
                            y_dis = servo_data['servo1']
                            
                            ctl.set_pwm_servo_pulse(1, y_dis, 20)
                            ctl.set_pwm_servo_pulse(2, x_dis, 20)
                            
                            AGC.runActionGroup(turn_right, times=5, lock_servos=lock_servos)
                        elif head_turn == 'left_right':
                            x_dis += d_x            
                            if x_dis > servo_data['servo2'] + 400 or x_dis < servo_data['servo2'] - 400:
                                if head_turn == 'left_right':
                                    head_turn = 'up_down'
                                d_x = -d_x
                        elif head_turn == 'up_down':
                            y_dis += d_y
                            if y_dis > servo_data['servo1'] + 300 or y_dis < servo_data['servo1']:
                                if head_turn == 'up_down':
                                    head_turn = 'left_right'
                                d_y = -d_y

                        ctl.set_pwm_servo_pulse(1, y_dis, 20)
                        ctl.set_pwm_servo_pulse(2, x_dis, 20)
                        time.sleep(0.02)
            else:
                time.sleep(0.01)
        else:
            time.sleep(0.01)

#启动动作的线程(start action thread)
th = threading.Thread(target=move)
th.daemon = True
th.start()

def run(img):
    global step 
    global turn
    global stop_detect, find_box
    global color, color_center_x, color_center_y, color_angle
    global object_color, object_center_x, object_center_y, object_angle

    if not __isRunning or stop_detect:
        if step == 5:
            object_center_x = 0
        elif step == 6:
            find_box = not find_box
            object_center_x = -2
            step = 1
            stop_detect = False
       
        return img
    
    data = asr.getResult()
    if data == 2:
       object_color = 'red'
       tts.TTSModuleSpeak('', '收到, 开始寻找红色')
    elif data == 3:
        object_color = 'green'
        tts.TTSModuleSpeak('', '好的，开始寻找绿色')
    elif data == 4:
        object_color = 'blue'
        tts.TTSModuleSpeak('', 'ok， 开始寻找蓝色')
    
    if object_color is None:
        object_center_x = -4
        return img
    
    color, color_center_x, color_center_y, color_angle = colorDetect(img, object_color)  # 颜色检测，返回颜色，中心坐标，角度
    
    # 如果是搬运阶段(if it is the transportation stage)
    if find_box:
        object_center_x, object_center_y, object_angle = color_center_x, color_center_y, color_angle
    else:
        tag_data = apriltagDetect(img) # apriltag检测(apriltag detection)
        
        if object_color is None:
            return img
        if tag_data[color_tag[object_color] - 1][0] != -1:  # 如果检测到目标arpiltag(if the target apriltag is detected)
            object_center_x, object_center_y, object_angle = tag_data[color_tag[object_color] - 1]
        else:  # 如果没有检测到目标arpiltag，就通过其他arpiltag来判断相对位置(if the target apriltag is not detected, then determine the relative position using other apriltags)
            turn = getTurn(color_tag[object_color], tag_data)
            if turn == 'None':
                object_center_x, object_center_y, object_angle = -1, -1, 0
            else:  # 完全没有检测到apriltag(if no apriltag is detected at all)
                object_center_x, object_center_y, object_angle = -3, -1, 0
    
    return img

def asr_init():
    asr.eraseWords()
    asr.setMode(2)
    asr.addWords(1, 'kai shi')
    asr.addWords(2, 'ban yun hong se')
    asr.addWords(3, 'ban yun lv se')
    asr.addWords(4, 'ban yun lan se')
    
    tts.TTSModuleSpeak('[h0][v10][m3]', '准备就绪')
    time.sleep(1)
    asr.getResult()

if __name__ == '__main__':
    init()
    start()
    
    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open()          
    AGC.runActionGroup('stand_slow')

    try:
        asr_init()
        print('''当前为口令模式，每次说指令前均需要说口令来激活(currently in password mode, activation required before each command by saying the password)
口令：开始(command: kai shi)
指令2：搬运红色(command2: transport red)
指令3：搬运绿色(command3: transport green)
指令4：搬运蓝色(command4: transport blue)
    ''')
    except:
        print('传感器初始化出错')
    while True:
        ret, img = my_camera.read()
        if ret:
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
