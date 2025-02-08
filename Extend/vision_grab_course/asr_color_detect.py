#!/usr/bin/python3
# coding=utf8
# 4.拓展课程学习\10.拓展课程之视觉抓取课程\第5课 TonyPi Pro语音控制抓取(4.Advanced Lessons\10.Vision Gripping Lesson\Lesson5 TonyPi Pro Voice Control Gripping)
import sys
import cv2
import math
import time
import threading
import numpy as np
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ASR as ASR
import hiwonder.Misc as Misc
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

#语音控制抓取(voice control gripping)

asr = ASR.ASR()
CentreX = 330
state = False
target_color = 'None'
color, color_x, color_y, angle = None, 0, 0, 0

lab_data = None
servo_data = None
def load_config():
    global lab_data
    global servo_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
    'None': (255, 255, 255)}

board = rrc.Board()
ctl = Controller(board)

# 初始位置(initial position)
def initMove():
    ctl.set_pwm_servo_pulse(1,servo_data['servo1'],1000)
    ctl.set_pwm_servo_pulse(2,servo_data['servo2'],1000) 
    ctl.set_bus_servo_pulse(17, 500, 1000)
    ctl.set_bus_servo_pulse(18, 500, 1000)

    

# 找出面积最大的轮廓(find out the contour with the maximal area)
# 参数为要比较的轮廓的列表(the parameter is the list to be compared)
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


def move():
    global state, target_color
    global lab_data
    global servo_data

    skip = True
    pulse2 = servo_data['servo2']
    dire = None
    while True:
        if state:
            time.sleep(1)
            if color != 'None':
                if dire is None:
                    if color_x > CentreX:
                        dire = 'right'
                    elif color_x < CentreX:
                        dire = 'left'
                if 15 > color_x - CentreX > 8:
                    AGC.runAction('right_move_10')
                elif -15 < color_x - CentreX < -8:
                    AGC.runAction('left_move_10')
                elif color_x - CentreX >= 20:
                    AGC.runAction('right_move_20')
                elif color_x - CentreX <= -20:
                    AGC.runAction('left_move_20')
                else:
                    board.set_buzzer(1900, 0.1, 0.9, 1)
                    if dire == 'left':
                        AGC.runAction('grab_squat_left')
                        time.sleep(0.5)
                        AGC.runAction('grab_squat_up_left')
                        time.sleep(0.5)
                        AGC.runAction('grab_stand_left')
                    elif dire == 'right':
                        AGC.runAction('grab_squat_right')
                        time.sleep(0.5)
                        AGC.runAction('grab_squat_up_right')
                        time.sleep(0.5)
                        AGC.runAction('grab_stand_right')
                    dire = None
                    state = False
                    target_color = 'None'
                    
                if pulse2 - servo_data['servo2'] >= 10:
                    pulse2 -= 20
                elif pulse2 - servo_data['servo2'] <= -10:
                    pulse2 += 20
                ctl.set_pwm_servo_pulse(2, pulse2, 30)
                
            else:
                dire = None
                if skip:
                    pulse2 = 1700
                    skip = False
                else:
                    pulse2 = 1360
                    skip = True
                ctl.set_pwm_servo_pulse(2, pulse2, 300)
                time.sleep(0.8)
        else: 
            time.sleep(0.01)

# 运行子线程(run sub-thread)
th = threading.Thread(target=move)
th.daemon = True
th.start()

def colorDetect(img):
    img_h, img_w = img.shape[:2]
    size = (img_w, img_h)
    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)
    
    center_max_distance = pow(img_w/2, 2) + pow(img_h, 2)
    color, center_x, center_y, angle = 'None', -1, -1, 0
    if target_color != 'None':
        frame_mask = cv2.inRange(frame_lab,
                                 (lab_data[target_color]['min'][0],
                                  lab_data[target_color]['min'][1],
                                  lab_data[target_color]['min'][2]),
                                 (lab_data[target_color]['max'][0],
                                  lab_data[target_color]['max'][1],
                                  lab_data[target_color]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(corrosion)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilation)
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find out contours)
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓(find out the contour with the maximal area)
        if area_max > 500:  # 有找到最大面积(the maximal area is found)
            rect = cv2.minAreaRect(areaMaxContour)#最小外接矩形(the minimum bounding rectangle)
            angle_ = rect[2]

            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点(the four vertices of the minimum bounding rectangle)
            for j in range(4):
                box[j, 0] = int(Misc.map(box[j, 0], 0, size[0], 0, img_w))
                box[j, 1] = int(Misc.map(box[j, 1], 0, size[1], 0, img_h))
            cv2.drawContours(img, [box], -1, range_rgb[target_color], 2)#画出四个点组成的矩形(draw the rectangle formed by the four points)
            #获取矩形的对角点(get the diagonal points of the rectangle)
            ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]            
            center_x_, center_y_ = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2)#中心点(center point)
            cv2.circle(img, (center_x_, center_y_), 5, (0, 255, 255), -1)#画出中心点(draw center point)
            distance = pow(center_x_ - img_w/2, 2) + pow(center_y_ - img_h, 2)
            if distance < center_max_distance:  # 寻找距离最近的物体来搬运(find the nearest object for transportation)
                center_max_distance = distance
                color = target_color
                center_x, center_y, angle = center_x_, center_y_, angle_
                    
    return color, center_x, center_y, angle

def run(img):
    global target_color, state
    global color, color_x, color_y, angle
    
    if not state:
        data = asr.getResult()
        if data:
            print("result:", data)
            if data == 2:
                target_color = 'red'
                state = True
            elif data == 3:
                target_color = 'green'
                state = True
            elif data == 4:
                target_color = 'blue'
                state = True
            else:
                target_color = 'None'
                state = False
            
    if state and target_color != 'None':
        color, color_x, color_y, angle = colorDetect(img)
#         print('Color:',color, color_x, color_y, angle)
        cv2.putText(img, "Color:"+color, (10, img.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.65, range_rgb[color], 2)
       
    return img


if __name__ == '__main__':
    from hiwonder.CalibrationConfig import *
    
    param_data = np.load(calibration_param_path + '.npz')
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
    
    asr.eraseWords()
    asr.setMode(2)
    asr.addWords(1, 'kai shi')
    asr.addWords(2, 'na hong se')
    asr.addWords(3, 'na lv se')
    asr.addWords(4, 'na lan se')

    load_config()
    initMove()
    
    camera = cv2.VideoCapture(-1)
    AGC.runActionGroup('stand_slow')
    
    while True:
    
        Time = time.time()
        ret,img = camera.read()
        if ret:
            #Time = time.time()
            frame = img.copy()
            frame = cv2.remap(frame.copy(), mapx, mapy, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            Frame = run(frame)
            d_time = time.time() - Time
            #print("d_time:",d_time)
            fps = int(1.0 / d_time)
            cv2.putText(Frame, 'FPS:'+ str(fps), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 2)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()
    
    
