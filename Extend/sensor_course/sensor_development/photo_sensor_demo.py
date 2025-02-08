#!/usr/bin/python3
# coding=utf8
#4.拓展课程学习\8.拓展课程之传感器应用开发课程\第5课 智能补光\第2节 智能补光(4.Advanced Lessons\8.Sensor Development Course\Lesson5 Intelligent Fill Light\2.Intelligent Fill Light)
import os
import sys
import cv2
import math
import time
import threading
import numpy as np
import gpiod
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.Camera as Camera
import hiwonder.Sonar as Sonar
import hiwonder.apriltag as apriltag
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 光敏控制超声波RGB(photosensitive control ultrasonic RGB)


servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

board = rrc.Board()
ctl = Controller(board)
# 初始位置(initial position)
def initMove():
    ctl.set_pwm_servo_pulse(1, 1500, 500)
    ctl.set_pwm_servo_pulse(2, servo_data['servo2'], 500)
    AGC.runActionGroup('lift_up')


st = 0
s = Sonar.Sonar()
s.setRGBMode(0)
s.setRGB(1, (0,0,0)) #设置超声波RGB关闭(set ultrasonic RGB to off)
s.setRGB(0, (0,0,0))

chip = gpiod.Chip('gpiochip4')
light = chip.get_line(8)
light.request(consumer="light", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
def move():
    global st
    
    while True:
        state = light.get_value() #读取引脚数字值(read pin numerical value)
        
        if state:
            time.sleep(0.1)
            if state:
                if st :            #这里做一个判断，防止反复响(make a judgment to prevent repeated echoes here)
                    st = 0
                    board.set_buzzer(1900, 0.1, 0.9, 1)   #设置蜂鸣器响0.1秒(set the buzzer to emit for 0.1 seconds)
                    s.setRGB(1, (255,255,255))  #设置超声波RGB亮白色(set the ultrasonic RGB to bright white)
                    s.setRGB(0, (255,255,255))
        else:
            if not st:
                st = 1
                s.setRGB(1, (0,0,0)) #设置超声波RGB关闭(set ultrasonic RGB to close)
                s.setRGB(0, (0,0,0))
            
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

            objective_x, objective_y = int(detection.center[0]), int(detection.center[1])  # 中心点(center point)
            
            object_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # 计算旋转角(calculate rotation angle)
            
            return [tag_family, tag_id, objective_x, objective_y]
            
    return None, None, None, None


def run(img):
    
    tag_family, tag_id, objective_x, objective_y = apriltagDetect(img) # apriltag检测(apriltag detection)
    print('Apriltag:',objective_x,objective_y)
        
    return img

if __name__ == '__main__':
    from hiwonder.CalibrationConfig import *
    
    param_data = np.load(calibration_param_path + '.npz')
    
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
  
    initMove()
    camera = cv2.VideoCapture(-1)
    
    while True:
        ret,img = camera.read()
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
    
