#!/usr/bin/python3
# coding=utf8
#4.拓展课程学习\8.拓展课程之传感器应用开发课程\第4课 形状识别\第2节 形状识别(4.Advanced Lessons\8.Sensor Development Course\Lesson4 Shape Recognition\2.Shape Recognition)
import sys
import cv2
import math
import time
import signal
import threading
import numpy as np
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
from hiwonder import dot_matrix_sensor
import hiwonder.Camera as Camera
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
 
# 点阵显示形状(dot-matrix display shape)
# 点阵接口：扩展板io7、io8(dot-matrix interface: expansion board io7、io8)

dms = dot_matrix_sensor.TM1640(dio=7, clk=8)

lab_data = None
servo_data = None
move_st = True

def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

board = rrc.Board()
ctl = Controller(board)

# 初始位置(initial position)
def inidmsove():
    ctl.set_pwm_servo_pulse(1, 1350, 500)
    ctl.set_pwm_servo_pulse(2, servo_data['servo2'], 500)


# 找出面积最大的轮廓(find out the contour with the maximal area)
# 参数为要比较的轮廓的列表(the parameter is the list of contour to be compared)
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓(iterate through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate contour mask)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 50:  # 只有在面积大于50时，最大面积的轮廓才是有效的，以过滤干扰(only contours with an area greater than 50 are considered valid, representing the largest area, to filter out interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the contour with the maximal contour)

shape_length = 0

def run():
    global shape_length
    
    while move_st:
        if shape_length == 3:
            print('三角形')
            ## 显示'三角形' (display 'triangle')
            dms.display_buf = (0x01, 0x03, 0x05, 0x09, 0x11, 0x21, 0x41, 0x81,
                              0x41, 0x21, 0x11, 0x09, 0x05, 0x03, 0x01, 0x00)
            dms.update_display()
            
        elif shape_length == 4:
            print('矩形')
            ## 显示'矩形'(display 'rectangle')
            dms.display_buf = (0x00, 0x00, 0x00, 0x00, 0xff, 0x81, 0x81, 0x81,
                              0x81, 0x81, 0x81,0xff, 0x00, 0x00, 0x00, 0x00)
            dms.update_display()
            
        elif shape_length >= 6:           
            print('圆')
            ## 显示'圆形'(display 'circle')
            dms.display_buf = (0x00, 0x00, 0x00, 0x00, 0x1c, 0x22, 0x41, 0x41,
                              0x41, 0x22, 0x1c,0x00, 0x00, 0x00, 0x00, 0x00)
            dms.update_display()
            
        else:
            ## 清屏(clear screen)
            dms.display_buf = [0] * 16
            dms.update_display()
            print('None')
            
       
        
# 运行子线程(run sub-thread)
th = threading.Thread(target=run)
th.daemon = True
th.start()

shape_list = []
action_finish = True

def Stop(signum, frame):
    global move_st
    move_st = False
    dms.display_buf = [0] * 16
    dms.update_display()
    print('关闭中...')
    AGC.runActionGroup('lift_down')

signal.signal(signal.SIGINT, Stop)


if __name__ == '__main__':
    from hiwonder.CalibrationConfig import *
    
    param_data = np.load(calibration_param_path + '.npz')
    
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
    
    load_config()
    inidmsove()
    my_camera = Camera.Camera()
    my_camera.camera_open()
    AGC.runActionGroup('stand_slow')
    AGC.runActionGroup('lift_up')
    while move_st:
        ret, img = my_camera.read()
        if ret:
            img_copy = img.copy()
            img_h, img_w = img.shape[:2]
            frame_gb = cv2.GaussianBlur(img_copy, (3, 3), 3)      
            frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)
            max_area = 0
            color_area_max = None    
            areaMaxContour_max = 0

            if action_finish:
                for i in lab_data:
                    if i != 'white' and i != 'black':
                        frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))  #对原图像和掩模进行位运算(perform the bitwise operation to original image and mask)
                        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6,6),np.uint8))  #开运算(opening operation)
                        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6,6),np.uint8)) #闭运算(closing operation)
                        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓(find out the contour)
                        areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓(find out the contour with the maximal area)
                        if areaMaxContour is not None:
                            if area_max > max_area:#找最大面积(find the maximal area)
                                max_area = area_max
                                color_area_max = i
                                areaMaxContour_max = areaMaxContour
            if max_area > 200:                   
                cv2.drawContours(img, areaMaxContour_max, -1, (0, 0, 255), 2)
                # 识别形状(recognize shape)
                # 周长  0.035 根据识别情况修改，识别越好，越小(perimeter: 0.035. Modify according to recognition. The better the recognition, the smaller)
                epsilon = 0.035 * cv2.arcLength(areaMaxContour_max, True)
                # 轮廓相似(contour similarity)
                approx = cv2.approxPolyDP(areaMaxContour_max, epsilon, True)
                shape_list.append(len(approx))
                if len(shape_list) == 30:
                    shape_length = int(round(np.mean(shape_list)))                            
                    shape_list = []
                    print(shape_length)
            # 纠正镜头畸变(correcting lens distortion)
            img = cv2.remap(img.copy(), mapx, mapy, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)  
            frame_resize = cv2.resize(img, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()

