#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import math
import time
import numpy as np

import hiwonder.Camera as Camera
import hiwonder.Misc as Misc
import hiwonder.yaml_handle as yaml_handle

# 颜色检测(color detection)
range_rgb = {
    'red': (0, 0, 255),
    'black': (0, 0, 0),   
}

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
            if contour_area_temp > 50:  # 只有在面积大于50时，最大面积的轮廓才是有效的，以过滤干扰(only contours with an area greater than 50 are considered valid; the contour with the largest area is used to filter out interference)
                areaMaxContour = c

    return areaMaxContour, contour_area_max  # 返回最大的轮廓(return the contour with the maximal area)


lab_data = None
def load_config():
    global lab_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

load_config()

size = (320, 240)
def run(img):

    detect_color = 'None'  
    draw_color = range_rgb["black"]	
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to the LAB space)

    max_area = 0 
    areaMaxContour_max = 0

    frame_mask = cv2.inRange(frame_lab,
                                        (lab_data['red']['min'][0],
                                         lab_data['red']['min'][1],
                                         lab_data['red']['min'][2]),
                                        (lab_data['red']['max'][0],
                                         lab_data['red']['max'][1],
                                         lab_data['red']['max'][2]))#对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
    eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(corrosion)
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilation)
    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓(find out contours)
    areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓(find out the contour with the maximal area)
    if areaMaxContour is not None:
        if area_max > max_area:#找最大面积(find the maximal area)
            max_area = area_max
            areaMaxContour_max = areaMaxContour
    if max_area > 200:  # 有找到最大面积(the maximal area is found)
        ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)  # 获取最小外接圆(get the minimum bounding circumcircle)
        centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
        centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, size[0], 0, img_w))            

        detect_color = 'red'
        draw_color = range_rgb["red"]

        cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)  #打印识别的颜色数据，格式：Color：***(print the recognized color data, format: Color: ***)
        cv2.circle(img, (centerX, centerY), radius, range_rgb["red"], 2)#画圆,即识别到颜色后圆圈框出(draw a circle around the recognized color)
  
    return img   #开启图像回传(enable image feedback)

if __name__ == '__main__':#用于加载回传画面参数，删除则执行程序无法调用回传(this is used to load parameters for image feedback. Deleting it will prevent the program from accessing the feedback)
    from CameraCalibration.CalibrationConfig import *
    
    #加载参数(load parameters)
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数(get parameters)
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open()
        
    print("Color_Recognize Init")
    print("Color_Recognize Start")
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
