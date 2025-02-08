#!/usr/bin/python3
# coding=utf8
#4.拓展课程学习\8.拓展课程之传感器应用开发课程\第1课 人脸追踪风扇\第2节 人脸风扇追踪(4.Advanced Lessons\8.Sensor Development Course\Lesson1 Face Tracking Fan\2. Human Fan Tracking)
import sys
import cv2
import math
import time
import threading
import numpy as np
import mediapipe as mp
import gpiod
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.Misc as Misc
import hiwonder.Camera as Camera
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle

# 人脸追踪控制风扇(face tracking control fan)

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 导入人脸识别模块(import human face recognition module)
face = mp.solutions.face_detection
# 自定义人脸识别方法，最小的人脸检测置信度0.5(custom human face recognition method, the minimum face detection confidence is 0.5)
face_detection = face.FaceDetection(min_detection_confidence=0.5)

servo_data = None
def load_config():
    global servo_data
    
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()
servo2_pulse = servo_data['servo2']

board = rrc.Board()
ctl = Controller(board)
# 初始位置(initial position)
def initMove():
    ctl.set_pwm_servo_pulse(1, 1600, 500)
    ctl.set_pwm_servo_pulse(2, servo2_pulse, 500)

d_pulse = 5
start_greet = False
robot_is_running = False

# 变量重置(variable reset)
def reset():
    global d_pulse
    global start_greet
    global servo2_pulse    

    d_pulse = 10
    start_greet = False
    servo2_pulse = servo_data['servo2']
    
# 初始化(initialization)
def init():
    print("FaceDetect Init")
    reset()
    initMove()

## 初始化引脚模式(initial pin mode)
chip = gpiod.Chip("gpiochip4")
fanPin1 = chip.get_line(8)
fanPin1.request(consumer="pin1", type=gpiod.LINE_REQ_DIR_OUT)

fanPin2 = chip.get_line(7)
fanPin2.request(consumer="pin2", type=gpiod.LINE_REQ_DIR_OUT)

def set_fan(start):
               
    if start == 1:
        ## 开启风扇, 顺时针(turn on the fan, clockwise)
        fanPin1.set_value(1)  # 设置引脚输出高电平(set pin output high voltage)
        fanPin2.set_value(0)  # 设置引脚输出低电平(set pin output low voltage)
    else:
        ## 关闭风扇(close fan)
        fanPin1.set_value(0)  # 设置引脚输出低电平(set pin output low voltage)
        fanPin2.set_value(0)  # 设置引脚输出低电平(set pin output low voltage)


start_ = False
def move():
    global start_greet, start_
    global d_pulse, servo2_pulse    
    
    while True:
        if robot_is_running:
            if start_greet: #判断是否识别到人脸的标志(the flag indicating whether a face is recognized)
                start_greet = False
                pulse = int(Misc.map(servo2_pulse, 1000, 2000, 600, 200)) #把头部舵机的脉宽映射到手部舵机(map the pulse width of the head servo to the hand servo)
                # 驱动手部舵机(drive the hand servo)
                ctl.set_bus_servo_pulse(16,700,1500)  
                time.sleep(1.5)
                if pulse < 500:
                    ctl.set_bus_servo_pulse(15,200,600)  
                    ctl.set_bus_servo_pulse(14,pulse,600)
                    time.sleep(0.6)
                else:
                    ctl.set_bus_servo_pulse(15,pulse-300,600)  
                    ctl.set_bus_servo_pulse(14,445,500)
                    time.sleep(0.6)
                set_fan(1) #开启风扇(turn on fan)
              
            else:
                set_fan(0) #关闭风扇(turn off fan)
                if servo2_pulse > 2000 or servo2_pulse < 1000:
                    d_pulse = -d_pulse
                #左右转动，检测(rotate left and right, and perform detection)
                servo2_pulse += d_pulse       
                ctl.set_pwm_servo_pulse(2, servo2_pulse, 60)
                time.sleep(0.06)
        else:
            time.sleep(0.01)
            
# 运行子线程(run sub-thread)
th = threading.Thread(target=move)
th.daemon = True
th.start()

size = (320, 240)
def run(img):
    global start_greet
       
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not robot_is_running:
        return img

    image_rgb = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB) # 将BGR图像转为RGB图像(convert BGR image to RGB image)
    results = face_detection.process(image_rgb) # 将每一帧图像传给人脸识别模块(pass each frame image to the face recognition module)
    if results.detections:   # 如果检测不到人脸那就返回None(if no face is detected, return None)
        for index, detection in enumerate(results.detections): # 返回人脸索引index(第几张脸)，和关键点的坐标信息(return the face index (which face), and the coordinates of the keypoints)
            bboxC = detection.location_data.relative_bounding_box # 设置一个边界框，接收所有的框的xywh及关键点信息(set a bounding box to receive the xywh (x-coordinate, y-coordinate, width, height) and keypoints information for all boxes)
            
            # 将边界框的坐标点,宽,高从比例坐标转换成像素坐标(convert the coordinates, width, and height of the bounding box from relative coordinates to pixel coordinates)
            bbox = (int(bboxC.xmin * img_w), int(bboxC.ymin * img_h),  
                   int(bboxC.width * img_w), int(bboxC.height * img_h))
            cv2.rectangle(img, bbox, (0,255,0), 2)  # 在每一帧图像上绘制矩形框(draw a rectangle on each frame image)
            x, y, w, h = bbox  # 获取识别框的信息,xy为左上角坐标点(retrieve information about the recognition box, where xy represents the coordinates of the top-left corner)
            center_x =  int(x + (w/2))
            if abs(center_x - img_w/2) < img_w/4:
                start_greet = True
            else:
                set_fan(0)
                start_greet = False
    
    return img

if __name__ == '__main__':
    from hiwonder.CalibrationConfig import *
    param_data = np.load(calibration_param_path + '.npz')
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
  
    init()
    robot_is_running = True
    my_camera = Camera.Camera()
    my_camera.camera_open()
    AGC.runActionGroup('stand_slow')
    while True:
        ret,img = my_camera.read()
        if ret:
            frame = img.copy()
            # 纠正镜头畸变(correct lens distortion)
            frame = cv2.remap(frame.copy(), mapx, mapy, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT) 
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    set_fan(0)
    my_camera.camera_close()
    cv2.destroyAllWindows()
