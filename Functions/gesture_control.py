#!/usr/bin/env python3
# encoding: utf-8
# 指尖轨迹点发布
import cv2
import math
import enum
import time
import queue
import threading
import numpy as np
import faulthandler
import mediapipe as mp
import hiwonder.Camera as Camera
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle
from hiwonder.Controller import Controller
import hiwonder.ros_robot_controller_sdk as rrc

faulthandler.enable()
def distance(point_1, point_2):
    """
    计算两个点间的距离(calculate the distance between two points)
    :param point_1: 点1
    :param point_2: 点2
    :return: 两点间的距离(distance between two points)
    """
    return math.sqrt((point_1[0] - point_2[0]) ** 2 + (point_1[1] - point_2[1]) ** 2)

def vector_2d_angle(v1, v2):
    """
    计算两向量间的夹角 -pi ~ pi(calculate the angle between two vectors -pi ~ pi)
    :param v1: 第一个向量(first vector)
    :param v2: 第二个向量(second vector)
    :return: 角度(angle)
    """
    d_v1_v2 = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos = v1.dot(v2) / (d_v1_v2)
    sin = np.cross(v1, v2) / (d_v1_v2)
    angle = np.degrees(np.arctan2(sin, cos))
    return angle

def get_hand_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标
    :param img: 像素坐标对应的图片
    :param landmarks: 归一化的关键点
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)

def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度
    :param landmarks: 手部关键点
    :return: 各个手指的角度
    """
    angle_list = []
    # thumb 大拇指
    angle_ = vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2])
    angle_list.append(angle_)
    # index 食指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8])
    angle_list.append(angle_)
    # middle 中指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12])
    angle_list.append(angle_)
    # ring 无名指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16])
    angle_list.append(angle_)
    # pink 小拇指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20])
    angle_list.append(angle_)
    angle_list = [abs(a) for a in angle_list]
    return angle_list

def h_gesture(angle_list):
    """
    通过二维特征确定手指所摆出的手势
    :param angle_list: 各个手指弯曲的角度
    :return : 手势名称字符串
    """
    thr_angle = 65.
    thr_angle_thumb = 53.
    thr_angle_s = 49.
    gesture_str = "none"
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "nico-nico-ni"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "two"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
        gesture_str = "three"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "OK"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "four"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "five"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "six"
    else:
        "none"
    return gesture_str

class State(enum.Enum):
    NULL = 0
    START = 1
    TRACKING = 2
    RUNNING = 3

def draw_points(img, points, thickness=4, color=(0, 0, 255)):
    points = np.array(points).astype(dtype=np.int64)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, thickness)
# 颜色阈值数据和头部舵机位置数据(color threshold data and head servo position data)
servo_data = None
board = rrc.Board()
ctl = Controller(board)
# 加载配置文件数据(load configuration file data)
def load_config():
    global servo_data
    
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)
    # print(servo_data)        

load_config()

# 初始化机器人舵机初始位置(initialize the servo initialization position of robot)
def initMove():
    ctl.set_pwm_servo_pulse(1, 1500, 500)
    ctl.set_pwm_servo_pulse(2, servo_data['servo2'], 500)
drawing = mp.solutions.drawing_utils

hand_detector = mp.solutions.hands.Hands(
     static_image_mode=False,
    max_num_hands=1,
    min_tracking_confidence=0.05,
    min_detection_confidence=0.6
)

state = State.NULL
count = 0
start_move = False
last_point = [0, 0]
points = []
points_list = []
running = False
def reset():
    global state, count, start_move, last_point, points, points_list
    state = State.NULL
    count = 0
    start_move = False
    last_point = [0, 0]
    points = []
    points_list = []

enter = False
running = False
# app初始化调用(app initialization calling)
def init():
    global enter
    print('gesture init')
    reset()
    initMove()
    enter = True
    AGC.runActionGroup('stand')

# app开始玩法调用(app start program calling)
def start():
    global running
    print('gesture start')
    running = True

# app停止玩法调用(app stop program calling)
def stop():
    global running
    print('gesture stop')
    reset()
    running = False
    initMove()
    AGC.runActionGroup('stand')

# app退出玩法调用(app exit program calling)
def exit():
    global running, enter
    print('gesture exit')
    reset()
    enter = False
    running = False

def action_thread():
    global last_point, start_move
    while True:
        if start_move and running:
            points = []
            left_and_right = [0]
            up_and_down = [0]
            for i in target_points:
                if i[0] - last_point[0] > 0:
                    left_and_right.append(1)
                else:
                    left_and_right.append(-1)
                if i[1] - last_point[1] > 0:
                    up_and_down.append(1)
                else:
                    up_and_down.append(-1)
                points.extend([i])
                last_point = i
            left_and_right = sum(left_and_right)
            up_and_down = sum(up_and_down)
            points = np.array(points)
            
            line = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
            vx, vy, x0, y0 = line.flatten()  
            projections = (points[:, 0] - x0) * vx + (points[:, 1] - y0) * vy

            t_min = projections.min()
            t_max = projections.max()

            line_segment_length = t_max - t_min
            angle = int(abs(math.degrees(math.acos(line[0][0]))))
            step = round(line_segment_length/70)
            print('>>>>>>', angle)
            if 90 >= angle > 60:
                if up_and_down > 0:
                    AGC.runActionGroup('back_fast', step, True)
                    print('down')
                else:
                    AGC.runActionGroup('go_forward', step, True)
                    print('up')
            elif 30 > angle >= 0:
                if left_and_right > 0:
                    AGC.runActionGroup('left_move', step, True)
                    print('right')
                else:
                    AGC.runActionGroup('right_move', step, True)
                    print('left')
            start_move = False
        else:
            time.sleep(0.1)

#启动动作的线程(start the thread of executing action)
threading.Thread(target=action_thread, daemon=True).start()

def run(image):
    global state, count, points, points_list, target_points, start_move

    image = cv2.flip(image, 1)
    display_image = image.copy()
    if not enter:
        return display_image
    if running:
        try:
            results = hand_detector.process(image)
            if results is not None and results.multi_hand_landmarks:
                gesture = "none"
                index_finger_tip = [0, 0]
                for hand_landmarks in results.multi_hand_landmarks:
                    drawing.draw_landmarks(
                        display_image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS)
                    landmarks = get_hand_landmarks(image, hand_landmarks.landmark)
                    angle_list = (hand_angle(landmarks))
                    gesture = (h_gesture(angle_list))
                    index_finger_tip = landmarks[8].tolist()
                if state != State.TRACKING:
                    if gesture == "one":  # 检测食指手势， 开始指尖追踪
                        count += 1
                        if count > 5:
                            count = 0
                            state = State.TRACKING
                            points = []
                            points_list = []
                    else:
                        count = 0

                elif state == State.TRACKING:
                    if gesture != "two":
                        if len(points) > 0:
                            last_point = points[-1]
                            if distance(last_point, index_finger_tip) < 5:
                                count += 1
                            else:
                                count = 0
                                points_list.append([int(index_finger_tip[0]), int(index_finger_tip[1])])
                                points.append(index_finger_tip)
                        else:
                            points_list.append([int(index_finger_tip[0]), int(index_finger_tip[1])])
                            points.append(index_finger_tip)
                    draw_points(display_image, points)
                if gesture == "five":
                    state = State.NULL
                    if len(points_list) > 10 and not start_move:
                        board.set_buzzer(1900, 0.1, 0.9, 1)
                        target_points = points_list
                        start_move = True
                    points = []
                    points_list = []
                    draw_points(display_image, points)
        except Exception as e:
            print(e)
    return display_image

if __name__ == "__main__":
    init()
    start()
    
    my_camera = Camera.Camera()
    my_camera.camera_open()              
    t = time.time()
    while True:
        ret, img = my_camera.read()
        if ret:
            frame = img.copy()
            frame = run(frame)           
            cv2.imshow('result_image', frame)
            d = time.time() - t
            if d < 0.02:
                key = cv2.waitKey(int((0.02 - d)*1000))
            else:
                key = cv2.waitKey(1)
            if key == 27:
                break
            t = time.time()
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
