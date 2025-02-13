#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2
import math
import copy
import threading
import numpy as np
import mediapipe as mp
from hiwonder import fps
from mediapipe import solutions
from mediapipe.tasks import python
import hiwonder.yaml_handle as yaml_handle
from hiwonder.Controller import Controller
from mediapipe.tasks.python import vision
import hiwonder.ros_robot_controller_sdk as rrc
from mediapipe.framework.formats import landmark_pb2
from mediapipe_visual import draw_pose_landmarks_on_image

board = rrc.Board()
ctl = Controller(board)

servo_data = None
# 加载配置文件数据(load configuration file data)
def load_config():
    global servo_data
    
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()

x_dis = servo_data['servo2']
y_dis = 1500

ctl.set_pwm_servo_pulse(1, y_dis, 500)
ctl.set_pwm_servo_pulse(2, x_dis, 500)

def val_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def vector_2d_angle(v1, v2):
    d_v1_v2 = np.linalg.norm(v1) * np.linalg.norm(v2)
    if d_v1_v2 == 0:
        return None

    cos = np.dot(v1, v2) / d_v1_v2
    cos = np.clip(cos, -1.0, 1.0)

    sin = np.cross(v1, v2) / d_v1_v2
    sin = np.clip(sin, -1.0, 1.0)
    angle = int(np.degrees(np.arctan2(sin, cos)))
    return angle

l1 = 0.06
l2 = 0.11
board = rrc.Board()
model_path = os.path.join(os.path.abspath(os.path.split(os.path.realpath(__file__))[0]), 'model/pose_landmarker_lite.task')
base_options = python.BaseOptions(model_asset_path=model_path)
options = vision.PoseLandmarkerOptions(
    base_options=base_options,
    output_segmentation_masks=True)
detector = vision.PoseLandmarker.create_from_options(options)
fps = fps.FPS()

last_angle = []
cap = cv2.VideoCapture(-1)
while True:
    ret, image = cap.read()
    if ret:
        image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 1)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        detection_result = detector.detect(mp_image)
        height, width, _ = image.shape
        pose_landmarks_list = detection_result.pose_landmarks
        annotated_image = np.copy(image)
        for idx in range(len(pose_landmarks_list)):
            pose_landmarks = pose_landmarks_list[idx]

            # Draw the pose landmarks.
            pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            pose_landmarks_proto.landmark.extend([
              landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
            ])
            mark = []
            landmarks = pose_landmarks[11:17]
            for landmark in landmarks:
                result = landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z)
                mark.append([int(result.x * width) , int(result.y * height)])
            for i in mark:
                cv2.circle(annotated_image, i, 10, (255, 255, 0), -1)
            left_p1 = mark[0]
            left_p2 = mark[2]
            left_p3 = mark[4]
            right_p1 = mark[1]
            right_p2 = mark[3]
            right_p3 = mark[5]
            # cv2.line(annotated_image, left_p1, (width, left_p1[1]), (255, 255, 0), 5)
            cv2.line(annotated_image, left_p1, left_p2, (255, 255, 0), 5)
            cv2.line(annotated_image, left_p2, left_p3, (255, 255, 0), 5)
            cv2.line(annotated_image, right_p1, right_p2, (255, 255, 0), 5)
            cv2.line(annotated_image, right_p2, right_p3, (255, 255, 0), 5)
            left_p0 = copy.deepcopy(mark[0])
            left_p0[0] = width
            right_p0 = copy.deepcopy(mark[1])
            right_p0[0] = 0
            # up-90 0 90down
            # up 0 1000down
            angle1 = vector_2d_angle(np.array(left_p1) - np.array(left_p0), np.array(left_p1) - np.array(left_p2))
            angle2 = vector_2d_angle(np.array(left_p2) - np.array(left_p1), np.array(left_p3) - np.array(left_p2)) 
            # 90 -90
            # 1000 0
            angle3 = vector_2d_angle(np.array(right_p1) - np.array(right_p0), np.array(right_p1) - np.array(right_p2))
            angle4 = vector_2d_angle(np.array(right_p2) - np.array(right_p1), np.array(right_p3) - np.array(right_p2))
            if angle1 is not None and angle2 is not None and angle3 is not None and angle4 is not None:
                # print(angle1, angle2, angle3, angle4) 
                x1 = l1 * math.cos(math.radians(angle1)) + l2 * math.cos((math.radians(angle2)) + math.radians(angle1))
                x2 = l1 * math.cos(math.radians(angle3)) + l2 * math.cos((math.radians(angle4)) + math.radians(angle3))
                # print(x1, x2)
                servo7 = int(val_map(angle1, -90, 90, 125, 875))
                servo6 = int(val_map(angle2, -90, 90, 125, 875))
                servo15 = int(val_map(angle3, -90, 90, 125, 875))
                servo14 = int(val_map(angle4, -90, 90, 125, 875))
                if servo6 > 875:
                    servo6 = 875
                if servo6 < 125:
                    servo6 = 125
                if servo7 > 875:
                    servo7 = 875
                if servo7 < 125:
                    servo7 = 125
                if servo14 > 875:
                    servo14 = 875
                if servo14 < 125:
                    servo14 = 125
                if servo15 > 875:
                    servo15 = 875
                if servo15 < 125:
                    servo15 = 125
                if last_angle:
                    if abs(last_angle[0] - servo6) < 30:
                        servo6 = last_angle[0]
                    if abs(last_angle[1] - servo7) < 30:
                        servo7 = last_angle[1]
                    if abs(last_angle[2] - servo14) < 30:
                        servo14 = last_angle[2]
                    if abs(last_angle[3] - servo15) < 30:
                        servo15 = last_angle[3]
                if x1 > 0 and x2 > 0:
                    board.bus_servo_set_position(0.1, [[7, servo7], [6, servo6], [14, servo14], [15, servo15]])
                elif x1 > 0 and x2 < 0:
                    board.bus_servo_set_position(0.05, [[7, servo7], [6, servo6]])
                elif x1 < 0 and x2 > 0:
                    board.bus_servo_set_position(0.05, [[14, servo14], [15, servo15]])

                solutions.drawing_utils.draw_landmarks(
                  annotated_image,
                  pose_landmarks_proto,
                  solutions.pose.POSE_CONNECTIONS,
                  solutions.drawing_styles.get_default_pose_landmarks_style())
                last_angle = [servo6, servo7, servo14, servo15]
        fps.update()
        result_image = fps.show_fps(cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
        cv2.imshow('pose_landmarker', result_image)
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:  # 按q或者esc退出(press q or esc to exit)
            break
cv2.destroyAllWindows()
