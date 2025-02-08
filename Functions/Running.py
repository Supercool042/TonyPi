#!/usr/bin/python3
# coding=utf8
import sys
import time
import threading

import Functions.KickBall as KickBall
import Functions.Transport as Transport
import Functions.ColorTrack as ColorTrack
import Functions.FaceDetect as FaceDetect
import Functions.lab_adjust as lab_adjust
import Functions.ColorDetect as ColorDetect
import Functions.VisualPatrol as VisualPatrol
import Functions.RemoteControl as RemoteControl
import Functions.ApriltagDetect as ApriltagDetect
import Functions.gesture_control as GestureControl 
import Extend.athletics_course.hurdles as Hurdles
import Extend.athletics_course.stairway as Stairway
import Extend.vision_grab_course.color_classify as ColorClassify

RunningFunc = 0
LastHeartbeat = 0
cam = None
open_once = False

FUNCTIONS = {
    1: RemoteControl, # 机体控制(remote control)
    2: KickBall,      # 自动踢球(auto shooting)
    3: GestureControl,# 颜色识别(color recognition)
    4: VisualPatrol,  # 智能巡线 (intelligent line follow)
    5: ColorTrack,    # 云台跟踪(pan-tilt tracking)
    6: FaceDetect,    # 人脸识别(face recognition)
    7: ApriltagDetect,# 标签识别(tag recognition)
    8: Transport,     # 智能搬运(intelligent transportation)
    9: lab_adjust,     # lab阈值调节(lab threshold adjustment)
    10: Hurdles ,     # 跨栏避障(hurdles)
    11: Stairway ,    # 上下台阶(go up and down stairs)
    12: ColorClassify # 色块分类(color block classification)
}

def doHeartbeat(tmp=()):
    global LastHeartbeat
    LastHeartbeat = time.time() + 7
    return (True, ())

def CurrentEXE():
    global RunningFunc
    return FUNCTIONS[RunningFunc]

def loadFunc(newf):
    global RunningFunc
    new_func = newf[0]

    doHeartbeat()

    if new_func < 1 or new_func > 12:
        return (False,  sys._getframe().f_code.co_name + ": Invalid argument")
    else:
        try:
            if RunningFunc > 1:
                FUNCTIONS[RunningFunc].exit()
            RunningFunc = newf[0]
            if not open_once:
                cam.camera_close()
                cam.camera_open()
            FUNCTIONS[RunningFunc].init()
        except Exception as e:
            print(e)
    return (True, (RunningFunc,))

def unloadFunc(tmp = ()):
    global RunningFunc
    if RunningFunc != 0:
        FUNCTIONS[RunningFunc].exit()
        RunningFunc = 0
    if not open_once:
        cam.camera_close()
    return (True, (0,))

def getLoadedFunc(newf):
    global RunningFunc
    return (True, (RunningFunc,))

def startFunc(tmp):
    global RunningFunc
    FUNCTIONS[RunningFunc].start()
    return (True, (RunningFunc,))

def stopFunc(tmp):
    global RunningFunc
    FUNCTIONS[RunningFunc].stop()
    return (True, (RunningFunc,))

def heartbeatTask():
    global LastHeartbeat
    global RunningFunc
    while True:
        try:
            if LastHeartbeat < time.time():
                if RunningFunc != 0:
                    unloadFunc()
            time.sleep(0.1)
        except Exception as e:
            print(e)

        except KeyboardInterrupt:
            break

threading.Thread(target=heartbeatTask, daemon=True).start()
