#!/usr/bin/python3
# coding=utf8
import sys
import os
import cv2
import time
import queue
import logging
import threading
import RPCServer
import MjpgServer
import numpy as np
# import cvfilter
import hiwonder.Camera as Camera
import hiwonder.yaml_handle as yaml_handle

import Functions.Running as Running

# 主线程，已经以后台的形式开机自启(the main thread is already set to start in the background upon boot)
# 自启方式systemd，自启文件/etc/systemd/system/tonypi.service(the autostart method is systemd, with the autostart file located at /etc/systemd/system/tonypi.service)
# sudo systemctl stop tonypi  此次关闭(closing this time)
# sudo systemctl disable tonypi 永久关闭(permanently close)
# sudo systemctl enable tonypi 永久开启(permanently open)
# sudo systemctl start tonypi 此次开启(open this time)

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

QUEUE_RPC = queue.Queue(10)

def task():
    while True:
        try:
            req, ret = QUEUE_RPC.get(False)
            event, params, *_ = ret
            ret[2] = req(params)  # 执行RPC命令(execute RPC command)
            event.set()
        except queue.Empty:
            time.sleep(0.01)

def startTonyPi():

    RPCServer.QUEUE = QUEUE_RPC

    threading.Thread(target=RPCServer.startRPCServer,
                     daemon=True).start()  # rpc服务器(rpc server)
    threading.Thread(target=task,
                     daemon=True).start()  # mjpg流服务器(mjpg stream server)
    
    threading.Thread(target=MjpgServer.startMjpgServer,
                     daemon=True).start()  # mjpg流服务器(mjpg stream server)
    loading_picture = cv2.imread('/home/pi/TonyPi/Functions/loading.jpg')
    cam = Camera.Camera()  # 相机读取(camera reading)
    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        cam.camera_open()

    Running.open_once = open_once
    Running.cam = cam

    while True:
        time.sleep(0.03)
        # 执行需要在本线程中执行的RPC命令(execute RPC commands that need to be executed in this thread)
        #####
        # 执行功能玩法程序：(perform function program)
        try:
            if Running.RunningFunc > 0 and Running.RunningFunc <= 12:
                if cam.frame is not None:
                    img = Running.CurrentEXE().run(cam.frame.copy())
                    if Running.RunningFunc == 9:
                        MjpgServer.img_show = np.vstack((img, cam.frame))
                    else:
                        MjpgServer.img_show = img
                else:
                    MjpgServer.img_show = loading_picture
            else:
                MjpgServer.img_show = cam.frame
        except KeyboardInterrupt:
            break
        except BaseException as e:
            print('error', e)

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    startTonyPi()
