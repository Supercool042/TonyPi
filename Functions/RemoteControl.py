#!/usr/bin/python3
# coding=utf8
import sys
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

board = rrc.Board()
ctl = Controller(board)

# 初始位置(initial position)
def initMove():
    ctl.set_pwm_servo_pulse(1, 1500, 500)
    ctl.set_pwm_servo_pulse(2, 1500, 500)
    agc.run_action_group('stand_slow')

def reset():
    return None

def init():
    initMove()
    print("RemoteControl Init")
    return None

def start():
    print("RemoteControl Start")
    return None

def stop():
    print("RemoteControl Stop")
    return None

def exit():
    print("RemoteControl Exit")
    return None

def run(img):
    return img
