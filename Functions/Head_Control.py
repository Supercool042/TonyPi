#!/usr/bin/python3
# coding=utf8
import sys
import time
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller

# 初始化机器人底层驱动(initialize the robot underlying drivers)
board = rrc.Board()
ctl = Controller(board)

def nod_head():
        ctl.set_pwm_servo_pulse(1, 1800, 200)
        time.sleep(0.2)
        ctl.set_pwm_servo_pulse(1, 1200, 200)
        time.sleep(0.2)
        ctl.set_pwm_servo_pulse(1, 1800, 200)
        time.sleep(0.2)
        ctl.set_pwm_servo_pulse(1, 1200, 200)
        time.sleep(0.2)
        ctl.set_pwm_servo_pulse(1, 1500, 100)
        time.sleep(0.1)
        

def shake_head():
        ctl.set_pwm_servo_pulse(2, 1800, 200)
        time.sleep(0.2)
        ctl.set_pwm_servo_pulse(2, 1200, 200)
        time.sleep(0.2)
        ctl.set_pwm_servo_pulse(2, 1800, 200)
        time.sleep(0.2)
        ctl.set_pwm_servo_pulse(2, 1200, 200)
        time.sleep(0.2)
        ctl.set_pwm_servo_pulse(2, 1500, 100)
        time.sleep(0.1)
        

if __name__ == '__main__':

    print("Head_Control Init")
    print("Head_Control Start")
    while True:
        print("I am nodding!")
        time.sleep(0.2)
        nod_head()
        time.sleep(0.5)
        
        print("I am shaking head!")
        time.sleep(0.2)
        shake_head()
        time.sleep(0.5)

        key = time.sleep(0.1)
        if key == 27:
            break
        
        