#!/usr/bin/python3
# coding=utf8
# 4.拓展课程学习\7.拓展课程之传感器基础开发课程\第6课 光线传感器模块实验(4.Advanced Lessons\7.Sensor Development Course\Lesson6 Light Sensor Module)
import os
import sys
import time
import gpiod
import hiwonder.ros_robot_controller_sdk as rrc

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


board = rrc.Board()
    
st = 0

chip = gpiod.Chip('gpiochip4')
light = chip.get_line(24)
light.request(consumer="light", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)

if __name__ == "__main__":
    while True:
        state = light.get_value()  #读取引脚数字值(read pin numerical value)
        print(state)
        if not state:
            if st :            #这里做一个判断，防止反复响(make a judgement to prevent repeated ringing here)
                st = 0
                board.set_buzzer(1900, 0.1, 0.9, 1) # 以1900Hz的频率，持续响0.1秒，关闭0.9秒，重复1次(at a frequency of 1900Hz, sound for 0.1 seconds, then pause for 0.9 seconds, repeat once)
                time.sleep(1)
                
        else:
            st = 1
            board.set_buzzer(1000, 0.0, 0.0, 1) # 关闭(close)
            
        board.set_buzzer(1000, 0.0, 0.0, 1) # 关闭(close)
