#!/usr/bin/python3
# coding=utf8
#4.拓展课程学习\8.拓展课程之传感器应用开发课程\第2课 能歌善舞\第2节 能歌善舞(4.Advanced Lessons\8.Sensor Development Course\Lesson2 Sing and Dance\2.Sing and Dance)
import os
import sys
import time
import signal
import threading
import gpiod
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.MP3 as MP3
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 触摸控制跳舞(touch control dance)

move_st = True
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
    ctl.set_pwm_servo_pulse(1, 1500, 500)
    ctl.set_pwm_servo_pulse(2, servo_data['servo2'], 500)
  
def Stop(signum, frame):
    global move_st
    print('关闭中...')
    move_st = False
    mp3.pause() #暂停(pause)

num = 0
time_ = 0
touch = False
state = True
pause_en = False
time_wait = False

def move(num_):
    global pause_en
    print(num)
    if num_ == '1':
        pause_en = True
        mp3.volume(30) #设置音量为30，注意在播放前设置(set the volume to 30. Remember to set it before playing)
        mp3.playNum(18) #播放歌曲18(play song 18)
        time.sleep(0.8)
        AGC.runActionGroup('18')
    elif num_ == '2':
        pause_en = True
        mp3.volume(30) 
        mp3.playNum(22) 
        time.sleep(0.8)
        AGC.runActionGroup('22')
    elif num_ == '3':
        pause_en = True
        mp3.volume(30) 
        mp3.playNum(24) 
        time.sleep(0.8)
        AGC.runActionGroup('24')
    else:
        time.sleep(0.3)
        board.set_buzzer(1900, 0.2, 0.8, 1) 
        time.sleep(0.1)
        board.set_buzzer(1900, 0.2, 0.8, 1) 
    
    pause_en = False

chip = gpiod.Chip('gpiochip4')
touchPin = chip.get_line(7)
touchPin.request(consumer="touch", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)

if __name__ == "__main__":  
    
    addr = 0x7b         #传感器iic地址(sensor I2C address)
    mp3 = MP3.MP3(addr)
    AGC.runActionGroup('stand_slow')
    initMove()
    
    while move_st:
        touch = touchPin.get_value()   #读取引脚数字值(read pin numerical vlaue)
        
        if touch:
           state = True
           
        elif not touch and state:
            num += 1
            state = False
            board.set_buzzer(1900, 0.1, 0.9, 1)    #设置蜂鸣器响(set buzzer to emit)
            if num == 1:
                time_wait = True
                time_ = time.time()
            time.sleep(0.1)
                
        if time_wait:
            if int(time.time() - time_) >= 1 :
                if not pause_en:
                    th = threading.Thread(target=move, args=str(num), daemon=True)
                    th.start()
                    num = 0
                    time_wait = False
                    
                elif pause_en:
                    print('pause')
                    num = 0
                    pause_en = False
                    time_wait = False
                    mp3.pause() #暂停(pause)
                    AGC.stopAction()
                    time.sleep(0.5)
                    AGC.runActionGroup('stand_slow')
        else:        
            time.sleep(0.01)
    
    
