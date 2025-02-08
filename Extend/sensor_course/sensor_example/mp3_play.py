#!/usr/bin/python3
# coding=utf8
# 4.拓展课程学习\7.拓展课程之传感器基础开发课程\第3课 MP3模块实验(4.Advanced Lessons\7.Sensor Development Course\Lesson3 MP3 Module)
import os
import sys
import time
import signal
import hiwonder.MP3 as MP3

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


move_st = True
addr = 0x7b         #MP3 module address
mp3 = MP3.MP3(addr)

def Stop(signum, frame):
    global move_st
    move_st = False
    mp3.pause() #pause song play
    print('\n')

signal.signal(signal.SIGINT, Stop)

skip = True

if __name__ == "__main__":
    while move_st:
        if skip:
            mp3.volume(20) #set the volume to 30, please set before play.  
            mp3.playNum(18) #play song num0018
            skip = False
        else:
            time.sleep(0.05)
        
