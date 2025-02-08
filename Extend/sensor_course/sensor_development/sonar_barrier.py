#!/usr/bin/python3
# coding=utf8
#4.拓展课程学习\8.拓展课程之传感器应用开发课程\第3课 躲避障碍\第2节 躲避障碍(4.Advanced Lessons\8.Sensor Development Course\Lesson3 Obstacle Avoidance\2.Obstacle Avoidance)
import os
import sys
import time
import threading
import numpy as np
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.Sonar as Sonar
import hiwonder.ActionGroupControl as AGC

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 超声波避障(ultrasonic obstacle avoidance)

board = rrc.Board()
ctl = Controller(board)

# 抬起左手(raise your left hand)
def hand_up():
    ctl.set_bus_servo_pulse(8, 330, 1000)
    time.sleep(0.3)
    ctl.set_bus_servo_pulse(7,860,1000)
    ctl.set_bus_servo_pulse(6,860,1000)
    time.sleep(1)
# 放下左手(put down your left hand)
def hand_down():
    ctl.set_bus_servo_pulse(7,800,1000)
    ctl.set_bus_servo_pulse(6,575,1000)
    time.sleep(0.3)
    ctl.set_bus_servo_pulse(8,725,1000)
    time.sleep(1)
# 向左边伸手(reach your hand to the left)
def hand_left():
    ctl.set_bus_servo_pulse(8,330,1000)
    time.sleep(0.3)
    ctl.set_bus_servo_pulse(7,500,1000)
    ctl.set_bus_servo_pulse(6,920,1000)
    time.sleep(1)

distance = 99999
#机器人移动子线程(robot movement sub-thread)
def move():
    global distance
    
    dist_left = []
    dist_right = []
    distance_left = 99999
    distance_right = 99999
    
    while True:
        if distance != 99999:
            if distance <= 300: #检测前方障碍物(detect obstacles ahead)
                distance = 99999
                hand_left() #向左边伸手(reach your hand to the left)
                time.sleep(1)
                #连续检测左边三次(continuously detect left side three times)
                for i in range(3):
                    dist_left.append(distance)
                    time.sleep(0.05)
                #取平均值(take average value)
                distance_left = round(np.mean(np.array(dist_left)))
                dist_left = []
                hand_up()
                
                if distance_left <= 300: #检测左边障碍物(detect obstacles on the left side)
                    distance_left = 99999
                    hand_down() # 放下左手(put down your left hand)
                    for i in range(5): #向右转(turn right)
                        AGC.runActionGroup('turn_right')
                        print("turn_right")
                        time.sleep(0.2)
                        
                    hand_up()
                    time.sleep(1)
                    # 连续检测右边三次(continuously detect right side three times)
                    for i in range(3):
                        dist_right.append(distance)
                        time.sleep(0.05)
                        
                    distance_right = round(np.mean(np.array(dist_right)))
                    dist_right = []
                    
                    if distance_right <= 300: #检测右边障碍物(detect obstacles on the right side)
                        distance_right = 99999
                        hand_down()
                        for i in range(5): #向左转(turn left)
                            AGC.runActionGroup('turn_left')
                            print("turn_left")
                            time.sleep(0.2)
                            
                        for i in range(5):#后退(move backward)
                            AGC.runActionGroup('back')
                            print("back")
                        hand_up()
                    else: #右边没有障碍物,则直走,前面已经右转(if there are no obstacles on the right, proceed straight. You have already turned right ahead)
                        AGC.runActionGroup('go_hand_up1')
                        print("go")
                else: #左边没有障碍物,则向左转(if there are no obstacles on the left, turn left)
                    hand_down()
                    for i in range(5):
                        AGC.runActionGroup('turn_left')
                        print("turn_left")
                        time.sleep(0.2)
                    hand_up()
            else:#前方没有障碍物,则直走(if there are no obstacles ahead, proceed straight)
                AGC.runActionGroup('go_hand_up1')
                print("go")
        else:   
            time.sleep(0.01)
            
#作为子线程开启(start as a sub-thread)
th = threading.Thread(target=move)
th.daemon = True
th.start()

if __name__ == "__main__":
    
    distance_list = []
    s = Sonar.Sonar()
    s.startSymphony()
    
    AGC.runActionGroup('stand_slow')
    time.sleep(1)
    hand_up()
    
    while True:
        
        distance_list.append(s.getDistance())
        #print("distance:",s.getDistance())
        #连续检测6次，取平均值(continuously detect 6 times and take the average)
        if len(distance_list) >= 6: 
            #print(distance_list)
            distance = int(round(np.mean(np.array(distance_list))))
            print(distance, 'mm')
            distance_list = []
            
        time.sleep(0.01)
            
        
                       
    
