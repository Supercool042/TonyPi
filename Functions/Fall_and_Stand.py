#!/usr/bin/python3
# coding=utf8
import sys
import time
import math
import hiwonder.ros_robot_controller_sdk as rrc
import hiwonder.ActionGroupControl as AGC



count1 = 0
count2 = 0

def stand_up():
    global count1, count2 
    
    try:
        accel_date = board.get_imu()
        angle_y = int(math.degrees(math.atan2(accel_date[0], accel_date[2]))) #将获得的数据转化为角度值(convert the obtained data into angle values)
        
        if abs(angle_y) > 160: #y轴角度大于160，count1加1，否则清零(if the y-axis angle is greater than 160, increment count1 by 1; otherwise, reset it to zero)
            count1 += 1
        else:
            count1 = 0

        if abs(angle_y) < 10: #y轴角度小于10，count2加1，否则清零(if the y-axis angle is less than 10, increment count2 by 1; otherwise, reset it to zero)
            count2 += 1
        else:
            count2 = 0

        time.sleep(0.1)
        
        if count1 >= 5: #往后倒了一定时间后起来(after a certain amount of time has passed, rise up from the backward position)
            count1 = 0  
            print("stand up back！")#打印执行的动作名(print the name of performed action)
            AGC.runActionGroup('stand_up_back')#执行动作(execute action)
        
        elif count2 >= 5: #往前倒了一定时间后起来(after tilting forward for a certain amount of time, rise up)
            count2 = 0
            print("stand up front！")#打印执行的动作名(print the name of the executed action)
            AGC.runActionGroup('stand_up_front')#执行动作 (execute action)
        
    except BaseException as e:
        print(e)

if __name__ == '__main__':
    # 初始化机器人底层驱动(initialize robot underlying driver)
    board = rrc.Board()
    board.enable_reception()   # 使能数据接收(enable data reception)
    time.sleep(1)

    print("Fall_and_Stand Init")
    print("Fall_and_Stand Start")

    try:
        while True :#循环检测机器人的状态(loop to check the status of the robot)
            stand_up()
            time.sleep(0.1)
    except KeyboardInterrupt:      
        print("接收到中断信号。退出循环。")
