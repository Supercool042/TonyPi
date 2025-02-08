#!/usr/bin/python3
# coding=utf8
import time
import VL53L0X
import threading
import hiwonder.ActionGroupControl as AGC


# Create a VL53L0X object
tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
tof.open()

# Start ranging
tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

timing = tof.get_timing()

if (timing < 20000):
    timing = 20000
print ("Timing %d ms" % (timing/1000))

AGC.runActionGroup('stand')  # 初始姿态(initial posture)

distance = None
last_distance = 999

#执行动作组线程(execute action group thread)
def move():
    global distance, last_distance
    while True:
        if distance is not None:
            if distance > 350: # 检测距离大于350mm时(when the detected distance is greater than 350mm)
                if last_distance <= 350: # 如果上次距离大于350mm, 说明是刚转到检测不到障碍物，但是没有完全转正(if the previous distance is greater than 350mm, it indicates that the robot has just turned and cannot detect obstacles, but has not fully straightened)
                    last_distance = distance
                    AGC.runActionGroup('turn_right', 3)  # 右转3次(turn right three times)
                else:
                    last_distance = distance
                    AGC.runActionGroup('go_forward')  # 直走(go straight)
            elif 150 < distance <= 350: # 检测距离在150-350mm时(when the detected distance is between 150 and 350 mm)
                last_distance = distance
                AGC.runActionGroup('turn_right')  # 右转(turn right)
            else:
                last_distance = distance
                AGC.runActionGroup('back_fast') # 后退(go backward)
        else:
            time.sleep(0.01)
        
#启动动作的线程(start the action thread)
th = threading.Thread(target=move)
th.daemon = True
th.start()

while True:
    distance = tof.get_distance() # 获取测距(obtain distance measurement)
    if (distance > 0):
        print ("%d mm" % distance)
    time.sleep(timing/1000000.00)
tof.stop_ranging()  # 关闭测距(close distance measurement)
tof.close()
