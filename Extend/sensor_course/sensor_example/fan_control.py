#!/usr/bin/python3
# coding=utf8
# 4.拓展课程学习\7.拓展课程之传感器基础开发课程\第1课 风扇模块实验(4.Advanced Lessons\7.Sensor Development Course\Lesson1 Fan Module)
import time
import gpiod

## 初始化引脚模式(initial pin mode)
chip = gpiod.Chip("gpiochip4")
fanPin1 = chip.get_line(8)
fanPin1.request(consumer="pin1", type=gpiod.LINE_REQ_DIR_OUT)

fanPin2 = chip.get_line(7)
fanPin2.request(consumer="pin2", type=gpiod.LINE_REQ_DIR_OUT)



# initial         
def init():
    start = False
    set_fan(0)
    print("Fan Control Init")	
    
#fan control 
def set_fan(start):
               
    if start == 1:
        ## 开启风扇, 顺时针(turn on the fan, clockwise)
        fanPin1.set_value(1)  # 设置引脚输出高电平(set pin output high voltage)
        fanPin2.set_value(0)  # 设置引脚输出低电平(set pin output low voltage)
    else:
        ## 关闭风扇(turn off the fan)
        fanPin1.set_value(0)  # 设置引脚输出低电平(set pin output low voltage)
        fanPin2.set_value(0)  # 设置引脚输出低电平(set pin output low voltage)

if __name__ == '__main__': 
    while True:
        try:
            set_fan(1)
        except KeyboardInterrupt:
            set_fan(0)
            break
    
    
