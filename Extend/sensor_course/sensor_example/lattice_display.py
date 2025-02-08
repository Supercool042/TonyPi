#!/usr/bin/python3
# coding=utf8
# 4.拓展课程学习\7.拓展课程之传感器基础开发课程\第5课 点阵模块实验(4.Advanced Lessons\7.Sensor Development Course\Lesson5 Dot Matrix Module)
import time
from hiwonder import dot_matrix_sensor

dms = dot_matrix_sensor.TM1640(dio=7, clk=8)
if __name__ == "__main__":

    while True:
        try:
            dms.display_buf=(0x7f, 0x08, 0x7f, 0x00, 0x7c, 0x54, 0x5c, 0x00,
                              0x7c, 0x40, 0x00,0x7c, 0x40, 0x38, 0x44, 0x38)
            dms.update_display()
            time.sleep(5)
        except KeyboardInterrupt:
            dms.display_buf = [0]*16
            dms.update_display()
            break


