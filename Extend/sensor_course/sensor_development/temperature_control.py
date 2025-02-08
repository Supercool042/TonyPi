#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/09/21
import smbus
import gpiod

class AHT10:
    CONFIG = [0x08, 0x00]
    MEASURE = [0x33, 0x00]

    def __init__(self, bus=1, addr=0x38):
        self.bus = smbus.SMBus(bus)
        self.addr = addr
        time.sleep(0.2) 

    def getData(self):
        byte = self.bus.read_byte(self.addr)
        self.bus.write_i2c_block_data(self.addr, 0xAC, self.MEASURE)
        time.sleep(0.5)
        data = self.bus.read_i2c_block_data(self.addr, 0x00)
        temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
        ctemp = ((temp*200) / 1048576) - 50
        hum = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4
        chum = int(hum * 100 / 1048576)
        
        return (ctemp, chum)

aht10 = AHT10()

## 初始化引脚模式(initial pin mode)
chip = gpiod.Chip("gpiochip4")
fanPin1 = chip.get_line(8)
fanPin1.request(consumer="pin1", type=gpiod.LINE_REQ_DIR_OUT)

fanPin2 = chip.get_line(7)
fanPin2.request(consumer="pin2", type=gpiod.LINE_REQ_DIR_OUT)

def set_fan(start):
    if start == 1:
        ## 开启风扇, 顺时针(turn on the fan, clockwise)
        fanPin1.set_value(1)  # 设置引脚输出高电平(set pin output high voltage)
        fanPin2.set_value(0)  # 设置引脚输出低电平(set pin output low voltage)
    else:
        ## 关闭风扇(close fan)
        fanPin1.set_value(0)  # 设置引脚输出低电平(set pin output low voltage)
        fanPin2.set_value(0)  # 设置引脚输出低电平(set pin output low voltage)

count = 0
while True:
    t = int(aht10.getData()[0])
    if t > 30:
        count += 1
        if count > 5:
            set_fan(1)
            count = 0
    else:
        count = 0
        set_fan(0)
