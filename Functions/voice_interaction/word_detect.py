#!/usr/bin/python3
# coding=utf8
import serial, os, sys
from speech import speech

cmd_dict = {b"\xaa\x55\x03\x00\xfb": 'wakeup',
            b"\xaa\x55\x02\x00\xfb": 'sleep',
            b"\xaa\x55\x00\x01\xfb": 'forward',
            b"\xaa\x55\x00\x02\xfb": 'back',
            b"\xaa\x55\x00\x03\xfb": 'turn_left',
            b"\xaa\x55\x00\x04\xfb": 'turn_right'}

class WonderEcho:
    def __init__(self, port):
        self.serialHandle = serial.Serial(None, 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=0.02)
        self.serialHandle.rts = False
        self.serialHandle.dtr = False
        self.serialHandle.setPort(port)
        self.serialHandle.open()
        self.serialHandle.reset_input_buffer()

    def detect(self):
        return self.serialHandle.read(5)

    def exit(self):
        self.serialHandle.close()

if __name__ == '__main__':
    wonderecho = WonderEcho('/dev/ttyUSB0')
    while True:
        try:
            res = wonderecho.detect()
            if res != b'':
                if res in cmd_dict:
                    print(cmd_dict[res])
        except KeyboardInterrupt:
            wonderecho.exit()
            break
