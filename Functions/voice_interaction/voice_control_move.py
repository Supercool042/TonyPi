#!/usr/bin/python3
# coding=utf8
import serial, os, sys
from speech import speech
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle
import hiwonder.ros_robot_controller_sdk as rrc

'''
	程序功能：语音控制TonyPi(program function: voice control TonyPi)

    运行效果：说唤醒词“小幻小幻”。听到回答“我在”之后，说出以下指令之一，机器人会执行相应动作,
              唤醒15秒之内不用再次唤醒，15秒之后会dong的表示进入休眠状态，此时需要再次唤醒，如果15秒内
              有持续的指令控制，会刷新15秒间隔
              指令：前进/后退/左转/右转

    对应教程文档路径：1 教程资料\10.语音交互课程\11.2 语音控制TonyPi
'''

# 添加当前脚本所在目录的上一级目录的绝对路径(add the absolute path of the parent directory of the directory where the current script is located)
last_dir_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(last_dir_path)
from ActionGroupDict import action_group_dict

# 初始化机器人底层驱动(initialize the robot's low-level drivers)
board = rrc.Board()
ctl = Controller(board)

#获取舵机配置数据(get servo configuration data)
servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

ctl.set_pwm_servo_pulse(1, 1500, 500)
ctl.set_pwm_servo_pulse(2, servo_data['servo2'], 500)
AGC.runActionGroup('stand')

cmd_dict = {b"\xaa\x55\x03\x00\xfb": 'wakeup',
            b"\xaa\x55\x02\x00\xfb": 'sleep',
            b"\xaa\x55\x00\x01\xfb": 'forward',
            b"\xaa\x55\x00\x02\xfb": 'back',
            b"\xaa\x55\x00\x03\xfb": 'turn_left',
            b"\xaa\x55\x00\x04\xfb": 'turn_right'}

audio_path = os.path.join(os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], 'audio'))) 

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
    speech.play_audio(os.path.join(audio_path, 'ready.wav'), volume=70)
    while True:
        try:
            res = wonderecho.detect()
            if res != b'':
                if res in cmd_dict:
                    if cmd_dict[res] == 'wakeup':
                        speech.play_audio(os.path.join(audio_path, 'wakeup.wav'), volume=70)
                        print('wakeup')
                    elif cmd_dict[res] == 'sleep':
                        speech.play_audio(os.path.join(audio_path, 'dong.wav'), volume=70) 
                        print('sleep')
                    elif cmd_dict[res] == 'forward':
                        speech.play_audio(os.path.join(audio_path, 'ok.wav'), volume=70)
                        AGC.runActionGroup('go_forward', 2 , True)
                        print('forward')
                    elif cmd_dict[res] == 'back':
                        speech.play_audio(os.path.join(audio_path, 'ok.wav'), volume=70)
                        AGC.runActionGroup('back', 2 , True)
                        print('back')
                    elif cmd_dict[res] == 'turn_left':
                        speech.play_audio(os.path.join(audio_path, 'ok.wav'), volume=70)
                        AGC.runActionGroup('turn_left', 2 , True)
                        print('turn_left')
                    elif cmd_dict[res] == 'turn_right':
                        speech.play_audio(os.path.join(audio_path, 'ok.wav'), volume=70)
                        AGC.runActionGroup('turn_right', 2 , True)
                        print('turn_right')
        except KeyboardInterrupt:
            wonderecho.exit()
            break
