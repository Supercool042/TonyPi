#!/usr/bin/python3
# coding=utf8
import sys
import os
import time

import hiwonder.TTS as TTS
import hiwonder.ASR as ASR
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle

'''
    程序功能：语音控制TonyPi(program function: voice control TonyPi)

    运行效果：   靠近语音识别模块的麦克风，先说唤醒词“开始”。当模块上的 STA 指示灯变为蓝色
                常亮时，再说其它词条，例如“qianjin（前进）”、“xianghoutui（向后退）”等。
                当识别到后，语音识别模块上的 STA 指示灯会熄灭，语音播报模块将播放“收到”的
                声音作为反馈，然后机器人便进行执行一次对应的动作。(running effect: "Approach the microphone near the voice recognition module, and say the wake-up word 'start.' 
                When the STA indicator light on the module turns solid blue, say other commands such as 'qianjin (forward)' or 'xianghoutui (move backward).' 
                When recognized, the STA indicator light on the voice recognition module will turn off, and the voice playback module will play the sound 'received' as feedback. Then the robot will execute the corresponding action once)
                    

    对应教程文档路径：  TonyPi智能视觉人形机器人\4.拓展课程学习\1.语音交互及智能搬运课程（语音模块选配）\第2课 语音控制TonyPi(corresponding tutorial file path: TonyPi Intelligent Vision Humanoid Robot\4.Expanded Courses\1.Voice Interaction and Intelligent Transportation(voice module optional)\Lesson2 Voice Control TonyPi)
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
try:
    asr = ASR.ASR()
    tts = TTS.TTS()

    asr.eraseWords()
    asr.setMode(2)
    asr.addWords(1, 'kai shi')
    asr.addWords(2, 'wang qian zou')
    asr.addWords(2, 'qian jin')
    asr.addWords(2, 'zhi zou')
    asr.addWords(3, 'wang hou tui')
    asr.addWords(4, 'xiang zuo yi')
    asr.addWords(5, 'xiang you yi')

    data = asr.getResult()
    ctl.set_pwm_servo_pulse(1, 1500, 500)
    ctl.set_pwm_servo_pulse(2, servo_data['servo2'], 500)
    AGC.runActionGroup('stand')
    action_finish = True
    tts.TTSModuleSpeak('[h0][v10][m3]', '准备就绪')
    print('''当前为口令模式，每次说指令前均需要说口令来激活(in password mode, activation requires speaking the passphrase before each command)
口令：开始(command: kai shi)
指令2：往前走(command2:wang qian zou)
指令2：前进(command2: qian jin)
指令2：直走(command2: zhi zou)
指令3：往后退(command3: wang hou tui)
指令4：向左移(command4: xiang zuo yi)
指令5：向右移(command5:xiang you yi)''')
    time.sleep(2)
except:
    print('传感器初始化出错')

while True:
    data = asr.getResult()
    if data:
        print('result:', data)
        tts.TTSModuleSpeak('', '收到')
        time.sleep(1)
        AGC.runActionGroup(action_group_dict[str(data - 1)], 2 ,True)
    time.sleep(0.01)
