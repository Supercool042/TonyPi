#!/usr/bin/python3
# coding=utf8
import os
import sys
import time
import math
import logging
import threading

from werkzeug.serving import run_simple
from werkzeug.wrappers import Request, Response

from jsonrpc import JSONRPCResponseManager, dispatcher

import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC
from ActionGroupDict import action_group_dict

import Functions.Running as Running
import Functions.KickBall as KickBall
import Functions.Transport as Transport
import Functions.lab_adjust as lab_adjust
import Functions.ColorTrack as ColorTrack
import Functions.VisualPatrol as VisualPatrol

# 远程调用api，框架jsonrpc(remote api call using the jsonrpc framework)
# 主要用于手机端和电脑端的客户端调用(primarily used for client-side calls between mobile and desktop applications)

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

__RPC_E01 = "E01 - Invalid number of parameter!"
__RPC_E02 = "E02 - Invalid parameter!"
__RPC_E03 = "E03 - Operation failed!"
__RPC_E04 = "E04 - Operation timeout!"
__RPC_E05 = "E05 - Not callable"

board = rrc.Board()
ctl = Controller(board)

ctl.enable_recv()        


ctl.set_buzzer(1900, 0.1, 0.9, 1) # 以1900Hz的频率，持续响0.1秒，关闭0.9秒，重复1次(at a frequency of 1900Hz, beep for 0.1 seconds, then silence for 0.9 seconds, repeating once)
time.sleep(0.1)

ColorTrack.initMove()
AGC.runActionGroup('stand_slow')


HWSONAR = None
QUEUE = None

# pwm舵机控制(pwm servo control)
# 参数1：时间（ms）(parameter1:time(ms))
# 参数2：舵机个数(parameter2:servo number)
# 参数3：舵机id（1或2）(parameter3:servo id(1 or 2))
# 参数4：舵机位置（500-2500）(parameter4:servo position(500-2500)
# ...舵机id(servo id)
# ...舵机位置(servo position)
# 例如：[1000, 2, 1, 1500, 5, 1500](such as [1000, 2, 1, 1500, 5, 1500])
@dispatcher.add_method
def SetPWMServo(*args, **kwargs):
    ret = (True, (), 'SetPWMServo')
    arglen = len(args)
    if 0 != (arglen % 2):
        return (False, __RPC_E01, 'SetPWMServo')
    try:
        servos = args[2:arglen:2]
        pulses = args[3:arglen:2]
        use_times = args[0]
        for s in servos:
            if s < 1 or s > 2:
                return (False, __RPC_E02, 'SetPWMServo')
        dat = zip(servos, pulses)
        for (s, p) in dat:
            ctl.set_pwm_servo_pulse(s, p ,use_times)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SetPWMServo')
    return ret

# 串口舵机控制(interface servo control)
# 参数1：时间（ms）(parameter1:time(ms))
# 参数2：舵机个数(parameter2:servo number)
# 参数3：舵机id(parameter3:servo id)
# 参数4：舵机位置（0-500）(parameter4:servo position(0-500)
# ...舵机id(servo id)
# ...舵机位置(servo position)
# ...
# 例如：[1000, 1, 1, 500](such as [1000, 1, 1, 500])
@dispatcher.add_method
def SetBusServoPulse(*args, **kwargs):
    ret = (True, (), 'SetBusServoPulse')   
    arglen = len(args)
    if (args[1] * 2 + 2) != arglen or arglen < 4:
        return (False, __RPC_E01, 'SetBusServoPulse')
    try:
        servos = args[2:arglen:2]
        pulses = args[3:arglen:2]
        use_times = args[0]
        for s in servos:
           if s < 1 or s > 16:
                return (False, __RPC_E02, 'SetBusServoPulse')
        dat = zip(servos, pulses)
        for (s, p) in dat:
            ctl.set_bus_servo_pulse(s, p ,use_times)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SetBusServoPulse')
    return ret

# 串口舵机偏差设置(interface servo deviation setting)
# 参数：舵机偏差（-125-125）(parameter: servo deviation(-125-125))
@dispatcher.add_method
def SetBusServoDeviation(*args):
    ret = (True, (), 'SetBusServoDeviation')
    arglen = len(args)
    if arglen != 2:
        return (False, __RPC_E01, 'SetBusServoDeviation')
    try:
        servo = args[0]
        deviation = args[1]
        ctl.set_bus_servo_deviation(servo, deviation)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SetBusServoDeviation')
    return ret

# 串口舵机偏差读取(interface servo deviation reading)
# 参数：readDeviation(parameter: readDeviation)
# 返回：1-16舵机偏差(return: 1-16 servo deviation)
@dispatcher.add_method
def GetBusServosDeviation(args):
    ret = (True, (), 'GetBusServosDeviation')
    data = []
    if args != "readDeviation":
        return (False, __RPC_E01, 'GetBusServosDeviation')
    try:
        for i in range(1, 16):
            dev = ctl.get_bus_servo_deviation(i)
            if dev is None:
                dev = 999
            data.append(dev)
        ret = (True, data, 'GetBusServosDeviation')
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'GetBusServosDeviation')
    return ret 

# 串口舵机偏差保存(interface servo deviation saving)
# 参数：downloadDeviation(parameter: downloadDeviation)
@dispatcher.add_method
def SaveBusServosDeviation(args):
    ret = (True, (), 'SaveBusServosDeviation')
    if args != "downloadDeviation":
        return (False, __RPC_E01, 'SaveBusServosDeviation')
    try:
        for i in range(1, 16):
            ctl.save_bus_servo_deviation(i)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'SaveBusServosDeviation')
    return ret 

# 串口舵机掉电(interface servo power loss)
# 参数：servoPowerDown(parameter:servoPowerDown)
@dispatcher.add_method
def UnloadBusServo(args):
    ret = (True, (), 'UnloadBusServo')
    if args != 'servoPowerDown':
        return (False, __RPC_E01, 'UnloadBusServo')
    try:
        for i in range(1, 16):
            ctl.unload_bus_servo(i)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'UnloadBusServo')
    return ret

# 获取串口舵机位置(obtain interface servo position)
# 参数：angularReadback(parameter: angularReadback)
# 返回：1-16舵机位置(return: 1-16 servo position)
@dispatcher.add_method
def GetBusServosPulse(args):
    ret = (True, (), 'GetBusServosPulse')
    data = []
    if args != 'angularReadback':
        return (False, __RPC_E01, 'GetBusServosPulse')
    try:
        for i in range(1, 16):
            pulse = ctl.get_bus_servo_pulse(i)
            if pulse is None:
                ret = (False, __RPC_E04, 'GetBusServosPulse')
                return ret
            else:
                data.append(pulse)
        ret = (True, data, 'GetBusServosPulse')
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'GetBusServosPulse')
    return ret 

# 停止当前动作(stop current action)
# 参数：stopAction(parameter:stopAction)
@dispatcher.add_method
def StopBusServo(args):
    ret = (True, (), 'StopBusServo')
    if args != 'stopAction':
        return (False, __RPC_E01, 'StopBusServo')
    try:     
        AGC.stopAction()
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'StopBusServo')
    return ret

# 停止当前动作组运行(stop current action group running)
# 参数：stopActionGroup(parameter: stopActionGroup)
@dispatcher.add_method
def StopActionGroup(args):
    ret = (True, (), 'StopActionGroup')
    if args != 'stopActionGroup':
        return (False, __RPC_E01, 'StopActionGroup')
    try:     
        AGC.stopActionGroup()
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'StopActionGroup')
    return ret

# 动作组运行(action group running)
# 参数1：动作编号（字符形式）(parameter1: action number (in character format))
# 参数2：动作次数（0表示循环）(parameter 2: Number of actions (0 indicates looping))
# 例如：['1', 2](such as: ['1', 2])
th = None
have_move = True
@dispatcher.add_method
def RunAction(*args_):
    global th
    global have_move 
    
    ret = (True, (), 'RunAction')
    actName = '0'
    times = 1
    
    if len(args_) != 2:
        return (False, __RPC_E01, 'RunAction')
    try:
        if args_[0] == '0':
            if have_move:
                AGC.stopActionGroup()
                have_move = False
        else:
            if th is not None:
                if not th.is_alive():
                    if args_[0] in action_group_dict:
                        actName = action_group_dict[args_[0]]
                    else:
                        actName = args_[0]
                    times = int(args_[1])
                    th = threading.Thread(target=AGC.runActionGroup, args=(actName, times))
                    th.start()
                    have_move = True
            else:
                if args_[0] in action_group_dict:
                    actName = action_group_dict[args_[0]]
                else:
                    actName = args_[0]
                times = int(args_[1])
                th = threading.Thread(target=AGC.runActionGroup, args=(actName, times))
                th.start()
                have_move = True
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03, 'RunAction')
    return ret

# 跌倒起立检测(falling and standing up detection)
def standup():
    count1 = 0
    count2 = 0
    count3 = 0

    for i in range(20):
        try:
            data = ctl.get_imu() #获取传感器值(get sensor value)
            print("imu",data)
            angle_y = int(math.degrees(math.atan2(data[0], data[2]))) #转化为角度值(convert to the angle value)
            
            if abs(angle_y) > 160: 
                count1 += 1
            else:
                count1 = 0
            if abs(angle_y) < 10:
                count2 += 1
            else:
                count2 = 0
            time.sleep(0.1)
            count3 += 1
            if count3 > 5 and count1 < 3 and count2 < 3:
                break
            if count1 >= 10: #往后倒了一定时间后起来(lying down for a certain period and then getting up)
                count1 = 0
                AGC.runActionGroup('stand_up_back')
                break
            elif count2 >= 10: #后前倒了一定时间后起来(falling forward for a certain period of time and then standing up)
                count2 = 0
                AGC.runActionGroup('stand_up_front')
                break
        except BaseException as e:
            print(e)

# 跌倒起立调用(fall and stand up invocation)
th2 = None
@dispatcher.add_method
def StandUp():
    global th2

    ret = (True, (), 'StandUp')

    if th2 is not None:
        if not th2.is_alive():
            th2 = threading.Thread(target=standup)
            th2.start()
        else:
            pass
    else:
        th2 = threading.Thread(target=standup)
        th2.start()

    return ret

def runbymainth(req, pas):
    if callable(req):
        #print('pas', req)
        event = threading.Event()
        ret = [event, pas, None]
        QUEUE.put((req, ret))
        count = 0
        #ret[2] =  req(pas)
        #print('ret', ret)
        while ret[2] is None:
            time.sleep(0.01)
            count += 1
            if count > 200:
                break
        if ret[2] is not None:
            if ret[2][0]:
                return ret[2]
            else:
                return (False, __RPC_E03 + " " + ret[2][1])
        else:
            return (False, __RPC_E04)
    else:
        return (False, __RPC_E05)

@dispatcher.add_method
def LoadFunc(new_func = 0):
    return runbymainth(Running.loadFunc, (new_func, ))

@dispatcher.add_method
def UnloadFunc():
    return runbymainth(Running.unloadFunc, ())

@dispatcher.add_method
def StartFunc():
    return runbymainth(Running.startFunc, ())

@dispatcher.add_method
def StopFunc():
    return runbymainth(Running.stopFunc, ())

@dispatcher.add_method
def FinishFunc():
    return runbymainth(Running.finishFunc, ())

@dispatcher.add_method
def Heartbeat():
    return runbymainth(Running.doHeartbeat, ())

@dispatcher.add_method
def GetRunningFunc():
    #return runbymainth("GetRunningFunc", ())
    return (True, (0,))

# 设置跟踪颜色(set tracking color)
# 参数：颜色（red，green，blue）(parameter: color(red, green, blue))
# 例如：[(red,)](such as: [(red,)])
@dispatcher.add_method
def SetTargetTrackingColor(*target_color):
    return runbymainth(ColorTrack.setTargetColor, target_color)

# 设置巡线颜色(set line following color)
# 参数：颜色（red，green，blue）(parameter: color(red, green, blue))
# 例如：[(red,)](such as: [(red,)])
@dispatcher.add_method
def SetVisualPatrolColor(*target_color):
    return runbymainth(VisualPatrol.setLineTargetColor, target_color)

# 设置踢球颜色(set auto shooting color)
# 参数：颜色（red，green，blue）(parameter: color(red, green, blue))
# 例如：[(red,)](such as: [(red,)])
@dispatcher.add_method
def SetBallColor(*target_color):
    return runbymainth(KickBall.setBallTargetColor, target_color)

# 设置颜色阈值(set color threshold)
# 参数：颜色lab(parameter: color lab)
# 例如：[{'red': ((0, 0, 0), (255, 255, 255))}]
@dispatcher.add_method
def SetLABValue(*lab_value):
    #print(lab_value)
    return runbymainth(lab_adjust.setLABValue, lab_value)

# 保存颜色阈值(save color threshold)
@dispatcher.add_method
def GetLABValue():
    return (True, lab_adjust.getLABValue()[1], 'GetLABValue')

# 保存颜色阈值(save color threshold)
@dispatcher.add_method
def SaveLABValue(color=''):
    return runbymainth(lab_adjust.saveLABValue, (color, ))

@dispatcher.add_method
def HaveLABAdjust():
    return (True, True, 'HaveLABAdjust')

@Request.application
def application(request):
    dispatcher["echo"] = lambda s: s
    dispatcher["add"] = lambda a, b: a + b
    response = JSONRPCResponseManager.handle(request.data, dispatcher)

    return Response(response.json, mimetype='application/json')

def startRPCServer():
    run_simple('', 9030, application)

if __name__ == '__main__':
    startRPCServer()
