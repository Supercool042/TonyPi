#!/usr/bin/python3
# coding=utf8
import sys
import os
import time
import json
import pygame
import asyncio
import threading
import websockets
import subprocess
import hiwonder.ActionGroupControl as AGC
import hiwonder.ros_robot_controller_sdk as rrc

board = rrc.Board()
#ps2手柄控制动作, 已以system方式自启动，无需再开启(the PS2 controller controls actions. It's already set to start automatically using the system, so there's no need to start it again)

key_map = {"PSB_CROSS":0, "PSB_CIRCLE":1, "PSB_SQUARE":3, "PSB_TRIANGLE":4,
        "PSB_L1": 6, "PSB_R1":7, "PSB_L2":8, "PSB_R2":9,
        "PSB_SELECT":10, "PSB_START":11, "PSB_L3":13, "PSB_R3":14}
# key_map = {"PSB_CROSS":2, "PSB_CIRCLE":1, "PSB_SQUARE":3, "PSB_TRIANGLE":0,
#         "PSB_L1": 4, "PSB_R1":5, "PSB_L2":6, "PSB_R2":7,
#         "PSB_SELECT":8, "PSB_START":9, "PSB_L3":10, "PSB_R3":11};
action_map = ["CROSS", "CIRCLE", "", "SQUARE", "TRIANGLE", "L1", "R1", "L2", "R2", "SELECT", "START", "", "L3", "R3"]

def joystick_init():
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.display.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        js=pygame.joystick.Joystick(0)
        js.init()
        jsName = js.get_name()
        print("Name of the joystick:", jsName)
        jsAxes=js.get_numaxes()
        print("Number of axif:",jsAxes)
        jsButtons=js.get_numbuttons()
        print("Number of buttons:", jsButtons)
        jsBall=js.get_numballs()
        print("Numbe of balls:", jsBall)
        jsHat= js.get_numhats()
        print("Number of hats:", jsHat)


async def call_rpc(method, params=None):
    websocket = None
    try:
        websocket = await websockets.connect('ws://192.168.149.1:7788/up')
        call = dict(jsonrpc='2.0', method=method)
        if params is not None:
            call['params'] = params
#         logger.debug(json.dumps(call))
        await websocket.send(json.dumps(call))
        await websocket.close()
    except Exception as e:
#         logger.error(e)
        if websocket is not None:
            await websocket.close()

async def run_action_set(action_set_name, repeat):
    await call_rpc('run_action_set', [action_set_name, repeat])

async def stop(action_set_name=None):
    await call_rpc('stop')
    if action_set_name is not None:
        await run_action_set(action_set_name, 1)
        
athletics_perform_finish = True
th = None
last_status = ''
connected = False
action_name = None
status = 'init'
last_buttons = key_map
aplay_thread = None
while True:
    if os.path.exists("/dev/input/js0") is True:
        if connected is False:
            joystick_init()
            jscount =  pygame.joystick.get_count()
            if jscount > 0:
                try:
                    js=pygame.joystick.Joystick(0)
                    js.init()
                    connected = True
                except Exception as e:
                    print(e)
            else:
                pygame.joystick.quit()
    else:
        if connected is True:
            connected = False
            js.quit()
            pygame.joystick.quit()
    if connected is True:
        pygame.event.pump()     
        actName = None
        times = 1
        
        try:
            buttons = {}
            for i in key_map:
                buttons[i] = js.get_button(key_map[i])
            if js.get_button(key_map["PSB_SELECT"]):
                time.sleep(0.01)
                if last_buttons != buttons:
                    os.system('amixer -q -D pulse set Master {}%'.format(20))
                    action_name = None
                    if js.get_button(key_map["PSB_START"]):
                        athletics_perform_finish = False
                        subprocess.Popen("python3 /home/pi/TonyPi/Extend/athletics_course/athletics_perform.py",shell=True)
                        time.sleep(1)
                    elif js.get_button(key_map["PSB_L1"]):
                        action_name = '21'
                        board.set_buzzer(1900, 0.1, 0.1, 2)
                    elif js.get_button(key_map["PSB_L2"]):
                        action_name = '22'
                        board.set_buzzer(1900, 0.1, 0.1, 2)
                    elif js.get_button(key_map["PSB_R1"]):
                        action_name = '23'
                        board.set_buzzer(1900, 0.1, 0.1, 2)
                    elif js.get_button(key_map["PSB_R2"]):
                        action_name = '24'
                        board.set_buzzer(1900, 0.1, 0.1, 2)
                    elif js.get_button(key_map["PSB_CIRCLE"]):
                        action_name = '18'
                        board.set_buzzer(1900, 0.1, 0.1, 2)
                    elif js.get_button(key_map["PSB_SQUARE"]):
                        action_name = '19'
                        board.set_buzzer(1900, 0.1, 0.1, 2)
                    elif js.get_button(key_map["PSB_TRIANGLE"]):
                        action_name = '20'
                        board.set_buzzer(1900, 0.1, 0.1, 2)
                    elif js.get_button(key_map["PSB_CROSS"]):
                        board.set_buzzer(1900, 0.1, 0.1, 2)
                        athletics_perform_finish = True
                        os.system("/home/pi/TonyPi/Extend/test.sh")
            elif athletics_perform_finish:
                if js.get_button(key_map["PSB_R1"]):
                    actName = 'right_kick'
                    # print("right_kick")
                if js.get_button(key_map["PSB_R2"]):
                    actName = 'turn_right'
                    # print("turn_right")
                if js.get_button(key_map["PSB_L1"]):
                    actName = 'left_kick'
                    # print("left_kick")
                if js.get_button(key_map["PSB_L2"]):
                    actName = 'turn_left'
                    # print("turn_left")
                if js.get_button(key_map["PSB_SQUARE"]): #正方形(square)
                    actName = 'twist'
                    # print("twist")
                if js.get_button(key_map["PSB_CIRCLE"]): #圈(circle)
                    actName = 'right_shot_fast'
                    # print("right_shot_fast")
                if js.get_button(key_map["PSB_TRIANGLE"]): #三角(triangle)
                    actName = 'wave'
                    # print("wave")
                if js.get_button(key_map["PSB_CROSS"]): #叉(cross)
                    actName = 'bow'
                    # print("bow")
                if  js.get_hat(0)[0] == 0  and  js.get_hat(0)[1] == 1:
                    last_status = 'go'
                    actName = 'go_forward'
                    # print("go_forward")
                    times = 0
                elif js.get_hat(0)[0] == 0  and  js.get_hat(0)[1] == -1:
                    last_status = 'back_fast'
                    actName = 'back_fast'
                    # print("back_fast")
                    times = 0
                elif js.get_hat(0)[0] == -1  and  js.get_hat(0)[1] == 0:
                    actName = 'left_move'
                    # print("left_move")
                elif js.get_hat(0)[0] == 1  and  js.get_hat(0)[1] == 0:
                    actName = 'right_move' 
                    # print("right_move") 
                else:
                    lx = js.get_axis(0)
                    ly = js.get_axis(1)
                    
                    if lx < -0.5 :
                        actName = 'left_move'
                    elif lx > 0.5:
                        actName = 'right_move'
                    if ly < -0.5 :
                        last_status = 'go'
                        actName = 'go_forward'
                        times = 0
                    elif ly > 0.5:
                        last_status = 'back_fast'
                        actName = 'back_fast'
                        times = 0
                    else:
                        if (last_status == 'go' or last_status == 'back_fast') and actName is None:
                            AGC.stopActionGroup()
                            last_status = ''
                if js.get_button(key_map["PSB_START"]):
                    if status == 'play':
                        AGC.stopAction()
                        if aplay_thread is not None:
                            aplay_thread.terminate()
                            aplay_thread.wait()
                            aplay_thread = None
                            time.sleep(0.5)
                        status = 'init'
                    actName = 'stand_slow'
                    board.set_buzzer(1900, 0.1, 0.9, 1) # 以1900Hz的频率，持续响0.1秒，关闭0.9秒，重复1次(at a frequency of 1900Hz, beep for 0.1 seconds, then silence for 0.9 seconds, repeating once)
            if action_name is not None:
                threading.Thread(target=AGC.runActionGroup, args=(action_name, 1, True), daemon=True).start()

                aplay_thread = subprocess.Popen(["aplay", "/home/pi/TonyPi/audio/{}.wav".format(action_name)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                status = 'play'
                action_name = None
            if actName is not None:
                if th is not None:
                    if not th.is_alive():
                        asyncio.run(run_action_set(actName, 1))
                        th = threading.Thread(target=AGC.runActionGroup, args=(actName, times), daemon=True)
                        th.start()
                else:
                    asyncio.run(run_action_set(actName, 1))
                    th = threading.Thread(target=AGC.runActionGroup, args=(actName, times), daemon=True)
                    th.start()
            last_buttons = buttons
        except Exception as e:
            print(e)
            connected = False          
    time.sleep(0.01)
