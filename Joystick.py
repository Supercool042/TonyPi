#!/usr/bin/python3
# coding=utf8
import sys
import os
import time
import json
import pygame
import asyncio
import threading
import subprocess
import websockets
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
        
shield = True
th = None
last_status = ''
connected = False
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
        #buttons = [js.get_button(i) for i in range(15)]
        #print(buttons)
        # hats = [js.get_hat(i) for i in range(1)]
        # print(hats)
        
        try:
            if js.get_button(key_map["PSB_R1"]):
                if shield:
                    actName = 'right_kick'
                    print("right_kick")
            if js.get_button(key_map["PSB_R2"]):
                if shield:
                    actName = 'turn_right'
                    print("turn_right")
            if js.get_button(key_map["PSB_L1"]):
                if shield:
                    actName = 'left_kick'
                    print("left_kick")
            if js.get_button(key_map["PSB_L2"]):
                if shield:
                    actName = 'turn_left'
                    print("turn_left")
            if js.get_button(key_map["PSB_SQUARE"]): #正方形(square)
                if shield:
                    actName = 'twist'
                    print("twist")
            if js.get_button(key_map["PSB_CIRCLE"]): #圈(circle)
                if shield:
                    actName = 'right_shot_fast'
                    print("right_shot_fast")
            if js.get_button(key_map["PSB_TRIANGLE"]): #三角(triangle)
                if shield:
                    actName = 'wave'
                    print("wave")
            if js.get_button(key_map["PSB_CROSS"]): #叉(cross)
                if shield:
                    actName = 'bow'
                    print("bow")
            if  js.get_hat(0)[0] == 0  and  js.get_hat(0)[1] == 1:
                if shield:
                    last_status = 'go'
                    actName = 'go_forward'
                    print("go_forward")
                times = 0
            elif js.get_hat(0)[0] == 0  and  js.get_hat(0)[1] == -1:
                if shield:
                    last_status = 'back'
                    actName = 'back_fast'
                    print("back_fast")
                times = 0
            elif js.get_hat(0)[0] == -1  and  js.get_hat(0)[1] == 0:
                if shield:
                    actName = 'left_move'
                    print("left_move")
            elif js.get_hat(0)[0] == 1  and  js.get_hat(0)[1] == 0:
                if shield:
                    actName = 'right_move' 
                    print("right_move") 
            else:
                if (last_status == 'go' or last_status == 'back') and actName is None:
                    AGC.stopActionGroup()
                    last_status = '' 
                 
            lx = js.get_axis(0)
            ly = js.get_axis(1)
            
            if lx < -0.5 :
                if shield:
                    actName = 'left_move'
            elif lx > 0.5:
                if shield:
                    actName = 'right_move'
            l3_state = js.get_button(key_map["PSB_L3"])
            if ly < -0.5 :
                if not l3_state:
                    if shield:
                        last_status = 'go'
                        actName = 'go_forward'
                    times = 0
            elif ly > 0.5:
                if not l3_state:
                    if shield:
                        last_status = 'back'
                        actName = 'back_fast'
                    times = 0
            else:
                if (last_status == 'go' or last_status == 'back') and actName is None:
                    AGC.stopActionGroup()
                    last_status = ''
            if js.get_button(key_map["PSB_START"]):
                if shield:
                    actName = 'stand_slow'
                    board.set_buzzer(1900, 0.1, 0.9, 1) # 以1900Hz的频率，持续响0.1秒，关闭0.9秒，重复1次(at a frequency of 1900Hz, beep for 0.1 seconds, then silence for 0.9 seconds, repeating once)
                
            if js.get_button(key_map["PSB_SELECT"]):
                time.sleep(0.01)
                if js.get_button(key_map["PSB_START"]):
                    shield = False
                    subprocess.Popen("python3 /home/pi/TonyPi/Extend/athletics_course/athletics_perform.py",shell=True)
                    time.sleep(1)
                else:
                    shield = True
                    os.system("/home/pi/TonyPi/Extend/test.sh")
                
            if th is not None:
                if actName is not None:
                    if not th.is_alive():
                        asyncio.run(run_action_set(actName, 1))
                        th = threading.Thread(target=AGC.runActionGroup, args=(actName, times), daemon=True)
                        th.start()
            else:
                asyncio.run(run_action_set(actName, 1))
                th = threading.Thread(target=AGC.runActionGroup, args=(actName, times), daemon=True)
                th.start()
        except Exception as e:
            print(e)
            connected = False          
    time.sleep(0.01)
