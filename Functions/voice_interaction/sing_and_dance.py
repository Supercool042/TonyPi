#!/usr/bin/python3
# coding=utf8
from speech import speech
import hiwonder.ActionGroupControl as AGC

action_name = '21'
speech.play_audio("/home/pi/TonyPi/audio/{}.wav".format(action_name), volume=70, block=False)
AGC.runActionGroup(action_name)

