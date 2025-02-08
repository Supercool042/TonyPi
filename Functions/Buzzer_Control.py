import time
import hiwonder.ros_robot_controller_sdk as rrc


board = rrc.Board()

board.set_buzzer(1900, 0.1, 0.9, 1) # 以1900Hz的频率，持续响0.1秒，关闭0.9秒，重复1次(at a frequency of 1900Hz, sound for 0.1 seconds, then pause for 0.9 seconds, repeat once)

time.sleep(1) # 延时(delay)

board.set_buzzer(1000, 0.5, 0.5, 1)

