# music.py

import time
from HiwonderSDK.hiwonder import MP3
from hsv import ServoController
from yaml_handle import get_yaml_data


class MusicController:
    def __init__(self):
        # 初始化舵机控制器
        self.servo_controller = ServoController()

        # 初始化 MP3 播放器
        self.mp3_player = MP3()

        # 读取舵机配置
        servo_config = get_yaml_data('/home/pi/TonyPi/servo_config.yaml')
        self.servo_positions = {
            'servo1': servo_config.get('servo1', 0),
            'servo2': servo_config.get('servo2', 0)
        }

        print("初始化完成：舵机控制器、MP3 播放器和舵机配置已加载。")

    def play_music(self):
        """
        播放音乐。
        """
        print("开始播放音乐...")
        # 假设 MP3 播放器有一个 start 方法
        self.mp3_player.start()

    def pause_music(self):
        """
        暂停音乐。
        """
        print("暂停音乐...")
        self.mp3_player.pause()

    def dance(self, speed):
        """
        控制机器人跳舞。
        :param speed: 跳舞速度。
        """
        print(f"机器人正在以速度 {speed} 跳舞！")

        # 根据速度调整舵机角度
        for servo, position in self.servo_positions.items():
            angle = int(position + speed * 10)  # 示例计算
            channel = 0 if servo == 'servo1' else 1
            self.servo_controller.set_servo_angle(channel, angle)
            print(f"设置 {servo} 角度为 {angle}")

        # 更新灯带颜色
        self.servo_controller.update_with_spectrum(int(speed * 10))

        # 播放音乐
        self.play_music()
        time.sleep(2)  # 模拟跳舞过程
        self.pause_music()  # 暂停音乐


# 添加入口点
if __name__ == "__main__":
    # 实例化 MusicController 并调用 dance 方法
    print("启动 MusicController...")
    music_controller = MusicController()
    music_controller.dance(speed=5)  # 假设速度为 5                self.adjust_serv