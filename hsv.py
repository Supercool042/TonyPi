import Adafruit_PCA9685
import time
import neopixel
import board
import colorsys


class ServoController:
    def __init__(self):
        try:
            # 初始化 PCA9685 舵机控制器，显式指定 I2C 总线编号和地址
            self.pwm = Adafruit_PCA9685.PCA9685(busnum=1, address=0x40)
            self.pwm.set_pwm_freq(50)  # 设置频率为 50Hz
        except RuntimeError as e:
            print(f"初始化舵机控制器失败: {e}")
            print("请确保以下条件满足：")
            print("- I2C接口已在Raspberry Pi上启用")
            print("- PCA9685板已正确连接到I2C总线")
            print("- 已安装必要的依赖库（如python-smbus和i2c-tools）")
            print("- 当前用户已加入'i2c'用户组")
            raise

        # 初始化 NeoPixel 灯带
        self.strip = neopixel.NeoPixel(board.D18, 30)  # 假设灯带连接到 GPIO 18，灯珠数量为 30

    def set_servo_angle(self, channel, angle):
        """
        设置舵机角度（0-180°）。
        :param channel: 舵机通道号（0-15）。
        :param angle: 目标角度（0-180°）。
        """
        if not (0 <= angle <= 180):
            raise ValueError("角度必须在 0-180° 之间")
        duty = int((angle * 4096) / 180 + 204.8)  # 换算为 PCA9685 占空比
        self.pwm.set_pwm(channel, 0, duty)

    def execute_action(self, action_sequence):
        """
        执行动作序列。
        :param action_sequence: 动作序列，格式为 [[舵机1角度, 舵机2角度,...], ...]。
        """
        for frame in action_sequence:
            for channel, angle in enumerate(frame):
                self.set_servo_angle(channel, angle)
            time.sleep(0.05)  # 动作过渡间隔

    def set_color(self, index, color):
        """
        设置单个灯珠颜色（RGB 格式）。
        :param index: 灯珠索引。
        :param color: RGB 颜色值（元组，如 (255, 0, 0)）。
        """
        self.strip[index] = color  # 设置灯珠颜色
        self.strip.show()          # 更新灯带显示

    def update_with_spectrum(self, spectrum):
        """
        根据频谱能量动态调整灯带颜色（HSV 转 RGB）。
        :param spectrum: 频谱能量值（0-255）。
        """
        h = spectrum / 255.0  # 色调随能量变化（0-1）
        r, g, b = self.hsv_to_rgb(h, 1.0, 1.0)
        for i in range(len(self.strip)):
            self.set_color(i, (r, g, b))

    def hsv_to_rgb(self, h, s, v):
        """
        HSV 转 RGB 算法。
        :param h: 色调（0-1）。
        :param s: 饱和度（0-1）。
        :param v: 明度（0-1）。
        :return: RGB 颜色值（元组，如 (255, 0, 0)）。
        """
        return tuple(int(c * 255) for c in colorsys.hsv_to_rgb(h, s, v))