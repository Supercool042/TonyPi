
import time
import yaml
import sys
import platform
import colorsys
from subprocess import check_output, CalledProcessError

# 硬件检查与依赖提示
def check_hardware_dependencies():
    # 检测系统环境
    if platform.system() != "Linux":
        print("警告：非Linux系统（如Windows/macOS），无法控制真实硬件！")
        print("----------------------------------------")
        return False

    # 1. PCA9685硬件连接检查
    try:
        # 检查I2C设备列表
        i2c_devices = check_output(["i2cdetect", "-y", "1"], universal_newlines=True)
        if "40" not in i2c_devices:
            print("错误：未检测到PCA9685舵机驱动板（0x40）！")
            print("请检查：")
            print("- SDA(GPIO2)和SCL(GPIO3)是否正确连接")
            print("- 驱动板是否单独供电（5V）")
            print("- 运行 `sudo i2cdetect -y 1` 确认设备地址")
            return False
    except CalledProcessError:
        print("错误：I2C工具未安装！请运行：")
        print("sudo apt-get install i2c-tools python-smbus")
        return False

    # 2. NeoPixel灯带依赖检查
    try:
        import RPi.GPIO
        import rpi_ws281x
    except ModuleNotFoundError:
        print("错误：缺少灯带驱动库！请运行：")
        print("sudo apt-get install build-essential")
        print("pip3 install rpi_ws281x adafruit-circuitpython-neopixel")
        return False

    # 3. 用户权限检查
    try:
        with open("/etc/group") as f:
            groups = f.read()
            if "i2c" not in groups or "gpio" not in groups:
                print("错误：当前用户缺少硬件权限！请运行：")
                print("sudo usermod -aG i2c pi")
                print("sudo usermod -aG gpio pi")
                print("sudo reboot")
                return False
    except PermissionError:
        print("错误：无权限读取用户组信息，请使用sudo运行！")
        return False

    return True

# 加载伺服器配置
servo_file_path = '/home/pi/TonyPi/servo_config.yaml'

def get_yaml_data(yaml_file):
    with open(yaml_file, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file)

def save_yaml_data(data, yaml_file):
    with open(yaml_file, 'w', encoding='utf-8') as file:
        yaml.dump(data, file)

class ServoController:
    def __init__(self):
        # 执行硬件和依赖检查
        if not check_hardware_dependencies():
            sys.exit(1)  # 检查不通过则退出

        try:
            import Adafruit_PCA9685
            self.pwm = Adafruit_PCA9685.PCA9685(busnum=1, address=0x40)
            self.pwm.set_pwm_freq(50)
        except RuntimeError as e:
            print(f"舵机控制器初始化失败: {e}")
            print("请确认：")
            print("- PCA9685驱动库已安装（pip3 install Adafruit_PCA9685）")
            print("- 驱动板电源正常（5V）")
            raise

        try:
            # 初始化 NeoPixel 灯带（带异常处理）
            import neopixel
            import board
            self.strip = neopixel.NeoPixel(board.D18, 30, brightness=0.5)
        except Exception as e:
            print(f"灯带初始化失败: {e}")
            print("请检查：")
            print("- 灯带是否连接GPIO18（物理引脚12）")
            print("- 灯带是否单独供电（5V，非树莓派供电）")
            print("- 运行 `gpio readall` 确认引脚状态")
            raise

        try:
            self.servo_config = get_yaml_data(servo_file_path)
        except FileNotFoundError:
            print(f"错误：未找到配置文件 {servo_file_path}！")
            print("请确认文件路径正确，或运行初始化脚本生成配置")
            raise

    def set_servo_angle(self, channel, angle):
        """设置舵机角度（带范围检查）"""
        if not (0 <= angle <= 180):
            raise ValueError("角度必须在 0-180° 之间")
        duty = int((angle * 4096) / 180 + 204.8)
        self.pwm.set_pwm(channel, 0, duty)

    def execute_action(self, action_sequence):
        """执行动作序列"""
        for frame in action_sequence:
            for channel, angle in enumerate(frame):
                self.set_servo_angle(channel, angle)
            time.sleep(0.05)  # 动作过渡间隔

    def set_color(self, index, color):
        """设置单个灯珠颜色（RGB 格式）"""
        self.strip[index] = color
        self.strip.show()

    def update_with_spectrum(self, spectrum):
        """根据频谱能量动态调整灯带颜色"""
        h = spectrum / 255.0
        r, g, b = self.hsv_to_rgb(h, 1.0, 1.0)
        for i in range(len(self.strip)):
            self.set_color(i, (r, g, b))

    def hsv_to_rgb(self, h, s, v):
        """HSV 转 RGB 算法"""
        return tuple(int(c * 255) for c in colorsys.hsv_to_rgb(h, s, v))

class MusicController:
    def __init__(self):
        self.servo_controller = ServoController()
        self.dance_actions = [
            [90, 90, 90, 90, 90, 90, 90, 90],  # 初始姿态
            [120, 60, 120, 60, 120, 60, 120, 60],  # 动作1
            [60, 120, 60, 120, 60, 120, 60, 120],  # 动作2
            [90, 90, 90, 90, 90, 90, 90, 90],  # 复位
        ]

    def dance(self, speed=50, music_controller=None):
        """控制机器人跳舞"""
        frame_delay = 0.05 / (speed / 50)  # 速度调整
        print(f"机器人正在以速度 {speed} 跳舞！")
        
        if music_controller:
            print("音乐控制功能已激活")
        
        for frame in self.dance_actions:
            self.servo_controller.execute_action([frame])
            time.sleep(frame_delay)
        
        # 结束动作
        self.servo_controller.set_color(0, (0, 255, 0))  # 绿色表示完成
        print("舞蹈执行完成")

if __name__ == '__main__':
    try:
        robot = MusicController()
        robot.dance(speed=70)  
    except ModuleNotFoundError as e:
        if "Adafruit_PCA9685" in str(e):
            print("错误：缺少舵机驱动库！请运行：")
            print("pip3 install Adafruit_PCA9685")
        elif "neopixel" in str(e):
            print("错误：缺少灯带驱动库！请运行：")
            print("pip3 install rpi_ws281x adafruit-circuitpython-neopixel")
        else:
            print(f"依赖缺失: {e}")
    except Exception as e:
        print(f"运行错误: {e}")    