from . import ros_robot_controller_sdk as rrc
import time

board = rrc.Board()


class Controller:
    def __init__(self, board, time_out=50):
        self.board = board
        self.time_out = time_out

    """
        function name：  获取总线舵机温度限制

        param：
                servo_id: 要获取温度限制的舵机id

        return：   返回舵机温度限制
    """ 
    def get_bus_servo_temp_limit(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_temp_limit(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)

    """
        function name：  获取总线舵机角度限制

        param：
                servo_id: 要获取角度限制的舵机ID

        return：   返回舵机角度限制
    """ 
    def get_bus_servo_angle_limit(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_angle_limit(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
    

    """
        function name：  获取总线舵机电压限制

        param：
                servo_id: 要获取电压限制的舵机id

        return：   返回舵机电压限制
    """ 
    def get_bus_servo_vin_limit(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_vin_limit(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
    

    """
        function name：  获取总线舵机ID

        param：

        return：   返回舵机ID
    """      
    def get_bus_servo_id(self):
        count = 0
        while True:
            data = self.board.bus_servo_read_id()
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
    

    """
        function name：  获取总线舵机位置

        param：
                servo_id: 要获取位置的舵机id

        return：   返回舵机位置
    """      
    def get_bus_servo_pulse(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_position(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
   

    """
        function name：  获取总线舵机电压

        param：
                servo_id: 要获取电压的舵机id

        return：   返回舵机电压
    """  
    def get_bus_servo_vin(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_vin(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
    

    """
        function name：  获取总线舵机温度

        param：
                servo_id: 要获取温度的舵机id

        return：   返回舵机温度
    """  
    def get_bus_servo_temp(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_temp(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
    

    """
        function name：  获取总线舵机偏差

        param：
                servo_id: 要获取偏差的舵机id

        return：   返回舵机偏差
    """    
    def get_bus_servo_deviation(self, servo_id):
        count = 0
        while True:
            data = self.board.bus_servo_read_offset(servo_id)
            count += 1
            if data is not None:
                return data[0]
            if count > self.time_out:
                return None
            time.sleep(0.01)
    

    """
        function name：  设置总线舵机位置

        param：
                servo_id: 要驱动的舵机id
                pulse:    舵机目标位置
                use_time: 转动需要的时间    
    """
    def set_bus_servo_pulse(self, servo_id, pulse, use_time):
        self.board.bus_servo_set_position(use_time/1000, [[servo_id, pulse]])

    """
        function name：  设置pwm舵机位置

        param：
                servo_id: 要驱动的舵机id
                pulse:    舵机目标位置
                use_time: 转动需要的时间    
    """
    def set_pwm_servo_pulse(self, servo_id, pulse, use_time):
        self.board.pwm_servo_set_position(use_time/1000, [[servo_id, pulse]])

    """
        function name：  设置总线舵机ID

        param：
                servo_id_now: 当前的舵机ID
                servo_id_new: 新的舵机ID
    """
    def set_bus_servo_id(self, servo_id_now, servo_id_new):
        self.board.bus_servo_set_id(servo_id_now, servo_id_new)

    """
        function name：  设置总线舵机偏差

        param：
                servo_id:  要设置的舵机ID
                deviation: 设置的舵机偏差
    """
    def set_bus_servo_deviation(self, servo_id, deviation):
        self.board.bus_servo_set_offset(servo_id, deviation)

    """
        function name：  设置总线舵机温度限制

        param：
                servo_id:  要设置的舵机ID
                temp_limit: 设置的温度限制
    """
    def set_bus_servo_temp_limit(self, servo_id, temp_limit):
        self.board.bus_servo_set_temp_limit(servo_id, temp_limit)

    """
        function name：  设置总线舵机角度限制

        param：
                servo_id:  要设置的舵机ID
                angle_limit: 设置的角度限制
    """
    def set_bus_servo_angle_limit(self, servo_id, angle_limit):
        self.board.bus_servo_set_angle_limit(servo_id, angle_limit)

    """
        function name：  设置总线舵机电压限制

        param：
                servo_id:  要设置的舵机ID
                vin_limit: 设置的电压限制
    """
    def set_bus_servo_vin_limit(self, servo_id, vin_limit):
        self.board.bus_servo_set_vin_limit(servo_id, vin_limit)

    """
        function name：  下载舵机偏差

        param：
                servo_id:  要下载偏差的舵机ID
    """    
    def save_bus_servo_deviation(self, servo_id):
        self.board.bus_servo_save_offset(servo_id)

    """
        function name：  舵机掉电

        param：
                servo_id:  要掉电的舵机ID
    """ 
    def unload_bus_servo(self, servo_id):
        self.board.bus_servo_enable_torque(servo_id, 1)

    """
        function name：  驱动蜂鸣器

        param：
                freq:       声音频率
                on_time：   开启时间
                off_time:   关闭时间
                repeat：    重复次数
    """ 
    def set_buzzer(self, freq, on_time, off_time, repeat=1):
        self.board.set_buzzer(freq, on_time, off_time, repeat=1)

    """
        function name：  获取imu数据

        param：
        
        return：返回IMU数据，ax, ay, az, gx, gy, gz
    """ 
    def get_imu(self):
        count = 0
        while True:
            data = self.board.get_imu()
            count += 1
            if data is not None:
                return data
            if count > self.time_out:
                return None
            time.sleep(0.01)

    """
        function name：  数据接收使能

        param：
        
        return：
    """ 
    def enable_recv(self):
        self.board.enable_reception(False)
        time.sleep(1)
        self.board.enable_reception(True)
        time.sleep(1)
        
    