import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense
from sklearn.preprocessing import MinMaxScaler
import threading
import time
from hsv import ServoController
from music import TonyPi  # 导入 TonyPi 类


class ChoreographyModel:
    def __init__(self, timesteps=1, features=50):  # 减少特征数以适应树莓派
        self.timesteps = timesteps
        self.features = features
        self.scaler = MinMaxScaler()
        self.model = self.build_model()

    def build_model(self):
        """简化模型结构以适应树莓派"""
        model = Sequential([
            LSTM(32, input_shape=(self.timesteps, self.features), return_sequences=False),  # 减少LSTM单元数
            Dense(self.features, activation="tanh")
        ])
        model.compile(
            optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
            loss="mse"
        )
        return model

    def train(self, X_train, y_train, epochs=30, batch_size=16):  # 减少训练参数
        """训练模型（输入形状：[样本数, 时间步, 特征数]）"""
        X_train = self.scaler.fit_transform(X_train)
        X_train = X_train.reshape(-1, self.timesteps, self.features)
        self.model.fit(X_train, y_train, epochs=epochs, batch_size=batch_size)

    def generate_action(self, input_data):
        """生成未来2秒动作序列（舵机角度指令）"""
        input_data = self.scaler.transform(input_data)
        input_data = input_data.reshape(1, self.timesteps, self.features)
        output = self.model.predict(input_data)
        return self.scaler.inverse_transform(output)


class TonyPiController:
    def __init__(self):
        self.robot = TonyPi()
        self.controller = ServoController()
        self.running = True

    def start(self):
        """启动TonyPi和音频处理线程"""
        print("启动TonyPi...")
        threading.Thread(target=self._run_tonypi, daemon=True).start()
        threading.Thread(target=self.real_time_dance_control, daemon=True).start()

    def _run_tonypi(self):
        """TonyPi主循环"""
        while self.running:
            time.sleep(0.1)
        print("TonyPi已停止运行。")

    def stop(self):
        """停止所有线程"""
        self.running = False

    def real_time_dance_control(self):
        """
        实时舞蹈控制逻辑
        根据音频特征生成动作序列，并控制TonyPi执行
        """
        choreography_model = ChoreographyModel(timesteps=1, features=50)
        nsdm_data = np.load("nsdm_data.npy")  # 加载NSDM数据集
        choreography_model.train(nsdm_data[:, :-1], nsdm_data[:, -1])  # 训练模型

        # 模拟实时音频特征提取
        while self.running:
            # 模拟从音频中提取特征
            bpm = np.random.uniform(80, 140)  # 模拟BPM
            spectral_centroid = np.random.uniform(500, 2000)  # 模拟频谱质心
            zero_crossing_rate = np.random.uniform(0.01, 0.1)  # 模拟过零率

            # 打印特征值
            print(f"BPM: {bpm:.2f}, 频谱质心: {spectral_centroid:.2f}, 过零率: {zero_crossing_rate:.4f}")

            # 动作生成逻辑
            if bpm > 100:
                input_data = np.random.rand(1, 50)  # 模拟输入数据
                action_sequence = choreography_model.generate_action(input_data)
                print("生成动作序列:", action_sequence.flatten())
                self.execute_actions(action_sequence.flatten())
            else:
                intensity = spectral_centroid / 1000
                self.robot.move(intensity=intensity)

            time.sleep(1)  # 每秒更新一次

    def execute_actions(self, action_sequence):
        """
        执行动作序列
        :param action_sequence: 动作序列（舵机角度指令）
        """
        for angle in action_sequence:
            self.controller.set_servo_angle(angle)
            time.sleep(0.1)  # 每个动作间隔0.1秒


if __name__ == "__main__":
    # 初始化TonyPi控制器
    controller = TonyPiController()
    controller.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("程序已退出。")
        controller.stop()