# 假设HiwonderSDK是自定义模块，实际使用时需要根据机器人SDK调整
# 这里使用模拟类以便代码能在普通环境运行
import cv2
import mediapipe as mp
import numpy as np
import time
import threading
import logging
from hsv import ServoController  # 引入现有的ServoController


# 配置日志记录
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class VideoToDance:
    def __init__(self):
        self.pose = mp.solutions.pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.joint_mapping = {
            "LEFT_SHOULDER": 0,
            "RIGHT_SHOULDER": 1,
            "LEFT_ELBOW": 2,
            "RIGHT_ELBOW": 3,
            "LEFT_HIP": 4,
            "RIGHT_HIP": 5,
            "LEFT_KNEE": 6,
            "RIGHT_KNEE": 7
        }

    def extract_keypoints(self, video_path, show_processing=False):
        cap = cv2.VideoCapture(video_path)
        fps = cap.get(cv2.CAP_PROP_FPS)
        keypoints_sequence = []

        if not cap.isOpened():
            logging.error(f"无法打开视频文件 {video_path}")
            return [], fps

        while cap.isOpened():
            success, image = cap.read()
            if not success:
                break

            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.pose.process(image)

            if results.pose_landmarks:
                frame_keypoints = []
                for idx, landmark in enumerate(results.pose_landmarks.landmark):
                    frame_keypoints.extend([landmark.x, landmark.y, landmark.z])
                keypoints_sequence.append(frame_keypoints)

                if show_processing:
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                    self.mp_drawing.draw_landmarks(
                        image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)
                    cv2.imshow('MediaPipe Pose', image)
                    if cv2.waitKey(1) & 0xFF == 27:
                        break

        cap.release()
        cv2.destroyAllWindows()
        return np.array(keypoints_sequence), fps


class KeypointToServo:
    def __init__(self):
        # 舵机角度限制参数（根据实际机器人调整）
        self.min_angles = np.array([30, 30, 20, 20, 40, 40, 30, 30])  # 各舵机最小角度
        self.max_angles = np.array([150, 150, 160, 160, 140, 140, 150, 150])  # 各舵机最大角度
        self.center_point = np.array([0.5, 0.5, 0])  # 人体中心点

        # 缩放因子，控制动作幅度
        self.scale_factor = np.array([1.0, 1.0, 0.5])  # 分别对应x,y,z方向缩放

    def convert(self, keypoints):
        """将关节点坐标转换为舵机角度"""
        # 提取关键关节点
        left_shoulder = keypoints[11 * 3:11 * 3 + 3]  # 左肩部
        right_shoulder = keypoints[12 * 3:12 * 3 + 3]  # 右肩部
        left_elbow = keypoints[13 * 3:13 * 3 + 3]  # 左肘部
        right_elbow = keypoints[14 * 3:14 * 3 + 3]  # 右肘部
        left_hip = keypoints[23 * 3:23 * 3 + 3]  # 左髋部
        right_hip = keypoints[24 * 3:24 * 3 + 3]  # 右髋部
        left_knee = keypoints[25 * 3:25 * 3 + 3]  # 左膝部
        right_knee = keypoints[26 * 3:26 * 3 + 3]  # 右膝部

        # 计算相对位置（相对于中心点）并应用缩放
        left_shoulder_rel = (left_shoulder - self.center_point) * self.scale_factor
        right_shoulder_rel = (right_shoulder - self.center_point) * self.scale_factor
        left_elbow_rel = (left_elbow - self.center_point) * self.scale_factor
        right_elbow_rel = (right_elbow - self.center_point) * self.scale_factor
        left_hip_rel = (left_hip - self.center_point) * self.scale_factor
        right_hip_rel = (right_hip - self.center_point) * self.scale_factor
        left_knee_rel = (left_knee - self.center_point) * self.scale_factor
        right_knee_rel = (right_knee - self.center_point) * self.scale_factor

        # 映射到舵机角度范围（根据实际机器人结构调整）
        servo_angles = np.zeros(8)  # 8个舵机

        # 肩部舵机
        servo_angles[0] = np.interp(left_shoulder_rel[0], [-0.3, 0.3], [150, 30])  # 左肩水平
        servo_angles[1] = np.interp(right_shoulder_rel[0], [-0.3, 0.3], [30, 150])  # 右肩水平

        # 肘部舵机
        servo_angles[2] = np.interp(left_elbow_rel[1], [0.2, 0.8], [30, 150])  # 左肘垂直
        servo_angles[3] = np.interp(right_elbow_rel[1], [0.2, 0.8], [150, 30])  # 右肘垂直

        # 髋部舵机
        servo_angles[4] = np.interp(left_hip_rel[0], [-0.2, 0.2], [140, 40])  # 左髋水平
        servo_angles[5] = np.interp(right_hip_rel[0], [-0.2, 0.2], [40, 140])  # 右髋水平

        # 膝部舵机
        servo_angles[6] = np.interp(left_knee_rel[1], [0.5, 1.0], [150, 30])  # 左膝弯曲
        servo_angles[7] = np.interp(right_knee_rel[1], [0.5, 1.0], [30, 150])  # 右膝弯曲

        # 应用角度限制
        servo_angles = np.clip(servo_angles, self.min_angles, self.max_angles)
        return servo_angles.astype(int)


class DanceExecutor:
    def __init__(self):
        self.servo = ServoController()  # 使用现有的ServoController
        self.frame_delay = 0.033  # 默认30FPS的帧间隔
        self.last_execution_time = 0

    def set_frame_rate(self, fps):
        """设置执行帧率"""
        self.frame_delay = 1.0 / fps

    def execute(self, servo_angles_sequence, show_preview=False):
        """执行舵机角度序列"""
        if show_preview:
            logging.info("执行舞蹈动作（按Ctrl+C终止）")

        for angles in servo_angles_sequence:
            # 记录开始时间
            start_time = time.time()

            # 设置各舵机角度
            for channel, angle in enumerate(angles):
                try:
                    self.servo.set_servo_angle(channel, angle)
                except Exception as e:
                    logging.error(f"设置舵机 {channel} 角度失败: {e}")
                    continue

            # 控制执行速度，保持与原视频一致的节奏
            elapsed = time.time() - start_time
            if elapsed < self.frame_delay:
                time.sleep(self.frame_delay - elapsed)

            if show_preview:
                logging.info(f"\r帧 {servo_angles_sequence.index(angles) + 1}/{len(servo_angles_sequence)}: " +
                             " ".join([f"舵机{ch}:{ang}°" for ch, ang in enumerate(angles)]))


def dance_with_video(video_path, show_processing=False, show_preview=True):
    """主函数：视频→舞蹈执行完整流程"""
    # 1. 初始化模块
    extractor = VideoToDance()
    converter = KeypointToServo()
    executor = DanceExecutor()

    # 2. 提取视频关节点
    logging.info(f"正在处理视频: {video_path}")
    keypoints_sequence, fps = extractor.extract_keypoints(video_path, show_processing)

    if len(keypoints_sequence) == 0:
        logging.error("未检测到有效姿态数据")
        return

    logging.info(f"成功提取 {len(keypoints_sequence)} 帧姿态数据，帧率: {fps:.1f} FPS")

    # 3. 设置执行帧率
    executor.set_frame_rate(fps)

    # 4. 转换为舵机角度
    logging.info("正在转换为舵机角度...")
    servo_angles_sequence = [converter.convert(kp) for kp in keypoints_sequence]

    # 5. 执行舞蹈动作
    logging.info("开始执行舞蹈...")
    execution_thread = threading.Thread(target=executor.execute, args=(servo_angles_sequence, show_preview))
    execution_thread.start()
    execution_thread.join()


if __name__ == "__main__":
    video_path = r"C:\Users\a8185\Desktop\B.mp4"  # 替换为实际视频路径
    dance_with_video(video_path, show_processing=False, show_preview=True)