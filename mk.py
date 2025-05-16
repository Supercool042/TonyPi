import cv2
import mediapipe as mp
import numpy as np
from hsv import ServoController  # 导入 TonyPi 的舵机和灯带控制类


class PoseController:
    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

        # 初始化 TonyPi 控制器
        self.servo_controller = ServoController()

    def extract_and_visualize_pose(self, video_source):
        """
        实时处理视频流，提取姿态并可视化。
        :param video_source: 视频源，可以是文件路径或摄像头索引（例如 0 表示默认摄像头）。
        """
        # 判断视频源类型
        if isinstance(video_source, int):
            print("使用树莓派摄像头...")
            cap = cv2.VideoCapture(video_source)
        else:
            print(f"从文件 {video_source} 读取视频...")
            cap = cv2.VideoCapture(video_source)

        # 设置分辨率和帧率以优化性能
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 15)

        while cap.isOpened():
            success, image = cap.read()
            if not success:
                print("无法读取视频帧，退出。")
                break

            # 转换为 RGB 格式
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.pose.process(image)

            # 转换回 BGR 格式以便显示
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # 可视化姿态关键点
            if results.pose_landmarks:
                self.mp_drawing.draw_landmarks(
                    image,
                    results.pose_landmarks,
                    self.mp_pose.POSE_CONNECTIONS,
                    self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                    self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2)
                )

                # 获取关键点坐标
                landmarks = results.pose_landmarks.landmark
                pose_coordinates = []
                for landmark in landmarks[:25]:
                    pose_coordinates.extend([landmark.x, landmark.y, landmark.z])

                # 动态调整舵机角度
                self.adjust_servos(pose_coordinates)

                # 动态调整灯带颜色
                self.update_leds_with_pose(landmarks)

            # 显示结果
            cv2.imshow('运动理解层可视化', image)

            # 按 q 键退出循环
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def adjust_servos(self, pose_coordinates):
        """
        根据姿态坐标调整舵机角度。
        :param pose_coordinates: 姿态关键点坐标列表。
        """
        # 示例：将第一个舵机的角度与第一个关键点的 x 坐标关联
        if len(pose_coordinates) > 0:
            angle = int(pose_coordinates[0] * 180)  # 将 x 坐标映射到 0-180°
            self.servo_controller.set_servo_angle(0, angle)

    def update_leds_with_pose(self, landmarks):
        """
        根据姿态动态调整灯带颜色。
        :param landmarks: 姿态关键点对象列表。
        """
        # 示例：根据头部关键点的 y 坐标调整灯带颜色
        head_y = landmarks[self.mp_pose.PoseLandmark.NOSE].y
        spectrum = int(head_y * 255)  # 将 y 坐标映射到 0-255
        self.servo_controller.update_with_spectrum(spectrum)


if __name__ == "__main__":
    # 使用树莓派摄像头时，将参数设置为 0
    video_source = 0

    # 使用视频文件时，指定文件路径
    # video_source = r'C:\Users\a8185\Desktop\B.mp4'

    try:
        pose_controller = PoseController()
        pose_controller.extract_and_visualize_pose(video_source)
    except KeyboardInterrupt:
        print("程序已退出。")