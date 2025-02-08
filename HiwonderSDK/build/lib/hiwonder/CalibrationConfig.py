#相邻两个角点间的实际距离，单位cm(the actual distance between adjacent corners, measured in centimeters)
corners_length = 2.1

#木块边长3cm(the wooden block has a side length of 3 cm)
square_length = 3

#标定棋盘大小, 列， 行, 指内角点个数，非棋盘格(calibrate the size of the chessboard: columns, rows, indicating the number of inner corner points, excluding the chessboard squares)
calibration_size = (7, 7)

#采集标定图像存储路径(storage path for collecting calibration images)
save_path = '/home/pi/TonyPi/Functions/CameraCalibration/calibration_images/'

#标定参数存储路径(storage path for calibration parameters)
calibration_param_path = '/home/pi/TonyPi/Functions/CameraCalibration/calibration_param'

#映射参数存储路径(storage path for mapping parameters)
map_param_path = '/home/pi/TonyPi/Functions/CameraCalibration/map_param'
