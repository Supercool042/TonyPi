#相邻两个角点间的实际距离，单位cm(the actual distance between adjacent two corners, in centimeters)
corners_length = 2.1

#木块边长3cm(the side length of the wooden block is 3cm)
square_length = 3

#标定棋盘大小, 列， 行, 指内角点个数，非棋盘格(calibrate chessboard size, columns, rows, referring to the number of inner corner points, not chessboard squares)
calibration_size = (7, 7)

#采集标定图像存储路径(the storage path for collecting calibration images)
save_path = '/home/pi/TonyPi/Functions/CameraCalibration/calibration_images/'

#标定参数存储路径(the storage path for calibration parameters)
calibration_param_path = '/home/pi/TonyPi/Functions/CameraCalibration/calibration_param'

#映射参数存储路径(the storage path for mapping parameters)
map_param_path = '/home/pi/TonyPi/Functions/CameraCalibration/map_param'
