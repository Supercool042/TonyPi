#!/usr/bin/python3
# coding=utf8
#4.拓展课程学习\8.拓展课程之传感器应用开发课程\第6课 口罩识别\第2节 口罩识别(4.Advanced Lessons\8.Sensor Development Course\Lesson6 Mask Recognition\Lesson2 Human Fan Tracking\2.Mask Recognition)
import os
import cv2
import sys
import time
import torch
import argparse
import numpy as np
from pathlib import Path
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import torch.backends.cudnn as cudnn
from utils.plots import Annotator, colors
from models.experimental import attempt_load
from utils.datasets import LoadImages, LoadStreams
from utils.torch_utils import load_classifier, select_device, time_sync
from utils.general import apply_classifier, check_img_size, check_imshow, \
     check_requirements, check_suffix, colorstr, increment_path, non_max_suppression, \
     print_args, save_one_box, scale_coords, set_logging, strip_optimizer, xyxy2xywh


#口罩识别程序

#导入MP3模块库文件
import hiwonder.MP3 as MP3
addr = 0x7b       #传感器iic地址(sensor I2C address)
mp3 = MP3.MP3(addr)
times = 0.0

servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

board = rrc.Board()
ctl = Controller(board)

# 初始位置(initial position)
def initMove():
    ctl.set_pwm_servo_pulse(1,1800,500)
    ctl.set_pwm_servo_pulse(2,servo_data['servo2'],500)
    AGC.runActionGroup('stand_slow')



FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

@torch.no_grad()
def run(weights=ROOT / 'weights/mask.pt',  # 权重文件路径(path to the weight file)
        source=0,  # file/dir/URL/glob, 0 for webcam
        imgsz=160,  # 推理大小(像素)(inference size (pixels))
        conf_thres=0.5,  # 置信度阈值(confidence threshold)
        iou_thres=0.45,  # NMS IOU 阈值(NMS IOU threshold)
        max_det=1000,  # 最大检测图像个数(maximum number of detected images)
        device='cpu',  # Cuda设备，即0或0、1、2、3或CPU(CUDA device, either 0 or 0,1,2,3 or CPU)
        view_img=False,  # show results
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        ):
    global times
    source = str(source)
    webcam = source.isnumeric() or source.endswith('.txt') or source.lower().startswith(
        ('rtsp://', 'rtmp://', 'http://', 'https://'))

    # 初始化(initialization)
    set_logging()
    device = select_device(device)
    half &= device.type != 'cpu'  # 半精度仅支持CUDA(half precision only supported with CUDA)

    # 加载权重文件(load weight file)
    w = str(weights[0] if isinstance(weights, list) else weights)
    classify, suffix, suffixes = False, Path(w).suffix.lower(), ['.pt', '.onnx', '.tflite', '.pb', '']
    check_suffix(w, suffixes)  # 检查权重是否有可接受的后缀(check if the weight file has an acceptable suffix)
    pt, onnx, tflite, pb, saved_model = (suffix == x for x in suffixes)  # 后端布尔值(backend boolean value)
    stride, names = 64, [f'class{i}' for i in range(1000)]  # 设置默认值(set default value)
    if pt:
        model = torch.jit.load(w) if 'torchscript' in w else attempt_load(weights, map_location=device)
        stride = int(model.stride.max())
        names = model.module.names if hasattr(model, 'module') else model.names  # 得到类名(get class name)
        if half:
            model.half() 
        if classify:  # 二级分类器(secondary classifier)
            modelc = load_classifier(name='resnet50', n=2)
            modelc.load_state_dict(torch.load('resnet50.pt', map_location=device)['model']).to(device).eval()
            
    imgsz = check_img_size(imgsz, s=stride)  # 检查图像大小(check image size)
    
    # 加载数据(load data)
    if webcam:
        view_img = check_imshow()
        cudnn.benchmark = True  # True设置为加速常量图像大小推断(set to True to accelerate constant image size inference)
        dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt)
        bs = len(dataset)
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt)
        bs = 1
    vid_path, vid_writer = [None] * bs, [None] * bs
    dt, seen = [0.0, 0.0, 0.0], 0
    for path, img, im0s, vid_cap in dataset:
        t1 = time_sync()
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1

        # 进行推理(perform inference)
        if pt:
            visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
            pred = model(img, augment=augment, visualize=visualize)[0]
            
        t3 = time_sync()
        dt[1] += t3 - t2

        # NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        dt[2] += time_sync() - t3

        # 预测过程(prediction process)
        for i, det in enumerate(pred):  # per image
            seen += 1
            if webcam:  # batch_size >= 1
                p, s, im0, frame = path[i], f'{i}: ', im0s[i].copy(), dataset.count
            else:
                p, s, im0, frame = path, '', im0s.copy(), getattr(dataset, 'frame', 0)
            s += '%gx%g ' % img.shape[2:]  # print string
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))
            if len(det):
                # 重新缩放img_size到im0大小(rescale img_size to the size of im0)
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    if view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        #得到推理结果(get inference results)
                        label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                        #打印结果(print result)
                        print(label)
                        if str(label[:-5]) == 'without_mask': #没有戴口罩(without wearing the mask)
                            if time.time()- times >= 5: #设置两次播放的时间间隔5秒(set the time interval between two plays to 5 seconds)
                                mp3.volume(30) #设置音量0~30,注意在播放前设置(set the volume from 0 to 30, ensuring it's set before playback)
                                mp3.playNum(6) #播放4号MP3文件(play MP3 file number 4)
                                times = time.time()
                            
                        elif str(label[:-5]) == 'withmask': #有戴口罩(wear a mask)
                            if time.time()- times >= 5: #设置两次播放的时间间隔5秒(set the time interval between two plays to 5 seconds)
                                mp3.volume(30) #设置音量为0~30，注意在播放前设置(set the volume from 0 to 30, ensuring it's set before playback)
                                mp3.playNum(5) #播放3号MP3文件(play MP3 file number 3)
                                times = time.time()
                            
                        annotator.box_label(xyxy, label, color=colors(c, True))
                    
            # 打印一帧的推断时间(orint the inference time for one frame)
            print(f'{s}Done. ({t3 - t2:.3f}s)')

            # 显示画面(display screen)
            img = annotator.result()
            if view_img:
                img = cv2.resize(img, (320, 240)) # 缩放画面(scale the screen)
                cv2.imshow(str(p), img)
                cv2.waitKey(1)


if __name__ == "__main__":
    
    initMove()
    
    while True:
        run()
    
    
