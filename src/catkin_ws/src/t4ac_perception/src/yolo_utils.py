"""

Authors: Javier Araluce
Last mod: J. Felipe Arango. 24/11/2022
"""
# General imports
import sys
import os
import cv2
import torch
import numpy as np
from numpy import random

# ROS imports
import t4ac_msgs.msg

# YOLOv5
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', "yolov5/")))
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords
from utils.plots import plot_one_box
from utils.torch_utils import select_device

def load_model(weights, device='0', imgsz=640):
    device = select_device(device)
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = int(model.stride.max())
    imgsz = check_img_size(imgsz, s=stride)
    if half:
        model.half()  # to FP16
    return model, imgsz, half, stride # model, 640, 32

def detect(model, imgs, imgsz, half, opt, stride, device=0, batch_size = 1):
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]
    model(torch.zeros(batch_size, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))    

    img_aux = np.zeros(imgs[0].shape)
    img_aux = pre_process_image(img_aux, imgsz, stride, half, device)
    img = np.zeros((batch_size, img_aux.shape[0], img_aux.shape[1], img_aux.shape[2]))
    # img = np.zeros((batch_size, 3, 288, 640))
    # img0 = np.zeros((batch_size, imgs.shape[1],  imgs.shape[2], 3))

    for i in range(batch_size):
        img[i,:,:,:] = pre_process_image(imgs[i], imgsz, stride, half, device)

    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float() 
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    pred = model(img, augment=opt.augment)[0]
    pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)

    return img, imgs, pred, names, colors

def pre_process_image(img0, imgsz, stride, half, device):
    img = letterbox(img = img0, new_shape = imgsz, stride = stride, auto=True, scaleFill=False)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1)
    img = np.ascontiguousarray(img)
    return img

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    shape = img.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, ratio, (dw, dh)

def process_image(img, im0, pred, names, colors, header, tfori=None):
    bb2d_list = t4ac_msgs.msg.Bounding_Box_2D_list()
    bb2d_list.header = header
    im0_copy = im0.copy()
    img = torch.unsqueeze(img, 0)
    for i, det in enumerate(pred):  # detections per image
        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        bb2d_list.bounding_box_2D_list = []
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            for *xyxy, conf, cls in reversed(det):
                det_type = f'{names[int(cls)]}'
                det_score = f'{conf:.2f}'
                label = '{} {}'.format(det_type, det_score)
                bb2d = t4ac_msgs.msg.Bounding_Box_2D()
                bb2d.type = det_type 
                bb2d.score = float(det_score)
                bb2d.x1 = xyxy[0].item()
                bb2d.y1 = xyxy[1].item()
                bb2d.x2 = xyxy[2].item()
                bb2d.y2 = xyxy[3].item()
                bb2d_list.bounding_box_2D_list.append(bb2d)
                plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)

    return im0, bb2d_list

def process_image_without_ROS(img, im0, pred, names, colors, tfori=None):
    im0_copy = im0.copy()
    img = torch.unsqueeze(img, 0)
    for i, det in enumerate(pred):  # detections per image
        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            for *xyxy, conf, cls in reversed(det):
                det_type = f'{names[int(cls)]}'
                det_score = f'{conf:.2f}'
                label = '{} {}'.format(det_type, det_score)
                bb2d = t4ac_msgs.msg.Bounding_Box_2D()
                bb2d.type = det_type 
                bb2d.score = float(det_score)
                bb2d.x1 = xyxy[0].item()
                bb2d.y1 = xyxy[1].item()
                bb2d.x2 = xyxy[2].item()
                bb2d.y2 = xyxy[3].item()
                plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)

    return im0