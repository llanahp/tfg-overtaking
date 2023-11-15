#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
"""

Authors: Javier Araluce
Last mod: J. Felipe Arango. 24/11/2022
"""

import os
import rospy
import numpy as np
import t4ac_msgs.msg
import sensor_msgs.msg

from PIL import Image
from types import SimpleNamespace
from yolo_utils import load_model, detect, process_image

import tf
import image_geometry
import sensor_msgs.msg
import visualization_msgs.msg

from modules.monitors.monitors_functions import apply_tf
from modules.monitors.ros_functions import get_detection_marker

def numpyImage_to_rosimage(numpy_img, frame_id, stamp):
    """
    """
    im_from_array = Image.fromarray(numpy_img)
    output_ros_image = sensor_msgs.msg.Image()
    output_ros_image.header.frame_id = frame_id
    output_ros_image.header.stamp = stamp
    output_ros_image.encoding = "bgr8"
    output_ros_image.width, output_ros_image.height = im_from_array.size
    output_ros_image.step = 3 * output_ros_image.width
    output_ros_image.data = im_from_array.tobytes()
    return im_from_array, output_ros_image

class Detector_2D_Yolo():
    def __init__(self) -> None:
        # Detector 2D
        YOLOV5_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '../', "yolov5/weights/"))
        YOLO_WEIGHT_PATH = YOLOV5_PATH + '/yolov5l.pt'
        self.yolo, self.imgsz, self.half, self.stride = load_model(YOLO_WEIGHT_PATH)
        # object idx: ['person', 'bicycle', 'car', 'motorcycle', 'bus', 'traffic light', 'stop sign']
        OBJECT_DETECTOR_2D_list = [0,1,2,3,5,9,11]
        opt = {
            "augment": False,
            "conf_thres": 0.45,
            "iou_thres": 0.45,
            "classes": OBJECT_DETECTOR_2D_list ,
            "agnostic_nms": False
            }
        self.opt = SimpleNamespace(**opt)

        # Frames
        self.map_frame = rospy.get_param('/t4ac/frames/map')
        self.camera_frame = rospy.get_param('/t4ac/frames/camera')

        camera_info = rospy.wait_for_message('/t4ac/perception/camera_info', sensor_msgs.msg.CameraInfo) 

        # ROS Publishers
        self.pub_camera_3D_obstacles_markers = rospy.Publisher(
            "/t4ac/perception/3D_camera_obstacles_markers", visualization_msgs.msg.MarkerArray, queue_size=20)
        
        # Camera Parameters
        camera_id = 'front_camera'
        
        cam_model = image_geometry.PinholeCameraModel()
        cam_model.fromCameraInfo(camera_info)
        K = np.array([camera_info.K]).reshape(3,3)
        P = np.array([camera_info.P]).reshape(3,4)
        R = np.array([camera_info.R]).reshape(3,3) # ??
        # R = np.insert(R,3,values=[0,0,0],axis=0)
        # R = np.insert(R,3,values=[0,0,0,1],axis=1)
        yaw_tf_center = (-1.57079632679 * 1.57079632679) / 90
        camera_tf = [0.0, 0, 1.65, -1.57079632679, 0, yaw_tf_center]
        red = [1.0,0.0,0.0,1.0]
        self.camera_dict = dict({
            'id': camera_id,
            'cam_model': cam_model,
            'width': camera_info.width,
            'height': camera_info.height,
            'K': K,
            'P': P,
            'P_hom': P,
            'P_hom_inv': np.linalg.pinv(P),
            'R': R,
            'x': camera_tf[0],
            'y': camera_tf[1],
            'z': camera_tf[2],
            'bbox_pub': rospy.Publisher(
                                '/t4ac/perception/YOLO_image_detections',
                                t4ac_msgs.msg.Bounding_Box_2D_list,
                                queue_size=20),
            'image_pub': rospy.Publisher(
                                '/t4ac/perception/YOLO_output_image',
                                sensor_msgs.msg.Image,
                                queue_size=20),
            'tf_camera2baselink': np.zeros((4,4)), # To be filled,
            'frame_id': self.camera_frame,
            'colour_marker': red
        })

        self.listener = tf.TransformListener()

        # ROS Subscribers
        self.sub_camera_img = rospy.Subscriber('/t4ac/perception/camera', sensor_msgs.msg.Image, self.callback_camera)

    def recover_img(self, img_msg):
        header = img_msg.header
        current_stamp = header.stamp
        
        # Recover image
        image = Image.frombytes(
                    mode = 'RGBA',
                    size = (img_msg.width, img_msg.height),
                    data = img_msg.data)

        image = image.convert('RGB')
        return header, current_stamp, image
        
    def detection2d_pipelane(self, img, header):
        image = [np.array(img)]
        # model forward
        img, img0, pred, names, colors = detect(
                self.yolo, image,
                imgsz= self.imgsz,
                half= self.half,
                stride= self.stride,
                opt= self.opt,
                batch_size = 1)
        
        bounding_box_2d_list_cameras = []
        for i in range(len(pred)):
            predicted_image, bounding_box_2d_list = process_image(img[i], img0[i], [pred[i]], names, colors, header, tfori=None)
            bounding_box_2d_list_cameras.append(bounding_box_2d_list)

        return bounding_box_2d_list_cameras[0], predicted_image

    def pix2realwworld(self, det):
        centroid_x = (det.x1 + det.x2)/2
        # Hipotesis suelo 
        pixels = np.array([[centroid_x],[det.y2], [1]]) # Punto con el centroide de la bbox abajo de ella, añadiendo un 1 para la tercera coordenada
        p_camera = np.dot(self.camera_dict['P_hom_inv'], pixels)  # Multiplicar por la inv de la P_hom

        # Suposición suelo 
        if (p_camera[1] != 0):
            K = self.camera_dict['z'] / p_camera[1]
        else:
            return None
        p_camera_meters = np.dot(p_camera, K)
        
        return p_camera_meters

    def create_3d_object_marker(self, p_camera_meters, tf_map2camera, i, map_frame, current_stamp):
        obstacle_local_position = t4ac_msgs.msg.Node(p_camera_meters[0], p_camera_meters[1], p_camera_meters[2])
        obstacle_global_position = apply_tf(obstacle_local_position, tf_map2camera)

        detected_3D_object_marker = get_detection_marker(
            i,
            obstacle_global_position, # obstacle_local_position
            map_frame, # self.camera_dict['frame_id']
            current_stamp,
            color=self.camera_dict['colour_marker'],
            camera=True)
        return detected_3D_object_marker

    def callback_camera(self, img_msg):
        
        # Transforms    
        try:
            (translation,quaternion) = self.listener.lookupTransform(
                self.map_frame, self.camera_frame, rospy.Time(0)) 
            rot_matrix = tf.transformations.quaternion_matrix(quaternion)
            
            tf_map2camera = rot_matrix
            tf_map2camera[:3,3] = tf_map2camera[:3,3] + translation 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("\033[1;93m"+"TF Map2Camera exception"+'\033[0;m')

        # 2D
        header, current_stamp, image = self.recover_img(img_msg)
        bounding_box_2d_list, predicted_image = self.detection2d_pipelane(image, header)
        cv_image, ros_image = numpyImage_to_rosimage(predicted_image, 'ego_vehicle/camera', img_msg.header.stamp)
        self.camera_dict['image_pub'].publish(ros_image)
        # 3D
        detected_3D_objects_marker_array = visualization_msgs.msg.MarkerArray()
        for i, det in enumerate(bounding_box_2d_list.bounding_box_2D_list):
            traffic_light_index = det.type.find('traffic')
            stop_index = det.type.find('stop')

            # Camera obstacles
            if traffic_light_index == -1 and stop_index == -1 and np.any(tf_map2camera):

                # pixel to real world coordinates
                p_camera_meters = self.pix2realwworld(det)

                # create and save 3d markers
                detected_3D_object_marker = self.create_3d_object_marker(
                    p_camera_meters, tf_map2camera, i, self.map_frame, current_stamp)

                detected_3D_objects_marker_array.markers.append(detected_3D_object_marker)

        # publish markers
        self.pub_camera_3D_obstacles_markers.publish(detected_3D_objects_marker_array)
        
def main():
    rospy.init_node('t4ac_2D_detection_node', anonymous=True)
    Detector_2D_Yolo()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down ROS T4AC Unified Perception module")
    
if __name__ == "__main__":
    main()
