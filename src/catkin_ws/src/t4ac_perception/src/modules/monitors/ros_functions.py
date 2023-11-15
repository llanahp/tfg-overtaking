#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
"""
Created on tue Nov 07 11:57:13 2021
@author: Carlos Gómez Huélamo
"""

import rospy
import visualization_msgs.msg

def get_detection_marker(index, detection, frame_id, stamp, color=None, camera=None):
    """
    """
    detected_3D_object_marker = visualization_msgs.msg.Marker()
    detected_3D_object_marker.header.stamp = stamp
    detected_3D_object_marker.header.frame_id = frame_id
    detected_3D_object_marker.action = visualization_msgs.msg.Marker.ADD
    detected_3D_object_marker.id = index
    detected_3D_object_marker.lifetime = rospy.Duration.from_sec(0.1)
    detected_3D_object_marker.type = visualization_msgs.msg.Marker.CUBE
    detected_3D_object_marker.pose.position.x = detection.x
    detected_3D_object_marker.pose.position.y = detection.y
    detected_3D_object_marker.pose.position.z = detection.z
   
    detected_3D_object_marker.scale.x = 1
    detected_3D_object_marker.scale.y = 1
    detected_3D_object_marker.scale.z = 1.5

    detected_3D_object_marker.color.r = color[0]
    detected_3D_object_marker.color.g = color[1]
    detected_3D_object_marker.color.b = color[2]
    detected_3D_object_marker.color.a = color[3]

    return detected_3D_object_marker