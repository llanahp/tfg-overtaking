#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
"""
Created on tue Nov 07 11:56:48 2021
@author: Carlos Gómez Huélamo
"""

import numpy as np
import t4ac_msgs.msg

# Auxiliar functions

def apply_tf(source_location, transform):  
    """
    Input: t4ac_msgs.msg.Node() in the source frame
    Output: t4ac_msgs.msg.Node() in the target frame 
    """
    centroid = np.array([0.0,0.0,0.0,1.0]).reshape(4,1)

    try:
        centroid[0,0] = source_location.x 
        centroid[1,0] = source_location.y
        centroid[2,0] = source_location.z
    except:
        centroid[0,0] = source_location[0] # LiDAR points (3,)
        centroid[1,0] = source_location[1]
        centroid[2,0] = source_location[2]

    aux = np.dot(transform,centroid) 

    target_location = t4ac_msgs.msg.Node()
    target_location.x = aux[0,0]
    target_location.y = aux[1,0]
    target_location.z = aux[2,0]

    return target_location