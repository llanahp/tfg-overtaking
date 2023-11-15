"""
Module that implement functions to generate  util marker objects to represent
different elements of the map in RVIZ

Authors: Alejandro D. and J. Felipe Arango.
Last mod: J. Felipe Arango. 24/11/2022
"""
# General imports
import sys 
import os
import math 
import numpy as np
from copy import deepcopy
# ROS imports
from geometry_msgs.msg import Quaternion

# T4AC imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from map_parser.map_utils import get_point_in_line
from map_parser.landmark import T4ac_LandmarkRoad, T4ac_LandmarkPose

def euler_to_quaternion(roll, pitch, yaw):
    """
    Return the orientation of our ego-vehicle in quaternion based on Euler angles (Roll = x, Pitch = y, Yaw = z)
    """
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    q = Quaternion (qx, qy, qz, qw)
    return q  

def recalculate_landmarks(map_object, map_name):
    landmarks = map_object.landmarks
    maps_debugged = ["Town01", "Town02", "Town03", "Town04", "Town05", "CampusUAH_v1_7"]
    if not map_name in maps_debugged:
                print('\033[1;93m'+'Warning: %s not debugged by map monitor.' % (map_name) +'\033[0;m')
                return landmarks
    landmarks_aux = []
    for landmark in landmarks:
        if landmark.type == "TrafficLight":
            if (map_name == "Town03"):
                # In these CARLA's HDMaps only one Signal_3Light_Post01 (of the three that each traffic light has) is positioned. 
                # Moreover, it is badly positioned, placed on the post and with a incorrect zOffset (We reset it to 0.0).
                # For the moment, we reposition the traffic lights, as we know the distances from Unreal Engine.
                zOffset = 5.25 # 6.4m from Signal_3Light_Post01 top
                left_t = 11.5
                center_t = 8.0
                right_t = 5.0
                # Create left-right traffic lights that are not defined by default.
                left_landmark = deepcopy(landmark)
                right_landmark = deepcopy(landmark)
                # Apply translations
                left_landmark.transform.location = get_point_in_line (landmark.transform.location.x, landmark.transform.location.y, 
                                            zOffset, left_t, math.radians(landmark.transform.rotation.yaw)+ math.pi/2)
                right_landmark.transform.location = get_point_in_line (landmark.transform.location.x, landmark.transform.location.y, 
                                            zOffset, right_t, math.radians(landmark.transform.rotation.yaw)+ math.pi/2)
                landmark.transform.location = get_point_in_line (landmark.transform.location.x, landmark.transform.location.y, 
                                            zOffset, center_t, math.radians(landmark.transform.rotation.yaw)+ math.pi/2)
                # Redefine IDs ->  Left = 1 1_ _ _ _   ;   Right = 2 2_ _ _ _
                left_landmark.id += 110000
                right_landmark.id += 220000
                # Add left-right traffic lights
                landmarks_aux.append(left_landmark)
                landmarks_aux.append(right_landmark)
            elif (map_name == "Town01" or map_name == "Town02"):
                zOffset = 2.35  # 3.5m from Signal_3Light_Post01 top
                landmark.transform.location.z = zOffset
            elif (map_name == "Town04"):
                zOffset = 5.25 # 6.4m from Signal_3Light_Post01 top
                t = 5.0
                landmark.transform.location = get_point_in_line (landmark.transform.location.x, landmark.transform.location.y, 
                                            zOffset, t, math.radians(landmark.transform.rotation.yaw)+ math.pi/2)
            elif (map_name == "Town05"):
                zOffset = 5.25 # 6.4m from Signal_3Light_Post01 top
                left_t = 10.5
                center_t = 8.0
                right_t = 5.0
                zOffset_post = 2.35 # 3.5m from Signal_3Light_Post01 top
                # Create left-center-right traffic lights that are not defined by default.
                left_landmark = deepcopy(landmark)
                center_landmark = deepcopy(landmark)
                right_landmark = deepcopy(landmark)
                # Apply translations
                left_landmark.transform.location = get_point_in_line (landmark.transform.location.x, landmark.transform.location.y, 
                                            zOffset, left_t, math.radians(landmark.transform.rotation.yaw)+ math.pi/2)
                center_landmark.transform.location = get_point_in_line (landmark.transform.location.x, landmark.transform.location.y, 
                                            zOffset, center_t, math.radians(landmark.transform.rotation.yaw)+ math.pi/2)
                right_landmark.transform.location = get_point_in_line (landmark.transform.location.x, landmark.transform.location.y, 
                                            zOffset, right_t, math.radians(landmark.transform.rotation.yaw)+ math.pi/2)
                landmark.transform.location.z = zOffset_post
                # Redefine IDs ->  Left = 1 1_ _ _ _   ;   Right = 2 2_ _ _ _
                left_landmark.id += 110000
                center_landmark.id += 220000
                right_landmark.id += 330000
                # Add left-right traffic lights
                landmarks_aux.append(left_landmark)
                landmarks_aux.append(center_landmark)
                landmarks_aux.append(right_landmark)

        elif landmark.type == "Crosswalk":
            # CARLA HDMaps do not have implemented validities for the objects (Crosswalk and Stops), 
            # so we do not know which lanes are affected. We assume that Crosswalk affects all lanes in which it is defined.
            distance_step = 0.5
            heading = math.radians(landmark.transform.rotation.yaw)
            # Look for driving-lanes on the Crosswalk - center
            ini_pose = landmark.transform.location
            for t in np.arange (-landmark.width/2, landmark.width/2, distance_step):
                pose = get_point_in_line(ini_pose.x, ini_pose.y, 0, t, heading+ math.pi/2)
                waypoint = map_object.get_waypoint (pose.x, pose.y, 0, True)
                landmark.affecting_roads = add_affecting_roads_2_landmark (landmark.affecting_roads, waypoint)

            # Look for driving-lanes on the Crosswalk - bottom
            ini_pose = get_point_in_line(landmark.transform.location.x, landmark.transform.location.y, 0, -landmark.length/2, heading)
            for t in np.arange (-landmark.width/2, landmark.width/2, distance_step):
                pose = get_point_in_line(ini_pose.x, ini_pose.y, 0, t, heading+ math.pi/2)
                waypoint = map_object.get_waypoint (pose.x, pose.y, 0, True)
                landmark.affecting_roads = add_affecting_roads_2_landmark (landmark.affecting_roads, waypoint)

            # Look for driving-lanes on the Crosswalk - top
            ini_pose = get_point_in_line(landmark.transform.location.x, landmark.transform.location.y, 0, +landmark.length/2, heading)
            for t in np.arange (-landmark.width/2, landmark.width/2, distance_step):
                pose = get_point_in_line(ini_pose.x, ini_pose.y, 0, t, heading+ math.pi/2)
                waypoint = map_object.get_waypoint (pose.x, pose.y, 0, True)
                landmark.affecting_roads = add_affecting_roads_2_landmark (landmark.affecting_roads, waypoint)

        elif landmark.type == "Stop":            
            # CARLA HDMaps do not have implemented validities for the objects (Crosswalk and Stops), 
            # so we do not know which lanes are affected. We assume that Crosswalk affects all lanes in which it is defined.
            distance_step = 0.5
            heading = math.radians(landmark.transform.rotation.yaw)
            # Look for driving-lanes on the Crosswalk
            ini_pose = landmark.transform.location
            for t in np.arange (-landmark.width/2, landmark.width/2, distance_step):
                pose = get_point_in_line(ini_pose.x, ini_pose.y, 0, t, heading+ math.pi/2)
                waypoint = map_object.get_waypoint (pose.x, pose.y, 0, True)
                landmark.affecting_roads = add_affecting_roads_2_landmark (landmark.affecting_roads, waypoint)
        else:
            pass
    landmarks += landmarks_aux
    return landmarks

def add_affecting_roads_2_landmark (affecting_roads, waypoint):    
    for i_road, road in enumerate(affecting_roads):
        if road.road_id == waypoint.road_id:
            if not waypoint.lane_id in road.lanes:
                affecting_roads[i_road].lanes.append(waypoint.lane_id)
            return affecting_roads
    
    affecting_road = T4ac_LandmarkRoad()
    affecting_road.road_id = waypoint.road_id
    affecting_road.pose = T4ac_LandmarkPose(waypoint.s, waypoint.lane_width/2, 0, 0)
    affecting_road.orientation = 'both'
    affecting_road.lanes.append(waypoint.lane_id) 
    affecting_roads.append(affecting_road)
    return affecting_roads