"""
================================================
In developement --> This module is not operative
Authors: Alejandro D. and J. Felipe Arango.
Last mod: J. Felipe Arango. 24/11/2022
================================================
Module to get affecting Regulatory Elements to the route
"""
# General imports
import sys 
import os
import glob
import math

# T4AC imports
from t4ac_msgs.msg import RegulatoryElement, MonitorizedRegElems

def get_regElems(current_waypoint, waypoint_route, segment_index, n1, 
                 map_object):
    """
    Get Regulatory Elements affecting the current lane of the route

    Args:
        current_waypoint: Current position of ego_vehicle 
        waypoint_route: Route as a list of Waypoint
        segment_index: (int) Index to locate in which segment of the route 
            is the ego_vehicle
        n1: (int) Number of waypoints to monitorize in front (current lane)

    Returns:
        regElems: (MonitorizedRegElems) List of RegulatoryElement() affecting
            any waypoint of the current lane in the route.
    """
    
    route = waypoint_route[segment_index:segment_index + n1]
    distance = 3.0 # The maximum distance to search for landmarks from the current waypoint

    regElems = MonitorizedRegElems()
    affecting_landmarks_ids = []

    # I have to improve distance (from ego_vehicle to landmark) calculation using s-t coordinate system
    # At the moment I use Euclidean distance
    dist_vehicle_2_landmark = 0
    last_pose = current_waypoint.transform.location
    
    for index, waypoint in enumerate(route):
        actual_pose = waypoint.transform.location
        dist_vehicle_2_landmark += math.sqrt(pow(actual_pose.x-last_pose.x,2) + pow(actual_pose.y-last_pose.y,2))
        last_pose = actual_pose

        landmarks_aux = map_object.get_affecting_landmarks(waypoint, distance)
        for landmark in landmarks_aux:
            if landmark.id not in affecting_landmarks_ids:
                regElem = RegulatoryElement()
                regElem.id = landmark.id
                regElem.type = landmark.type 
                regElem.height = landmark.height 
                regElem.width = landmark.width 
                regElem.length = landmark.length 
                regElem.pose = landmark.transform
                regElem.distance = dist_vehicle_2_landmark
                regElems.reg_elems.append(regElem)
                affecting_landmarks_ids.append(landmark.id)       

        if (index==0):  ##The first waypoint is always behind the ego_vehicle
            last_pose = current_waypoint.transform.location
            dist_vehicle_2_landmark = 0.0

    return regElems



