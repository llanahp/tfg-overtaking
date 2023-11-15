#!/usr/bin/env python3
"""
================
Waypoint Class
================
Waypoints are described as 3D directed points. They have a Transform which locates the waypoint 
in a road and orientates it according to the lane. They also store the road information belonging 
to said point regarding its lane and lane markings.

Authors: Alejandro D. and J. Felipe Arango.
Last mod: J. Felipe Arango. 24/11/2022
"""
# General imports
import math
import numpy as np
from scipy.spatial import KDTree

# T4AC imports
from .builder_classes import T4ac_Transform, T4ac_Location, T4ac_Rotation

class T4ac_Waypoint:
    """
    For initializing a waypoint in a specific position, the location
    parameter must be passed in T4ac_Location format. If not, other option
    is not passing any parameter an set the parameters after initializing the
    T4ac_Waypoint object.
    """
    def __init__(self, location=T4ac_Location("", "", ""), 
                       rotation=T4ac_Rotation("", "", "")):
        self.id = "" # (int)
        self.transform = T4ac_Transform(location, rotation)
        self.road_id = "" # (int)
        self.section_id = "" # (int)
        self.lane_id = "" # (int)
        self.s = "" # (float - meters)
        self.junction = "" # junction's ID, if -1 it is not junction (int)
        self.lane_width = "" # (float)
        self.lane_change = "" #can be none, right, left or both (string)
        self.lane_type = "" # (string)
        self.right_lane_marking = ""
        self.left_lane_marking = ""
        self.vmax = ""
        self.vunit = ""
        self.nLanes = ""         # Number of lanes in same direction
        self.lanePosition = ""   # Position of the current lane,
                                # starting from 1 to the right

    def get_closer_wp(self, waypoint_list):
        """
        Return closer wp given a wp list
        It can be usefull to get road and lane info of the self waypoint
        """
        closer_distance = 10000 # High arbitrary value
        for wp in waypoint_list:
            distance = math.sqrt((wp.transform.location.x-self.transform.location.x)**2 + 
                                 (wp.transform.location.y-self.transform.location.y)**2 +
                                 (wp.transform.location.z-self.transform.location.z)**2)
            if distance < closer_distance:
                closer_distance = distance
                closer_wp = wp
        return closer_wp

    def get_closer_right_wp(self, waypoint_list, kdtree):
        """
        Calculates closer right waypoint only considering it it is in a 
        different road or lane

        Args:
            self
            waypoint_list: (list)
            kdtree: (scipy.spatial.kdtree.KDTree)

        Returns:
            closer_right_wp
        """
        k = -1
        if self.lane_id < 0: k = 1

        alpha_radians = math.radians(self.transform.rotation.yaw)
        x = self.transform.location.x + math.cos(alpha_radians)*self.lane_width*(-k)
        y = self.transform.location.y - math.sin(alpha_radians)*self.lane_width*k
        z = self.transform.location.z

        current_location_array = np.array((x, y, z))
        closer_dist, closer_point = kdtree.query(
            current_location_array, 1)
        right_waypoint = waypoint_list[closer_point.numerator]
        return right_waypoint

    def get_closer_left_wp(self, waypoint_list, kdtree):
        """
        Calculates closer left waypoint only considering it it is in a 
        different road or lane

        Args:
            self
            waypoint_list: (list)
            kdtree: (scipy.spatial.kdtree.KDTree)

        Returns:
            closer_left_wp
        """
        k = -1
        if self.lane_id < 0: k = 1

        alpha_radians = math.radians(self.transform.rotation.yaw)
        x = self.transform.location.x - math.cos(alpha_radians)*self.lane_width*(-k)
        y = self.transform.location.y + math.sin(alpha_radians)*self.lane_width*k
        z = self.transform.location.z

        current_location_array = np.array((x, y, z))
        closer_dist, closer_point = kdtree.query(
            current_location_array, 1)
        left_waypoint = waypoint_list[closer_point.numerator]
        return left_waypoint

    def distance(self, waypoint):
        """
        Calculate distance from current waypoint to other waypoint

        Args:
            waypoint: Goal waypoint to compute distance from current

        Returns:
            distance: (float) euclidean distance
        """
        distance = math.sqrt(
            (waypoint.transform.location.x-self.transform.location.x)**2 + 
            (waypoint.transform.location.y-self.transform.location.y)**2)

        return distance

    def get_lanePosition(self, lane_id, road):
        """
        Returns the number of lanes in the current road with same direction
        and the position of the current lane, starting to count from 1 from 
        the right to the left

        Args:
            lane_id: (int) Id of the current lane
            road: (T4ac_Road) Road containing the current lane

        Returns:
            nLanes: (int) Number of lanes
            lanePosition: (int) Position of the current lane
        """

        nLanes = 0
        lanePosition = 0

        if lane_id < 0:
            for lane in road.lanes.laneSections[0].right:
                if lane.type == "driving":
                    nLanes += 1
                if lane.id == lane_id:
                    lanePosition = nLanes
        elif lane_id > 0:
            for lane in road.lanes.laneSections[0].left:
                if lane.type == "driving":
                    nLanes += 1
                if lane.id == lane_id:
                    lanePosition = nLanes

        return nLanes, lanePosition