#!/usr/bin/env python3
"""
Receive path planning route, ego_vehicle_position and xodr map.
With that info calculate monitorized elements.

Authors: Alejandro D. and J. Felipe Arango.
Last mod: J. Felipe Arango. 24/11/2022
"""
# General imports
import sys 
import os

# ROS imports
import rospy
from nav_msgs.msg import Odometry
from t4ac_msgs.msg import Path, MonitorizedLanes, MonitorizedRegElems

# T4AC imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from map_parser.map_object import MapObject
from modules.route_module import path_to_waypoint_route, calculate_route_segment_centers, get_route_segment_index
from modules.calculus_module import braking_n_distance
from modules.monitor_module import calculate_lanes, calculate_intersections, calculate_regElems
from modules.utils import recalculate_landmarks


class MapMonitor:
    """
    Class for the map monitor. 
    Here are two main callbacks: one when a new route is published an other 
    when localization of th evehicle is published (localization is supposed 
    to be constantly published) 
    """

    def __init__(self, map_name, map_path, map_flag=0):
        # Map
        self.map_object = MapObject(map_name, map_path, map_flag)
        # CARLA's HDMaps have errors in the placement of the landmarks, therefore, we made some modifications:
        self.map_object.landmarks = recalculate_landmarks (self.map_object, map_name) 
        # Ego vehicle localization 
        self.route_segment_centers = None
        self.segment_index = -1
        self.ego_vehicle_location = None
        self.ego_vehicle_waypoint = None
        # Route
        # self.location_route = []
        self.waypoint_route = []
        self.flag_goal_reached = 0
        # Monitor
        self.n_min = 15
        self.monitor_flag = 0

        # Monitor ROS Publishers
        self.lanes_monitor_pub = rospy.Publisher(
            "/t4ac/mapping/monitor/lanes", MonitorizedLanes,
            queue_size=10)#, latch=True)
        self.intersections_monitor_pub = rospy.Publisher(
            "/t4ac/mapping/monitor/intersections", 
            MonitorizedLanes, queue_size=10)#, latch=True)
        self.regElems_monitor_pub = rospy.Publisher(
           "/t4ac/mapping/monitor/regElems", 
           MonitorizedRegElems, queue_size=10)#, latch=True)

        # Monitor ROS Subscribers
        self.route_sub = rospy.Subscriber("/t4ac/planning/route", Path, self.route_callback)
        self.localization_sub = rospy.Subscriber("/t4ac/localization/pose", Odometry, self.localization_callback)

    ### Route Callback ###
    def route_callback(self, path_route):
        """
        Callback function called when a route is published by path planner

        Args:
            path_route: Route of type nav_msgs/Path.msg

        Returns: 
            Set route_location and route_waypoint with new route published.
            Also check if vehicle is inside the route and in which segment.
        """
        # Generate route as a list of t4ac.Waypoint
        self.waypoint_route = path_to_waypoint_route(
            path_route, self.map_object)
        # Check conditions for map monitor
        if self.ego_vehicle_waypoint is not None:
            # Obtenemos lista de t4ac.Waypoint equidistantes a los puntos de self.waypoint_route
            self.route_segment_centers = (
                calculate_route_segment_centers(self.waypoint_route))
            # Get segment_index (if is outside of the route segment_index is -1)
            # Obtenemos el indice de la ruta donde nos encontramos (desde 0 hasta len(waypoint_route)-1)
            # Creo que esto no hace falta aqui, ya se hace cada vez que recibimos loc
            self.segment_index = get_route_segment_index(
                self.route_segment_centers, self.ego_vehicle_waypoint)
            self.monitor_flag = 1
        else:
            self.segment_index = -1
            # print("Warning! self.ego_vehicle_waypoint in map_monitor.py is None")
        
    ### Ego_vehicle_position Callback ###
    def localization_callback(self, ego_vehicle_odometry):
        """
        Callback function called when a ego_vehicle position is published

        Args:
            ego_vehicle_odometry: Current local UTM position of the 
                vehicle of type nav_msgs/Odometry.msg

        Returns: 
            Activates map monitor if possible
        """
        # Get ego-vehicle Odometry as T4ac_Waypoint
        self.ego_vehicle_waypoint = self.map_object.get_waypoint(
                ego_vehicle_odometry.pose.pose.position.x, 
                ego_vehicle_odometry.pose.pose.position.y, 
                ego_vehicle_odometry.pose.pose.position.z)
        
        # Calculate number of waypoints to monitorize depending on the velocity
        # 'n' is number of waypoints to monitorize in the frontside and 'n2' in
        # the backside
        n_max = braking_n_distance(ego_vehicle_odometry)
        if n_max < self.n_min: n_max = self.n_min
        n2_max = int(n_max/2)
        
        # Check conditions to monitorize
        if len(self.waypoint_route) > 0:
            if self.route_segment_centers is not None:
                self.segment_index = get_route_segment_index(
                    self.route_segment_centers, self.ego_vehicle_waypoint)
                # print(">>> self.segment_index = ", self.segment_index)
            else:
                pass
                # print("Warning! self.route_segment_centers == None in " \
                #       "map_monitor.py")
        else:
            pass
            # print("Warning! self.waypoint_route < 0 in map_monitor.py", 
                # self.segment_index)
            self.segment_index = -1

        # If conditions are ok, monitorize
        if self.segment_index >= 0:              
            # Check if current segment is last segment of the route
            if (self.segment_index == (len(self.waypoint_route)-1)):
                # or self.segment_index == (len(self.waypoint_route)-2)):
                if self.flag_goal_reached == 0:
                    print("Congratz, goal reached!")
                    self.flag_goal_reached = 1
            else:
                # Monitorize 
                if self.monitor_flag == 1:
                    print(">>> Monitoring...")
                    self.monitor_flag = 0

                # Set n front and n2 back waypoints to monitorize
                if (self.segment_index + n_max) < len(self.waypoint_route):
                    n1 = n_max
                else: 
                    n1 = len(self.waypoint_route) - self.segment_index
                if (self.segment_index - n2_max) < 0:
                    n2 = self.segment_index
                else:
                    n2 = n2_max
        
                # Calculate and publish monitorized lanes
                lanes = calculate_lanes(
                    self.map_object.map_waypoints, self.map_object.map_kdtree, 
                    self.segment_index, self.waypoint_route[:], n1, n2)
                if lanes is not None:
                    self.lanes_monitor_pub.publish(lanes)
                #else:
                    #print("Warning! lanes == None in map_monitor.py")

                # Calculate and publish monitorized intersections
                intersection_lanes = calculate_intersections(
                    self.waypoint_route[:], self.segment_index, n1, 
                    self.map_object)
                if intersection_lanes:
                    self.intersections_monitor_pub.publish(intersection_lanes)
                #else:
                    #print("Warning! intersection_lanes == None in map_monitor.py")

                # Calculate and publish regulatory elements
                regElems = calculate_regElems(
                    self.ego_vehicle_waypoint, self.waypoint_route[:], 
                    self.segment_index, n1, self.map_object)
                if regElems:
                    self.regElems_monitor_pub.publish(regElems)
                else:
                    #print("Warning! regElems == None in map_monitor.py")
                    regElems = MonitorizedRegElems()
                    self.regElems_monitor_pub.publish(regElems)

def map_monitor():
    rospy.init_node("map_monitor_node", anonymous=True)
    map_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..', "maps/xodr/"))
    map_name = rospy.get_param('t4ac/mapping/map_name') 
    
    map_monitor = MapMonitor(map_name, map_path)    
    rospy.spin()

if __name__ == '__main__':
    try:
        map_monitor()
    except rospy.ROSInterruptException:
        pass

