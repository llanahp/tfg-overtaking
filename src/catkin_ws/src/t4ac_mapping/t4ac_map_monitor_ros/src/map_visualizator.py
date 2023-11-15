#!/usr/bin/env python3
"""
Map Visualizator without using PythonAPI

Represent static elements of the map in RVIZ using markers.
Elements represented are:
    - Lanes -> grey
    - Stop signals -> red
    - Yield signals -> blue
    - TrafficLight signals -> green
    - Crosswalks -> white

Authors: Alejandro D. and J. Felipe Arango.
Last mod: J. Felipe Arango. 24/11/2022
"""
# General imports
import sys 
import os

# ROS imports
import rospy
from visualization_msgs.msg import MarkerArray, Marker

# T4AC imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from map_parser.map_object import MapObject
from modules.markers_module import get_lanes_markers, get_reg_elements_markers

map_visualizator_pub = rospy.Publisher(
    "/t4ac/mapping/map/lanes_marker", Marker, queue_size=1)
reg_elements_markers_pub = rospy.Publisher(
    "/t4ac/mapping/map/reg_elements_marker", MarkerArray, queue_size=1)

def main():
    rospy.init_node("map_visualizator_node", anonymous=True)
    rate = rospy.Rate(1)
    map_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..', "maps/xodr/"))
    map_name = rospy.get_param("/t4ac/mapping/map_name")
    previous_map_name = None

    map_object = MapObject(map_name, map_path)
    waypoints = map_object.map_waypoints
    lane_markers = get_lanes_markers(waypoints, rgb=[192,192,192])
    landmarks = map_object.landmarks  
    reg_elements_markers = get_reg_elements_markers(landmarks)

    while not rospy.is_shutdown(): #() and not KeyboardInterrupt):
        map_name = rospy.get_param("/t4ac/mapping/map_name")
        if map_name != previous_map_name:

            map_object = MapObject(map_name, map_path)

            waypoints = map_object.map_waypoints
            lane_markers = get_lanes_markers(waypoints, rgb=[0.5,0.5,0.5])
            landmarks = map_object.landmarks  
            reg_elements_markers = get_reg_elements_markers(landmarks)
            
            previous_map_name = map_name
        
        map_visualizator_pub.publish(lane_markers)
        reg_elements_markers_pub.publish(reg_elements_markers)

        rate.sleep()

if __name__ == '__main__':
    print("Start Map visualizator node")
    main()