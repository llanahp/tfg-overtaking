#!/usr/bin/env python3
"""
Module to implement callbacks for monitorized elements. Every time a
monitorized element is published, there is a callback to represent it
in RVIZ.

Authors: Alejandro D. and J. Felipe Arango.
Last mod: J. Felipe Arango. 24/11/2022
"""
# General imports
import sys 
import os

# ROS imports
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from t4ac_msgs.msg import MonitorizedLanes, MonitorizedRegElems

# T4AC imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from modules.markers_module import get_lane
from modules.utils import euler_to_quaternion

class MonitorVisualizator:
    """
    Class for the monitor visualizator.
    Represents all the monitorized elements in ROS topics as markers, to 
    be visualized in RVIZ.
    """

    def __init__(self):
        # ROS Subscribers
        self.lanes_monitor_sub = rospy.Subscriber(
            "/t4ac/mapping/monitor/lanes", 
            MonitorizedLanes, self.lanes_callback)
        self.intersections_monitor_sub = rospy.Subscriber(
            "/t4ac/mapping/monitor/intersections", 
            MonitorizedLanes, self.intersections_callback)
        self.regElems_monitor_sub = rospy.Subscriber(
            "/t4ac/mapping/monitor/regElems", 
            MonitorizedRegElems, self.regElems_callback)

        # ROS Publishers
        self.lanes_monitor_visualizator_pub = rospy.Publisher(
            "/t4ac/mapping/monitor/lanes_marker", 
            MarkerArray, queue_size = 10)
        self.intersections_monitor_visualizator_pub = rospy.Publisher(
            "/t4ac/mapping/monitor/intersections_marker", 
            MarkerArray, queue_size = 10)
        self.regElems_monitor_visualizator_pub = rospy.Publisher(
            "/t4ac/mapping/monitor/regElems_marker", 
            MarkerArray, queue_size = 10)

    def lanes_callback(self, lanes):
        """
        Callback function called when lanes are published by the map_monitor
        in /t4ac/mapping/monitor/lanes

        Args:
            lanes: (t4ac_msgs.msg.MonitorizedLanes)
                Monitorized lanes pusblished by the map_monitor

        Returns:
            Publish lane markers to be visualized in RVIZ
        """
        standard_lane_markers = MarkerArray()
        for lane in lanes.lanes:
            if lane.role == "current_front" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += get_lane(
                        lane, [1,0,0], "current_front_", 4, 0.2, 0.1, 0.5, 0)

            elif lane.role == "current_back" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += get_lane(
                        lane, [0,0,1], "current_back_", 4, 0.2, 0.1, 0.5, 2)

            elif lane.role == "right_front" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += get_lane(
                        lane, [1,1,0], "right_front_", 8, 0.4, 0.1, 0.5, 4)

            elif lane.role == "right_back" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += get_lane(
                        lane, [1,1,0], "right_back_", 8, 0.4, 0.1, 0.5, 6, 0.5)

            elif lane.role == "left_front" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += get_lane(
                        lane, [1,1,0], "left_front_", 8, 0.4, 0.1, 0.5, 8)

            elif lane.role == "left_back" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += get_lane(
                        lane, [1,1,0], "left_back_", 8, 0.4, 0.1, 0.5, 10, 0.5)

        self.lanes_monitor_visualizator_pub.publish(standard_lane_markers)


    def intersections_callback(self, intersection_lanes):
        """
        Callback function called when intersection_lanes are published by 
        the map_monitor in /t4ac/mapping/monitor/intersections

        Args:
            intersection_lanes: (t4ac_msgs.msg.MonitorizedLanes).
                Monitorized intersection_lanes pusblished by the map_monitor

        Returns:
            Publish lane_intersection markers to be visualized in RVIZ
        """
        intersection_lane_markers = MarkerArray()
        i = 0
        for lane in intersection_lanes.lanes:
            if lane.role == "merge": # Colour yellow
                intersection_lane_markers.markers += ( 
                    get_lane(
                        lane, [1,1,0], "merge_"+str(i), 4, 0.2, 0.05, 1, i))
                i += 1

            elif lane.role == "split": # Colour orange
                intersection_lane_markers.markers += (
                    get_lane(
                        lane, [1,0.5,0], "split_"+str(i), 4, 0.2, 0.05, 1, i))
                i += 1

            elif lane.role == "cross": # Colour purple
                intersection_lane_markers.markers += (
                    get_lane(
                        lane, [1,0,1], "cross_"+str(i), 4, 0.2, 0.05, 1, i))
                i += 1

        self.intersections_monitor_visualizator_pub.publish(intersection_lane_markers)

    def regElems_callback(self, regElems):
        """
        Callback function called when regElems are published by 
        the map_monitor in /t4ac/mapping/monitor/regElems

        Args:
            regElems: (t4ac_msgs.msg.MonitorizedRegElem).
                Monitorized Regulatory Elements pusblished by the map_monitor

        Returns:
            Publish Regulatory Element markers to be visualized in RVIZ
        """
    
        reg_elements_markers = MarkerArray()

        if len(regElems.reg_elems) > 0:
            for regElem in regElems.reg_elems:
                reg_element_marker = Marker()
                reg_element_marker.header.frame_id = "map"
                reg_element_marker.header.stamp = rospy.Time.now()
                reg_element_marker.ns = regElem.type + "-monitor"
                reg_element_marker.id = regElem.id
                reg_element_marker.action = Marker.ADD
                reg_element_marker.type = Marker.CUBE
                reg_element_marker.lifetime = rospy.Duration.from_sec(0.2)
                reg_element_marker.scale.x = regElem.length 
                reg_element_marker.scale.y = regElem.width
                reg_element_marker.scale.z = regElem.height
                reg_element_marker.pose.position.x = regElem.pose.location.x 
                reg_element_marker.pose.position.y = regElem.pose.location.y 
                # HDMap's z-coordinate is from the road to bottom edge of the landmark
                reg_element_marker.pose.position.z = regElem.pose.location.z  + (regElem.height/2)

                q = euler_to_quaternion (regElem.pose.rotation.roll, 
                                        regElem.pose.rotation.pitch, 
                                        regElem.pose.rotation.yaw)
                reg_element_marker.pose.orientation = q

                if (regElem.type == "Stop"):
                    reg_element_marker.color.r = 1.0
                    reg_element_marker.color.g = 0.0
                    reg_element_marker.color.b = 0.0
                    reg_element_marker.color.a = 1.0
                elif (regElem.type == "TrafficLight"):
                    reg_element_marker.color.r = 0.0
                    reg_element_marker.color.g = 1.0
                    reg_element_marker.color.b = 0.0
                    reg_element_marker.color.a = 1.0
                elif (regElem.type == "Crosswalk"):
                    reg_element_marker.color.r = 1.0
                    reg_element_marker.color.g = 1.0
                    reg_element_marker.color.b = 1.0
                    reg_element_marker.color.a = 1.0
                else:
                    reg_element_marker.color.r = 0.0
                    reg_element_marker.color.g = 0.0
                    reg_element_marker.color.b = 0.0
                    reg_element_marker.color.a = 1.0

                reg_elements_markers.markers.append(reg_element_marker)
            self.regElems_monitor_visualizator_pub.publish(reg_elements_markers)


def monitor_visualizator():
    # Init node
    rospy.init_node("monitor_visualizator_node", anonymous=True)
    monitor_visualizator = MonitorVisualizator()
    rospy.spin()

if __name__ == '__main__':
    try:
        monitor_visualizator()
    except rospy.ROSInterruptException:
        pass
