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

# ROS imports
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# T4AC imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from .monitor_classes import Node3D
from .utils import euler_to_quaternion

def get_lanes_markers(waypoint_list, rgb = [0,0,0]):
    """
    Get markers to represent map topology defined by every lane.

    Args:
        waypoint_list: List of waypoints centered in every lane of the map
            at a given distance.

    Returns:
        points: Point markers defining every lane of the map.
    """
    points = Marker()
    points.header.frame_id = "map"
    points.header.stamp = rospy.Time.now()
    points.ns = "map_visualizator_lanes"
    points.action = Marker.ADD
    points.pose.orientation.w = 1.0
    points.id = 0
    points.type = Marker.POINTS
    points.color.r = rgb[0]
    points.color.g = rgb[1]
    points.color.b = rgb[2]
    points.color.a = 1.0
    points.scale.x = 0.2
    points.scale.y = 0.2

    points.lifetime = rospy.Duration.from_sec(0)

    # Fill points.points list with every wp defining lanes
    for waypoint in waypoint_list:
        node_right = Node3D()
        node_left = Node3D()
        yaw = waypoint.transform.rotation.yaw
        alpha = yaw
        alpha_radians = math.radians(alpha)
        distance = waypoint.lane_width/2

        k = -1
        if waypoint.lane_id < 0: k = 1

        node_right.x = waypoint.transform.location.x + math.cos(alpha_radians)*distance*(-k)
        node_right.y = waypoint.transform.location.y - math.sin(alpha_radians)*distance*k
        node_right.z = waypoint.transform.location.z

        node_left.x = waypoint.transform.location.x - math.cos(alpha_radians)*distance*(-k)
        node_left.y = waypoint.transform.location.y + math.sin(alpha_radians)*distance*k
        node_left.z = waypoint.transform.location.z

        p1 = Point()
        p1.x = node_right.x
        p1.y = node_right.y
        p1.z = node_right.z

        p2 = Point()
        p2.x = node_left.x
        p2.y = node_left.y
        p2.z = node_left.z

        points.points.append(p1)
        points.points.append(p2)
    return points

def get_reg_elements_markers(landmarks):
    reg_elements_markers = MarkerArray()
    for landmark in landmarks:
        reg_element_marker = Marker()
        reg_element_marker.header.frame_id = "map"
        reg_element_marker.header.stamp = rospy.Time.now()
        reg_element_marker.ns = landmark.type
        reg_element_marker.id = landmark.id
        reg_element_marker.action = Marker.ADD
        reg_element_marker.type = Marker.CUBE
        reg_element_marker.scale.x = landmark.length 
        reg_element_marker.scale.y = landmark.width
        reg_element_marker.scale.z = landmark.height
        reg_element_marker.pose.position.x = landmark.transform.location.x 
        reg_element_marker.pose.position.y = landmark.transform.location.y 
        # HDMap's z-coordinate is from the road to bottom edge of the landmark
        reg_element_marker.pose.position.z = landmark.transform.location.z + (landmark.height/2)

        q = euler_to_quaternion (landmark.transform.rotation.roll, 
                                landmark.transform.rotation.pitch, 
                                landmark.transform.rotation.yaw)
        reg_element_marker.pose.orientation = q

        if (landmark.type == "Stop"):
            reg_element_marker.color.r = 1.0
            reg_element_marker.color.g = 0.0
            reg_element_marker.color.b = 0.0
            reg_element_marker.color.a = 0.3
        elif (landmark.type == "TrafficLight"):
            reg_element_marker.color.r = 0.0
            reg_element_marker.color.g = 1.0
            reg_element_marker.color.b = 0.0
            reg_element_marker.color.a = 0.3
        elif (landmark.type == "Crosswalk"):
            reg_element_marker.color.r = 1.0
            reg_element_marker.color.g = 1.0
            reg_element_marker.color.b = 1.0
            reg_element_marker.color.a = 0.3
        else:
            reg_element_marker.color.r = 0.0
            reg_element_marker.color.g = 0.0
            reg_element_marker.color.b = 0.0
            reg_element_marker.color.a = 0.3

        reg_elements_markers.markers.append(reg_element_marker)
    return reg_elements_markers

def get_waypoint(waypoint, rgb = [0,0,0], lifetime = 0.2, scale = 0.2):
    """
    Get way marker to represent a carla.Waypoint in RVIZ

    Args:
        waypoint: carla.Waypoint()
        rgb: Colour
        lifetime: Lifetime of the marker in seconds
        scale: Scale for the marker

    Returns:
        waypoint_marker: Marker to represent a waypoint in RVIZ
    """
    waypoint_marker = Marker()
    waypoint_marker.header.frame_id = "map"
    waypoint_marker.header.stamp = rospy.Time.now()
    waypoint_marker.ns = "waypoint_marker"
    waypoint_marker.action = Marker.ADD
    waypoint_marker.pose.orientation.w = 1.0
    waypoint_marker.id = 0
    waypoint_marker.type = Marker.POINTS
    waypoint_marker.color.r = rgb[0]
    waypoint_marker.color.g = rgb[1]
    waypoint_marker.color.b = rgb[2]
    waypoint_marker.color.a = 1.0
    waypoint_marker.scale.x = scale
    waypoint_marker.scale.y = scale
    waypoint_marker.lifetime = rospy.Duration(lifetime)

    point = Point()
    point.x = waypoint.transform.location.x
    point.y = waypoint.transform.location.y
    point.z = waypoint.transform.location.z
    waypoint_marker.points.append(point)

    return waypoint_marker

def get_way(way, rgb = [0,0,0], lifetime = 0.2, scale = 0.2, 
            marker_type=4, way_id=0):
    """
    Get way marker to represent a way in RVIZ

    Args:
        way: List of carla.Waypoint()
        rgb: Colour
        lifetime: Lifetime of the marker in seconds
        scale: Scale for the marker
        marker_type: (int) LINE_STRIP (4) or POINTS (8)
        way_id: (int)


    Returns:
        way_marker: Marker to represent a way in RVIZ
    """
    way_marker = Marker()
    way_marker.header.frame_id = "map"
    way_marker.header.stamp = rospy.Time.now()
    way_marker.ns = "way_marker"+str(way_id)
    way_marker.action = Marker.ADD
    way_marker.pose.orientation.w = 1.0
    way_marker.id = way_id
    way_marker.type = marker_type
    way_marker.color.r = rgb[0]
    way_marker.color.g = rgb[1]
    way_marker.color.b = rgb[2]
    way_marker.color.a = 1.0
    way_marker.scale.x = scale
    way_marker.scale.y = scale
    way_marker.lifetime = rospy.Duration(lifetime)

    for waypoint in way:
        point = Point()
        point.x = waypoint.transform.location.x
        point.y = waypoint.transform.location.y
        point.z = waypoint.transform.location.z
        way_marker.points.append(point)
    return way_marker

def get_lane(lane, rgb = [0,0,0], name = "", marker_type = 4,
             scale = 0.2, extra_z = 0, lifetime = 0.2, id=0, a=1):
    """
    Get lane marker to represent monitorized lane.

    Args:
        lane: Monitorized lane to represent of type t4ac_msgs.msg.Lane()
        rbg: Colour to represent the lane
        name: Differential name of the marker for namespace
        marker_type: (int) Type of marker, usually LINE_STRIP (4) or POINTS (8)
        scale: Scale for the marker
        extra_z: (int) Extra value for coordinate z
        lifetime: Lifetime of the marker in seconds
        id: (int)
        a: (float) 0 < a < 1

    Returns:
        lane_marker_right: Marker to represent right way of monitorized lane.
        lane_marker_left: Marker to represent left way of monitorized lane.
    """
    # Right way of lane 
    lane_marker_right = Marker()
    lane_marker_right.header.frame_id = "map"
    lane_marker_right.header.stamp = rospy.Time.now()
    # lane_marker_right.ns = str(lane.role)+"lane_marker_right"
    lane_marker_right.ns = "lane_" + name + "_marker_right"
    lane_marker_right.action = Marker.ADD
    lane_marker_right.pose.orientation.w = 1.0
    lane_marker_right.id = id
    lane_marker_right.type = marker_type
    lane_marker_right.color.r = rgb[0]
    lane_marker_right.color.g = rgb[1]
    lane_marker_right.color.b = rgb[2]
    lane_marker_right.color.a = a
    lane_marker_right.scale.x = scale
    lane_marker_right.scale.y = scale
    lane_marker_right.lifetime = rospy.Duration(lifetime)

    for node in lane.right.way:
        point = Point()
        point.x = node.x
        point.y = node.y
        point.z = node.z + extra_z
        lane_marker_right.points.append(point)

    # Left way of lane
    lane_marker_left = Marker()
    lane_marker_left.header.frame_id = "map"
    lane_marker_left.header.stamp = rospy.Time.now()
    # lane_marker_left.ns = str(lane.role)+"lane_marker_left"
    lane_marker_left.ns = "lane_" + name + "_marker_left"
    lane_marker_left.action = Marker.ADD
    lane_marker_left.pose.orientation.w = 1.0
    lane_marker_left.id = id
    lane_marker_left.type = marker_type
    lane_marker_left.color.r = rgb[0]
    lane_marker_left.color.g = rgb[1]
    lane_marker_left.color.b = rgb[2]
    lane_marker_left.color.a = a
    lane_marker_left.scale.x = scale
    lane_marker_left.scale.y = scale
    lane_marker_left.lifetime = rospy.Duration(lifetime)

    for node in lane.left.way:
        point = Point()
        point.x = node.x
        point.y = node.y
        point.z = node.z + extra_z
        lane_marker_left.points.append(point)

    return lane_marker_right, lane_marker_left

def get_nodes(nodes, rgb = [0,0,0], name = "", marker_type = 4,
             scale = 0.2, extra_z = 0, lifetime = 0.2):
    """
    Get nodes marker to represent a list of nodes in RVIZ

    Args:
        nodes: List of monitor_classes.Node3D
        rbg: Colour to represent the lane
        name: Differential name of the marker for namespace
        marker_type: (int) Type of marker, usually LINE_STRIP (4) or POINTS (8)
        scale: Scale for the marker
        extra_z: (int) Extra value for coordinate z
        lifetime: Lifetime of the marker in seconds

    Returns:
        nodes: Marker to represent a way in RVIZ
    """
    nodes_marker = Marker()
    nodes_marker.header.frame_id = "map"
    nodes_marker.header.stamp = rospy.Time.now()
    nodes_marker.ns = "nodes_marker" + name
    nodes_marker.action = Marker.ADD
    nodes_marker.pose.orientation.w = 1.0
    nodes_marker.id = 0
    nodes_marker.type = marker_type
    nodes_marker.color.r = rgb[0]
    nodes_marker.color.g = rgb[1]
    nodes_marker.color.b = rgb[2]
    nodes_marker.color.a = 1.0
    nodes_marker.scale.x = scale
    nodes_marker.lifetime = rospy.Duration(lifetime)

    for node in nodes:
        point = Point()
        point.x = node.x
        point.y = -node.y
        point.z = node.z + extra_z
        nodes_marker.points.append(point)
    return nodes_marker
