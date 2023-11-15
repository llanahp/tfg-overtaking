"""
This file calculates a route and plots on RVIZ both the route 
and the lane of the route

The workflow to debug using this file is:
    - Launch a roscore
    - Launch RVIZ
    - Launch the map_visualizator
    - Launch global_planner from Planning Layer
    - Run this file
    - Select 2D Pose Estimate in RVIZ for initial_xyz
    - Select 2D Nav Goal in RVIZ for goal_xyz
    - Visualize results in RVIZ
"""
import sys
import os

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from lane_waypoint_planner import LaneWaypointPlanner
from modules import markers_module

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from t4ac_mapping_layer.t4ac_map_monitor_ros.src.modules.lanes_module import calculate_lane

rospy.init_node("odometry_simulator_node", anonymous=True)

map_name = rospy.get_param("/t4ac/mapping/map_name")
map_path = rospy.get_param("/t4ac/mapping/map_path")
lwp = LaneWaypointPlanner(map_name, map_path)

def initial_pose_callback(initial_pose):
    global initial_xyz
    global route_flag
    route_flag = 1
    initial_xyz =(initial_pose.pose.pose.position.x, 
                  initial_pose.pose.pose.position.y, 
                  initial_pose.pose.pose.position.z)
    current_odometry = Odometry()
    current_odometry.pose.pose.position.x = initial_pose.pose.pose.position.x
    current_odometry.pose.pose.position.y = initial_pose.pose.pose.position.y
    current_odometry.pose.pose.position.z = initial_pose.pose.pose.position.z
    while(route_flag == 1):
        odometry_pub.publish(current_odometry)
        rospy.Rate(10).sleep()

def goal_callback(goal_pose):
    global initial_xyz
    global route_flag
    route_flag = 0
    goal_xyz = (goal_pose.pose.position.x,
                goal_pose.pose.position.y,
                goal_pose.pose.position.z)
    route = lwp.calculate_waypoint_route(3, initial_xyz, goal_xyz)
    route_lane = calculate_lane(route)

    central_way_marker = markers_module.get_way(route, [1, 0, 0], -1, 0.4, 4, 0)
    right_way_marker = markers_module.get_nodes(route_lane.right_way, [1, 0, 0], "right", 4, 0.4, 0, -1)
    left_way_marker = markers_module.get_nodes(route_lane.left_way, [1, 0, 0], "left", 4, 0.4, 0, -1)

    way_pub.publish(central_way_marker)
    way_pub.publish(right_way_marker)
    way_pub.publish(left_way_marker)

    route_flag = 1

initial_pose_sub = rospy.Subscriber(
        "/initialpose", PoseWithCovarianceStamped, initial_pose_callback)
goal_subscriber = rospy.Subscriber(
        "/t4ac/planning/goal", PoseStamped, goal_callback)

odometry_pub = rospy.Publisher("/t4ac/localization/pose", Odometry, queue_size=1)
way_pub = rospy.Publisher("/t4ac/debug/route_lanes", markers_module.visualization_msgs.msg.Marker, queue_size=1)

rospy.spin()