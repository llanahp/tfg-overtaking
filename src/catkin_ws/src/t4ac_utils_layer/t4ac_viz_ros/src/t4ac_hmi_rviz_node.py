#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on tue Feb 12 13:43:17 2022
@author: Santiago Montiel Marín and Carlos Gómez Huélamo
"""

import rospy
from std_msgs.msg import ColorRGBA
from jsk_rviz_plugins.msg import OverlayText
import t4ac_msgs.msg
import std_msgs.msg
import nav_msgs.msg
import math

class T4AC_HMI_RVIZ:
    """
    """
    def __init__(self) -> None:
        """
        """

        # OverlayText fields: 

        # uint8 ADD=0
        # uint8 DELETE=1
        # uint8 action
        # int32 width
        # int32 height
        # int32 left
        # int32 top
        # std_msgs/ColorRGBA bg_color
        # float32 r
        # float32 g
        # float32 b
        # float32 a
        # int32 line_width
        # float32 text_size
        # string font
        # std_msgs/ColorRGBA fg_color
        # float32 r
        # float32 g
        # float32 b
        # float32 a
        # string text

        # Auxiliar variables

        self.loc_ego = t4ac_msgs.msg.Node(50000,50000,0)
        rospy.set_param('/rqt_gauges/topic1', '/t4ac/hmi/ego_linear_velocity')
        rospy.set_param('/rqt_gauges/gauge_name1', 'km/h')

        # ROS communications

        self.sub_front_obstacle = rospy.Subscriber("/t4ac/perception/monitors/front_obstacle",
                                                   t4ac_msgs.msg.Obstacle,
                                                   self.front_obstacle_callback)
        self.sub_decision_making_state = rospy.Subscriber("/t4ac/decision_making/state",
                                                          std_msgs.msg.String,
                                                          self.decision_making_state_callback)
        self.sub_current_traffic_light = rospy.Subscriber("/t4ac/perception/monitors/traffic_sign_list",
                                                   t4ac_msgs.msg.Traffic_Sign_List,
                                                   self.current_traffic_light_callback)
        self.sub_current_stop = rospy.Subscriber("/t4ac/mapping/current_stop",
                                                   t4ac_msgs.msg.RegulatoryElement,
                                                   self.current_stop_callback)
        self.sub_cmd_vel = rospy.Subscriber('/t4ac/control/cmd_vel', 
                                            t4ac_msgs.msg.CarControl, 
                                            self.read_cmd_vel_callback)                                           
        self.sub_localization_pose = rospy.Subscriber("/t4ac/localization/pose",
                                                   nav_msgs.msg.Odometry,
                                                   self.localization_pose_callback)

        self.pub_front_obstacle_hmi = rospy.Publisher("/t4ac/hmi/front_obstacle", OverlayText, queue_size=10)
        self.pub_decision_making_state_hmi = rospy.Publisher("/t4ac/hmi/decision_making_state", OverlayText, queue_size=10)
        self.pub_current_traffic_light_hmi = rospy.Publisher("/t4ac/hmi/current_traffic_light", OverlayText, queue_size=10)
        self.pub_current_stop_hmi = rospy.Publisher("/t4ac/hmi/current_stop", OverlayText, queue_size=10)
        self.pub_ego_linear_velocity_hmi = rospy.Publisher("/t4ac/hmi/ego_linear_velocity", std_msgs.msg.Float64, queue_size=10)

    def read_cmd_vel_callback(self,cmd_vel):
        """
        Return the current cmd_vel command
        """

        cmd_vel_ = std_msgs.msg.Float64(round(cmd_vel.velocity*3.6,2))

    def localization_pose_callback(self, localization_pose_msg):
        """
        """

        self.loc_ego.x = localization_pose_msg.pose.pose.position.x
        self.loc_ego.y = localization_pose_msg.pose.pose.position.y
        ego_linear_vel = localization_pose_msg.twist.twist.linear.x * 3.6 # m/s -> km/h 

        self.pub_ego_linear_velocity_hmi.publish(int(ego_linear_vel))

    def decision_making_state_callback(self, decision_making_state_msg):
        """
        """

        decision_making_state_hmi = OverlayText()

        # Action

        decision_making_state_hmi.action = 0 # Action: ADD: 0, DELETE: 1 (Uint8)

        # Geometric parameters

        decision_making_state_hmi.width = 600 # Width of the overlay (int32)
        decision_making_state_hmi.height = 20 # Height of the overlay (int32)
        decision_making_state_hmi.left = 35 # X-coordinate of the top-left point of the overlay (int32)
        decision_making_state_hmi.top = 35 # Y-coordinate of the top-left point of the overlay (int32)

        # Text parameters

        decision_making_state_hmi.text = f"Current behaviour: {decision_making_state_msg.data}" # Text of the box (string)
        decision_making_state_hmi.text_size = 12 # Text size (int32)
        decision_making_state_hmi.fg_color = ColorRGBA(0.8, 0.8, 0.0, 1.0) # Color of the text (red) (ColorRGBA)
        decision_making_state_hmi.bg_color = ColorRGBA(0.0, 0.2, 0.8, 0.25) # Color of the background (transparent blue)

        self.pub_decision_making_state_hmi.publish(decision_making_state_hmi)

    def front_obstacle_callback(self, front_obstacle_msg):
        """
        """

        dist2ego = round(front_obstacle_msg.dist2ego,2)
        object_type = front_obstacle_msg.type

        front_obstacle_hmi = OverlayText()

        # Action

        front_obstacle_hmi.action = 0 # Action: ADD: 0, DELETE: 1 (Uint8)

        # Geometric parameters

        front_obstacle_hmi.width = 600 # Width of the overlay (int32)
        front_obstacle_hmi.height = 20 # Height of the overlay (int32)
        front_obstacle_hmi.left = 35 # X-coordinate of the top-left point of the overlay (int32)
        front_obstacle_hmi.top = 55 # Y-coordinate of the top-left point of the overlay (int32)

        # Text parameters

        if dist2ego != 50000:
            front_obstacle_hmi.text = f"Relevant {object_type} at {dist2ego} m" # Text of the box (string)
        else:
            front_obstacle_hmi.text = "No relevant obstacle around"
        front_obstacle_hmi.text_size = 12 # Text size (int32)
        front_obstacle_hmi.fg_color = ColorRGBA(0.8, 0.8, 0.0, 1.0) # Color of the text (red) (ColorRGBA)
        front_obstacle_hmi.bg_color = ColorRGBA(0.0, 0.2, 0.8, 0.25) # Color of the background (transparent blue)

        self.pub_front_obstacle_hmi.publish(front_obstacle_hmi)

    def current_traffic_light_callback(self, traffic_sign_list_msg):
        """
        """

        current_traffic_light_hmi = OverlayText()
        current_traffic_light = traffic_sign_list_msg.traffic_sign_list[0]

        if self.loc_ego.x != 50000:
            dist2ego = 50000
            if current_traffic_light.global_pose.pose.position.x != 50000:
                tl_global_x = current_traffic_light.global_pose.pose.position.x
                tl_global_y = current_traffic_light.global_pose.pose.position.y
                dist2ego = round(math.sqrt(pow(self.loc_ego.x-tl_global_x,2)+pow(self.loc_ego.y-tl_global_y,2)),2)
            else:
                dist2ego = 50000

            current_traffic_light_type = "None"
            if current_traffic_light.type == "traffic light_g":
                current_traffic_light_type = "Green"
            elif current_traffic_light.type == "traffic light_y":
                current_traffic_light_type = "Yellow"
            elif current_traffic_light.type == "traffic light_r":
                current_traffic_light_type = "Red"

            # Action

            current_traffic_light_hmi.action = 0 # Action: ADD: 0, DELETE: 1 (Uint8)

            # Geometric parameters

            current_traffic_light_hmi.width = 600 # Width of the overlay (int32)
            current_traffic_light_hmi.height = 20 # Height of the overlay (int32)
            current_traffic_light_hmi.left = 35 # X-coordinate of the top-left point of the overlay (int32)
            current_traffic_light_hmi.top = 35 # Y-coordinate of the top-left point of the overlay (int32)

            # Text parameters

            if dist2ego != 50000:
                current_traffic_light_hmi.text = f"{current_traffic_light_type} traffic light at {dist2ego} m" # Text of the box (string)
            else:
                current_traffic_light_hmi.text = "No relevant traffic light"
            current_traffic_light_hmi.text_size = 12 # Text size (int32)
            current_traffic_light_hmi.fg_color = ColorRGBA(0.8, 0.8, 0.0, 1.0) # Color of the text (red) (ColorRGBA)
            current_traffic_light_hmi.bg_color = ColorRGBA(0.0, 0.2, 0.8, 0.25) # Color of the background (transparent blue)

            self.pub_current_traffic_light_hmi.publish(current_traffic_light_hmi)

    def current_stop_callback(self, current_stop_msg):
        """
        """

        current_stop_hmi = OverlayText()

        if self.loc_ego.x != 50000:
            dist2ego = 50000
            if current_stop_msg.global_location.pose.position.x != 50000:
                stop_global_x = current_stop_msg.global_location.pose.position.x
                stop_global_y = current_stop_msg.global_location.pose.position.y
                dist2ego = round(math.sqrt(pow(self.loc_ego.x-stop_global_x,2)+pow(self.loc_ego.y-stop_global_y,2)),2)
            else:
                dist2ego = 50000
            
            # Action

            current_stop_hmi.action = 0 # Action: ADD: 0, DELETE: 1 (Uint8)

            # Geometric parameters

            current_stop_hmi.width = 600 # Width of the overlay (int32)
            current_stop_hmi.height = 20 # Height of the overlay (int32)
            current_stop_hmi.left = 35 # X-coordinate of the top-left point of the overlay (int32)
            current_stop_hmi.top = 35 # Y-coordinate of the top-left point of the overlay (int32)

            # Text parameters

            if dist2ego != 50000:
                current_stop_hmi.text = f"Stop signal at: {dist2ego} m" # Text of the box (string)
            else:
                current_stop_hmi.text = "No relevant stop signal"
            current_stop_hmi.text_size = 12 # Text size (int32)
            current_stop_hmi.fg_color = ColorRGBA(0.8, 0.8, 0.0, 1.0) # Color of the text (red) (ColorRGBA)
            current_stop_hmi.bg_color = ColorRGBA(0.0, 0.2, 0.8, 0.25) # Color of the background (transparent blue)

            self.pub_current_stop_hmi.publish(current_stop_hmi)

def main(args=None):
    """
    """
    # Initialize ROS node

    rospy.init_node("t4ac_hmi_rviz_node")
    overlay_node = T4AC_HMI_RVIZ()
    rate = rospy.Rate(10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down ROS T4AC HMI RVIZ node")

if __name__ == "__main__":
    main()
