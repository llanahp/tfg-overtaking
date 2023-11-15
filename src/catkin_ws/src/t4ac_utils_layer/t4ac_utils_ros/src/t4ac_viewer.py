#!/usr/bin/env python3
"""
T4ac architecture status viewer

Authors: J. Felipe Arango.
Last mod: J. Felipe Arango. 24/11/2022

Requirement: sudo apt install ros-melodic-jsk-rviz-plugins    ||   sudo apt install ros-noetic-jsk-rviz-plugins
"""

# General imports
import math

# ROS imports
import rospy
import tf
from std_msgs.msg import ColorRGBA, String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from jsk_rviz_plugins.msg import OverlayText

# T4AC imports
from t4ac_msgs.msg import CarControl, MonitorizedRegElems



class t4ac_viewer():
    def __init__(self) -> None:
        ################################### ROS CONFIGURATION ########################################
        rospy.init_node("t4ac_viewer_node", anonymous=True)

        ########## Publishers 
        text_pub = rospy.Publisher("/t4ac/utils/status", OverlayText, queue_size=1)
        ########## Subscribers 
        #Mapping
        regElems_monitor_sub = rospy.Subscriber("/t4ac/mapping/monitor/regElems", MonitorizedRegElems, self.regElems_callback)
        #Localization
        pose_sub = rospy.Subscriber("/t4ac/localization/pose", Odometry, self.loc_callback)
        #Control
        cmd_vel_sub = rospy.Subscriber("/t4ac/control/cmd_vel", CarControl, self.control_callback)
        #Perception
        yolo_sub = rospy.Subscriber("/t4ac/decision_making/state", String, self.yolo_callback)
        #Decision
        decision_sub = rospy.Subscriber("/t4ac/decision_making/action", String, self.decision_callback)

        #Timer
        rate = 5
        timer = rospy.Rate(rate)

        ################################### Variables ###############################################
        #Local
        text_mgs = OverlayText()
        text_mgs.width = 800
        text_mgs.height = 120
        text_mgs.left = 10
        text_mgs.top = 10
        text_mgs.text_size = 12
        text_mgs.line_width = 2
        text_mgs.font = "DejaVu Sans Mono"
        text_mgs.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        text_mgs.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)

        error_str = """<span style="color: red;"> no data received!</span> \n"""
        
        #Self parameters
        self.regElems_str = error_str
        self.loc_str = error_str
        self.control_str = error_str
        self.perc_str = error_str
        self.yolo_before_t = 0.0
        self.decision_str = error_str

        while not rospy.is_shutdown(): 
            #Update variables
            try:
                map_name = rospy.get_param("/t4ac/mapping/map_name")
                mapping_str = " map = " + map_name + " | map monitor: "
            except KeyError:
                mapping_str = """<span style="color: red;"> map not defined!</span> \n"""

            header_str = """<span style="color: green;">      CURRENT STATE OF T4AC ARCHITECTURE</span> \n"""
            mapping_str = """<span style="color: green;">MAPPING: </span> """ + mapping_str + self.regElems_str
            self.loc_str = """<span style="color: green;">LOCALIZATION: </span>""" + self.loc_str
            self.control_str = """<span style="color: green;">CONTROL: </span>""" + self.control_str
            # self.perc_str = """<span style="color: green;">AGENT: </span>""" + self.perc_str
            self.decision_str = """<span style="color: green;">ACTION: </span>""" + self.decision_str
            
            text_mgs.text = header_str + mapping_str + self.loc_str + self.control_str + self.perc_str + self.decision_str

            text_pub.publish(text_mgs)
            #Reset variables
            self.regElems_str = error_str
            self.loc_str = error_str
            self.control_str = error_str
            # self.perc_str = error_str
            self.decision_str = error_str

            timer.sleep()
    
    def regElems_callback(self, regElems):
        min_distance = 1000
        nearest_index = 0
        if len(regElems.reg_elems) > 0:
            for index, regElem in enumerate(regElems.reg_elems):
                if (regElem.distance < min_distance):
                    nearest_index = index
                    min_distance = regElem.distance
            
            self.regElems_str = (regElems.reg_elems[nearest_index].type + " is %.*fm from our vehicle \n" 
                                                        % (2, regElems.reg_elems[nearest_index].distance))
        else:
            self.regElems_str = "No relevant regulatory elements \n"        
    
    
    def loc_callback(self, data):
        loc_xy = (data.pose.pose.position.x,
                            data.pose.pose.position.y)
        q = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q)

        vx = data.twist.twist.linear.x
        vy = data.twist.twist.linear.x
        speed = math.sqrt(vx**2 + vy**2) * 3.6
        
        self.loc_str = "x = %.*f | y = %.*f | yaw = %.*f° | speed = %.*fkm/h \n" % (2, loc_xy[0], 2, loc_xy[1], 2, math.degrees(euler[2]), 2, speed)

    def control_callback(self, data):
        self.control_str = "speed = %.*f km/h | steer = %.*f \n" % (2, data.velocity*3.6, 2, data.steer)

    def yolo_callback(self, data):
        self.perc_str = data.data + "\n" 

    def decision_callback(self, data):
        self.decision_str = data.data + "\n" 
        

def main():
    try:
        t4ac_viewer()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down t4ac_viewer node")
    
if __name__ == "__main__":
    main()





