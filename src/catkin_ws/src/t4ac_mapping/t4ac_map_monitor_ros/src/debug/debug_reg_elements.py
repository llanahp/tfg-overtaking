#!/usr/bin/python3
"""
Map Debug without using PythonAPI

Represent static elements of the map in RVIZ using markers.
Elements represented are:
    - Lanes -> grey
    - Stop signals -> red
    - Yield signals -> blue
    - TrafficLight signals -> green
    - Crosswalks -> white

Authors: J. Felipe Arango.
Last mod: J. Felipe Arango. 24/11/2022
"""
#General Imports
import os
import sys 
import argparse
import time

#ROS Imports
import rospy
import roslaunch
import rospkg
import rosgraph
from visualization_msgs.msg import Marker, MarkerArray


#T4AC Imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from map_parser.map_object import MapObject
from t4ac_map_monitor_ros.src.modules import markers_module
from t4ac_map_monitor_ros.src.modules.utils import recalculate_landmarks

class test_node:
    def __init__(self, args):
        # Mapping config
        map_flag = 0
        self.map_name = args.map
        self.map_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..', "maps/xodr/"))
        self.map_object = MapObject(self.map_name, self.map_path, map_flag)
        # CARLA's HDMaps have errors in the placement of the landmarks, therefore, we made some modifications:
        self.map_object.landmarks = recalculate_landmarks (self.map_object, self.map_name) 

        #ROS config
        os.system('roscore &')
        rospy.init_node('test_node', anonymous=True)

        self.map_pub = rospy.Publisher("/t4ac/mapping/map/lanes_marker", Marker, queue_size=10)
        self.roads_ids_pub = rospy.Publisher("/t4ac/mapping/map/roads_ids", MarkerArray, queue_size=10)
        self.reg_elements_pub = rospy.Publisher("/t4ac/mapping/map/reg_elements", MarkerArray, queue_size=10)
        self.reg_elements_ids_pub = rospy.Publisher("/t4ac/mapping/map/reg_elements_ids", MarkerArray, queue_size=10)

        ## Launch Localization and RVIZ
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        try:
            rosgraph.Master('/rostopic').getPid()
        except:
            launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files=[], is_core=True)
            launch.start()

        rp = rospkg.RosPack()
        launch_path = rp.get_path('t4ac_viz_ros') + '/launch/mapping_debug_rviz.launch'

        t4ac_architecture_launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        t4ac_architecture_launch.start()
        time.sleep(2)
        rospy.loginfo("RVIZ started")
        
        self.rate = rospy.Rate(1)

# ==============================================================================
# -- bridge_loop() -------------------------------------------------------------
# ==============================================================================

def bridge_loop(our_node):
    try:  
        waypoints = our_node.map_object.map_waypoints  ##Default distance_between_waypoints = 0.2
        lanes_markers = markers_module.get_lanes_markers(waypoints, rgb=[192,192,192])
        distance_between_waypoints = 3.0
        waypoints = our_node.map_object.generate_waypoints(our_node.map_object.roads, distance_between_waypoints)
        roads_id_markers = get_roads_id_markers(waypoints)

        landmarks = our_node.map_object.landmarks  
        reg_elements_markers = markers_module.get_reg_elements_markers(landmarks)
        reg_elements_id_markers = get_landmarks_id_markers(landmarks)

        ##############################################################
        landmark_id = 465
        for i in range (0, len(landmarks)-1):
            if (landmarks[i].id == landmark_id):
                print ("landmark found!!!")
                break

        print ("len(landmarks): ", len(landmarks))
        print ("id: ", landmarks[i].id)
        print ("name: ", landmarks[i].name)
        print ("is_dynamic: ", landmarks[i].is_dynamic)
        print ("height: ", landmarks[i].height)
        print ("width: ", landmarks[i].width)
        print ("length: ", landmarks[i].length)
        print ("roll: ", landmarks[i].roll)
        print ("pitch: ", landmarks[i].pitch)
        print ("type: ", landmarks[i].type)

        for affecting_road in landmarks[i].affecting_roads:
            print ("road_id: ", affecting_road.road_id, " ; s: ", affecting_road.pose.s, " ; lanes: ", affecting_road.lanes)

        ###############################################################

        while not rospy.is_shutdown():
            our_node.map_pub.publish(lanes_markers)
            our_node.roads_ids_pub.publish(roads_id_markers)
            our_node.reg_elements_pub.publish(reg_elements_markers)
            our_node.reg_elements_ids_pub.publish(reg_elements_id_markers)
            our_node.rate.sleep()

    except:
        print('\033[1;31m'+'Error during bridge loop, exiting...' +'\033[0;m')

    finally:
        print('\033[1;93m'+'Loop finished.' +'\033[0;m')
        del landmarks

def get_roads_id_markers(road_waypoints):
    """
    Get a list of waypoints with a certain distance between them for every 
    lane and centered inside of it. 
    Returns a marker to visualized the road/lanes id of every waypoint in RVIZ.

    Args:
        road_waypoints: (list) List containing waypoints

    Returns:
        roads_id_markers: List of marker object to be visualized on RVIZ
    """
    roads_id_markers = MarkerArray()
    cnt = 0 

    truee = True

    for waypoint in road_waypoints:
        if (waypoint.road_id == 1050 or waypoint.road_id == 1049):
        #if (truee):
            cnt += 1
            road_id_marker = Marker()
            road_id_marker.header.frame_id = "map"
            road_id_marker.header.stamp = rospy.Time.now()
            road_id_marker.ns = "roads_ids"
            road_id_marker.id = cnt
            road_id_marker.action = Marker.ADD
            road_id_marker.type = Marker.TEXT_VIEW_FACING
            road_id_marker.color.r = 1
            road_id_marker.color.g = 0
            road_id_marker.color.b = 0
            road_id_marker.color.a = 1.0
            road_id_marker.scale.z = 1.0
            road_id_marker.lifetime = rospy.Duration(0)
            road_id_marker.pose.position.x = waypoint.transform.location.x 
            road_id_marker.pose.position.y = waypoint.transform.location.y + 0.5 #Space for lane-id
            road_id_marker.pose.position.z = waypoint.transform.location.z
            road_id_marker.pose.orientation.w = 1.0
            road_id_marker.text = str(waypoint.road_id)
            roads_id_markers.markers.append(road_id_marker)

            lane_id_marker = Marker()
            lane_id_marker.header.frame_id = "map"
            lane_id_marker.header.stamp = rospy.Time.now()
            lane_id_marker.ns = "lanes_ids"
            lane_id_marker.id = cnt
            lane_id_marker.action = Marker.ADD
            lane_id_marker.type = Marker.TEXT_VIEW_FACING
            lane_id_marker.color.r = 0
            lane_id_marker.color.g = 0
            lane_id_marker.color.b = 1
            lane_id_marker.color.a = 1.0
            lane_id_marker.scale.z = 1.0
            lane_id_marker.lifetime = rospy.Duration(0)
            lane_id_marker.pose.position.x = waypoint.transform.location.x
            lane_id_marker.pose.position.y = waypoint.transform.location.y - 0.5 #Space for lane-id
            lane_id_marker.pose.position.z = waypoint.transform.location.z
            lane_id_marker.pose.orientation.w = 1.0
            lane_id_marker.text = str(waypoint.lane_id)
            roads_id_markers.markers.append(lane_id_marker)
            
    return roads_id_markers

def get_landmarks_id_markers(landmarks):
    """
   
    """
    landmarks_id_markers = MarkerArray()

    for landmark in landmarks:
        landmark_id_marker = Marker()
        landmark_id_marker.header.frame_id = "map"
        landmark_id_marker.header.stamp = rospy.Time.now()
        landmark_id_marker.ns = "landmarks_ids"
        landmark_id_marker.id = landmark.id
        landmark_id_marker.action = Marker.ADD
        landmark_id_marker.type = Marker.TEXT_VIEW_FACING
        landmark_id_marker.color.r = 0.0
        landmark_id_marker.color.g = 1.0
        landmark_id_marker.color.b = 1.0
        landmark_id_marker.color.a = 1.0
        landmark_id_marker.scale.z = 1.5
        landmark_id_marker.lifetime = rospy.Duration(0)
        landmark_id_marker.pose.position.x = landmark.transform.location.x 
        landmark_id_marker.pose.position.y = landmark.transform.location.y
        landmark_id_marker.pose.position.z = landmark.transform.location.z
        landmark_id_marker.pose.orientation.w = 1.0
        landmark_id_marker.text = str(landmark.id)
        landmarks_id_markers.markers.append(landmark_id_marker)   
    return landmarks_id_markers

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    # CARLA general parameters
    argparser = argparse.ArgumentParser(description='Techs4AgeCar Mapping Layer Debug\n')
    argparser.add_argument('--map', default='Town03',
                            help='Name of the map (default: Town03)')
    args = argparser.parse_args()
    print('\033[1;32m'+'Debugging map %s.' % (args.map) +'\033[0;m')

    try:
        our_node = test_node(args)
        bridge_loop(our_node)
    except:
        print('\033[1;31m'+'Error during map_object creation, exiting...' +'\033[0;m')
    finally:
        print('\033[1;93m'+'Debug finished.' +'\033[0;m')

        script_path = os.path.expandvars('$AIVATAR_PROJECT_PATH') + '/utils_docker/clear_run_evaluation.sh'
        os.system(script_path)
        del our_node

if __name__ == "__main__":
    main()