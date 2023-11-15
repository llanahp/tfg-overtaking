#!/usr/bin/env python3
"""
T4ac architecture validation

Authors: J. Felipe Arango.
Last mod: J. Felipe Arango. 24/11/2022
"""

import rospy
import visualization_msgs.msg
import carla 
import numpy as np

class t4ac_validation():
    def __init__(self) -> None:
        ################################### CARLA CONFIGURATION ######################################
        # Carla Utils for GT poses
        client = carla.Client('127.0.0.1', 2000)
        self.world = client.get_world()

        ################################### ROS CONFIGURATION ########################################
        rospy.init_node("t4ac_validation_node", anonymous=True)

        ########## Publishers 
        self.pub_GT_3D_obstacles_markers = rospy.Publisher(
            "/t4ac/perception/3D_GT_obstacles_markers", visualization_msgs.msg.MarkerArray, queue_size=20)

        ########## Subscribers 
        camera_3D_obstacles_sub = rospy.Subscriber("/t4ac/perception/3D_camera_obstacles_markers", visualization_msgs.msg.MarkerArray, self.obstacles_callback)
        
        ################################### Variables ###############################################
        # Frames
        self.map_frame = rospy.get_param('/t4ac/frames/map')
        self.camera_frame = rospy.get_param('/t4ac/frames/camera')

        while not rospy.is_shutdown(): 
            continue

    def obstacles_callback(self, data):
        gt_3D_objects_marker_array = self.getGTdetection(actors = self.world.get_actors())
        error = self.CalculateError(data.markers, gt_3D_objects_marker_array.markers)
        print('Perception error: ', error)

        # publish markers
        self.pub_GT_3D_obstacles_markers.publish(gt_3D_objects_marker_array)
    
    def getGTdetection(self, actors):
        gt_3D_objects_marker_array = visualization_msgs.msg.MarkerArray()
        current_stamp = rospy.get_rostime()
        for i, actor in enumerate(actors.filter('vehicle.*')):
            location = actor.get_location()
            location.y *= -1
            # obstacle_global_position = t4ac_msgs.msg.Node(location.x, location.y, location.z)
            color = [0.0,0.0,1.0,1.0]
            detected_3D_object_marker = self.get_detection_marker(
                i,
                location, # Obstacle position [x y z]
                self.map_frame, # self.camera_dict['frame_id']
                current_stamp,
                color)
            gt_3D_objects_marker_array.markers.append(detected_3D_object_marker)
        
        for i, actor in enumerate(actors.filter('walker.*')):
            location = actor.get_location()
            location.y *= -1
            # obstacle_global_position = t4ac_msgs.msg.Node(location.x, location.y, location.z)
            color = [0.0,1.0,0.0,1.0]
            detected_3D_object_marker = self.get_detection_marker(
                i,
                location, # Obstacle position [x y z]
                self.map_frame, # self.camera_dict['frame_id']
                current_stamp,
                color)
            gt_3D_objects_marker_array.markers.append(detected_3D_object_marker)

        return gt_3D_objects_marker_array
    
    def get_detection_marker(self, index, detection, frame_id, stamp, color):
        """
        """
        detected_3D_object_marker = visualization_msgs.msg.Marker()
        detected_3D_object_marker.header.stamp = stamp
        detected_3D_object_marker.header.frame_id = frame_id
        detected_3D_object_marker.action = visualization_msgs.msg.Marker.ADD
        detected_3D_object_marker.id = index
        detected_3D_object_marker.lifetime = rospy.Duration.from_sec(0.1)
        detected_3D_object_marker.type = visualization_msgs.msg.Marker.CUBE
        detected_3D_object_marker.pose.position.x = detection.x
        detected_3D_object_marker.pose.position.y = detection.y
        detected_3D_object_marker.pose.position.z = detection.z
        detected_3D_object_marker.scale.x = 1
        detected_3D_object_marker.scale.y = 1
        detected_3D_object_marker.scale.z = 1

        detected_3D_object_marker.color.r = color[0]
        detected_3D_object_marker.color.g = color[1]
        detected_3D_object_marker.color.b = color[2]
        detected_3D_object_marker.color.a = color[3]

        return detected_3D_object_marker

    def CalculateError(self, detected_position, ground_truth_position):
        
        error_list = []
        for detection in detected_position:
            detection_array = np.array([detection.pose.position.x, detection.pose.position.y])
            dist_pre = 1000
            for gt in ground_truth_position:
                gt_array = np.array([gt.pose.position.x, gt.pose.position.y])
                dist = np.linalg.norm(detection_array - gt_array)
                
                if dist < dist_pre:
                    dist_pre = dist
            error_list.append(dist_pre)

        error = np.mean(error_list)
        return error

def main():
    try:
        t4ac_validation()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down t4ac_validation node")
    
if __name__ == "__main__":
    main()





