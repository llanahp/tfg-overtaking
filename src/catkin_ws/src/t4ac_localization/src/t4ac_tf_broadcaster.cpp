/***
Authors: J. Felipe Arango.
Last mod: J. Felipe Arango. 24/11/2022
**/

// INCLUIR LA LIBRERIA DE ROS
#include "ros/ros.h"

#include <iostream>
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>

#include "nav_msgs/Odometry.h"

// Frames

std::string world_frame, map_frame, baseLink_frame, 
            lidar_frame, camera_frame, gnss_frame, 
			front_axis_center_frame, front_bumper_center_frame;

std::vector<double> world_to_map_tf, baseLink_to_lidar_tf, 
					baseLink_to_camera_tf, baseLink_to_gnss_tf, 
					baseLink_to_front_axis_center_tf, baseLink_to_front_bumper_center_tf;

std::vector<std::string> frames;
std::vector<std::vector<double>> transforms;

ros::Subscriber pose_sub;

// INCLUIR RESTO DE LIBERIAS A UTILIZAR

	/*********************Euler angles <-> Quaternions ******************/
/*	tf::Quaternion q;
  	q.setRPY(0, 0, 0.785398); //Euler angles(rad) -> Quaternions
	cout << "QUATERNION--> "<< "q.x(): " << q.x() << "  q.y(): " << q.y() << "  q.z(): " << q.z() << "  q.w(): " << q.w() << endl;

	tf::Quaternion q1(0, 0, 7.6512093668e-16, 1);
	tf::Matrix3x3 m(q1);
    	double roll, pitch, yaw;
    	m.getRPY(roll, pitch, yaw); //Quaternions -> Euler angles
	cout << "EULER--> "<< "roll: " << roll << "  pitch: " << pitch << "  yaw: " << yaw << endl; 
*/
	/********************************************************************/ 
// DECLARAR LAS VARIABLES A UTILIZAR
tf::TransformListener *tf_listener;

void pose_Callback(const nav_msgs::Odometry::ConstPtr& msg){
	nav_msgs::Odometry pose_data = *msg;
	
	static tf::TransformBroadcaster broadcaster_tf;
	tf::Transform transform;
	tf::Quaternion q; 
	std::string target_frame, source_frame;
	double tx, ty, tz, roll, pitch, yaw;

	transform.setOrigin(tf::Vector3(pose_data.pose.pose.position.x, pose_data.pose.pose.position.y, pose_data.pose.pose.position.z)); // Translation
	tf::Quaternion q_pose(pose_data.pose.pose.orientation.x, pose_data.pose.pose.orientation.y, pose_data.pose.pose.orientation.z, pose_data.pose.pose.orientation.w);
	transform.setRotation(q_pose); // Rotation -> Quaternions
	broadcaster_tf.sendTransform(tf::StampedTransform(transform, pose_data.header.stamp, map_frame, baseLink_frame));

	int cnt = 0;
	for (std::vector<std::vector<double>>::iterator tf_it = transforms.begin() ; tf_it != transforms.end(); ++tf_it)
	{
		std::vector<double> tf = *tf_it;
		tx = tf[0];
		ty = tf[1];
		tz = tf[2];
		roll = tf[3];
		pitch = tf[4];
		yaw = tf[5];
		q.setRPY(roll, pitch, yaw);
		transform.setOrigin(tf::Vector3(tx, ty, tz)); // Translation
		transform.setRotation(q); // Rotation -> Quaternions

		if (cnt==0) {
			target_frame = frames[0]; // World
			source_frame = frames[1]; // Map
		}
		else{
			target_frame = frames[2]; // Base_link
			source_frame = frames[2+cnt]; // Map
		}
		cnt++;
		broadcaster_tf.sendTransform(tf::StampedTransform(transform, pose_data.header.stamp, target_frame, source_frame));
	}
}

// PROGRAMA PRINCIPAL
int main (int argc, char **argv){
	// INICIALIZAR ROS
	ros::init(argc, argv, "t4ac_tf_broadcaster_node");
	ros::NodeHandle nh;

	// Frames: Third argument represents the default value if ROS does not find the parameter of the first argument
	nh.param<std::string>("/t4ac/frames/world", world_frame, "world");
	frames.push_back(world_frame);
	nh.param<std::string>("/t4ac/frames/map", map_frame, "map");
	frames.push_back(map_frame);
	nh.param<std::string>("/t4ac/frames/base_link", baseLink_frame, "base_link");
	frames.push_back(baseLink_frame);
	nh.param<std::string>("/t4ac/frames/lidar", lidar_frame, "lidar");
	frames.push_back(lidar_frame);
	nh.param<std::string>("/t4ac/frames/camera", camera_frame, "camera");
	frames.push_back(camera_frame);
	nh.param<std::string>("/t4ac/frames/gnss", gnss_frame, "gnss");
	frames.push_back(gnss_frame);
	nh.param<std::string>("/t4ac/frames/front_axis_center", front_axis_center_frame, "front_axis_center");
	frames.push_back(front_axis_center_frame);
	nh.param<std::string>("/t4ac/frames/front_bumper_center", front_bumper_center_frame, "front_bumper_center");
	frames.push_back(front_bumper_center_frame);

	// // Transforms: TF: x,y,z (trans), roll,pitch,yaw (rot) 
	std::vector<double> generic_transform{0,0,0,0,0,0};

	nh.param<std::vector<double>>("/t4ac/tf/world_to_map_tf", world_to_map_tf, generic_transform);
	transforms.push_back(world_to_map_tf);
	nh.param<std::vector<double>>("/t4ac/tf/base_link_to_lidar_tf", baseLink_to_lidar_tf, generic_transform);
	transforms.push_back(baseLink_to_lidar_tf);
	nh.param<std::vector<double>>("/t4ac/tf/base_link_to_camera_tf", baseLink_to_camera_tf, generic_transform);
	transforms.push_back(baseLink_to_camera_tf);
	nh.param<std::vector<double>>("/t4ac/tf/base_link_to_gnss_tf", baseLink_to_gnss_tf, generic_transform);
	transforms.push_back(baseLink_to_gnss_tf);
	nh.param<std::vector<double>>("/t4ac/tf/base_link_to_front_axis_center_tf", baseLink_to_front_axis_center_tf, generic_transform);  
	transforms.push_back(baseLink_to_front_axis_center_tf);
	nh.param<std::vector<double>>("/t4ac/tf/base_link_to_front_bumper_center_tf", baseLink_to_front_bumper_center_tf, generic_transform); 
	transforms.push_back(baseLink_to_front_bumper_center_tf);

	tf_listener = new tf::TransformListener(ros::Duration(5.0)); 

	pose_sub = nh.subscribe("/t4ac/localization/pose", 1, pose_Callback);
		
	ros::spin();	
	return 0;
}
